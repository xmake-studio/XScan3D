#include "ahrs.h"

#include <TroykaIMU.h>

// Everything in this file runs on core1 unless the comment says otherwise.
// The only exceptions are the four entry points at the bottom, which are core0
// and touch nothing but the seqlock.

namespace {

Madgwick      filter;
Gyroscope     gyroscope;
Accelerometer accelerometer;

// --- Shared state -----------------------------------------------------------
// A seqlock, not a mutex. Core1 must never block: a stalled AHRS tick is the
// exact defect this whole change exists to remove, and core0 must never block
// either because it is holding the step train up. So the writer never waits,
// and the reader retries on the rare tear -- at 100 Hz against a reader that
// polls far faster, a retry is close to unobservable and always terminates.
//
// Odd sequence means a write is in progress. Both fences are load-bearing: the
// stores to the payload must not be hoisted above the odd store nor sunk below
// the even one, and Cortex-M0+ will happily reorder them otherwise.
//
// Core1 keeps its own working copies of all of this and publishes whole
// structs; nothing below is ever mutated in place while a reader might be
// looking at it.
volatile uint32_t g_seq = 0;

AhrsSample g_sample = {0, 1.0f, 0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
AhrsStatus g_status = {};
bool       g_live   = false;   // the filter has produced at least one output

// Core0 bumps the request, core1 bumps the ack when it has honoured it. Each
// is written by exactly one core and is a naturally aligned 32-bit word, so
// neither needs the seqlock -- and a request raised mid-calibration collapses
// into the one already running rather than queueing a second.
volatile uint32_t g_calibReq = 0;
volatile uint32_t g_calibAck = 0;

// Core1 waits on this so its bus probe cannot race core0's pin setup.
volatile bool g_go = false;

inline void fence() { __atomic_thread_fence(__ATOMIC_SEQ_CST); }

// core1 only. Publishes both payloads under one sequence bump. `s` may be null
// to republish the status alone, which is how a calibration announces itself.
void publish(const AhrsStatus &st, const AhrsSample *s) {
  g_seq++;
  fence();
  if (s) {
    g_sample = *s;
    g_live   = true;
  }
  g_status = st;
  fence();
  g_seq++;
}

// --- Input conditioning -----------------------------------------------------
// One-pole low pass, one instance per axis. The coefficient is fixed because
// the tick rate is: core1 holds AHRS_HZ to within a few microseconds, which is
// the whole reason the filter lives here, so recomputing alpha from a measured
// dt would only add jitter of its own.
//
// The first sample is loaded straight in rather than filtered towards from
// zero. Starting at zero would feed the Madgwick filter a fake gravity vector
// swinging up from nothing over the first few hundred milliseconds, and it
// would faithfully track that.
struct OnePole {
  float y = 0.0f;
  bool  primed = false;

  float operator()(float x, float alpha) {
    if (!primed) {
      primed = true;
      y = x;
    } else {
      y += alpha * (x - y);
    }
    return y;
  }
};

// alpha for a one-pole at `cutoff` Hz sampled at AHRS_HZ.
constexpr float lpfAlpha(float cutoff) {
  return (2.0f * (float)M_PI * cutoff / (float)AHRS_HZ) /
         (2.0f * (float)M_PI * cutoff / (float)AHRS_HZ + 1.0f);
}

const float kAccelAlpha = lpfAlpha(AHRS_ACCEL_CUTOFF_HZ);
const float kGyroAlpha  = lpfAlpha(AHRS_GYRO_CUTOFF_HZ);

OnePole g_lpAx, g_lpAy, g_lpAz;
OnePole g_lpGx, g_lpGy, g_lpGz;

// --- Bus discovery ----------------------------------------------------------
// Candidate I2C pin pairs, restricted to GPIOs this board is not already using
// and to combinations the RP2040's pin mux actually supports (I2C0 SDA on
// 0/4/8/12.., SCL on 1/5/9/13..; I2C1 SDA on 2/6/10/14/26, SCL on
// 3/7/11/15/27). The stepper owns 1-8 and the lidar 12/13.
struct I2CCandidate {
  TwoWire *bus;
  uint8_t  sda, scl;
};
const I2CCandidate kI2C[IMU_SCAN_CANDIDATES] = {
  {&Wire1, IMU_SDA_PIN, IMU_SCL_PIN},  // the configured wiring
  {&Wire1, 14, 15},
  {&Wire1, 26, 27},
  {&Wire,   0,  9},
};

TwoWire *g_bus = nullptr;

bool findImuBus(AhrsStatus &st) {
  for (uint8_t c = 0; c < IMU_SCAN_CANDIDATES; c++) {
    TwoWire *bus = kI2C[c].bus;
    bus->end();
    bus->setSDA(kI2C[c].sda);
    bus->setSCL(kI2C[c].scl);
    bus->setClock(100000);
    bus->begin();

    // Probe with an address-only transaction. endTransmission() reports the
    // ACK without ever calling read(), so a bus with nothing on it returns an
    // error instead of blocking the way a read would.
    uint8_t found = 0;
    for (uint8_t a = I2C_ADDR_LO; a <= I2C_ADDR_HI; a++) {
      bus->beginTransmission(a);
      if (bus->endTransmission() == 0) {
        if (found < AHRS_MAX_ADDRS) st.addr[found] = a;
        found++;
      }
    }

    if (found) {
      g_bus     = bus;
      st.sda    = kI2C[c].sda;
      st.scl    = kI2C[c].scl;
      st.found  = found;
      return true;
    }
    bus->end();
  }
  return false;
}

// --- Calibration ------------------------------------------------------------
// The gyro's zero-rate offset is the single biggest error source in a 30 s
// sweep: an uncorrected 1 deg/s bias walks the yaw 30 deg across one scan.
// Averaging with the rig stationary knocks it down to the noise floor.
//
// This blocks core1 for ~2 s, which is now free: core0 keeps stepping, keeps
// draining the lidar and keeps answering the host throughout. On core0 this
// was a hard stall inside setup() with the USB link already open.
void calibrateGyro(AhrsStatus &st) {
  double sx = 0.0, sy = 0.0, sz = 0.0;
  for (int i = 0; i < AHRS_BIAS_SAMPLES; i++) {
    float x, y, z;
    gyroscope.readRotationRadXYZ(x, y, z);
    sx += x; sy += y; sz += z;
    delay(1000 / AHRS_HZ);
  }
  st.bias[0] = (float)(sx / AHRS_BIAS_SAMPLES);
  st.bias[1] = (float)(sy / AHRS_BIAS_SAMPLES);
  st.bias[2] = (float)(sz / AHRS_BIAS_SAMPLES);
  st.calibCount++;
  filter.reset();
  // The low passes have been running on pre-calibration gyro values and the
  // filter has just been reset, so let both re-prime from the next real
  // sample rather than carrying stale state across the discontinuity.
  g_lpAx = g_lpAy = g_lpAz = OnePole();
  g_lpGx = g_lpGy = g_lpGz = OnePole();
}

}  // namespace

// --- core1 ------------------------------------------------------------------

void ahrs::core1Main() {
  while (!g_go) tight_loop_contents();
  fence();

  AhrsStatus st = {};

  filter.begin();
  filter.setFrequency(AHRS_HZ);
  // Must come after begin(): it seeds beta with the library default, which is
  // far too permissive for a rig that vibrates. See ahrs.h. Zeta stays 0 --
  // the gyro bias is measured explicitly by calibrateGyro(), and letting the
  // filter chase it as well just gives two estimators fighting over the same
  // error.
  filter.setSettings(AHRS_BETA, 0.0f);

  st.ok = findImuBus(st);
  if (st.ok) {
    gyroscope.begin(*g_bus);
    accelerometer.begin(*g_bus);
    g_calibAck = g_calibReq;   // the boot calibration is not a host request
    st.calibrating = true;
    publish(st, nullptr);
    calibrateGyro(st);
    st.calibrating = false;
  }
  st.ready = true;
  publish(st, nullptr);

  // Free-running deadline, advanced by exactly one period per tick so the rate
  // cannot drift with the cost of a read. Unsigned differences throughout, so
  // it stays correct across the micros() wrap every ~71 minutes.
  uint32_t next = micros();

  for (;;) {
    if (g_calibReq != g_calibAck) {
      st.calibrating = true;
      publish(st, nullptr);          // let core0 announce the hold-still
      if (st.ok) calibrateGyro(st);
      g_calibAck     = g_calibReq;
      st.calibrating = false;
      next = micros();               // the 2 s hold is not a pile of late ticks
      publish(st, nullptr);
      continue;
    }

    const int32_t due = (int32_t)(micros() - next);
    if (due < 0) {
      tight_loop_contents();
      continue;
    }
    next += AHRS_PERIOD_US;

    // A whole period late means something took longer than 10 ms -- an I2C
    // retry, most likely. Resync rather than trying to catch up: a burst of
    // back-to-back ticks would feed the filter samples whose real dt is far
    // below the fixed dt it assumes, which is worse than the gap itself.
    if (due >= (int32_t)AHRS_PERIOD_US) {
      if (st.late < 0xFFFF) st.late++;
      next = micros() + AHRS_PERIOD_US;
    }

    if (!st.ok) continue;   // no bus: nothing to read, nothing to say

    AhrsSample s;
    s.t_us = micros();
    float ax, ay, az, gx, gy, gz;
    accelerometer.readAccelerationGXYZ(ax, ay, az);
    gyroscope.readRotationRadXYZ(gx, gy, gz);
    gx -= st.bias[0];
    gy -= st.bias[1];
    gz -= st.bias[2];

    // Band-limit before the filter, and publish the filtered values rather
    // than the raw ones: the host's gravity-direction check and the live
    // attitude readout both want the same signal the quaternion was built
    // from, not a noisier parallel copy of it.
    s.ax = g_lpAx(ax, kAccelAlpha);
    s.ay = g_lpAy(ay, kAccelAlpha);
    s.az = g_lpAz(az, kAccelAlpha);
    s.gx = g_lpGx(gx, kGyroAlpha);
    s.gy = g_lpGy(gy, kGyroAlpha);
    s.gz = g_lpGz(gz, kGyroAlpha);

    // Six-DOF on purpose: the magnetometer sits centimetres from an energised
    // stepper, so its heading is worse than the drift it would correct. Roll
    // and pitch -- the axes the sweep actually tilts through -- stay
    // accel-corrected.
    filter.update(s.gx, s.gy, s.gz, s.ax, s.ay, s.az);
    filter.readQuaternion(s.qw, s.qx, s.qy, s.qz);

    publish(st, &s);
  }
}

// --- core0 ------------------------------------------------------------------

void ahrs::begin() {
  fence();
  g_go = true;
}

bool ahrs::read(AhrsSample &out) {
  uint32_t s0;
  bool live;
  do {
    s0 = g_seq;
    fence();
    out  = g_sample;
    live = g_live;
    fence();
  } while ((s0 & 1) || s0 != g_seq);
  return live;
}

AhrsStatus ahrs::status() {
  AhrsStatus out;
  uint32_t s0;
  do {
    s0 = g_seq;
    fence();
    out = g_status;
    fence();
  } while ((s0 & 1) || s0 != g_seq);
  return out;
}

void ahrs::requestCalibration() { g_calibReq++; }
