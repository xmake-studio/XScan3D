#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "lidar_parser.h"
#include "stepper.h"

// The AHRS runs on core1, alone. On core0 it was a cooperative task competing
// with the step train, the lidar drain and the USB writes, so its 100 Hz tick
// was 100 Hz only on average: an I2C read that landed behind a burst of lidar
// frames arrived late, and Madgwick integrates with a fixed dt regardless. The
// error that produced is exactly the slow yaw walk the gyro bias calibration
// exists to remove. Core1 has nothing else to do, so the period is now held to
// within a few microseconds and the filter's dt is honest.
//
// The split is strict, and it is the whole reason this is safe without locks
// around the peripherals:
//   core0  Serial (all of it), the stepper, the lidar UART
//   core1  I2C, the IMU, the Madgwick filter
// Nothing is touched from both. In particular core1 never writes to Serial --
// USB CDC is not reentrant, and a diagnostic that corrupted the sample stream
// would cost more than it explained. Core1 records what it wants to say in the
// status block instead and core0 does the talking.

// --- I2C -------------------------------------------------------------------
// The default Wire pins on this core are GPIO4/5, which are STEP_PIN_MS3 and
// STEP_PIN_RESET on the A4988. Letting the IMU library call Wire.begin() with
// the defaults put the I2C block and the stepper on the same two pads: the
// motor's pinMode() then stole them back, and the next I2C read wedged the bus
// before a single byte of output had been written. Hence explicit pins, and
// the static_asserts below so this cannot recur silently.
//
// Confirmed wiring: the IMU is on I2C1, SDA GPIO10 / SCL GPIO11.
//
// Note that GPIO9 + GPIO10 is not a legal pair on this chip. The pin mux fixes
// each GPIO to one role on one block: 9 can only be I2C0 SCL, 10 can only be
// I2C1 SDA. Worse, TwoWire::setSDA() does not report an illegal pin, it calls
// panic() -- the chip halts, which from the host looks exactly like firmware
// that never booted. On core1 that is even quieter than it used to be, since
// core0 keeps answering commands while the AHRS never comes up; the status
// block below is what makes that state visible instead of merely silent.
#define IMU_SDA_PIN 10
#define IMU_SCL_PIN 11

// Valid SDA pins are I2C0 {0,4,8,12,16,20,24,28}, I2C1 {2,6,10,14,18,22,26};
// valid SCL are I2C0 {1,5,9,13,17,21,25,29}, I2C1 {3,7,11,15,19,23,27}. Catch
// a bad edit here rather than in a runtime panic with no output.
static_assert(IMU_SDA_PIN % 4 == 2, "IMU_SDA_PIN is not a valid I2C1 SDA pin");
static_assert(IMU_SCL_PIN % 4 == 3, "IMU_SCL_PIN is not a valid I2C1 SCL pin");
static_assert(IMU_SDA_PIN < STEP_PIN_ENABLE || IMU_SDA_PIN > STEP_PIN_DIR,
              "IMU SDA collides with the stepper");
static_assert(IMU_SCL_PIN < STEP_PIN_ENABLE || IMU_SCL_PIN > STEP_PIN_DIR,
              "IMU SCL collides with the stepper");
static_assert(IMU_SDA_PIN != LIDAR_RX_PIN && IMU_SDA_PIN != LIDAR_TX_PIN &&
              IMU_SCL_PIN != LIDAR_RX_PIN && IMU_SCL_PIN != LIDAR_TX_PIN,
              "IMU pins collide with the lidar UART");

// The configured pair is tried first; the rest are a fallback so a wiring
// change reports itself in the boot log instead of just going quiet.
#define IMU_SCAN_CANDIDATES 4

// Address range worth probing (7-bit, excluding reserved ranges).
#define I2C_ADDR_LO 0x08
#define I2C_ADDR_HI 0x77

static_assert(STEP_PIN_MS3 == 4 && STEP_PIN_RESET == 5,
              "stepper moved: recheck the I2C candidate pins in ahrs.cpp");

// --- Rates ------------------------------------------------------------------
#define AHRS_HZ        100
#define AHRS_PERIOD_US (1000000UL / AHRS_HZ)

// Samples averaged to estimate gyro bias. At 100 Hz this is ~2 s, and it must
// run with the rig stationary or the whole sweep inherits the error.
#define AHRS_BIAS_SAMPLES 200

// --- Noise rejection --------------------------------------------------------
// The lidar spins a few centimetres from the IMU and its imbalance shows up as
// a continuous buzz on the accelerometer. Two things then turn that buzz into
// the ~1 deg jitter that was visible on the attitude readout:
//
//   1. The library's default Madgwick beta is derived from a 40 deg/s gyro
//      error estimate, which works out at ~0.6. That is enormous: it tells the
//      filter to trust the accelerometer almost completely, so every vibration
//      spike is taken as a genuine change in the gravity direction and yanks
//      the quaternion with it. The platform moves at ~2 deg/s during a sweep,
//      so the filter needs nothing like that much authority to track it.
//      0.03 still pulls out real gyro drift within a second or two while
//      ignoring buzz.
//
//   2. Nothing band-limited the inputs. The vibration sits well above the
//      motion we care about, so a one-pole low pass on each axis removes it
//      before the filter ever sees it.
//
// Both are deliberately conservative: raise AHRS_BETA if the attitude is slow
// to settle after a real move, lower the cutoffs if buzz still gets through.
#define AHRS_BETA 0.03f

// One-pole cutoffs, Hz. The accelerometer is the noisy one and only has to
// track a 2 deg/s tilt, so it is filtered hard. The gyro is much quieter and
// carries the fast motion, so it keeps a wider band.
#define AHRS_ACCEL_CUTOFF_HZ 3.0f
#define AHRS_GYRO_CUTOFF_HZ  20.0f

// How many addresses of a busy bus we bother remembering for the boot log.
#define AHRS_MAX_ADDRS 8

// Everything core1 produces, as one consistent set. Taken as a snapshot rather
// than field by field: a quaternion assembled from two different filter ticks
// is not a rotation, and it would be stamped onto lidar samples as if it were.
struct AhrsSample {
  uint32_t t_us;                // micros() at the start of the tick
  float    qw, qx, qy, qz;      // sensor->world
  float    ax, ay, az;          // g
  float    gx, gy, gz;          // rad/s, bias-corrected
};

// What core1 would have printed, had it been allowed to print.
struct AhrsStatus {
  bool     ready;               // core1 has finished bringing the IMU up
  bool     ok;                  // ...and something actually answered
  bool     calibrating;         // holding still, averaging the gyro
  uint8_t  sda, scl;            // the pair that won
  uint8_t  found;               // devices that answered on it
  uint8_t  addr[AHRS_MAX_ADDRS];
  float    bias[3];             // rad/s, subtracted from every gyro read
  uint32_t calibCount;          // bumped on each completed calibration
  uint16_t late;                // ticks that missed their deadline outright
};

namespace ahrs {

// Called from core0. Releases core1 from its start gate; nothing touches I2C
// before this returns, so core0's pin setup cannot race the bus probe.
void begin();

// Core1's entry point. Never returns. Call it from loop1() and nothing else.
void core1Main();

// Core0: latest consistent snapshot. Returns false until the filter has
// produced its first output (no IMU means never), in which case `out` is the
// identity rotation and zeroed vectors -- usable, just not informative.
bool read(AhrsSample &out);

// Core0: ask for a fresh gyro bias. Returns immediately; watch
// status().calibCount to see it land. Ignored if one is already in flight.
void requestCalibration();

// Core0: a consistent copy of the block above.
AhrsStatus status();

}  // namespace ahrs
