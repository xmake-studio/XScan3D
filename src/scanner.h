#pragma once

#include <Arduino.h>

#include "ahrs.h"
#include "lidar_parser.h"
#include "protocol.h"   // SCAN_MODE_*, the command letters and their limits
#include "stepper.h"

// The Scanner runs entirely on core0: the motor, the lidar link and the USB
// protocol. The IMU and the Madgwick filter live on core1 -- see ahrs.h for
// why, and for the core-ownership rules that keep the split lock-free. This
// class only ever reads snapshots out of ahrs::, it never touches I2C.

// --- Scan geometry ----------------------------------------------------------
// The platform tilts +/-SCAN_DEGREES about the stepper's axis (which is the
// sensor's Y) while the lidar keeps sweeping its own plane, so the two
// rotations together cover a band of the sphere. A sweep runs -SCAN_DEGREES ->
// +SCAN_DEGREES. Both of these are only the power-on defaults now; the host
// can override them at runtime with the 'a' and 't' commands.
#define SCAN_DEGREES 30.0f
#define SCAN_TIME    30.0f   // seconds for the full 2*SCAN_DEGREES sweep

// 2 motor revolutions per platform revolution.
#define SCAN_GEAR_RATIO 2.0f

// Finer microstepping buys smoothness, which matters more than torque here:
// the sweep is slow and the load is a few hundred grams of sensor.
#define SCAN_MICROSTEP MICROSTEP_SIXTEENTH

// Getting to the start of the sweep is dead time, so it runs fast and ramped.
#define SCAN_TRAVEL_SPEED 800.0f    // microsteps/s
#define SCAN_TRAVEL_ACCEL 1600.0f   // microsteps/s^2

// Parking overshoots slightly and the sensor rings; let it die out before the
// data that has to be accurate starts flowing.
#define SCAN_SETTLE_MS 1500

// --- Idle power -------------------------------------------------------------
// Holding torque costs the full coil current and dumps it into the A4988 and
// the motor as heat, for as long as the rig sits there doing nothing -- which
// between scans is most of its life. It also bakes that heat into the IMU
// sitting centimetres away, and a drifting bias is the one error a 30 s sweep
// cannot absorb. So the coils are released once the platform has been parked
// and still for this long.
//
// The cost is real and worth stating plainly: with the coils cold the shaft is
// backdrivable, so a platform that is not balanced about its axis will sag,
// and currentPosition() then means nothing. If yours does sag, either balance
// it or set this to 0 to keep the old always-energised behaviour. The grace
// period also keeps a back-to-back 's' from chattering the driver, and lets
// the parking wobble die before the brake comes off.
#define SCAN_IDLE_DISABLE_MS 750

// --- Stepped mode -----------------------------------------------------------
// A continuous sweep asks the AHRS to track a moving platform while the lidar
// buzzes it, and the pose stamped on a frame is only as good as the filter's
// instantaneous output. Stepped mode removes both problems by never capturing
// while moving: it stops at each stop, waits out the ring-down, averages the
// IMU over a window with the platform genuinely stationary, then freezes that
// pose and stamps it on every frame captured before the next move.
//
// It is much slower -- these three windows are paid at every one of steps+1
// stops -- which is the trade. Use it when a continuous scan comes out smeared.
#define SCAN_STEPS_DEFAULT   60
#define SCAN_STEP_SETTLE_MS  250   // ring-down after a move, before averaging
#define SCAN_STEP_AVERAGE_MS 200   // IMU averaging window, ~20 ticks at 100 Hz
#define SCAN_STEP_CAPTURE_MS 400   // lidar dwell, ~2 revs of a 300 RPM lidar

// Travel between adjacent stops is a fraction of a degree, so the ramp never
// reaches speed and only adds ring. Moves at this rate, unramped.
#define SCAN_STEP_TRAVEL_SPEED 400.0f

#define TELEM_PERIOD_MS 100

enum ScanState : uint8_t {
  SCAN_IDLE = 0,
  SCAN_PARKING,    // travelling to -SCAN_DEGREES
  SCAN_SETTLING,   // parked, waiting for the wobble to stop
  SCAN_SWEEPING,   // the run that produces the cloud
  SCAN_DONE,
  // Stepped mode. Appended rather than inserted: the host decodes this as a
  // plain integer and inserting would silently relabel every existing capture.
  SCAN_STEP_MOVE,     // travelling to the next stop
  SCAN_STEP_SETTLE,   // stopped, waiting for the ring-down
  SCAN_STEP_AVERAGE,  // stopped and quiet, averaging the IMU
  SCAN_STEP_CAPTURE,  // stopped, streaming lidar against the frozen pose
};

// Owns the motor and the USB link. main() just pumps it.
class Scanner {
 public:
  void begin();

  // Non-blocking; call as often as possible from loop().
  void update();

  ScanState state() const { return state_; }

 private:
  void handleCommands();
  void runCommand(char cmd, const char *arg, bool hasArg);
  void emitConfig();
  void serviceAhrs();
  void serviceLidar();
  void serviceTelemetry();
  void serviceIdlePower();
  void advanceStateMachine();

  void startScan();
  void abort(const char *why);
  void emitImuStatus();
  void emitEvent(const char *fmt, ...);

  // --- Stepped mode ---------------------------------------------------------
  void beginAverage();          // enter SCAN_STEP_AVERAGE with a clean sum
  void accumulateAverage();     // fold in each new AHRS tick; called from update()
  void finishAverage();         // normalise the sum into poseQ_
  void advanceStep();           // move to the next stop, or finish the scan
  float stepAngle(uint16_t i) const;   // platform degrees at stop `i`

  // Energises the coils if they were released, and warns once when the
  // released interval means the angle can no longer be trusted.
  void engageMotor();

  float platformDeg() const;
  long  platformDegToSteps(float deg) const;

  Stepper  motor_;
  ScanState state_    = SCAN_IDLE;
  uint32_t stateAt_   = 0;   // millis() when the current state was entered
  uint32_t nextTelem_ = 0;   // millis()

  // The last snapshot pulled off core1, refreshed once per loop and stamped
  // onto every sample and telemetry record.
  AhrsSample ahrs_ = {0, 1.0f, 0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  // Edges of core1's status that core0 has already announced. Core1 cannot
  // write to Serial, so this is how its news reaches the host.
  bool     imuReported_  = false;
  bool     calibSeen_    = false;
  uint32_t calibCount_   = 0;
  uint16_t lateSeen_     = 0;

  uint16_t dropped_ = 0;     // samples binned because the host stopped reading

  // Live sweep parameters, seeded from the defaults above and overridable by
  // the host. Changing them mid-sweep is refused rather than applied, so the
  // geometry of a scan in flight cannot shift under the host's feet.
  float scanDegrees_ = SCAN_DEGREES;
  float scanTime_    = SCAN_TIME;

  // Stepped mode. mode_ is latched into scanMode_ at startScan() so a mode
  // change that arrives mid-scan cannot switch state machines under the sweep.
  uint8_t  mode_      = SCAN_MODE_CONTINUOUS;
  uint8_t  scanMode_  = SCAN_MODE_CONTINUOUS;
  uint16_t steps_     = SCAN_STEPS_DEFAULT;
  uint16_t dwellMs_   = SCAN_STEP_CAPTURE_MS;
  uint16_t stepIndex_ = 0;      // which stop we are at, 0..steps_

  // The averaged pose for the current stop, stamped on every frame captured
  // there. Seeded to identity so a stop that saw no IMU ticks at all emits a
  // valid rotation rather than a zero quaternion the host cannot normalise.
  float poseQw_ = 1.0f, poseQx_ = 0.0f, poseQy_ = 0.0f, poseQz_ = 0.0f;

  // Running sum for the averaging window. Doubles because a 200 ms window is
  // only ~20 terms but the components are near-unity and near-cancelling.
  double  sumQw_ = 0.0, sumQx_ = 0.0, sumQy_ = 0.0, sumQz_ = 0.0;
  uint16_t avgCount_ = 0;
  uint32_t avgLastT_ = 0;       // t_us of the last tick folded in, to skip repeats

  char    line_[24];         // one command line being assembled
  uint8_t lineLen_ = 0;

  // Set when the coils are released, cleared once the host has been told the
  // angle may have moved, so the warning is one per release and not per scan.
  bool angleStale_ = false;
};
