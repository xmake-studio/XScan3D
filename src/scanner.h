#pragma once

#include <Arduino.h>

#include "lidar_parser.h"
#include "protocol.h"   // SCAN_MODE_*, the command letters and their limits
#include "stepper.h"

// The Scanner is the whole application: the motor, the lidar link and the USB
// protocol, all on core0. There is no second core and no sensor fusion -- the
// lidar sits directly on the stepper shaft, so the step count is the pose.

// --- Scan geometry ----------------------------------------------------------
// The lidar is bolted to the shaft with its scan plane vertical, and the shaft
// turns about the world vertical axis. Half a turn therefore sweeps that
// vertical plane through every azimuth and covers the whole sphere, which is
// why the default half-sweep is the full 90 the limits allow: -90 -> +90.
//
// Both of these are only the power-on defaults; the host can override them at
// runtime with the 'a' and 't' commands.
#define SCAN_DEGREES 90.0f
#define SCAN_TIME    30.0f   // seconds for the full 2*SCAN_DEGREES sweep

// The shaft carries the lidar directly -- no gearing, no linkage, no backlash.
// One motor revolution is one platform revolution, so the position maths below
// is the motor's own and there is no ratio to apply.

// The full 1/16. With the lidar sitting straight on the shaft there is almost
// nothing to turn, so the torque that coarser stepping buys is wasted; what
// finer stepping buys instead is smoothness, and that is the whole game here.
// At 200 full steps/rev this is 3200 microsteps per turn, or 0.1125 deg each.
#define SCAN_MICROSTEP MICROSTEP_SIXTEENTH

// Getting to the start of the sweep is dead time, so it runs fast and ramped.
#define SCAN_TRAVEL_SPEED 1600.0f   // microsteps/s, = 180 deg/s
#define SCAN_TRAVEL_ACCEL 3200.0f   // microsteps/s^2

// Parking overshoots slightly and the sensor rings; let it die out before the
// data that has to be accurate starts flowing.
#define SCAN_SETTLE_MS 1500

// --- Idle power -------------------------------------------------------------
// Holding torque costs the full coil current and dumps it into the A4988 and
// the motor as heat, for as long as the rig sits there doing nothing -- which
// between scans is most of its life. So the coils are released once the
// platform has been parked and still for this long.
//
// The cost is real and worth stating plainly: with the coils cold the shaft is
// backdrivable, so a platform that is not balanced about its axis will sag, and
// currentPosition() then means nothing. If yours does sag, either balance it or
// set this to 0 to keep the always-energised behaviour. The grace period also
// keeps a back-to-back 's' from chattering the driver, and lets the parking
// wobble die before the brake comes off.
#define SCAN_IDLE_DISABLE_MS 750

// --- Stepped mode -----------------------------------------------------------
// A continuous sweep stamps each frame with the angle the shaft was passing
// through as it arrived, which is exact but smears anything the lidar's own
// buzz adds. Stepped mode removes that: it stops at each stop, waits out the
// ring-down, then captures with the shaft genuinely stationary.
//
// It is much slower -- both windows are paid at every one of steps+1 stops --
// which is the trade. Use it when a continuous scan comes out smeared.
#define SCAN_STEPS_DEFAULT   60
#define SCAN_STEP_SETTLE_MS  250   // ring-down after a move, before capturing
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
  SCAN_STEP_MOVE,     // travelling to the next stop
  SCAN_STEP_SETTLE,   // stopped, waiting for the ring-down
  SCAN_STEP_CAPTURE,  // stopped, streaming lidar at a fixed angle
  SCAN_UNWRAP,        // turning the shaft to unwind the tether
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
  void serviceLidar();
  void serviceTelemetry();
  void serviceIdlePower();
  void advanceStateMachine();

  void startScan();
  void startUnwrap(float deg);
  void abort(const char *why);
  void emitEvent(const char *fmt, ...);

  void advanceStep();                  // move to the next stop, or finish
  float stepAngle(uint16_t i) const;   // shaft degrees at stop `i`

  // Energises the coils if they were released, and warns once when the
  // released interval means the angle can no longer be trusted.
  void engageMotor();

  float platformDeg() const;
  long  platformDegToSteps(float deg) const;

  Stepper  motor_;
  ScanState state_    = SCAN_IDLE;
  uint32_t stateAt_   = 0;   // millis() when the current state was entered
  uint32_t nextTelem_ = 0;   // millis()

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

  char    line_[24];         // one command line being assembled
  uint8_t lineLen_ = 0;

  // Set when the coils are released, cleared once the host has been told the
  // angle may have moved, so the warning is one per release and not per scan.
  bool angleStale_ = false;
};
