#pragma once

#include <Arduino.h>

// --- Wiring -----------------------------------------------------------------
// A4988 soldered pin-side-down onto the RP2040 Zero, so the driver's pin order
// maps straight onto consecutive GPIOs.
#define STEP_PIN_ENABLE 1   // active LOW: LOW = coils energised
#define STEP_PIN_MS1    2
#define STEP_PIN_MS2    3
#define STEP_PIN_MS3    4
#define STEP_PIN_RESET  5   // active LOW: LOW = translator held at home state
#define STEP_PIN_SLEEP  6   // active LOW: LOW = sleep
#define STEP_PIN_STEP   7
#define STEP_PIN_DIR    8

// --- Motor ------------------------------------------------------------------
#define STEP_FULL_STEPS_PER_REV 200   // 1.8 deg/step

// --- Driver timing (A4988 datasheet minima, with margin) --------------------
#define STEP_PULSE_US   2   // STEP high time, min 1 us
#define STEP_SETUP_US   2   // DIR setup before STEP rising edge, min 200 ns
#define STEP_WAKE_US 1000   // charge-pump settling after leaving sleep, min 1 ms

// Microstep resolution. The value is the divisor, so it doubles as the
// steps-per-full-step multiplier used in all the position maths.
enum Microstep : uint8_t {
  MICROSTEP_FULL  = 1,
  MICROSTEP_HALF  = 2,
  MICROSTEP_QTR   = 4,
  MICROSTEP_EIGHTH = 8,
  MICROSTEP_SIXTEENTH = 16,
};

class Stepper {
 public:
  Stepper(uint8_t stepPin   = STEP_PIN_STEP,
          uint8_t dirPin    = STEP_PIN_DIR,
          uint8_t enablePin = STEP_PIN_ENABLE,
          uint8_t ms1Pin    = STEP_PIN_MS1,
          uint8_t ms2Pin    = STEP_PIN_MS2,
          uint8_t ms3Pin    = STEP_PIN_MS3,
          uint8_t resetPin  = STEP_PIN_RESET,
          uint8_t sleepPin  = STEP_PIN_SLEEP);

  // Drives every pin to a known state: awake, out of reset, disabled (coils
  // cold), full-step. Call from setup().
  void begin(Microstep res = MICROSTEP_SIXTEENTH);

  // --- Power ----------------------------------------------------------------
  void enable();               // energise coils (holding torque, draws current)
  void disable();              // release coils; the shaft can be turned by hand
  bool isEnabled() const { return enabled_; }

  // Sleep cuts the regulator and most of the chip: much lower quiescent draw
  // than disable(), but the translator's step position is preserved. Waking
  // blocks for STEP_WAKE_US.
  void sleep();
  void wake();
  bool isAsleep() const { return asleep_; }

  // Pulses RESET, returning the translator to its home microstep. The motor
  // may physically jump by up to one full step, so currentPosition() is not
  // meaningful across a reset -- setCurrentPosition() afterwards if it matters.
  void resetDriver();

  // --- Microstepping --------------------------------------------------------
  // Rescales currentPosition/targetPosition so a change of resolution keeps the
  // same physical angle. Speeds and accelerations are in steps, so they scale
  // with the resolution too -- call setSpeed()/setAcceleration() after this if
  // you want to hold a physical rate.
  void setMicrostep(Microstep res);
  Microstep microstep() const { return res_; }
  long stepsPerRev() const { return (long)STEP_FULL_STEPS_PER_REV * res_; }

  // --- Motion profile -------------------------------------------------------
  void setMaxSpeed(float stepsPerSec);       // clamped to > 0
  void setAcceleration(float stepsPerSecSq); // 0 disables ramping
  float maxSpeed() const { return maxSpeed_; }
  float acceleration() const { return accel_; }

  // --- Targets --------------------------------------------------------------
  void moveTo(long absolute);
  void move(long relative);
  void moveToAngle(float degrees);           // absolute, from position 0
  void moveAngle(float degrees);             // relative

  long currentPosition() const { return pos_; }
  void setCurrentPosition(long p);           // also clears the target and speed
  long targetPosition() const { return target_; }
  long distanceToGo() const { return target_ - pos_; }
  float currentAngle() const { return 360.0f * pos_ / stepsPerRev(); }
  float currentSpeed() const { return dir_ ? speed_ : -speed_; }
  bool isRunning() const { return pos_ != target_ || speed_ != 0.0f; }

  // --- Running --------------------------------------------------------------
  // Non-blocking: emits at most one step, when one is due. Poll from loop().
  // Returns true while there is still motion left to do.
  bool run();

  // Blocks until the target is reached. Enables the driver if needed.
  void runToPosition();
  void runToNewPosition(long absolute);

  // Ignores acceleration and free-runs at `stepsPerSec` (sign = direction)
  // until stopNow() or a new target. Poll run() as usual.
  void setConstantSpeed(float stepsPerSec);

  // Ramps down over the shortest distance the current acceleration allows and
  // retargets there. With no acceleration set this is the same as stopNow().
  void stop();

  // Drops the target and the speed immediately. Loses steps under load.
  void stopNow();

  // Emits one step in `forward`, ignoring the profile and the target. Blocks
  // for the pulse. Useful for jogging and homing.
  void stepOnce(bool forward);

 private:
  void applyMicrostepPins();
  void setDirection(bool forward);
  void pulse();
  void computeNextInterval();

  const uint8_t stepPin_, dirPin_, enablePin_;
  const uint8_t ms1Pin_, ms2Pin_, ms3Pin_;
  const uint8_t resetPin_, sleepPin_;

  Microstep res_     = MICROSTEP_FULL;
  bool enabled_      = false;
  bool asleep_       = false;
  bool dir_          = true;   // true = forward / increasing position

  long  pos_         = 0;
  long  target_      = 0;

  float maxSpeed_    = 1000.0f;
  float accel_       = 0.0f;
  bool  constant_    = false;

  // Austin's ramp: n is the step index along the ramp, cn the current interval.
  long     rampStep_ = 0;
  float    cn_       = 0.0f;   // us
  float    c0_       = 0.0f;   // us, first interval of a ramp
  float    cmin_     = 1000.0f;// us, interval at maxSpeed_
  float    speed_    = 0.0f;   // steps/s, unsigned
  uint32_t interval_ = 0;      // us until the next step, 0 = none due
  uint32_t lastStep_ = 0;      // micros() of the last pulse
};
