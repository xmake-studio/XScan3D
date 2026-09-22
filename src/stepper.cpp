#include "stepper.h"

#include <math.h>

Stepper::Stepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin,
                 uint8_t ms1Pin, uint8_t ms2Pin, uint8_t ms3Pin,
                 uint8_t resetPin, uint8_t sleepPin)
    : stepPin_(stepPin), dirPin_(dirPin), enablePin_(enablePin),
      ms1Pin_(ms1Pin), ms2Pin_(ms2Pin), ms3Pin_(ms3Pin),
      resetPin_(resetPin), sleepPin_(sleepPin) {}

void Stepper::begin(Microstep res) {
  pinMode(stepPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);
  pinMode(enablePin_, OUTPUT);
  pinMode(ms1Pin_, OUTPUT);
  pinMode(ms2Pin_, OUTPUT);
  pinMode(ms3Pin_, OUTPUT);
  pinMode(resetPin_, OUTPUT);
  pinMode(sleepPin_, OUTPUT);

  digitalWrite(stepPin_, LOW);
  digitalWrite(dirPin_, HIGH);
  digitalWrite(enablePin_, HIGH);  // start disabled: no current, no heat
  digitalWrite(resetPin_, HIGH);   // out of reset
  digitalWrite(sleepPin_, HIGH);   // awake
  enabled_ = false;
  asleep_  = false;
  dir_     = true;
  delayMicroseconds(STEP_WAKE_US);

  res_ = res;
  applyMicrostepPins();
  setMaxSpeed(maxSpeed_);
  lastStep_ = micros();
}

// --- Power ------------------------------------------------------------------

void Stepper::enable() {
  digitalWrite(enablePin_, LOW);
  enabled_ = true;
}

void Stepper::disable() {
  digitalWrite(enablePin_, HIGH);
  enabled_ = false;
}

void Stepper::sleep() {
  if (asleep_) return;
  digitalWrite(sleepPin_, LOW);
  asleep_ = true;
}

void Stepper::wake() {
  if (!asleep_) return;
  digitalWrite(sleepPin_, HIGH);
  asleep_ = false;
  // The charge pump has to come back up before a STEP edge will be honoured.
  delayMicroseconds(STEP_WAKE_US);
  lastStep_ = micros();
}

void Stepper::resetDriver() {
  digitalWrite(resetPin_, LOW);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(resetPin_, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  lastStep_ = micros();
}

// --- Microstepping ----------------------------------------------------------

void Stepper::applyMicrostepPins() {
  // A4988 truth table, MS3:MS2:MS1.
  bool ms1, ms2, ms3;
  switch (res_) {
    case MICROSTEP_HALF:       ms1 = true;  ms2 = false; ms3 = false; break;
    case MICROSTEP_QTR:        ms1 = false; ms2 = true;  ms3 = false; break;
    case MICROSTEP_EIGHTH:     ms1 = true;  ms2 = true;  ms3 = false; break;
    case MICROSTEP_SIXTEENTH:  ms1 = true;  ms2 = true;  ms3 = true;  break;
    case MICROSTEP_FULL:
    default:                   ms1 = false; ms2 = false; ms3 = false; break;
  }
  digitalWrite(ms1Pin_, ms1 ? HIGH : LOW);
  digitalWrite(ms2Pin_, ms2 ? HIGH : LOW);
  digitalWrite(ms3Pin_, ms3 ? HIGH : LOW);
}

void Stepper::setMicrostep(Microstep res) {
  if (res == res_) return;
  // Keep the physical angle: positions are in microsteps, so rescale them.
  pos_    = (pos_    * (long)res) / (long)res_;
  target_ = (target_ * (long)res) / (long)res_;
  res_ = res;
  applyMicrostepPins();
  computeNextInterval();
}

// --- Motion profile ---------------------------------------------------------

void Stepper::setMaxSpeed(float stepsPerSec) {
  if (stepsPerSec <= 0.0f) stepsPerSec = 1.0f;
  if (maxSpeed_ == stepsPerSec && cmin_ > 0.0f) return;
  maxSpeed_ = stepsPerSec;
  cmin_ = 1e6f / stepsPerSec;
  // Rejoin the ramp at the step index matching the new ceiling, so a speed
  // change mid-move doesn't restart the acceleration from zero.
  if (rampStep_ > 0 && accel_ > 0.0f) {
    rampStep_ = (long)((maxSpeed_ * maxSpeed_) / (2.0f * accel_));
    computeNextInterval();
  }
}

void Stepper::setAcceleration(float stepsPerSecSq) {
  if (stepsPerSecSq < 0.0f) stepsPerSecSq = -stepsPerSecSq;
  if (accel_ == stepsPerSecSq) return;
  if (stepsPerSecSq > 0.0f) {
    // Austin, "Generate stepper-motor speed profiles in real time" (eq. 15),
    // with the 0.676 correction for the first-step truncation error.
    if (accel_ > 0.0f) rampStep_ = (long)(rampStep_ * (accel_ / stepsPerSecSq));
    c0_ = 0.676f * sqrtf(2.0f / stepsPerSecSq) * 1e6f;
  } else {
    rampStep_ = 0;
  }
  accel_ = stepsPerSecSq;
  computeNextInterval();
}

// --- Targets ----------------------------------------------------------------

void Stepper::moveTo(long absolute) {
  if (target_ == absolute && !constant_) return;
  constant_ = false;
  target_ = absolute;
  computeNextInterval();
}

void Stepper::move(long relative) { moveTo(pos_ + relative); }

void Stepper::moveToAngle(float degrees) {
  moveTo((long)lroundf(degrees * stepsPerRev() / 360.0f));
}

void Stepper::moveAngle(float degrees) {
  move((long)lroundf(degrees * stepsPerRev() / 360.0f));
}

void Stepper::setCurrentPosition(long p) {
  pos_ = target_ = p;
  rampStep_ = 0;
  speed_ = 0.0f;
  interval_ = 0;
  constant_ = false;
}

void Stepper::setConstantSpeed(float stepsPerSec) {
  if (stepsPerSec == 0.0f) { stopNow(); return; }
  constant_ = true;
  dir_ = stepsPerSec > 0.0f;
  speed_ = fabsf(stepsPerSec);
  if (speed_ > maxSpeed_) speed_ = maxSpeed_;
  rampStep_ = 0;
  cn_ = 1e6f / speed_;
  interval_ = (uint32_t)cn_;
  // Free-run: keep the target out of reach in the chosen direction.
  target_ = dir_ ? 0x7FFFFFFFL : -0x7FFFFFFFL;
}

void Stepper::stop() {
  if (speed_ == 0.0f) { stopNow(); return; }
  constant_ = false;
  if (accel_ <= 0.0f) { stopNow(); return; }
  long stepsToStop = (long)((speed_ * speed_) / (2.0f * accel_)) + 1;
  moveTo(dir_ ? pos_ + stepsToStop : pos_ - stepsToStop);
}

void Stepper::stopNow() {
  target_ = pos_;
  rampStep_ = 0;
  speed_ = 0.0f;
  interval_ = 0;
  constant_ = false;
}

// --- Running ----------------------------------------------------------------

void Stepper::setDirection(bool forward) {
  if (dir_ == forward) return;
  dir_ = forward;
  digitalWrite(dirPin_, forward ? HIGH : LOW);
  delayMicroseconds(STEP_SETUP_US);
}

void Stepper::pulse() {
  digitalWrite(stepPin_, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin_, LOW);
}

// Decides the delay before the next step, and which way it goes. Sets
// interval_ to 0 when the move is finished.
void Stepper::computeNextInterval() {
  const long distanceTo = target_ - pos_;

  if (constant_) {
    interval_ = (uint32_t)cn_;
    return;
  }

  if (accel_ <= 0.0f) {
    // No ramping: step at max speed, stop dead at the target.
    if (distanceTo == 0) { interval_ = 0; speed_ = 0.0f; return; }
    setDirection(distanceTo > 0);
    speed_ = maxSpeed_;
    cn_ = cmin_;
    interval_ = (uint32_t)cmin_;
    return;
  }

  const long stepsToStop = (long)((speed_ * speed_) / (2.0f * accel_));

  if (distanceTo == 0 && stepsToStop <= 1) {
    interval_ = 0;
    speed_ = 0.0f;
    rampStep_ = 0;
    return;
  }

  if (distanceTo > 0) {
    if (rampStep_ > 0) {
      // Accelerating: start braking if we are inside the stopping distance,
      // or if we are still coasting the wrong way.
      if (stepsToStop >= distanceTo || !dir_) rampStep_ = -stepsToStop;
    } else if (rampStep_ < 0) {
      // Braking: resume accelerating if there is room again.
      if (stepsToStop < distanceTo && dir_) rampStep_ = -rampStep_;
    }
  } else if (distanceTo < 0) {
    if (rampStep_ > 0) {
      if (stepsToStop >= -distanceTo || dir_) rampStep_ = -stepsToStop;
    } else if (rampStep_ < 0) {
      if (stepsToStop < -distanceTo && !dir_) rampStep_ = -rampStep_;
    }
  }

  if (rampStep_ == 0) {
    cn_ = c0_;
    setDirection(distanceTo > 0);
  } else {
    cn_ -= (2.0f * cn_) / (4.0f * rampStep_ + 1.0f);
    if (cn_ < cmin_) cn_ = cmin_;
  }
  rampStep_++;
  interval_ = (uint32_t)cn_;
  speed_ = 1e6f / cn_;
}

bool Stepper::run() {
  if (interval_ == 0) {
    computeNextInterval();
    if (interval_ == 0) return false;
  }

  // Unsigned subtraction, so this stays correct across the micros() wrap.
  const uint32_t now = micros();
  if (now - lastStep_ < interval_) return true;

  pulse();
  lastStep_ = now;
  pos_ += dir_ ? 1 : -1;

  computeNextInterval();
  return interval_ != 0 || pos_ != target_;
}

void Stepper::runToPosition() {
  if (!enabled_) enable();
  if (asleep_) wake();
  while (run()) {
    // Steps are emitted from run(); nothing else to do but let it finish.
  }
}

void Stepper::runToNewPosition(long absolute) {
  moveTo(absolute);
  runToPosition();
}

void Stepper::playTone(uint16_t freqHz, uint16_t ms, uint8_t swing,
                       uint16_t fadeMs) {
  if (freqHz == 0) { delay(ms); lastStep_ = micros(); return; }
  if (swing == 0) swing = 1;
  const uint32_t period = 1000000UL / freqHz;
  uint32_t cycles = ((uint32_t)ms * freqHz) / 1000UL;
  if (cycles == 0) cycles = 1;
  uint32_t fade = ((uint32_t)fadeMs * freqHz) / 1000UL;
  if (fade > cycles / 2) fade = cycles / 2;

  const bool savedDir = dir_;
  uint32_t t = micros();
  for (uint32_t i = 0; i < cycles; i++) {
    // Swing for this cycle: ramps 1 -> swing over the fade-in, and mirrors
    // that over the fade-out.
    uint32_t s = swing;
    if (fade > 0) {
      const uint32_t edge = i < cycles - 1 - i ? i : cycles - 1 - i;
      if (edge < fade) s = 1 + (swing - 1) * edge / fade;
    }
    // 2*s pulses per cycle, evenly spaced, so the pitch holds while the
    // amplitude changes.
    const uint32_t gap = period / (2 * s);
    for (uint8_t half = 0; half < 2; half++) {
      setDirection(half == 0);
      for (uint32_t k = 0; k < s; k++) {
        pulse();
        t += gap;
        while ((int32_t)(micros() - t) < 0) {}
      }
    }
  }
  setDirection(savedDir);
  lastStep_ = micros();
}

void Stepper::stepOnce(bool forward) {
  setDirection(forward);
  pulse();
  pos_ += forward ? 1 : -1;
  target_ = pos_;
  lastStep_ = micros();
}
