#include "scanner.h"

#include <stdarg.h>
#include <stdio.h>

#include "lidar_parser.h"
#include "protocol.h"

namespace {

void putMagic(uint8_t *p, uint8_t tag) {
  p[0] = PKT_MAGIC0;
  p[1] = PKT_MAGIC1;
  p[2] = PKT_MAGIC2;
  p[3] = tag;
}

}  // namespace

// --- Geometry ---------------------------------------------------------------
// The lidar sits on the shaft, so platform angle and motor angle are the same
// number. These two exist anyway because everything below reads better for
// saying which of the two it means.

float Scanner::platformDeg() const {
  return motor_.currentAngle();
}

long Scanner::platformDegToSteps(float deg) const {
  return lroundf(deg * motor_.stepsPerRev() / 360.0f);
}

// Stop `i` of steps_ intervals, so i == 0 is -scanDegrees_ and i == steps_ is
// +scanDegrees_. Computed from the index rather than accumulated, so the last
// stop lands exactly on +scanDegrees_ instead of wherever rounding drifted to.
float Scanner::stepAngle(uint16_t i) const {
  if (steps_ == 0) return -scanDegrees_;
  return -scanDegrees_ + (2.0f * scanDegrees_ * i) / steps_;
}

// --- Setup ------------------------------------------------------------------

void Scanner::begin() {
  // Proof of life before a single peripheral is touched. Every hardware init
  // below can fail or block, and without this the whole class of boot hang
  // looks identical to dead firmware from the host's side.
  emitEvent("boot: scanner starting");

  // The motor claims its pins first, so nothing else can take them later.
  motor_.begin(SCAN_MICROSTEP);
  motor_.setMaxSpeed(SCAN_TRAVEL_SPEED);
  motor_.setAcceleration(SCAN_TRAVEL_ACCEL);
  motor_.enable();
  // Wherever the rig happens to be sitting is angle zero until told otherwise.
  motor_.setCurrentPosition(0);
  emitEvent("boot: stepper on GPIO%d-%d, 1/%d step, %ld/rev",
            STEP_PIN_ENABLE, STEP_PIN_DIR, (int)SCAN_MICROSTEP,
            motor_.stepsPerRev());

  lidarBegin();
  emitEvent("boot: lidar uart on RX%d/TX%d", LIDAR_RX_PIN, LIDAR_TX_PIN);

  nextTelem_ = millis();
  state_     = SCAN_IDLE;
  stateAt_   = millis();

  emitConfig();
  emitEvent("ready: sweep %+.0f..%+.0f deg in %.0f s", -scanDegrees_,
            scanDegrees_, scanTime_);
}

// --- Emission ---------------------------------------------------------------

void Scanner::emitEvent(const char *fmt, ...) {
  char text[PKT_EVENT_MAX];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(text)) n = sizeof(text);
  if (Serial.availableForWrite() < n + 5) return;

  uint8_t hdr[5];
  putMagic(hdr, PKT_EVENT_TAG);
  hdr[4] = (uint8_t)n;
  Serial.write(hdr, sizeof(hdr));
  Serial.write((const uint8_t *)text, n);
}

// --- Services ---------------------------------------------------------------

void Scanner::serviceLidar() {
  LidarFrame frame;
  while (lidarPoll(frame)) {
    // Keep the step train running between frames: a burst of queued frames
    // must not stall the motor and dent the constant-rate sweep.
    motor_.run();

    // If the host has stopped draining the pipe, drop the sample rather than
    // block here -- a blocked write would stall the motor and corrupt the
    // sweep geometry for every point that follows.
    if ((size_t)Serial.availableForWrite() < sizeof(PktSample)) {
      if (dropped_ < 0xFFFF) dropped_++;
      continue;
    }

    PktSample s;
    putMagic(s.magic, PKT_SAMPLE_LEN);
    s.t_us        = micros();
    s.platformDeg = platformDeg();
    s.lidarSpeed  = frame.speed;
    s.rawAngle    = frame.rawAngle;
    for (uint8_t i = 0; i < LIDAR_POINTS; i++) {
      s.dist[i] = frame.points[i].valid ? frame.points[i].distance
                                        : LIDAR_DIST_INVALID;
    }
    Serial.write((const uint8_t *)&s, sizeof(s));
  }
}

void Scanner::serviceTelemetry() {
  if ((int32_t)(millis() - nextTelem_) < 0) return;
  nextTelem_ += TELEM_PERIOD_MS;

  if ((size_t)Serial.availableForWrite() < sizeof(PktTelem)) return;

  PktTelem t;
  putMagic(t.magic, PKT_TELEM_LEN);
  t.t_us        = micros();
  t.platformDeg = platformDeg();
  t.state       = (uint8_t)state_;
  t.reserved    = 0;
  t.dropped     = dropped_;
  Serial.write((const uint8_t *)&t, sizeof(t));
}

// --- Idle power -------------------------------------------------------------

void Scanner::serviceIdlePower() {
  if (SCAN_IDLE_DISABLE_MS == 0) return;
  if (state_ != SCAN_IDLE) return;
  if (!motor_.isEnabled() || motor_.isRunning()) return;
  if (millis() - stateAt_ < SCAN_IDLE_DISABLE_MS) return;

  motor_.disable();
  angleStale_ = true;
  emitEvent("motor: coils released at %+.2f deg (idle)", platformDeg());
}

void Scanner::engageMotor() {
  if (motor_.isEnabled()) return;
  motor_.enable();
  if (angleStale_) {
    angleStale_ = false;
    // Not an error -- with the coils cold the shaft turns freely, so this is
    // the honest statement that the angle is only as good as the rig's balance
    // since the release. 'h' re-zeros it if the platform was moved on purpose.
    emitEvent("motor: re-energised, angle assumed unchanged at %+.2f deg",
              platformDeg());
  }
}

// --- Stepped mode -----------------------------------------------------------

void Scanner::advanceStep() {
  if (stepIndex_ >= steps_) {
    emitEvent("stepped scan complete: %u stops, dropped=%u",
              (unsigned)(steps_ + 1), (unsigned)dropped_);
    state_   = SCAN_DONE;
    stateAt_ = millis();
    motor_.setMaxSpeed(SCAN_TRAVEL_SPEED);
    motor_.setAcceleration(SCAN_TRAVEL_ACCEL);
    motor_.moveTo(0);
    return;
  }

  stepIndex_++;
  // Unramped: the gap between adjacent stops is a fraction of a degree, so a
  // ramp would spend the whole move accelerating and decelerating and leave
  // more ring behind than it saved.
  motor_.setAcceleration(0.0f);
  motor_.setMaxSpeed(SCAN_STEP_TRAVEL_SPEED);
  motor_.moveTo(platformDegToSteps(stepAngle(stepIndex_)));
  state_   = SCAN_STEP_MOVE;
  stateAt_ = millis();
}

// --- Commands ---------------------------------------------------------------

void Scanner::emitConfig() {
  // Every write on this link is guarded: a blocked Serial.write would stall
  // loop(), and stalling loop() stops the step train mid-sweep. Losing a
  // status message is always preferable to denting the scan geometry.
  if ((size_t)Serial.availableForWrite() < sizeof(PktConfig)) return;
  PktConfig c;
  putMagic(c.magic, PKT_CONFIG_LEN);
  c.scanDegrees = scanDegrees_;
  c.scanTime    = scanTime_;
  c.mode        = mode_;
  c.reserved    = 0;
  c.steps       = steps_;
  c.settleMs    = SCAN_STEP_SETTLE_MS;
  c.captureMs   = dwellMs_;
  Serial.write((const uint8_t *)&c, sizeof(c));
}

// Accumulates a line, then dispatches. A lone command letter with no newline
// still fires as soon as the next letter arrives, so a serial monitor works.
void Scanner::handleCommands() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;

    if (c == '\r' || c == '\n') {
      if (lineLen_ > 0) {
        line_[lineLen_] = '\0';
        runCommand(line_[0], line_ + 1, lineLen_ > 1);
        lineLen_ = 0;
      }
      continue;
    }
    if (c == ' ' || c == '\t') continue;

    // A second command letter means the previous line had no argument and no
    // terminator; flush it rather than gluing the two together.
    if (lineLen_ > 0 && isalpha(c)) {
      line_[lineLen_] = '\0';
      runCommand(line_[0], line_ + 1, lineLen_ > 1);
      lineLen_ = 0;
    }

    if (lineLen_ < sizeof(line_) - 1) {
      line_[lineLen_++] = (char)c;
    } else {
      lineLen_ = 0;  // overlong garbage; drop it and resync on the next line
    }
  }
}

void Scanner::runCommand(char cmd, const char *arg, bool hasArg) {
  switch (cmd) {
    case CMD_START:  startScan(); break;
    case CMD_ABORT:  abort("host"); break;

    case CMD_HOME:
      // Deliberately works with the coils released: park the platform by hand,
      // then 'h' to call that zero.
      motor_.setCurrentPosition(0);
      angleStale_ = false;
      emitEvent("home: platform angle is now 0");
      break;

    case CMD_UNWRAP:
      startUnwrap(hasArg ? atof(arg) : SCAN_UNWRAP_DEG);
      break;

    case CMD_ANGLE:
    case CMD_TIME: {
      if (!hasArg) { emitConfig(); break; }
      if (state_ != SCAN_IDLE && state_ != SCAN_DONE) {
        emitEvent("busy: cannot change settings mid-sweep");
        break;
      }
      const float v = atof(arg);
      if (cmd == CMD_ANGLE) {
        if (v < SCAN_DEGREES_MIN || v > SCAN_DEGREES_MAX) {
          emitEvent("angle %.2f out of range %.0f..%.0f", v,
                    SCAN_DEGREES_MIN, SCAN_DEGREES_MAX);
          break;
        }
        scanDegrees_ = v;
      } else {
        if (v < SCAN_TIME_MIN || v > SCAN_TIME_MAX) {
          emitEvent("time %.2f out of range %.0f..%.0f", v, SCAN_TIME_MIN,
                    SCAN_TIME_MAX);
          break;
        }
        scanTime_ = v;
      }
      emitConfig();
      break;
    }

    case CMD_MODE:
    case CMD_STEPS:
    case CMD_DWELL: {
      if (!hasArg) { emitConfig(); break; }
      if (state_ != SCAN_IDLE && state_ != SCAN_DONE) {
        emitEvent("busy: cannot change settings mid-sweep");
        break;
      }
      const long v = atol(arg);
      if (cmd == CMD_MODE) {
        if (v != SCAN_MODE_CONTINUOUS && v != SCAN_MODE_STEPPED) {
          emitEvent("mode %ld unknown (0=continuous, 1=stepped)", v);
          break;
        }
        mode_ = (uint8_t)v;
      } else if (cmd == CMD_STEPS) {
        if (v < SCAN_STEPS_MIN || v > SCAN_STEPS_MAX) {
          emitEvent("steps %ld out of range %d..%d", v, SCAN_STEPS_MIN,
                    SCAN_STEPS_MAX);
          break;
        }
        steps_ = (uint16_t)v;
      } else {
        if (v < SCAN_DWELL_MIN || v > SCAN_DWELL_MAX) {
          emitEvent("dwell %ld out of range %d..%d ms", v, SCAN_DWELL_MIN,
                    SCAN_DWELL_MAX);
          break;
        }
        dwellMs_ = (uint16_t)v;
      }
      emitConfig();
      break;
    }

    case CMD_STATUS:
      nextTelem_ = millis();
      emitConfig();
      emitEvent("state=%u platform=%+.2f deg motor=%s dropped=%u resync=%lu",
                (unsigned)state_, platformDeg(),
                motor_.isEnabled() ? "on" : "off", (unsigned)dropped_,
                (unsigned long)lidarResyncBytes());
      break;

    default:
      emitEvent("unknown command '%c'", cmd);
      break;
  }
}

// --- State machine ----------------------------------------------------------

void Scanner::startScan() {
  if (state_ != SCAN_IDLE && state_ != SCAN_DONE) {
    emitEvent("busy: already scanning");
    return;
  }
  dropped_ = 0;
  // Latch the mode for the duration: parking and settling are shared, and the
  // branch out of SETTLING must go where the scan started, not where a command
  // that arrived in the meantime points.
  scanMode_  = mode_;
  stepIndex_ = 0;
  emitConfig();  // pin the geometry this scan was taken with into the stream
  engageMotor();
  motor_.setMaxSpeed(SCAN_TRAVEL_SPEED);
  motor_.setAcceleration(SCAN_TRAVEL_ACCEL);
  motor_.moveTo(platformDegToSteps(-scanDegrees_));
  state_   = SCAN_PARKING;
  stateAt_ = millis();
  emitEvent("parking to %+.1f deg", -scanDegrees_);
  if (scanMode_ == SCAN_MODE_STEPPED) {
    // The runtime is set by the dwell windows, not by scanTime_, so state it
    // up front rather than letting the operator infer it from the duration
    // field the UI is still showing.
    const float secs = (steps_ + 1) *
                       (SCAN_STEP_SETTLE_MS + dwellMs_) / 1000.0f;
    emitEvent("stepped: %u stops of %.2f deg, ~%.0f s plus travel",
              (unsigned)(steps_ + 1), (2.0f * scanDegrees_) / steps_, secs);
  }
}

// Turn the shaft to unwind the tether that runs up it, then call where it
// landed home. Re-homing is the whole point: the sweep is symmetric about home,
// so without it the very next scan would drive straight back through the turn
// just taken out and wind the twist right back in.
void Scanner::startUnwrap(float deg) {
  if (state_ != SCAN_IDLE && state_ != SCAN_DONE) {
    emitEvent("busy: cannot unwrap mid-sweep");
    return;
  }
  if (!(deg > -SCAN_UNWRAP_DEGMAX - 0.001f &&
        deg < SCAN_UNWRAP_DEGMAX + 0.001f) || deg == 0.0f) {
    emitEvent("unwrap %.1f out of range +/-%.0f", deg, SCAN_UNWRAP_DEGMAX);
    return;
  }
  engageMotor();
  motor_.setMaxSpeed(SCAN_TRAVEL_SPEED);
  motor_.setAcceleration(SCAN_TRAVEL_ACCEL);
  motor_.move(platformDegToSteps(deg));
  state_   = SCAN_UNWRAP;
  stateAt_ = millis();
  emitEvent("unwrap: turning %+.1f deg", deg);
}

void Scanner::abort(const char *why) {
  motor_.stop();
  state_   = SCAN_IDLE;
  stateAt_ = millis();
  emitEvent("aborted (%s)", why);
}

void Scanner::advanceStateMachine() {
  switch (state_) {
    case SCAN_PARKING:
      if (!motor_.isRunning()) {
        state_   = SCAN_SETTLING;
        stateAt_ = millis();
        emitEvent("settling %u ms", (unsigned)SCAN_SETTLE_MS);
      }
      break;

    case SCAN_SETTLING:
      if (millis() - stateAt_ >= SCAN_SETTLE_MS) {
        if (scanMode_ == SCAN_MODE_STEPPED) {
          // Already parked at stop 0 and already settled, so the per-step
          // settle window would be redundant here; go straight to capturing.
          emitEvent("stepped: capturing stop 1/%u at %+.2f deg",
                    (unsigned)(steps_ + 1), stepAngle(0));
          state_   = SCAN_STEP_CAPTURE;
          stateAt_ = millis();
          break;
        }
        // Constant rate for the sweep itself: with acceleration disabled the
        // driver steps at exactly maxSpeed, so shaft angle is linear in time
        // and the azimuth slices come out evenly spaced. The ramp that makes
        // travel fast would bunch points at both ends of the sweep.
        const float platformDegPerSec = (2.0f * scanDegrees_) / scanTime_;
        const float stepsPerSec =
            platformDegPerSec * motor_.stepsPerRev() / 360.0f;
        motor_.setAcceleration(0.0f);
        motor_.setMaxSpeed(stepsPerSec);
        motor_.moveTo(platformDegToSteps(scanDegrees_));
        state_   = SCAN_SWEEPING;
        stateAt_ = millis();
        emitEvent("sweeping at %.2f microsteps/s", stepsPerSec);
      }
      break;

    case SCAN_SWEEPING:
      if (!motor_.isRunning()) {
        emitEvent("sweep complete in %.1f s, dropped=%u",
                  (millis() - stateAt_) / 1000.0f, (unsigned)dropped_);
        state_   = SCAN_DONE;
        stateAt_ = millis();
        motor_.setMaxSpeed(SCAN_TRAVEL_SPEED);
        motor_.setAcceleration(SCAN_TRAVEL_ACCEL);
        motor_.moveTo(0);
      }
      break;

    case SCAN_DONE:
      if (!motor_.isRunning()) {
        state_   = SCAN_IDLE;
        stateAt_ = millis();   // also starts the idle-release countdown
        emitEvent("idle");
      }
      break;

    case SCAN_UNWRAP:
      if (!motor_.isRunning()) {
        motor_.setCurrentPosition(0);
        angleStale_ = false;
        state_      = SCAN_IDLE;
        stateAt_    = millis();
        emitEvent("unwrap done: this is now home");
      }
      break;

    case SCAN_STEP_MOVE:
      if (!motor_.isRunning()) {
        state_   = SCAN_STEP_SETTLE;
        stateAt_ = millis();
      }
      break;

    case SCAN_STEP_SETTLE:
      if (millis() - stateAt_ >= SCAN_STEP_SETTLE_MS) {
        state_   = SCAN_STEP_CAPTURE;
        stateAt_ = millis();
      }
      break;

    case SCAN_STEP_CAPTURE:
      if (millis() - stateAt_ >= dwellMs_) advanceStep();
      break;

    case SCAN_IDLE:
    default:
      break;
  }
}

void Scanner::update() {
  motor_.run();
  handleCommands();
  serviceLidar();
  serviceTelemetry();
  advanceStateMachine();
  serviceIdlePower();
}
