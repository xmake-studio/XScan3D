#include "scanner.h"

#include <stdarg.h>
#include <stdio.h>

#include "ahrs.h"
#include "lidar_parser.h"
#include "protocol.h"

// Core0 only. The IMU, the filter and the I2C bus belong to core1; see ahrs.h.

namespace {

void putMagic(uint8_t *p, uint8_t tag) {
  p[0] = PKT_MAGIC0;
  p[1] = PKT_MAGIC1;
  p[2] = PKT_MAGIC2;
  p[3] = tag;
}

}  // namespace

// --- Geometry ---------------------------------------------------------------

float Scanner::platformDeg() const {
  return motor_.currentAngle() / SCAN_GEAR_RATIO;
}

long Scanner::platformDegToSteps(float deg) const {
  return lroundf(deg * SCAN_GEAR_RATIO * motor_.stepsPerRev() / 360.0f);
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
  emitEvent("boot: stepper on GPIO%d-%d", STEP_PIN_ENABLE, STEP_PIN_DIR);

  lidarBegin();
  emitEvent("boot: lidar uart on RX%d/TX%d", LIDAR_RX_PIN, LIDAR_TX_PIN);

  // Core1 has been spinning on its start gate since boot; releasing it here
  // means its I2C probe cannot land in the middle of the pin setup above.
  // Bring-up and the ~2 s gyro bias average then run in parallel with the rest
  // of this function -- boot no longer stalls on them, and the host can drive
  // the rig immediately. serviceAhrs() reports the outcome when it arrives.
  ahrs::begin();
  emitEvent("boot: ahrs handed to core1 at %d Hz", AHRS_HZ);

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

// Everything core1 wanted to say about the bus, said from the core that owns
// Serial. Repeated on demand because the boot log is lost to any host that
// connects after the fact, which is every host.
void Scanner::emitImuStatus() {
  const AhrsStatus st = ahrs::status();

  if (!st.ready) {
    emitEvent("imu: core1 still bringing the bus up");
    return;
  }
  if (!st.ok) {
    emitEvent("imu: ABSENT - no I2C device on any candidate pair "
              "(10/11, 14/15, 26/27, 0/9)");
    return;
  }

  emitEvent("imu: SDA%u/SCL%u, %u device(s), late=%u", st.sda, st.scl,
            st.found, (unsigned)st.late);
  const uint8_t n = st.found < AHRS_MAX_ADDRS ? st.found : AHRS_MAX_ADDRS;
  for (uint8_t i = 0; i < n; i++) {
    emitEvent("i2c: device 0x%02X on SDA%u/SCL%u", st.addr[i], st.sda, st.scl);
  }
  emitEvent("gyro bias %.4f %.4f %.4f rad/s", st.bias[0], st.bias[1],
            st.bias[2]);
}

// --- Services ---------------------------------------------------------------

// Core0's half of the AHRS: pull the latest snapshot, and turn core1's status
// edges into events. No I2C, no filter maths, and nothing here can block.
void Scanner::serviceAhrs() {
  ahrs::read(ahrs_);

  const AhrsStatus st = ahrs::status();

  if (st.ready && !imuReported_) {
    imuReported_ = true;
    emitImuStatus();
    if (!st.ok) {
      // Keep running: the stepper angle alone still reconstructs a cloud, so a
      // missing IMU costs slop compensation rather than the whole scan.
      emitEvent("IMU not found - running without AHRS, "
                "reconstruct with --from-stepper");
    }
  }

  if (st.calibrating && !calibSeen_) {
    calibSeen_ = true;
    emitEvent("gyro: averaging bias, hold still");
  } else if (!st.calibrating && calibSeen_) {
    calibSeen_ = false;
  }

  if (st.calibCount != calibCount_) {
    calibCount_ = st.calibCount;
    if (imuReported_) {
      emitEvent("gyro bias %.4f %.4f %.4f rad/s", st.bias[0], st.bias[1],
                st.bias[2]);
    }
  }

  // A late tick means core1 missed a 10 ms deadline, which should not happen
  // now that it has the core to itself. If it starts happening the AHRS is
  // silently degrading, so say so rather than burying it in the '?' output.
  if (st.late != lateSeen_) {
    lateSeen_ = st.late;
    emitEvent("ahrs: %u late tick(s)", (unsigned)st.late);
  }
}

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

    // Re-read rather than reusing the snapshot from the top of the loop: a
    // burst of queued frames can span several 10 ms AHRS ticks, and the point
    // of stamping each frame is that the pose is the one from its own moment.
    ahrs::read(ahrs_);

    // ...except while stepping, where the platform is deliberately stationary
    // and the pose worth stamping is the average taken over the quiet window,
    // not whatever the filter happens to read through the lidar's buzz. Every
    // frame of a stop therefore carries the identical rotation.
    const bool frozen = (state_ == SCAN_STEP_CAPTURE);

    PktSample s;
    putMagic(s.magic, PKT_SAMPLE_LEN);
    s.t_us        = micros();
    s.qw          = frozen ? poseQw_ : ahrs_.qw;
    s.qx          = frozen ? poseQx_ : ahrs_.qx;
    s.qy          = frozen ? poseQy_ : ahrs_.qy;
    s.qz          = frozen ? poseQz_ : ahrs_.qz;
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
  t.ax = ahrs_.ax; t.ay = ahrs_.ay; t.az = ahrs_.az;
  t.gx = ahrs_.gx; t.gy = ahrs_.gy; t.gz = ahrs_.gz;
  t.qw = ahrs_.qw; t.qx = ahrs_.qx; t.qy = ahrs_.qy; t.qz = ahrs_.qz;
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

void Scanner::beginAverage() {
  sumQw_ = sumQx_ = sumQy_ = sumQz_ = 0.0;
  avgCount_ = 0;
  avgLastT_ = 0;
  state_    = SCAN_STEP_AVERAGE;
  stateAt_  = millis();
}

// Folds one AHRS tick into the running sum. Called every update() while
// averaging; core1 publishes at 100 Hz and core0 loops far faster than that,
// so the t_us check is what turns "every loop" into "every new sample".
void Scanner::accumulateAverage() {
  ahrs::read(ahrs_);
  if (ahrs_.t_us == avgLastT_) return;
  avgLastT_ = ahrs_.t_us;

  // q and -q are the same rotation, so a sum taken without fixing the sign can
  // cancel to nothing. Align every term with the first one before adding.
  float w = ahrs_.qw, x = ahrs_.qx, y = ahrs_.qy, z = ahrs_.qz;
  if (avgCount_ > 0) {
    const double dot = sumQw_ * w + sumQx_ * x + sumQy_ * y + sumQz_ * z;
    if (dot < 0.0) { w = -w; x = -x; y = -y; z = -z; }
  }
  sumQw_ += w; sumQx_ += x; sumQy_ += y; sumQz_ += z;
  avgCount_++;
}

// The linear-then-normalise average. Exact averaging on the rotation manifold
// would be an eigenvector problem, but the whole point of the settle window is
// that these quaternions differ by a fraction of a degree, and over that span
// the two agree to far better than the sensor does.
void Scanner::finishAverage() {
  const double n = sqrt(sumQw_ * sumQw_ + sumQx_ * sumQx_ +
                        sumQy_ * sumQy_ + sumQz_ * sumQz_);
  if (avgCount_ == 0 || n < 1e-9) {
    // No IMU, or nothing published during the window. Keep the previous pose
    // and say so -- silently stamping identity would put this stop's points in
    // a completely wrong place, which is much harder to spot than a warning.
    emitEvent("step %u: no IMU samples to average, reusing last pose",
              (unsigned)stepIndex_);
  } else {
    poseQw_ = (float)(sumQw_ / n);
    poseQx_ = (float)(sumQx_ / n);
    poseQy_ = (float)(sumQy_ / n);
    poseQz_ = (float)(sumQz_ / n);
  }
  state_   = SCAN_STEP_CAPTURE;
  stateAt_ = millis();
}

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
  c.gearRatio   = SCAN_GEAR_RATIO;
  c.mode        = mode_;
  c.reserved    = 0;
  c.steps       = steps_;
  c.settleMs    = SCAN_STEP_SETTLE_MS;
  c.averageMs   = SCAN_STEP_AVERAGE_MS;
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

    case CMD_ZERO: {
      const AhrsStatus st = ahrs::status();
      if (st.ready && !st.ok) { emitEvent("no IMU to calibrate"); break; }
      // Fire and forget: core1 does the 2 s average while core0 keeps serving
      // the host. serviceAhrs() emits the new bias when it lands.
      ahrs::requestCalibration();
      emitEvent("recalibrating, hold still");
      break;
    }

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
      emitImuStatus();
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
                       (SCAN_STEP_SETTLE_MS + SCAN_STEP_AVERAGE_MS + dwellMs_) /
                       1000.0f;
    emitEvent("stepped: %u stops of %.2f deg, ~%.0f s plus travel",
              (unsigned)(steps_ + 1), (2.0f * scanDegrees_) / steps_, secs);
  }
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
          // settle window would be redundant here; go straight to averaging.
          emitEvent("stepped: capturing stop 1/%u at %+.2f deg",
                    (unsigned)(steps_ + 1), stepAngle(0));
          beginAverage();
          break;
        }
        // Constant rate for the sweep itself: with acceleration disabled the
        // driver steps at exactly maxSpeed, so platform angle is linear in
        // time and the elevation slices come out evenly spaced. The ramp that
        // makes travel fast would bunch points at both ends of the sweep.
        const float platformDegPerSec = (2.0f * scanDegrees_) / scanTime_;
        const float stepsPerSec = platformDegPerSec * SCAN_GEAR_RATIO *
                                  motor_.stepsPerRev() / 360.0f;
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

    case SCAN_STEP_MOVE:
      if (!motor_.isRunning()) {
        state_   = SCAN_STEP_SETTLE;
        stateAt_ = millis();
      }
      break;

    case SCAN_STEP_SETTLE:
      if (millis() - stateAt_ >= SCAN_STEP_SETTLE_MS) beginAverage();
      break;

    case SCAN_STEP_AVERAGE:
      // The samples themselves are folded in by accumulateAverage() on every
      // pass of update(); this only decides when the window has closed.
      if (millis() - stateAt_ >= SCAN_STEP_AVERAGE_MS) finishAverage();
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
  serviceAhrs();
  if (state_ == SCAN_STEP_AVERAGE) accumulateAverage();
  serviceLidar();
  serviceTelemetry();
  advanceStateMachine();
  serviceIdlePower();
}
