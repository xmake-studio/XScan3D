#include "scanner.h"

#include <stdarg.h>
#include <stdio.h>

#include "calib_store.h"
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

Adafruit_NeoPixel statusLed(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

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

void setStatus(int r, int g, int b) {
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
}

// --- Setup ------------------------------------------------------------------

void Scanner::begin() {
  // Proof of life before a single peripheral is touched. Every hardware init
  // below can fail or block, and without this the whole class of boot hang
  // looks identical to dead firmware from the host's side.
  emitEvent("boot: scanner starting");

  statusLed.begin();
  statusLed.setBrightness(255);
  setStatus(255, 0, 255);  // magenta: booting

  // The motor claims its pins first, so nothing else can take them later.
  motor_.begin(SCAN_MICROSTEP);
  motor_.setMaxSpeed(SCAN_TRAVEL_SPEED);
  motor_.setAcceleration(SCAN_TRAVEL_ACCEL);
  //motor_.enable();
  // Wherever the rig happens to be sitting is angle zero until told otherwise.
  motor_.setCurrentPosition(0);
  emitEvent("boot: stepper on GPIO%d-%d, 1/%d step, %ld/rev",
            STEP_PIN_ENABLE, STEP_PIN_DIR, (int)SCAN_MICROSTEP,
            motor_.stepsPerRev());

  lidarBegin();
  emitEvent("boot: lidar uart on RX%d/TX%d", LIDAR_RX_PIN, LIDAR_TX_PIN);

  calib::begin();
  if (calib::length())
    emitEvent("boot: calibration in flash, %u bytes", (unsigned)calib::length());
  else
    emitEvent("boot: no calibration in flash");

  nextTelem_ = millis();
  state_     = SCAN_IDLE;
  stateAt_   = millis();

  emitConfig();
  emitEvent("ready: sweep %+.0f..%+.0f deg in %.0f s", -scanDegrees_,
            scanDegrees_, scanTime_);

  setStatus(0, 255, 0);  // green: ready
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
    s.endAngle    = frame.endAngle;
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
  setStatus(0, 255, 0);  // green: motor disabled
  angleStale_ = true;
  emitEvent("motor: coils released at %+.2f deg (idle)", platformDeg());
}

void Scanner::engageMotor() {
  if (motor_.isEnabled()) return;
  motor_.enable();
  setStatus(255, 0, 0);  // red: motor enabled
  if (angleStale_) {
    angleStale_ = false;
    // Not an error -- with the coils cold the shaft turns freely, so this is
    // the honest statement that the angle is only as good as the rig's balance
    // since the release. 'h' re-zeros it if the platform was moved on purpose.
    emitEvent("motor: re-energised, angle assumed unchanged at %+.2f deg",
              platformDeg());
  }
}

// --- Chime ------------------------------------------------------------------

void Scanner::playChime() {
  if (motor_.isRunning()) return;
  engageMotor();

  // Played at the scan's own 1/16 resolution, so there is no mode switch and
  // the angle is safe wherever home happens to be.
  //
  // G5 then C6, a rising fourth: {Hz, ms, swing in microsteps}. The lower note
  // gets the smaller swing to come out level with the higher one.
  static const uint16_t notes[][3] = {
      {784, 150, 3}, {0, 30, 0}, {1047, 250, 4},
  };
  for (const auto &n : notes)
    motor_.playTone(n[0], n[1], (uint8_t)n[2], SCAN_CHIME_FADE_MS);
}

// --- Stepped mode -----------------------------------------------------------

void Scanner::advanceStep() {
  if (stepIndex_ >= steps_) {
    emitEvent("stepped scan complete: %u stops, dropped=%u",
              (unsigned)(steps_ + 1), (unsigned)dropped_);
    // Stays at the far end, as a continuous sweep does (see SCAN_SWEEPING).
    state_   = SCAN_DONE;
    stateAt_ = millis();
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

// --- Calibration ------------------------------------------------------------

// One chunk per call, and only while idle: a sweep's samples own the pipe, and
// a chunk that does not fit now simply goes out on a later pass.
void Scanner::serviceCalibTx() {
  if (!txPending_) return;
  if (state_ != SCAN_IDLE && state_ != SCAN_DONE) return;

  const uint16_t total = calib::length();
  if (txOff_ > total) txOff_ = 0;   // the blob shrank under a transfer
  const uint16_t n = min<uint16_t>(PKT_CALIB_CHUNK, total - txOff_);
  if ((size_t)Serial.availableForWrite() < sizeof(PktCalibHdr) + n) return;

  PktCalibHdr h;
  putMagic(h.magic, PKT_CALIB_TAG);
  h.total  = total;
  h.offset = txOff_;
  h.crc    = calib::crc();
  h.n      = (uint8_t)n;
  Serial.write((const uint8_t *)&h, sizeof(h));
  if (n) Serial.write(calib::data() + txOff_, n);
  txOff_ += n;
  if (txOff_ >= total) txPending_ = false;
}

void Scanner::beginCalibWrite(const char *arg, bool hasArg) {
  // Without a well-formed length the bytes that follow cannot be told apart
  // from commands, so this is the one refusal that cannot consume them. The
  // host always sends the length.
  char *end = nullptr;
  const unsigned long len = hasArg ? strtoul(arg, &end, 10) : 0;
  if (!hasArg || !end || *end != ',') {
    emitEvent("calib: write needs w<len>,<crc>");
    return;
  }
  rxLen_     = (uint16_t)min<unsigned long>(len, 0xFFFF);
  rxCrc_     = strtoul(end + 1, nullptr, 10);
  rxGot_     = 0;
  rxDiscard_ = len > CALIB_MAX_LEN;
  rxAt_      = millis();
  rxActive_  = true;
  if (rxLen_ == 0) finishCalibWrite();
}

void Scanner::serviceCalibRx() {
  if (rxActive_ && millis() - rxAt_ > CALIB_RX_TIMEOUT_MS) {
    rxActive_ = false;
    emitEvent("calib: write timed out after %u of %u bytes",
              (unsigned)rxGot_, (unsigned)rxLen_);
  }
}

void Scanner::finishCalibWrite() {
  rxActive_ = false;
  if (rxDiscard_) {
    emitEvent("calib: %u bytes is over the %d byte limit, not saved",
              (unsigned)rxLen_, CALIB_MAX_LEN);
  } else if (rxLen_ && calib::crc32(calibRx_, rxLen_) != rxCrc_) {
    emitEvent("calib: bad crc, not saved");
  } else if (state_ != SCAN_IDLE || motor_.isRunning()) {
    // The flash write stops the world for tens of milliseconds, which a
    // moving motor would feel as a dent in its step train.
    emitEvent("calib: not saved, platform busy");
  } else {
    bool changed = false;
    if (!calib::store(calibRx_, rxLen_, &changed))
      emitEvent("calib: flash write failed");
    else if (!rxLen_)
      emitEvent("calib: erased");
    else
      emitEvent(changed ? "calib: saved %u bytes" : "calib: unchanged, %u bytes",
                (unsigned)rxLen_);
    stateAt_ = millis();   // restart the idle-release countdown
  }
  // Whatever happened, answer with what the flash now holds.
  txOff_     = 0;
  txPending_ = true;
}

// Accumulates a line, then dispatches. A lone command letter with no newline
// still fires as soon as the next letter arrives, so a serial monitor works.
void Scanner::handleCommands() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;

    // Raw calibration bytes are counted off before any line parsing: they may
    // hold anything, including letters that would otherwise run as commands.
    if (rxActive_) {
      if (!rxDiscard_) calibRx_[rxGot_] = (uint8_t)c;
      rxGot_++;
      rxAt_ = millis();
      if (rxGot_ >= rxLen_) finishCalibWrite();
      continue;
    }

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

    case CMD_BEEP:
      if (state_ != SCAN_IDLE) {
        emitEvent("busy: cannot chime mid-sweep");
        break;
      }
      playChime();
      stateAt_ = millis();   // restart the idle-release countdown
      break;

    case CMD_CALIB_READ:
      txOff_     = 0;
      txPending_ = true;
      break;

    case CMD_CALIB_WRITE:
      beginCalibWrite(arg, hasArg);
      break;

    case CMD_STATUS:
      nextTelem_ = millis();
      emitConfig();
      emitEvent("state=%u platform=%+.2f deg motor=%s dropped=%u resync=%lu "
                "badcrc=%lu",
                (unsigned)state_, platformDeg(),
                motor_.isEnabled() ? "on" : "off", (unsigned)dropped_,
                (unsigned long)lidarResyncBytes(),
                (unsigned long)lidarChecksumErrors());
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
  // Far enough to reach the stop even from the far end of a sweep that was
  // aborted there, however wide the sweep has been set.
  const float travel =
      max(SCAN_HOME_DEG, 2.0f * scanDegrees_ + SCAN_HOME_BACKOFF + 15.0f);
  motor_.setMaxSpeed(SCAN_HOME_SPEED);
  motor_.setAcceleration(SCAN_TRAVEL_ACCEL);
  motor_.move(-platformDegToSteps(travel));
  state_   = SCAN_HOMING;
  stateAt_ = millis();
  emitEvent("homing: driving %.0f deg into the end stop", travel);
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
    case SCAN_HOMING:
      if (!motor_.isRunning()) {
        // The field now sits within half an electrical cycle of the stop.
        // Renumber so that backing off by at least SCAN_HOME_BACKOFF lands on
        // -scanDegrees_, shifting the count only by whole electrical cycles:
        // the driver's current table is indexed by the step count, and keeping
        // count and phase in step keeps the microstep error a fixed function
        // of the reported angle from one scan to the next.
        const long cycle = 4L * motor_.stepsPerRev() / STEP_FULL_STEPS_PER_REV;
        const long start = platformDegToSteps(-scanDegrees_);
        const long want  = start - platformDegToSteps(SCAN_HOME_BACKOFF);
        const long pos   = motor_.currentPosition();
        long k = (want - pos) / cycle;
        if (pos + k * cycle > want) k--;   // round toward the stop
        motor_.setCurrentPosition(pos + k * cycle);
        motor_.setMaxSpeed(SCAN_TRAVEL_SPEED);
        motor_.setAcceleration(SCAN_TRAVEL_ACCEL);
        motor_.moveTo(start);
        state_   = SCAN_PARKING;
        stateAt_ = millis();
        emitEvent("end stop reached; backing off %.1f deg to %+.1f deg",
                  360.0f * (start - motor_.currentPosition()) /
                      motor_.stepsPerRev(),
                  -scanDegrees_);
      }
      break;

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
        // Stays at the far end instead of returning to the middle: the next
        // scan's homing then covers most of its travel in free air and only
        // grinds against the stop for the last few degrees.
        state_   = SCAN_DONE;
        stateAt_ = millis();
      }
      break;

    case SCAN_DONE:
      if (!motor_.isRunning()) {
#if SCAN_CHIME_ENABLED
        playChime();
#endif
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
  serviceCalibRx();
  serviceLidar();
  serviceTelemetry();
  serviceCalibTx();
  advanceStateMachine();
  serviceIdlePower();
}
