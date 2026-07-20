#pragma once

#include <Arduino.h>

#include "lidar_parser.h"

// Wire format for the USB link, MCU -> host. Everything is little-endian and
// packed; the RP2040 and every host we care about are little-endian, and the
// structs below are laid out so no padding is inserted.
//
// Three record types share the 55 AA 03 xx prefix, where xx is the total record
// length in bytes (variable-length records carry 0x00 and an explicit count).
// The host re-syncs on the 4-byte magic, so boot chatter and dropped bytes cost
// at most one record.
//
// tools/scan_proto.py mirrors these layouts. Change one, change the other.

#define PKT_MAGIC0 0x55
#define PKT_MAGIC1 0xAA
#define PKT_MAGIC2 0x03

#define PKT_SAMPLE_LEN 48
#define PKT_TELEM_LEN  56
#define PKT_CONFIG_LEN 26
#define PKT_EVENT_TAG  0x09  // length is carried in the payload, not the tag

// One lidar frame plus the platform orientation at the moment it arrived.
// Self-contained on purpose: the host needs no interpolation to place the 8
// points in the world, because the pose it wants is already attached.
struct __attribute__((packed)) PktSample {
  uint8_t  magic[4];
  uint32_t t_us;                       // micros() when the frame completed
  float    qw, qx, qy, qz;             // AHRS quaternion, sensor->world
  float    platformDeg;                // stepper-derived tilt, gearing applied
  uint16_t lidarSpeed;                 // lidar's own spin rate, 1/64 RPM
  uint16_t rawAngle;                   // uncalibrated frame start angle
  uint16_t dist[LIDAR_POINTS];         // raw words, bit 15 = no return
};
static_assert(sizeof(PktSample) == PKT_SAMPLE_LEN, "PktSample layout drifted");

// Low-rate housekeeping. The raw accelerometer vector is here so the host can
// verify which quaternion convention actually rotates gravity to world -Z
// instead of the operator guessing (see scan_proto.resolve_frame).
//
// The orientation is repeated here as well as on every sample, because the
// sample stream only exists while the lidar is spinning. A host that wants to
// show live attitude -- or to tell a wedged AHRS from a wedged lidar -- needs
// it from a source that keeps ticking when the lidar is silent.
struct __attribute__((packed)) PktTelem {
  uint8_t  magic[4];
  uint32_t t_us;
  float    ax, ay, az;                 // g
  float    gx, gy, gz;                 // rad/s, bias-corrected
  float    qw, qx, qy, qz;             // AHRS quaternion, sensor->world
  float    platformDeg;
  uint8_t  state;                      // ScanState
  uint8_t  reserved;
  uint16_t dropped;                    // samples lost to a backed-up USB pipe
};
static_assert(sizeof(PktTelem) == PKT_TELEM_LEN, "PktTelem layout drifted");

// Scan modes. Continuous is the original behaviour: the platform crosses the
// whole sweep at a constant rate while the lidar streams. Stepped trades time
// for quiet -- it moves, stops, lets the ringing die, averages the IMU with
// the platform stationary, then captures with that one settled pose stamped on
// every frame. The lidar's own vibration is still there, but it is no longer
// being integrated into a pose that is moving at the same time.
#define SCAN_MODE_CONTINUOUS 0
#define SCAN_MODE_STEPPED    1

// The sweep parameters actually in force. Emitted whenever they change and on
// demand, so a UI that just connected can show the device's real settings
// instead of assuming its own defaults took.
struct __attribute__((packed)) PktConfig {
  uint8_t  magic[4];
  float    scanDegrees;                // sweep runs -this .. +this
  float    scanTime;                   // seconds for the full sweep
  float    gearRatio;                  // motor revs per platform rev
  uint8_t  mode;                       // SCAN_MODE_*
  uint8_t  reserved;
  uint16_t steps;                      // stepped: intervals, so steps+1 stops
  uint16_t settleMs;                   // stepped: ring-down after each move
  uint16_t averageMs;                  // stepped: IMU averaging window
  uint16_t captureMs;                  // stepped: lidar dwell at each stop
};
static_assert(sizeof(PktConfig) == PKT_CONFIG_LEN, "PktConfig layout drifted");

// Human-readable notices: magic, uint8 length, then that many ASCII bytes.
#define PKT_EVENT_MAX 96

// --- Host -> MCU commands ---------------------------------------------------
// Line oriented: one command letter, an optional decimal argument, then a
// newline. A bare letter still works when typed into a serial monitor.
#define CMD_START  's'  // park, settle, then sweep
#define CMD_ABORT  'x'  // stop where you are, go idle
#define CMD_HOME   'h'  // call the current platform angle zero
#define CMD_ZERO   'z'  // re-bias the gyro and reset the AHRS (hold still)
#define CMD_STATUS '?'  // emit telemetry + config + a text event
#define CMD_ANGLE  'a'  // "a25.0"  set the half-sweep in degrees
#define CMD_TIME   't'  // "t45.0"  set the sweep duration in seconds
#define CMD_MODE   'm'  // "m1"     0 = continuous, 1 = stepped
#define CMD_STEPS  'n'  // "n60"    stepped: intervals across the sweep
#define CMD_DWELL  'd'  // "d400"   stepped: lidar capture time per stop, ms

// Guard rails for the above, so a fat-fingered UI value cannot drive the
// platform into its end stops or ask for a step rate the motor cannot hold.
#define SCAN_DEGREES_MIN 1.0f
#define SCAN_DEGREES_MAX 90.0f
#define SCAN_TIME_MIN    2.0f
#define SCAN_TIME_MAX    600.0f
#define SCAN_STEPS_MIN   2
#define SCAN_STEPS_MAX   2000
#define SCAN_DWELL_MIN   50
#define SCAN_DWELL_MAX   5000
