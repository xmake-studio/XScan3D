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

#define PKT_SAMPLE_LEN 32
#define PKT_TELEM_LEN  16
#define PKT_CONFIG_LEN 20
#define PKT_EVENT_TAG  0x09  // length is carried in the payload, not the tag

// One lidar frame plus the shaft angle at the moment it arrived. Self-contained
// on purpose: the host needs no interpolation to place the 8 points in the
// world, because the angle it wants is already attached.
//
// There is no orientation field. The lidar is bolted straight to the stepper
// shaft and the shaft turns about the world vertical, so the step count *is*
// the pose -- exactly, with no drift and nothing to fuse.
struct __attribute__((packed)) PktSample {
  uint8_t  magic[4];
  uint32_t t_us;                       // micros() when the frame completed
  float    platformDeg;                // shaft angle, degrees about vertical
  uint16_t lidarSpeed;                 // lidar's own spin rate, 1/64 RPM
  uint16_t rawAngle;                   // uncalibrated frame start angle
  uint16_t dist[LIDAR_POINTS];         // raw words, bit 15 = no return
};
static_assert(sizeof(PktSample) == PKT_SAMPLE_LEN, "PktSample layout drifted");

// Low-rate housekeeping. The angle is repeated here as well as on every sample,
// because the sample stream only exists while the lidar is spinning: a host
// that wants to show where the platform is sitting -- or to tell a wedged motor
// from a wedged lidar -- needs it from a source that keeps ticking when the
// lidar is silent.
struct __attribute__((packed)) PktTelem {
  uint8_t  magic[4];
  uint32_t t_us;
  float    platformDeg;
  uint8_t  state;                      // ScanState
  uint8_t  reserved;
  uint16_t dropped;                    // samples lost to a backed-up USB pipe
};
static_assert(sizeof(PktTelem) == PKT_TELEM_LEN, "PktTelem layout drifted");

// Scan modes. Continuous is the original behaviour: the platform crosses the
// whole sweep at a constant rate while the lidar streams. Stepped trades time
// for quiet -- it moves, stops, lets the ringing die, then captures with the
// shaft genuinely stationary. The lidar's own vibration is still there, but the
// angle stamped on a frame is no longer a moving target.
#define SCAN_MODE_CONTINUOUS 0
#define SCAN_MODE_STEPPED    1

// The sweep parameters actually in force. Emitted whenever they change and on
// demand, so a UI that just connected can show the device's real settings
// instead of assuming its own defaults took.
struct __attribute__((packed)) PktConfig {
  uint8_t  magic[4];
  float    scanDegrees;                // sweep runs -this .. +this
  float    scanTime;                   // seconds for the full sweep
  uint8_t  mode;                       // SCAN_MODE_*
  uint8_t  reserved;
  uint16_t steps;                      // stepped: intervals, so steps+1 stops
  uint16_t settleMs;                   // stepped: ring-down after each move
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
#define CMD_HOME   'h'  // call the current shaft angle zero
#define CMD_STATUS '?'  // emit telemetry + config + a text event
#define CMD_ANGLE  'a'  // "a90.0"  set the half-sweep in degrees
#define CMD_TIME   't'  // "t45.0"  set the sweep duration in seconds
#define CMD_MODE   'm'  // "m1"     0 = continuous, 1 = stepped
#define CMD_STEPS  'n'  // "n60"    stepped: intervals across the sweep
#define CMD_DWELL  'd'  // "d400"   stepped: lidar capture time per stop, ms
#define CMD_UNWRAP 'u'  // "u90"    turn the shaft this far, then re-home

// Guard rails for the above, so a fat-fingered UI value cannot drive the
// platform into its end stops or ask for a step rate the motor cannot hold.
//
// The sweep is symmetric about home, so the travel is twice SCAN_DEGREES.
//
// 90 (a 180 degree sweep) is enough to cover the whole sphere, because the
// lidar's scan plane is vertical and half a turn of the shaft already carries
// that full plane through every azimuth.
//
// 180 (a 360 degree sweep) is for one-side scanning, where the host keeps only
// half of each lidar revolution to dodge the rangefinder's lateral standoff.
// Half a plane is a pole-to-pole arc rather than a full circle, so it needs the
// whole turn to sweep the sphere. Past that nothing new is scanned and the
// tether only takes on more twist.
#define SCAN_DEGREES_MIN 1.0f
#define SCAN_DEGREES_MAX 180.0f
#define SCAN_TIME_MIN    2.0f
#define SCAN_TIME_MAX    6000.0f
#define SCAN_STEPS_MIN   2
#define SCAN_STEPS_MAX   2000
#define SCAN_DWELL_MIN   50
#define SCAN_DWELL_MAX   5000

// The tether runs up the shaft, so it twists as the platform turns. Unwrap
// turns the shaft a quarter turn and calls the result home, which shifts the
// whole sweep range with it rather than leaving the next scan to wind the
// tether straight back up.
#define SCAN_UNWRAP_DEG    90.0f
#define SCAN_UNWRAP_DEGMAX 180.0f
