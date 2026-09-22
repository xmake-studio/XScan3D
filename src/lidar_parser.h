#pragma once

#include <Arduino.h>

// --- Wiring -----------------------------------------------------------------
#define LIDAR_RX_PIN 5    // uart1 RX
#define LIDAR_TX_PIN 20   // lidar has no RX; uart1 still wants a TX, and GPIO20
                          // is not broken out on the RP2040 Zero
#define LIDAR_BAUD   115200

// --- Frame layout -----------------------------------------------------------
#define LIDAR_FRAME_LEN     28
#define LIDAR_POINTS        8
#define LIDAR_HDR0          0x55
#define LIDAR_HDR1          0xAA
#define LIDAR_VERSION       0x02
#define LIDAR_PKT_TYPE      0x08

// 55 AA 02 08 | speed | startAngle | 8 x dist | endAngle | check, all u16 LE.
// `check` is the Neato XV-11 style 15-bit checksum over the 13 words before
// it (header included) -- see lidarChecksum() in lidar_parser.cpp.
#define LIDAR_CHECK_OFFSET  26

// Bit 15 of a distance word is a "no return" flag (observed as the bare value
// 0x8000 at dropouts), so the magnitude is 15 bits.
#define LIDAR_DIST_INVALID  0x8000
#define LIDAR_DIST_MASK     0x7FFF

// Output format on the USB serial port:
//   1 = raw 28-byte frames, verbatim, for tools/lidar_viz.py
//   0 = human-readable text, for eyeballing in the monitor
#define LIDAR_OUTPUT_BINARY 1

struct LidarPoint {
  uint16_t distance;  // 15-bit magnitude, units TBD (looks like mm)
  bool     valid;     // false when the lidar reported no return
};

struct LidarFrame {
  uint8_t    raw[LIDAR_FRAME_LEN];  // the frame exactly as received
  uint16_t   speed;                 // offset 4, near-constant: motor speed
  uint16_t   rawAngle;              // offset 6, uncalibrated angle of point 0
  uint16_t   endAngle;              // offset 24, uncalibrated angle of point 7
  uint16_t   check;                 // offset 26, checksum (already verified)
  LidarPoint points[LIDAR_POINTS];
};

// Opens the UART on the pins above. Call from setup().
void lidarBegin();

// Drains whatever bytes have arrived and, if a complete frame with a correct
// checksum was assembled, fills `out` and returns true. Poll from loop().
bool lidarPoll(LidarFrame &out);

// Writes one frame to Serial in whichever format LIDAR_OUTPUT_BINARY selects.
void lidarEmit(const LidarFrame &f);

// Bytes discarded so far while re-syncing to the frame header.
uint32_t lidarResyncBytes();

// Complete frames rejected so far because the checksum did not match.
uint32_t lidarChecksumErrors();
