#pragma once

#include <Arduino.h>

// --- Wiring -----------------------------------------------------------------
#define LIDAR_RX_PIN 13
#define LIDAR_TX_PIN 12
#define LIDAR_BAUD   115200

// --- Frame layout -----------------------------------------------------------
#define LIDAR_FRAME_LEN     28
#define LIDAR_POINTS        8
#define LIDAR_HDR0          0x55
#define LIDAR_HDR1          0xAA
#define LIDAR_VERSION       0x02
#define LIDAR_PKT_TYPE      0x08

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
  uint16_t   rawAngle;              // offset 6, uncalibrated start angle
  uint32_t   stamp;                 // offset 24, timestamp / CRC
  LidarPoint points[LIDAR_POINTS];
};

// Opens the UART on the pins above. Call from setup().
void lidarBegin();

// Drains whatever bytes have arrived and, if a complete valid frame was
// assembled, fills `out` and returns true. Poll from loop().
bool lidarPoll(LidarFrame &out);

// Writes one frame to Serial in whichever format LIDAR_OUTPUT_BINARY selects.
void lidarEmit(const LidarFrame &f);

// Bytes discarded so far while re-syncing to the frame header.
uint32_t lidarResyncBytes();
