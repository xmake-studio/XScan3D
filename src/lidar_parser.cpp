#include "lidar_parser.h"

// On the RP2040 the monitor rides native USB (Serial), so the hardware UARTs
// are both free. Serial1 is uart0, which is what GPIO12/13 are wired to.
static SerialUART &lidarSerial = Serial1;

static uint8_t  buf[LIDAR_FRAME_LEN];
static uint8_t  idx = 0;
static uint32_t resyncBytes = 0;

static inline uint16_t rd16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rd32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void lidarBegin() {
  // Pin assignment must happen before begin() on this core.
  lidarSerial.setRX(LIDAR_RX_PIN);
  lidarSerial.setTX(LIDAR_TX_PIN);
  lidarSerial.setFIFOSize(256);  // covers a couple of frames between polls
  lidarSerial.begin(LIDAR_BAUD, SERIAL_8N1);
}

uint32_t lidarResyncBytes() {
  return resyncBytes;
}

// Validates the fixed prefix as bytes arrive so a mid-stream desync costs a few
// bytes instead of a whole frame.
static bool prefixOk(uint8_t pos, uint8_t b) {
  switch (pos) {
    case 0: return b == LIDAR_HDR0;
    case 1: return b == LIDAR_HDR1;
    case 2: return b == LIDAR_VERSION;
    case 3: return b == LIDAR_PKT_TYPE;
    default: return true;
  }
}

static void decode(LidarFrame &out) {
  memcpy(out.raw, buf, LIDAR_FRAME_LEN);

  out.speed    = rd16(&buf[4]);
  out.rawAngle = rd16(&buf[6]);
  out.stamp    = rd32(&buf[24]);

  for (uint8_t i = 0; i < LIDAR_POINTS; i++) {
    uint16_t v = rd16(&buf[8 + i * 2]);
    out.points[i].valid    = (v & LIDAR_DIST_INVALID) == 0;
    out.points[i].distance = v & LIDAR_DIST_MASK;
  }
}

bool lidarPoll(LidarFrame &out) {
  while (lidarSerial.available()) {
    uint8_t b = lidarSerial.read();

    if (!prefixOk(idx, b)) {
      resyncBytes += idx + 1;
      // The offending byte may itself be the start of the next frame.
      idx = (b == LIDAR_HDR0) ? 1 : 0;
      if (idx == 1) buf[0] = b;
      continue;
    }

    buf[idx++] = b;

    if (idx == LIDAR_FRAME_LEN) {
      idx = 0;
      decode(out);
      return true;
    }
  }
  return false;
}

void lidarEmit(const LidarFrame &f) {
#if LIDAR_OUTPUT_BINARY
  // Verbatim passthrough. The host re-syncs on the same 55 AA 02 08 header,
  // so any stray text (boot log, panic dump) is skipped rather than misparsed.
  Serial.write(f.raw, LIDAR_FRAME_LEN);
#else
  Serial.printf("RAWANG=%5u SPEED=%5u STAMP=0x%08X |", f.rawAngle, f.speed,
                f.stamp);
  for (uint8_t i = 0; i < LIDAR_POINTS; i++) {
    if (f.points[i].valid) {
      Serial.printf(" %5u", f.points[i].distance);
    } else {
      Serial.print("     -");
    }
  }
  Serial.println();
#endif
}
