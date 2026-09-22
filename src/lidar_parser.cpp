#include "lidar_parser.h"

// On the RP2040 the monitor rides native USB (Serial), so the hardware UARTs
// are both free. Serial2 is uart1, the only UART that can receive on GPIO5.
static SerialUART &lidarSerial = Serial2;

static uint8_t  buf[LIDAR_FRAME_LEN];
static uint8_t  idx = 0;
static uint32_t resyncBytes = 0;
static uint32_t checksumErrors = 0;

static inline uint16_t rd16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
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

uint32_t lidarChecksumErrors() {
  return checksumErrors;
}

// Neato XV-11 style: shift-and-add the 13 LE words ahead of the checksum,
// then fold the 32-bit accumulator down to 15 bits. Verified against every
// frame of a 20 s raw capture (7545/7545), header words included.
static uint16_t lidarChecksum(const uint8_t *f) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < LIDAR_CHECK_OFFSET; i += 2) {
    acc = (acc << 1) + rd16(&f[i]);
  }
  return ((acc & 0x7FFF) + (acc >> 15)) & 0x7FFF;
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
  out.endAngle = rd16(&buf[24]);
  out.check    = rd16(&buf[LIDAR_CHECK_OFFSET]);

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
      if (lidarChecksum(buf) == rd16(&buf[LIDAR_CHECK_OFFSET])) {
        idx = 0;
        decode(out);
        return true;
      }
      // A bad frame usually means the header we locked onto was a lookalike
      // inside a data word, or bytes were lost. Slide to the next spot that
      // still reads as a valid prefix so the real frame start isn't skipped.
      checksumErrors++;
      uint8_t k = 1;
      for (; k < LIDAR_FRAME_LEN; k++) {
        uint8_t n = 0;
        while (k + n < LIDAR_FRAME_LEN && prefixOk(n, buf[k + n])) n++;
        if (k + n == LIDAR_FRAME_LEN) break;
      }
      resyncBytes += k;
      idx = LIDAR_FRAME_LEN - k;
      memmove(buf, &buf[k], idx);
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
  Serial.printf("RAWANG=%5u END=%5u SPEED=%5u |", f.rawAngle, f.endAngle,
                f.speed);
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
