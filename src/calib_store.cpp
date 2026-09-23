#include "calib_store.h"

#include <EEPROM.h>
#include <string.h>

#include "protocol.h"

namespace calib {

namespace {

// Laid at the start of the emulated EEPROM, the blob right after it.
struct __attribute__((packed)) Header {
  uint32_t magic;
  uint16_t length;
  uint16_t version;   // of this header, not of the blob inside
  uint32_t crc;
};

constexpr uint32_t kMagic   = 0x4C414358;  // "XCAL"
constexpr uint16_t kVersion = 1;
constexpr size_t   kSector  = 4096;

static_assert(sizeof(Header) + CALIB_MAX_LEN <= kSector,
              "calibration does not fit the EEPROM sector");

uint16_t length_ = 0;
uint32_t crc_    = 0;

const Header *header() {
  return reinterpret_cast<const Header *>(EEPROM.getConstDataPtr());
}

}  // namespace

uint32_t crc32(const uint8_t *p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  while (n--) {
    c ^= *p++;
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

void begin() {
  EEPROM.begin(kSector);
  const Header *h = header();
  length_ = 0;
  crc_    = 0;
  // Erased flash reads 0xFF throughout, which fails the magic; a torn write
  // fails the CRC. Either way there is simply no calibration.
  if (h->magic != kMagic || h->version != kVersion) return;
  if (h->length == 0 || h->length > CALIB_MAX_LEN) return;
  if (crc32(data(), h->length) != h->crc) return;
  length_ = h->length;
  crc_    = h->crc;
}

const uint8_t *data() {
  return EEPROM.getConstDataPtr() + sizeof(Header);
}

uint16_t length() { return length_; }
uint32_t crc() { return crc_; }

bool store(const uint8_t *p, uint16_t n, bool *changed) {
  if (changed) *changed = false;
  if (n > CALIB_MAX_LEN) return false;
  const uint32_t c = n ? crc32(p, n) : 0;
  if (n == length_ && c == crc_ && (n == 0 || memcmp(p, data(), n) == 0))
    return true;

  uint8_t *raw = EEPROM.getDataPtr();   // marks the sector dirty
  Header h;
  h.magic   = n ? kMagic : 0xFFFFFFFFu;
  h.length  = n;
  h.version = kVersion;
  h.crc     = c;
  memcpy(raw, &h, sizeof(h));
  if (n) memcpy(raw + sizeof(h), p, n);
  if (!EEPROM.commit()) return false;

  length_ = n;
  crc_    = c;
  if (changed) *changed = true;
  return true;
}

}  // namespace calib
