#pragma once

#include <Arduino.h>

// The rig's calibration, kept in the last sector of flash (the core's EEPROM
// emulation), so it survives power cycles and firmware uploads alike: picotool
// only rewrites the sectors the new image occupies.
//
// The contents are the host's business -- see protocol.h. Here it is only a
// blob with a length and a CRC, loaded once at boot and rewritten on demand.
namespace calib {

// Reads the sector. A blank or corrupt one reads as "nothing stored".
void begin();

const uint8_t *data();
uint16_t length();   // 0 = nothing stored
uint32_t crc();      // of data()[0 .. length()), 0 when nothing is stored

// Replaces the stored blob. Skips the flash write when nothing changed, so a
// host that re-sends what is already there costs no erase cycle. Returns
// false when the blob is too long to store.
//
// Blocks for ~50 ms with interrupts off while the sector is erased and
// programmed: only call it with the motor stopped.
bool store(const uint8_t *p, uint16_t n, bool *changed = nullptr);

// CRC-32 (IEEE 802.3, reflected, as zlib and the host compute it).
uint32_t crc32(const uint8_t *p, size_t n);

}  // namespace calib
