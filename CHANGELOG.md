# Changelog

## [0.2.0] - 2026-10-15

Protocol fixes checked against the HLK-LD6004 protocol PDF and a working ESP-IDF driver.
Before this release, the library did not work on real hardware.

### Fixed

- Zero-length ACK frames have no DATA_CKSUM byte. The parser waited for one, which swallowed
  the next frame's SOF, so every `set*()`/`request*()` call timed out.
- Control commands (TYPE 0x0201) now send a 4-byte int32 LE command, not 1 byte.
- Target frames (0x0A04): `target_num` is an int32, so targets start at offset 4, not 1.
- Zone occupancy (0x0A0A) is 4 × uint32, not 4 × uint8.
- `setZone()` sends a 4-byte int32 `area_id` (28-byte payload, LEN 0x1C), not 1 byte.
- Operating mode (0x0A12) and P20 (0x0A15) reports are 1 byte. They were rejected
  because the parser required 4 bytes.
- Frames larger than the 128-byte buffer (e.g. 0x0A08 point clouds) were truncated and
  produced checksum errors. They are now consumed and dropped cleanly.
- 0x0A08 point-cloud frames no longer overwrite the tracked targets.
- After a header-checksum failure (false SOF inside a payload), the parser rescans the
  consumed bytes so the following real frame is not lost.
- Zero-length outgoing frames (firmware query) no longer send a trailing DATA_CKSUM.
- Removed `src/main.cpp`: Arduino IDE compiles everything in `src/`, so its
  `setup()`/`loop()` conflicted with every user sketch.
- Examples: moved off GPIO 9/10 (SPI flash pins on ESP32-WROOM) to 16/17, enlarged the
  RX buffer, and removed `delay(100)` from `loop()`, which overflowed the default 256-byte RX buffer.

### Changed

- `request*()` methods now block until the sensor's report frame arrives, not just the
  ACK, so the matching getter is valid when they return true.
- `requestZones()` waits for all three zone reports (0x0A0B/0x0A0C/0x0A16).
- Commands are retried once on timeout, as protocol §6 specifies.
- The first command after `begin()` waits for the sensor to finish booting (first frame
  received, or 2.5 s), because the sensor ignores commands while booting.
- Outgoing frame IDs increment per frame.

### Added

- `getZone(areaId, ZoneBounds&)` to read cached zone coordinates.
- `bytesReceived` diagnostic counter.
- `examples/Diagnostics` hardware bring-up sketch.

## [0.1.0] - 2026-03-26

Initial release.

### Features

- Basic presence detection: `isPersonDetected()`, `getTargetCount()`, `getTarget()` (US1)
- Connection health: `isConnected()`, `lastUpdateAge()` (US4)
- Diagnostic counters: `framesReceived`, `checksumErrors`, `resetCounters()` (US5)
- Zone configuration: `setZone()`, `requestZones()`, `getZoneOccupancy()` (US2)
- P20 GPIO output mode: `setP20Mode()`, `requestP20Status()`, `p20Mode()` (US6)
- Extended operating modes: `setOperatingMode()`, `requestOperatingMode()`, `operatingMode()` (US7)
- Conditional zone initialization: `setZoneIfUnset()` (US8)
- Full sensor configuration: sensitivity, trigger speed, hold delay, installation mode,
  UART output interval, Z-axis range, low-power sleep time, stay zone lifecycle,
  firmware version (US3)

### Performance (ESP32 at 240 MHz, toolchain-xtensa-esp32 8.4.0)

- **Library code (flash/text)**: 5049 bytes
- **Class instance (SRAM/BSS)**: 612 bytes
  - Breakdown: 128B data buffer + 300B zone cache (12 × ZoneCoords) + 184B other fields
  - SC-004 targets < 4096B flash / < 512B SRAM; flash met, SRAM is 612B due to the
    mandatory 300B zone coordinate cache (12 entries × 25B). The 1024B protocol maximum
    for the data buffer has been reduced to 128B (largest actual frame = 96B zone coords).
- **Debug build flash overhead**: +464 bytes (SC-010 limit: +1024B ✓)
- **No heap allocation** (`new`/`malloc` free): ✓
- **No WiFi/network dependency**: ✓
- **`update()` latency**: < 50 µs per call with no bytes available (hardware benchmark TBD)
