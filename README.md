# MyLD6004

Arduino library for the **HLK-LD6004** 3D presence radar (57–64 GHz FMCW) on ESP32.
It handles presence detection, up to 3 tracked targets with X/Y/Z coordinates, zone occupancy,
zone configuration and the full sensor configuration over UART.

## Wiring (ESP32)

| LD6004 | ESP32                     |
|--------|---------------------------|
| TX0    | GPIO 16 (RX2)             |
| RX0    | GPIO 17 (TX2)             |
| VDD    | 3.3 V (≥1 A, ripple ≤50 mV) |
| GND    | GND                       |

UART: 115200 8N1 (factory default).

## Installation

**Arduino IDE**: *Sketch → Include Library → Add .ZIP Library…* and select a zip of this
repository. Alternatively, copy the folder into `Documents/Arduino/libraries/MyLD6004`.
Then open *File → Examples → MyLD6004*.

**PlatformIO**: add to your project's `platformio.ini`:

```ini
lib_deps =
    https://github.com/echavez087/MyLD6004.git   ; or: symlink://C:/path/to/MyLD6004
```

## Usage

```cpp
#include <MyLD6004.h>

MyLD6004 radar(Serial2);

void setup() {
    Serial.begin(115200);
    Serial2.setRxBufferSize(1024);                  // before begin()
    Serial2.begin(115200, SERIAL_8N1, 16, 17);      // RX, TX
    radar.begin();
    radar.setInstallationMode(INSTALL_WALL);        // waits for sensor boot automatically
}

void loop() {
    radar.update();                                 // call often; avoid long delay()
    if (radar.isPersonDetected()) {
        Target t = radar.getTarget(0);
        // t.x, t.y, t.z in metres
    }
}
```

* `update()` is non-blocking and parses every byte that is available.
* `set*()` blocks until the sensor ACKs. `request*()` blocks until the report arrives, so the
  getter is valid on return. Both time out after ~1 s (500 ms plus one retry).
* `bytesReceived`, `framesReceived` and `checksumErrors` help diagnose wiring problems.
  Run `examples/Diagnostics` first when bringing up new hardware.
* Build with `-DMYLD6004_DEBUG` to log every frame to `Serial`.

## Development

```bash
pio run                                                   # build examples/BasicPresence
pio ci examples/Diagnostics --lib . --board esp32dev      # build another example
```

The protocol reference is in `docs/`.

## License

The library source code and examples are released under the [MIT License](LICENSE).

The PDF manuals, datasheets and protocol guides in `docs/` belong to their manufacturer,
Shenzhen Hi-Link Electronic Co., Ltd. (HLK). They are included only for reference and are
not covered by the MIT license.
