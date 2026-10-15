/**
 * BasicPresence — MyLD6004 Arduino Library Example
 *
 * Demonstrates basic presence detection with the HLK-LD6004 3D radar,
 * supporting both wall-mount and ceiling-mount installations.
 *
 * ── Installation modes ───────────────────────────────────────────────────────
 *
 *   WALL MOUNT  (sensor on a wall, facing into the room)
 *     X  = horizontal (left/right, metres from sensor centre)
 *     Y  = depth (distance from the wall, metres)
 *     Z  = height (metres)
 *
 *   CEILING MOUNT  (sensor on the ceiling, facing down)
 *     X  = horizontal (left/right, metres from sensor centre)
 *     Y  = horizontal (front/back, metres from sensor centre)
 *     Z  = height of the person (metres)
 *
 * Set INSTALLATION_MODE below to match your physical installation before
 * flashing.  The sensor persists this setting across power cycles, but it is
 * re-applied here on every boot to guarantee it matches the sketch.
 *
 * ── Wiring (ESP32) ───────────────────────────────────────────────────────────
 *   LD6004 TX0 → GPIO 16 (ESP32 RX2)
 *   LD6004 RX0 → GPIO 17 (ESP32 TX2)
 *   LD6004 VDD → 3.3 V  (≥1 A, ripple ≤50 mV)
 *   LD6004 GND → GND
 */

#include <MyLD6004.h>

// ── Choose your installation ──────────────────────────────────────────────────
//   INSTALL_WALL    – sensor mounted on a wall, facing horizontally into the room
//   INSTALL_CEILING – sensor mounted on the ceiling, facing downward
#define INSTALLATION_MODE INSTALL_WALL

#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17

// ── Object ────────────────────────────────────────────────────────────────────
MyLD6004 radar(Serial2);

uint32_t lastPrint = 0;

// ── setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    // The radar streams ~10 frames/s; a larger RX buffer keeps bytes from being
    // dropped while loop() is busy (must be set before begin()).
    Serial2.setRxBufferSize(1024);
    Serial2.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    radar.begin();

    Serial.println("MyLD6004 BasicPresence starting...");

    // Apply installation mode so the sensor outputs coordinates in the correct
    // reference frame for your mounting position. This first command waits for
    // the sensor to finish booting (up to 2.5 s after power-on).
    if (!radar.setInstallationMode(INSTALLATION_MODE)) {
        Serial.println("WARNING: setInstallationMode got no ACK – check wiring "
                       "(TX/RX crossed?) and power.");
    } else {
#if INSTALLATION_MODE == INSTALL_CEILING
        Serial.println("Installation mode: CEILING (X/Y = floor plane, Z = height)");
#else
        Serial.println("Installation mode: WALL    (X = left/right, Y = depth, Z = height)");
#endif
    }
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop()
{
    // Call as often as possible; never block loop() with long delay() calls.
    radar.update();

    if (millis() - lastPrint < 500) return;
    lastPrint = millis();

    if (!radar.isConnected()) {
        Serial.printf("Sensor offline – last data: %lu ms ago\n",
                      (unsigned long)radar.lastUpdateAge());
        return;
    }

    if (radar.isPersonDetected()) {
        Serial.printf("Person detected: %d target(s)\n", radar.getTargetCount());
        for (uint8_t i = 0; i < radar.getTargetCount(); i++) {
            Target t = radar.getTarget(i);
            Serial.printf("  Target %d: x=%.3f m  y=%.3f m  z=%.3f m  id=%ld\n",
                          i, t.x, t.y, t.z, (long)t.clusterId);
        }
    } else {
        Serial.println("No person detected.");
    }
}
