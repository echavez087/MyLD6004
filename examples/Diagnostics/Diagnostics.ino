/**
 * Diagnostics — MyLD6004 Arduino Library Example
 *
 * Hardware bring-up check for the HLK-LD6004. On boot it reads back the
 * firmware version and every configuration value, then prints link
 * statistics, targets and zone occupancy once per second.
 *
 * What to look for:
 *   bytes = 0              → nothing arriving: check LD6004 TX0 → ESP32 RX, power, baud
 *   frames = 0, bytes > 0  → wrong baud rate or noise on the line
 *   cksum errors growing   → RX buffer overruns or a noisy line
 *   queries FAIL           → ESP32 TX → LD6004 RX0 not connected
 *
 * ── Wiring (ESP32) ───────────────────────────────────────────────────────────
 *   LD6004 TX0 → GPIO 16 (ESP32 RX2)
 *   LD6004 RX0 → GPIO 17 (ESP32 TX2)
 *   LD6004 VDD → 3.3 V  (≥1 A, ripple ≤50 mV)
 *   LD6004 GND → GND
 */

#include <MyLD6004.h>

#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17

MyLD6004 radar(Serial2);

uint32_t lastPrint = 0;

static const char* okFail(bool ok) { return ok ? "ok  " : "FAIL"; }

void printConfig()
{
    bool ok;

    ok = radar.queryFirmware();
    FirmwareInfo fw = radar.getFirmwareVersion();
    Serial.printf("[%s] Firmware        project=%u v%u.%u.%u\n", okFail(ok),
                  fw.projectName, fw.majorVersion, fw.subVersion, fw.modifiedVersion);

    ok = radar.requestSensitivity();
    Serial.printf("[%s] Sensitivity     %u (0=low 1=med 2=high)\n", okFail(ok), radar.getSensitivity());

    ok = radar.requestTriggerSpeed();
    Serial.printf("[%s] Trigger speed   %u (0=slow 1=med 2=fast)\n", okFail(ok), radar.getTriggerSpeed());

    ok = radar.requestInstallationMode();
    Serial.printf("[%s] Install mode    %u (0=ceiling 1=wall)\n", okFail(ok), radar.getInstallationMode());

    ok = radar.requestOperatingMode();
    Serial.printf("[%s] Operating mode  %u\n", okFail(ok), radar.operatingMode());

    ok = radar.requestP20Status();
    Serial.printf("[%s] P20 mode        %u\n", okFail(ok), radar.p20Mode());

    ok = radar.requestHoldDelay();
    Serial.printf("[%s] Hold delay      %lu s\n", okFail(ok), (unsigned long)radar.getHoldDelay());

    ok = radar.requestLowPowerSleepTime();
    Serial.printf("[%s] LP sleep time   %lu ms\n", okFail(ok), (unsigned long)radar.getLowPowerSleepTime());

    ok = radar.requestOutputInterval();
    Serial.printf("[%s] Output interval %lu frames\n", okFail(ok), (unsigned long)radar.getOutputInterval());

    ok = radar.requestStayZoneLifecycle();
    Serial.printf("[%s] Stay lifecycle  %lu\n", okFail(ok), (unsigned long)radar.getStayZoneLifecycle());

    float zMin, zMax;
    ok = radar.requestZAxisRange();
    radar.getZAxisRange(zMin, zMax);
    Serial.printf("[%s] Z-axis range    %.2f .. %.2f m\n", okFail(ok), zMin, zMax);

    ok = radar.requestZones();
    Serial.printf("[%s] Zones (0-3 interference, 4-7 detection, 8-11 dwell)\n", okFail(ok));
    for (uint8_t id = 0; id < 12; id++) {
        ZoneBounds z;
        if (radar.getZone(id, z)) {
            Serial.printf("       zone %2u  x[%.2f, %.2f]  y[%.2f, %.2f]  z[%.2f, %.2f]\n",
                          id, z.xMin, z.xMax, z.yMin, z.yMax, z.zMin, z.zMax);
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial2.setRxBufferSize(1024);
    Serial2.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    radar.begin();

    Serial.println("\nMyLD6004 Diagnostics — reading sensor configuration...");
    printConfig();
    Serial.println();
}

void loop()
{
    radar.update();

    if (millis() - lastPrint < 1000) return;
    lastPrint = millis();

    Serial.printf("link=%s age=%lums bytes=%lu frames=%lu cksumErr=%lu | zones=%d%d%d%d | targets=%u\n",
                  radar.isConnected() ? "UP" : "DOWN",
                  (unsigned long)radar.lastUpdateAge(),
                  (unsigned long)radar.bytesReceived,
                  (unsigned long)radar.framesReceived,
                  (unsigned long)radar.checksumErrors,
                  radar.getZoneOccupancy(0), radar.getZoneOccupancy(1),
                  radar.getZoneOccupancy(2), radar.getZoneOccupancy(3),
                  radar.getTargetCount());

    for (uint8_t i = 0; i < radar.getTargetCount(); i++) {
        Target t = radar.getTarget(i);
        Serial.printf("    #%u id=%ld  x=%.2f y=%.2f z=%.2f  dop=%ld\n",
                      i, (long)t.clusterId, t.x, t.y, t.z, (long)t.dopIdx);
    }
}
