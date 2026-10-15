#pragma once
#include <Arduino.h>

// ── Enumerations ───────────────────────────────────────────────────────────────

/** Detection sensitivity (control cmds 0x0A-0x0C, report 0x0A0E). */
enum SensitivityLevel : uint8_t { SENSITIVITY_LOW = 0, SENSITIVITY_MEDIUM, SENSITIVITY_HIGH };
/** Trigger speed (control cmds 0x0E-0x10, report 0x0A0F). */
enum TriggerSpeed     : uint8_t { TRIGGER_SLOW = 0, TRIGGER_MEDIUM, TRIGGER_FAST };
/** Installation method (control cmds 0x13/0x14, report 0x0A11). */
enum InstallationMode : uint8_t { INSTALL_CEILING = 0, INSTALL_WALL };
/** Operating mode (report 0x0A12 values 0-4). */
enum OperatingMode    : uint8_t {
    MODE_NORMAL = 0, MODE_LOW_POWER,
    MODE_RADAR_OFF_HIGH, MODE_RADAR_OFF_LOW, MODE_STRONG_REFLECTION
};
/** P20 GPIO output mode (report 0x0A15 values 0-5). */
enum P20Mode          : uint8_t {
    P20_HIGH_PRESENT = 0, P20_LOW_PRESENT,
    P20_ALWAYS_LOW, P20_ALWAYS_HIGH, P20_PULSE_LOW, P20_PULSE_HIGH
};

// ── Public Structs ─────────────────────────────────────────────────────────────

/** One tracked person from a 0x0A04 frame. */
struct Target {
    float   x, y, z;       // Cartesian coordinates in metres, relative to antenna centre
    int32_t dopIdx;         // Doppler index (~0 = micro-motion / breathing)
    int32_t clusterId;      // Cluster ID, stable across frames while the target is tracked
};

/** Firmware version reported by a 0xFFFF frame. */
struct FirmwareInfo {
    uint8_t projectName;        // 0 = general sensing, 5 = 3D point cloud
    uint8_t majorVersion;
    uint8_t subVersion;
    uint8_t modifiedVersion;
};

/** Bounds of one interference / detection / dwell zone, in metres. */
struct ZoneBounds {
    float xMin, xMax, yMin, yMax, zMin, zMax;
};

// ── Class ──────────────────────────────────────────────────────────────────────

class MyLD6004 {
public:

    // ── Lifecycle ──────────────────────────────────────────────────────────────

    /** Construct with a reference to the Arduino serial stream connected to the LD6004. */
    explicit MyLD6004(Stream& stream);

    /**
     * Initialise internal state and record the start time. Must be called once in setup().
     * The sensor ignores commands for ~2.5 s after power-on; commands sent before the first
     * frame arrives (or 2.5 s have passed since begin()) wait for that automatically.
     */
    void begin();

    /** Process all bytes currently available in the stream; call in every loop() iteration. */
    void update();

    // ── Presence & Targets ─────────────────────────────────────────────────────

    /** Returns true if at least one person is currently detected. */
    bool isPersonDetected() const;

    /** Returns the number of tracked targets in the latest frame (0-3). */
    uint8_t getTargetCount() const;

    /** Returns the target at local index (0-based). Returns zeroed Target if index >= getTargetCount(). */
    Target getTarget(uint8_t index) const;

    // ── Zone Occupancy ─────────────────────────────────────────────────────────

    /**
     * Returns the occupancy state of a detection zone.
     * zoneIndex is a local index 0-3 (0 = zone ID 4, 1 = zone ID 5, etc.).
     * Returns false if zoneIndex >= 4.
     */
    bool getZoneOccupancy(uint8_t zoneIndex) const;

    // ── Zone Configuration ─────────────────────────────────────────────────────

    /**
     * Set one of the 12 zones (TYPE 0x0202). IDs 0-3 = interference, 4-7 = detection,
     * 8-11 = dwell. Blocks until ACK (up to ~1 s with one retry). Returns true on success.
     * In 2D mode use zMin = -6 and zMax = 6.
     */
    bool setZone(uint8_t areaId,
                 float xMin, float xMax,
                 float yMin, float yMax,
                 float zMin, float zMax);

    /**
     * Query all zone coordinates (TYPE 0x0201 cmd 0x02). Blocks until all three response
     * frames (0x0A0B, 0x0A0C, 0x0A16) arrive (up to ~1 s). Read results with getZone().
     */
    bool requestZones();

    /** Copy the cached bounds of zone areaId (0-11) into out. Returns false if not yet received. */
    bool getZone(uint8_t areaId, ZoneBounds& out) const;

    // ── Sensor Configuration ───────────────────────────────────────────────────
    // set*() calls block until the sensor ACKs; request*() calls block until the
    // sensor's report frame arrives, so the matching getter is valid on return.
    // Both time out after ~1 s (500 ms, one retry). When the sensor is in low-power
    // mode the first command wakes it; the retry covers the case where it is missed.

    /** Set detection sensitivity. Returns true on ACK. */
    bool setSensitivity(SensitivityLevel level);

    /** Query sensitivity (cmd 0x0D); on success getSensitivity() holds the value. */
    bool requestSensitivity();

    /** Returns the last sensitivity value received from the radar. */
    SensitivityLevel getSensitivity() const;

    /** Set trigger speed. Returns true on ACK. */
    bool setTriggerSpeed(TriggerSpeed speed);

    /** Query trigger speed (cmd 0x11); on success getTriggerSpeed() holds the value. */
    bool requestTriggerSpeed();

    /** Returns the last trigger speed value received from the radar. */
    TriggerSpeed getTriggerSpeed() const;

    /** Set hold delay in seconds (TYPE 0x0203, default 30). Returns true on ACK. */
    bool setHoldDelay(uint32_t seconds);

    /** Query hold delay (cmd 0x05); on success getHoldDelay() holds the value. */
    bool requestHoldDelay();

    /** Returns the last hold-delay value (seconds) received from the radar. */
    uint32_t getHoldDelay() const;

    /** Set installation mode (ceiling or wall). Returns true on ACK. */
    bool setInstallationMode(InstallationMode mode);

    /** Query installation mode (cmd 0x15); on success getInstallationMode() holds the value. */
    bool requestInstallationMode();

    /** Returns the last installation mode received from the radar. */
    InstallationMode getInstallationMode() const;

    /** Set UART output interval in frames (TYPE 0x0207, 1 = every frame). Returns true on ACK. */
    bool setOutputInterval(uint32_t frames);

    /** Query output interval (cmd 0x27); on success getOutputInterval() holds the value. */
    bool requestOutputInterval();

    /** Returns the last UART output interval (frames) received from the radar. */
    uint32_t getOutputInterval() const;

    // ── Firmware ───────────────────────────────────────────────────────────────

    /** Query firmware version (TYPE 0xFFFF); on success getFirmwareVersion() holds the value. */
    bool queryFirmware();

    /** Returns the last firmware info received from the radar. */
    FirmwareInfo getFirmwareVersion() const;

    // ── Connection Health ──────────────────────────────────────────────────────

    /**
     * Returns true if a valid frame has been received within the last 2000 ms.
     * Returns false if no frame has ever been received.
     */
    bool isConnected() const;

    /**
     * Returns elapsed milliseconds since the last successfully parsed frame.
     * Before any frame is received, returns time since begin() was called.
     */
    uint32_t lastUpdateAge() const;

    // ── Diagnostic Counters ────────────────────────────────────────────────────

    uint32_t bytesReceived;    // Incremented on every byte read from the stream
    uint32_t framesReceived;   // Incremented on each successful frame parse
    uint32_t checksumErrors;   // Incremented on each checksum failure

    /** Reset bytesReceived, framesReceived and checksumErrors to 0. Does not affect any other state. */
    void resetCounters();

    // ── P20 GPIO Mode ──────────────────────────────────────────────────────────

    /** Set P20 output mode. Returns false immediately if mode is out of range, else true on ACK. */
    bool setP20Mode(P20Mode mode);

    /** Query P20 mode (cmd 0x1F); on success p20Mode() holds the value. */
    bool requestP20Status();

    /** Returns the last P20 mode received from the radar (TYPE 0x0A15). */
    P20Mode p20Mode() const;

    // ── Operating Mode ─────────────────────────────────────────────────────────

    /** Set operating mode. Returns false immediately if mode is out of range, else true on ACK. */
    bool setOperatingMode(OperatingMode mode);

    /** Query operating mode (cmd 0x18); on success operatingMode() holds the value. */
    bool requestOperatingMode();

    /** Returns the last operating mode received from the radar (TYPE 0x0A12). */
    OperatingMode operatingMode() const;

    // ── Conditional Zone Initialization ────────────────────────────────────────

    /**
     * Write a zone only if it is currently unset (all coordinate bounds == 0.0).
     * Queries zone state first if not already cached. Returns true if the zone was
     * written, false if already configured, out of range, or if the query timed out.
     */
    bool setZoneIfUnset(uint8_t areaId,
                        float xMin, float xMax,
                        float yMin, float yMax,
                        float zMin, float zMax);

    // ── Z-Axis Range ───────────────────────────────────────────────────────────

    /** Set Z-axis detection range (TYPE 0x0204, 3D only). Returns true on ACK. */
    bool setZAxisRange(float zMin, float zMax);

    /** Query Z-axis range (cmd 0x12); on success getZAxisRange() holds the values. */
    bool requestZAxisRange();

    /** Populate zMin and zMax with the last Z-axis range received from the radar. */
    void getZAxisRange(float& zMin, float& zMax) const;

    // ── Low-Power Sleep Time ───────────────────────────────────────────────────

    /** Set low-power sleep interval in milliseconds (TYPE 0x0205, default 500). Returns true on ACK. */
    bool setLowPowerSleepTime(uint32_t ms);

    /** Query low-power sleep time (cmd 0x19); on success getLowPowerSleepTime() holds the value. */
    bool requestLowPowerSleepTime();

    /** Returns the last low-power sleep time (ms) received from the radar. */
    uint32_t getLowPowerSleepTime() const;

    // ── Stay Zone Lifecycle ────────────────────────────────────────────────────

    /** Set dwell-zone target lifecycle (TYPE 0x0206, default 300, decreases 20/s). Returns true on ACK. */
    bool setStayZoneLifecycle(uint32_t value);

    /** Query dwell-zone lifecycle (cmd 0x26); on success getStayZoneLifecycle() holds the value. */
    bool requestStayZoneLifecycle();

    /** Returns the last stay zone lifecycle value received from the radar. */
    uint32_t getStayZoneLifecycle() const;

private:

    // ── Parser state ───────────────────────────────────────────────────────────

    enum ParserState : uint8_t {
        WAIT_SOF, READ_ID_HI, READ_ID_LO, READ_LEN_HI, READ_LEN_LO,
        READ_TYPE_HI, READ_TYPE_LO, READ_HEAD_CKSUM,
        READ_DATA, READ_DATA_CKSUM
    };

    // ── Zone coordinate cache entry ────────────────────────────────────────────

    struct ZoneCoords {
        ZoneBounds b;
        bool       valid;
    };

    // ── Private fields ─────────────────────────────────────────────────────────

    Stream*     _stream;

    // Parser
    uint8_t     _state;         // ParserState
    uint16_t    _frameId;
    uint16_t    _dataLen;
    uint16_t    _frameType;
    uint8_t     _headChecksum;
    uint8_t     _dataChecksum;
    uint16_t    _dataPos;
    bool        _discard;       // payload larger than _dataBuf: validate but don't store
    // 128 bytes: largest frame this library decodes is a zone-coord report (96-112 bytes).
    // Larger frames (e.g. 0x0A08 point clouds) are consumed and dropped.
    uint8_t     _dataBuf[128];

    // Command state
    uint16_t    _txFrameId;
    uint16_t    _lastAckedType;   // TYPE of the last zero-length (ACK) frame
    uint16_t    _lastRespType;    // TYPE of the last frame carrying data
    uint8_t     _zoneCacheGroupMask;

    // Cached sensor state
    Target      _targets[3];
    uint8_t     _targetCount;
    bool        _personDetected;
    bool        _zoneOccupancy[4];
    ZoneCoords  _zoneCache[12];
    FirmwareInfo _firmware;

    // Cached configuration state
    uint8_t     _sensitivity;
    uint8_t     _triggerSpeed;
    uint32_t    _holdDelay;
    uint8_t     _installMode;
    uint8_t     _operatingMode;
    uint8_t     _p20Mode;
    uint32_t    _outputInterval;
    float       _zAxisMin;
    float       _zAxisMax;
    uint32_t    _lowPowerSleepTime;
    uint32_t    _stayZoneLifecycle;

    // Connection health & diagnostics
    uint32_t    _lastFrameTime;
    uint32_t    _initTime;
    bool        _initialized;

    // ── Private helpers ────────────────────────────────────────────────────────

    void _feedByte(uint8_t b);
    void _handleFrame();
    void _sendFrame(uint16_t type, const uint8_t* data, uint16_t len);
    bool _sendAndWait(uint16_t type, const uint8_t* data, uint16_t len,
                      uint16_t waitType, bool waitForData);
    bool _sendControl(uint32_t cmd, uint16_t reportType);
    bool _sendU32(uint16_t type, uint32_t value);
    void _waitSensorReady();
    bool _isZoneUnset(uint8_t areaId) const;

    void _parseTargetFrame();
    void _parseZoneOccupancyFrame();
    void _parseZoneCoordsFrame(uint8_t cacheOffset);
};
