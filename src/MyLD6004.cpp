#include "MyLD6004.h"
#include <string.h>

// ── Protocol constants ────────────────────────────────────────────────────────

static const uint8_t  TF_SOF            = 0x01;

static const uint16_t TYPE_CONTROL      = 0x0201;
static const uint16_t TYPE_SET_ZONE     = 0x0202;
static const uint16_t TYPE_SET_HOLD     = 0x0203;
static const uint16_t TYPE_SET_Z_RANGE  = 0x0204;
static const uint16_t TYPE_SET_SLEEP    = 0x0205;
static const uint16_t TYPE_SET_STAY     = 0x0206;
static const uint16_t TYPE_SET_INTERVAL = 0x0207;

static const uint16_t TYPE_TARGETS      = 0x0A04;
static const uint16_t TYPE_ZONE_STATE   = 0x0A0A;
static const uint16_t TYPE_ZONE_NOISE   = 0x0A0B;
static const uint16_t TYPE_ZONE_DETECT  = 0x0A0C;
static const uint16_t TYPE_HOLD         = 0x0A0D;
static const uint16_t TYPE_SENSITIVITY  = 0x0A0E;
static const uint16_t TYPE_TRIGGER      = 0x0A0F;
static const uint16_t TYPE_Z_RANGE      = 0x0A10;
static const uint16_t TYPE_INSTALL      = 0x0A11;
static const uint16_t TYPE_OP_MODE      = 0x0A12;
static const uint16_t TYPE_SLEEP        = 0x0A13;
static const uint16_t TYPE_P20          = 0x0A15;
static const uint16_t TYPE_ZONE_STAY    = 0x0A16;
static const uint16_t TYPE_STAY_LIFE    = 0x0A17;
static const uint16_t TYPE_INTERVAL     = 0x0A18;
static const uint16_t TYPE_FIRMWARE     = 0xFFFF;

static const uint32_t CMD_TIMEOUT_MS    = 500;
static const uint8_t  CMD_ATTEMPTS      = 2;     // protocol §6: "if there is no reply, resend"
static const uint32_t BOOT_SETTLE_MS    = 2500;  // sensor ignores commands while booting
static const uint32_t CONNECTED_MS      = 2000;

// ── Enum-to-command-code lookup tables (protocol §6.1) ────────────────────────

static const uint8_t SENS_CMD[3]  = {0x0A, 0x0B, 0x0C};   // SENSITIVITY_LOW/MEDIUM/HIGH
static const uint8_t TRIG_CMD[3]  = {0x0E, 0x0F, 0x10};   // TRIGGER_SLOW/MEDIUM/FAST
static const uint8_t INST_CMD[2]  = {0x13, 0x14};          // INSTALL_CEILING/WALL
static const uint8_t P20_CMD[6]   = {0x1D, 0x1E, 0x20, 0x21, 0x22, 0x23};
static const uint8_t OM_CMD[5]    = {0x17, 0x16, 0x1B, 0x1C, 0x24};

// ── Little-endian helpers (DATA section is LE; header fields are BE) ─────────

static uint32_t readU32(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float readF32(const uint8_t* p)
{
    uint32_t u = readU32(p);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static void writeU32(uint8_t* p, uint32_t v)
{
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

static void writeF32(uint8_t* p, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    writeU32(p, u);
}

// ── Constructor ───────────────────────────────────────────────────────────────

MyLD6004::MyLD6004(Stream& stream)
    : bytesReceived(0), framesReceived(0), checksumErrors(0)
{
    _stream = &stream;

    _state          = WAIT_SOF;
    _frameId        = 0;
    _dataLen        = 0;
    _frameType      = 0;
    _headChecksum   = 0;
    _dataChecksum   = 0;
    _dataPos        = 0;
    _discard        = false;
    memset(_dataBuf, 0, sizeof(_dataBuf));

    _txFrameId           = 0;
    _lastAckedType       = 0;
    _lastRespType        = 0;
    _zoneCacheGroupMask  = 0;

    _targetCount    = 0;
    _personDetected = false;
    memset(_targets,      0, sizeof(_targets));
    memset(_zoneOccupancy, 0, sizeof(_zoneOccupancy));
    memset(_zoneCache,    0, sizeof(_zoneCache));
    memset(&_firmware,    0, sizeof(_firmware));

    _sensitivity        = 0;
    _triggerSpeed       = 0;
    _holdDelay          = 0;
    _installMode        = 0;
    _operatingMode      = 0;
    _p20Mode            = 0;
    _outputInterval     = 0;
    _zAxisMin           = 0.0f;
    _zAxisMax           = 0.0f;
    _lowPowerSleepTime  = 0;
    _stayZoneLifecycle  = 0;

    _lastFrameTime = 0;
    _initTime      = 0;
    _initialized   = false;
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

void MyLD6004::begin()
{
    _initTime      = millis();
    _lastFrameTime = 0;
    _state         = WAIT_SOF;
    _initialized   = true;
}

void MyLD6004::update()
{
    while (_stream->available()) {
        bytesReceived++;
        _feedByte((uint8_t)_stream->read());
    }
}

void MyLD6004::_feedByte(uint8_t b)
{
    switch (_state) {

    case WAIT_SOF:
        if (b == TF_SOF) {
            _headChecksum = TF_SOF;
            _state = READ_ID_HI;
        }
        break;

    case READ_ID_HI:
        _frameId = (uint16_t)b << 8;
        _headChecksum ^= b;
        _state = READ_ID_LO;
        break;

    case READ_ID_LO:
        _frameId |= b;
        _headChecksum ^= b;
        _state = READ_LEN_HI;
        break;

    case READ_LEN_HI:
        _dataLen = (uint16_t)b << 8;
        _headChecksum ^= b;
        _state = READ_LEN_LO;
        break;

    case READ_LEN_LO:
        _dataLen |= b;
        _headChecksum ^= b;
        _state = READ_TYPE_HI;
        break;

    case READ_TYPE_HI:
        _frameType = (uint16_t)b << 8;
        _headChecksum ^= b;
        _state = READ_TYPE_LO;
        break;

    case READ_TYPE_LO:
        _frameType |= b;
        _headChecksum ^= b;
        _state = READ_HEAD_CKSUM;
        break;

    case READ_HEAD_CKSUM:
        // Head checksum = ~XOR of (SOF + ID_HI + ID_LO + LEN_HI + LEN_LO + TYPE_HI + TYPE_LO)
        if (b != (uint8_t)(~_headChecksum)) {
            checksumErrors++;
#ifdef MYLD6004_DEBUG
            Serial.print(F("[MyLD6004] HEAD CKSUM FAIL type=0x"));
            Serial.println(_frameType, HEX);
#endif
            // The SOF was probably a 0x01 inside another frame's payload. A real
            // SOF may be among the 7 bytes consumed since, so rescan them.
            _state = WAIT_SOF;
            const uint8_t replay[7] = {
                (uint8_t)(_frameId >> 8),   (uint8_t)_frameId,
                (uint8_t)(_dataLen >> 8),   (uint8_t)_dataLen,
                (uint8_t)(_frameType >> 8), (uint8_t)_frameType, b
            };
            for (uint8_t i = 0; i < 7; i++) _feedByte(replay[i]);
            break;
        }
        if (_dataLen == 0) {
            // Zero-length frames (ACKs) carry no DATA_CKSUM byte on the wire
            // (protocol §5.1 example, confirmed on hardware). Waiting for one
            // would swallow the next frame's SOF and desync the parser.
            _handleFrame();
            framesReceived++;
            _lastFrameTime = millis();
            _state = WAIT_SOF;
            break;
        }
        _dataPos      = 0;
        _dataChecksum = 0;
        _discard      = (_dataLen > sizeof(_dataBuf));
        _state        = READ_DATA;
        break;

    case READ_DATA:
        if (!_discard) _dataBuf[_dataPos] = b;
        _dataPos++;
        _dataChecksum ^= b;
        if (_dataPos >= _dataLen) {
            _state = READ_DATA_CKSUM;
        }
        break;

    case READ_DATA_CKSUM:
        if (b == (uint8_t)(~_dataChecksum)) {
            if (!_discard) _handleFrame();
            framesReceived++;
            _lastFrameTime = millis();
#ifdef MYLD6004_DEBUG
            Serial.print(F("[MyLD6004] FRAME OK type=0x"));
            Serial.print(_frameType, HEX);
            Serial.print(F(" len="));
            Serial.println(_dataLen);
#endif
        } else {
            checksumErrors++;
#ifdef MYLD6004_DEBUG
            Serial.print(F("[MyLD6004] DATA CKSUM FAIL type=0x"));
            Serial.println(_frameType, HEX);
#endif
        }
        _state = WAIT_SOF;
        break;
    }
}

// ── Frame dispatcher ──────────────────────────────────────────────────────────

void MyLD6004::_handleFrame()
{
    if (_dataLen == 0) {
        // ACK: the sensor replies to every command with the same TYPE and no DATA
        _lastAckedType = _frameType;
        return;
    }

    // Single-value reports may be 1 byte (uint8) or 4 bytes (uint32 LE);
    // byte 0 is the value either way.
    const uint8_t v8 = _dataBuf[0];
    const bool has4  = (_dataLen >= 4);

    switch (_frameType) {
    case TYPE_TARGETS:     _parseTargetFrame();        break;
    case TYPE_ZONE_STATE:  _parseZoneOccupancyFrame(); break;
    case TYPE_ZONE_NOISE:
        _parseZoneCoordsFrame(0);
        _zoneCacheGroupMask |= 0x01;
        break;
    case TYPE_ZONE_DETECT:
        _parseZoneCoordsFrame(4);
        _zoneCacheGroupMask |= 0x02;
        break;
    case TYPE_ZONE_STAY:
        _parseZoneCoordsFrame(8);
        _zoneCacheGroupMask |= 0x04;
        break;
    case TYPE_HOLD:        if (has4) _holdDelay = readU32(_dataBuf);         break;
    case TYPE_SENSITIVITY: _sensitivity  = (v8 > 2) ? 2 : v8;                break;
    case TYPE_TRIGGER:     _triggerSpeed = (v8 > 2) ? 2 : v8;                break;
    case TYPE_Z_RANGE:
        if (_dataLen >= 8) {
            _zAxisMin = readF32(&_dataBuf[0]);
            _zAxisMax = readF32(&_dataBuf[4]);
        }
        break;
    case TYPE_INSTALL:     _installMode   = (v8 > 1) ? 1 : v8;               break;
    case TYPE_OP_MODE:     _operatingMode = (v8 > 4) ? 4 : v8;               break;
    case TYPE_SLEEP:       if (has4) _lowPowerSleepTime = readU32(_dataBuf); break;
    case TYPE_P20:         _p20Mode = (v8 > 5) ? 5 : v8;                     break;
    case TYPE_STAY_LIFE:   if (has4) _stayZoneLifecycle = readU32(_dataBuf); break;
    case TYPE_INTERVAL:    if (has4) _outputInterval = readU32(_dataBuf);    break;
    case TYPE_FIRMWARE:
        if (has4) {
            _firmware.projectName     = _dataBuf[0];
            _firmware.majorVersion    = _dataBuf[1];
            _firmware.subVersion      = _dataBuf[2];
            _firmware.modifiedVersion = _dataBuf[3];
        }
        break;
    default:
        // 0x0A08 point cloud, 0x0A14 low-power status, and unknown types are ignored
        break;
    }

    _lastRespType = _frameType;
}

// ── Frame sender ──────────────────────────────────────────────────────────────

void MyLD6004::_sendFrame(uint16_t type, const uint8_t* data, uint16_t len)
{
    // SOF, ID (BE), LEN (BE), TYPE (BE), HEAD_CKSUM, [DATA (LE), DATA_CKSUM]
    uint8_t header[8];
    header[0] = TF_SOF;
    header[1] = (uint8_t)(_txFrameId >> 8);
    header[2] = (uint8_t)(_txFrameId & 0xFF);
    header[3] = (uint8_t)(len >> 8);
    header[4] = (uint8_t)(len & 0xFF);
    header[5] = (uint8_t)(type >> 8);
    header[6] = (uint8_t)(type & 0xFF);
    _txFrameId++;

    uint8_t headXor = 0;
    for (uint8_t i = 0; i < 7; i++) headXor ^= header[i];
    header[7] = (uint8_t)(~headXor);
    _stream->write(header, 8);

    // Zero-length frames end at HEAD_CKSUM (protocol §5.1: "01 00 01 00 00 FF FF FF")
    if (len > 0 && data != nullptr) {
        uint8_t dataXor = 0;
        for (uint16_t i = 0; i < len; i++) dataXor ^= data[i];
        _stream->write(data, len);
        _stream->write((uint8_t)(~dataXor));
    }
}

// ── Command helpers ───────────────────────────────────────────────────────────

void MyLD6004::_waitSensorReady()
{
    // Once the sensor is streaming it has finished booting; otherwise give it
    // up to BOOT_SETTLE_MS after begin() before the first command.
    while (_lastFrameTime == 0 && (millis() - _initTime) < BOOT_SETTLE_MS) {
        update();
        yield();
    }
}

bool MyLD6004::_sendAndWait(uint16_t type, const uint8_t* data, uint16_t len,
                            uint16_t waitType, bool waitForData)
{
    _waitSensorReady();
    for (uint8_t attempt = 0; attempt < CMD_ATTEMPTS; attempt++) {
        _lastAckedType = 0;
        _lastRespType  = 0;
        _sendFrame(type, data, len);
        uint32_t start = millis();
        while ((millis() - start) < CMD_TIMEOUT_MS) {
            update();
            if (waitForData ? (_lastRespType == waitType) : (_lastAckedType == waitType)) {
                return true;
            }
            yield();
        }
    }
    return false;
}

bool MyLD6004::_sendControl(uint32_t cmd, uint16_t reportType)
{
    // Control command DATA is an int32 LE (protocol §6.1: "01 00 00 00")
    uint8_t buf[4];
    writeU32(buf, cmd);
    if (reportType == 0) {
        return _sendAndWait(TYPE_CONTROL, buf, 4, TYPE_CONTROL, false);
    }
    return _sendAndWait(TYPE_CONTROL, buf, 4, reportType, true);
}

bool MyLD6004::_sendU32(uint16_t type, uint32_t value)
{
    uint8_t buf[4];
    writeU32(buf, value);
    return _sendAndWait(type, buf, 4, type, false);
}

bool MyLD6004::_isZoneUnset(uint8_t areaId) const
{
    const ZoneCoords& z = _zoneCache[areaId];
    return z.valid &&
           z.b.xMin == 0.0f && z.b.xMax == 0.0f &&
           z.b.yMin == 0.0f && z.b.yMax == 0.0f &&
           z.b.zMin == 0.0f && z.b.zMax == 0.0f;
}

// ── Presence & Targets ────────────────────────────────────────────────────────

bool MyLD6004::isPersonDetected() const { return _personDetected; }
uint8_t MyLD6004::getTargetCount() const { return _targetCount; }

Target MyLD6004::getTarget(uint8_t index) const
{
    if (index >= _targetCount) {
        Target t = {};
        return t;
    }
    return _targets[index];
}

// ── Zone Occupancy ────────────────────────────────────────────────────────────

bool MyLD6004::getZoneOccupancy(uint8_t zoneIndex) const
{
    if (zoneIndex >= 4) return false;
    return _zoneOccupancy[zoneIndex];
}

// ── Zone Configuration ────────────────────────────────────────────────────────

bool MyLD6004::setZone(uint8_t areaId,
                       float xMin, float xMax,
                       float yMin, float yMax,
                       float zMin, float zMax)
{
    if (areaId >= 12) return false;

    // DATA: [int32 areaId, 6 × float] = 28 bytes, all LE (protocol §6.2, LEN 0x1C)
    uint8_t buf[28];
    writeU32(&buf[0], areaId);
    writeF32(&buf[4],  xMin);
    writeF32(&buf[8],  xMax);
    writeF32(&buf[12], yMin);
    writeF32(&buf[16], yMax);
    writeF32(&buf[20], zMin);
    writeF32(&buf[24], zMax);

    bool ok = _sendAndWait(TYPE_SET_ZONE, buf, sizeof(buf), TYPE_SET_ZONE, false);
    if (ok) {
        _zoneCache[areaId].valid = false;
    }
    return ok;
}

bool MyLD6004::requestZones()
{
    _waitSensorReady();
    uint8_t buf[4];
    writeU32(buf, 0x02);
    for (uint8_t attempt = 0; attempt < CMD_ATTEMPTS; attempt++) {
        _zoneCacheGroupMask = 0;
        _sendFrame(TYPE_CONTROL, buf, 4);
        uint32_t start = millis();
        while ((millis() - start) < CMD_TIMEOUT_MS) {
            update();
            if ((_zoneCacheGroupMask & 0x07) == 0x07) return true;
            yield();
        }
    }
    return false;
}

bool MyLD6004::getZone(uint8_t areaId, ZoneBounds& out) const
{
    if (areaId >= 12 || !_zoneCache[areaId].valid) return false;
    out = _zoneCache[areaId].b;
    return true;
}

// ── Sensor Configuration ──────────────────────────────────────────────────────

bool MyLD6004::setSensitivity(SensitivityLevel level)
{
    if (level >= 3) return false;
    return _sendControl(SENS_CMD[level], 0);
}

bool MyLD6004::requestSensitivity()      { return _sendControl(0x0D, TYPE_SENSITIVITY); }
SensitivityLevel MyLD6004::getSensitivity() const { return (SensitivityLevel)_sensitivity; }

bool MyLD6004::setTriggerSpeed(TriggerSpeed speed)
{
    if (speed >= 3) return false;
    return _sendControl(TRIG_CMD[speed], 0);
}

bool MyLD6004::requestTriggerSpeed()     { return _sendControl(0x11, TYPE_TRIGGER); }
TriggerSpeed MyLD6004::getTriggerSpeed() const { return (TriggerSpeed)_triggerSpeed; }

bool MyLD6004::setHoldDelay(uint32_t seconds) { return _sendU32(TYPE_SET_HOLD, seconds); }
bool MyLD6004::requestHoldDelay()        { return _sendControl(0x05, TYPE_HOLD); }
uint32_t MyLD6004::getHoldDelay() const  { return _holdDelay; }

bool MyLD6004::setInstallationMode(InstallationMode mode)
{
    if (mode >= 2) return false;
    return _sendControl(INST_CMD[mode], 0);
}

bool MyLD6004::requestInstallationMode() { return _sendControl(0x15, TYPE_INSTALL); }
InstallationMode MyLD6004::getInstallationMode() const { return (InstallationMode)_installMode; }

bool MyLD6004::setOutputInterval(uint32_t frames) { return _sendU32(TYPE_SET_INTERVAL, frames); }
bool MyLD6004::requestOutputInterval()   { return _sendControl(0x27, TYPE_INTERVAL); }
uint32_t MyLD6004::getOutputInterval() const { return _outputInterval; }

// ── Firmware ──────────────────────────────────────────────────────────────────

bool MyLD6004::queryFirmware()
{
    return _sendAndWait(TYPE_FIRMWARE, nullptr, 0, TYPE_FIRMWARE, true);
}

FirmwareInfo MyLD6004::getFirmwareVersion() const { return _firmware; }

// ── Connection Health ─────────────────────────────────────────────────────────

bool MyLD6004::isConnected() const
{
    return _lastFrameTime != 0 && (millis() - _lastFrameTime) < CONNECTED_MS;
}

uint32_t MyLD6004::lastUpdateAge() const
{
    if (_lastFrameTime != 0) return millis() - _lastFrameTime;
    return millis() - _initTime;
}

// ── Diagnostic Counters ───────────────────────────────────────────────────────

void MyLD6004::resetCounters()
{
    bytesReceived  = 0;
    framesReceived = 0;
    checksumErrors = 0;
}

// ── P20 GPIO Mode ─────────────────────────────────────────────────────────────

bool MyLD6004::setP20Mode(P20Mode mode)
{
    if (mode >= 6) return false;
    return _sendControl(P20_CMD[mode], 0);
}

bool MyLD6004::requestP20Status()        { return _sendControl(0x1F, TYPE_P20); }
P20Mode MyLD6004::p20Mode() const        { return (P20Mode)_p20Mode; }

// ── Operating Mode ────────────────────────────────────────────────────────────

bool MyLD6004::setOperatingMode(OperatingMode mode)
{
    if (mode >= 5) return false;
    return _sendControl(OM_CMD[mode], 0);
}

bool MyLD6004::requestOperatingMode()    { return _sendControl(0x18, TYPE_OP_MODE); }
OperatingMode MyLD6004::operatingMode() const { return (OperatingMode)_operatingMode; }

// ── Conditional Zone Init ─────────────────────────────────────────────────────

bool MyLD6004::setZoneIfUnset(uint8_t areaId,
                               float xMin, float xMax,
                               float yMin, float yMax,
                               float zMin, float zMax)
{
    if (areaId >= 12) return false;
    if (!_zoneCache[areaId].valid) {
        if (!requestZones()) return false;
    }
    if (!_isZoneUnset(areaId)) return false;
    return setZone(areaId, xMin, xMax, yMin, yMax, zMin, zMax);
}

// ── Z-Axis Range ──────────────────────────────────────────────────────────────

bool MyLD6004::setZAxisRange(float zMin, float zMax)
{
    uint8_t buf[8];
    writeF32(&buf[0], zMin);
    writeF32(&buf[4], zMax);
    return _sendAndWait(TYPE_SET_Z_RANGE, buf, 8, TYPE_SET_Z_RANGE, false);
}

bool MyLD6004::requestZAxisRange()       { return _sendControl(0x12, TYPE_Z_RANGE); }

void MyLD6004::getZAxisRange(float& zMin, float& zMax) const
{
    zMin = _zAxisMin;
    zMax = _zAxisMax;
}

// ── Low-Power Sleep Time ──────────────────────────────────────────────────────

bool MyLD6004::setLowPowerSleepTime(uint32_t ms) { return _sendU32(TYPE_SET_SLEEP, ms); }
bool MyLD6004::requestLowPowerSleepTime() { return _sendControl(0x19, TYPE_SLEEP); }
uint32_t MyLD6004::getLowPowerSleepTime() const { return _lowPowerSleepTime; }

// ── Stay Zone Lifecycle ───────────────────────────────────────────────────────

bool MyLD6004::setStayZoneLifecycle(uint32_t value) { return _sendU32(TYPE_SET_STAY, value); }
bool MyLD6004::requestStayZoneLifecycle() { return _sendControl(0x26, TYPE_STAY_LIFE); }
uint32_t MyLD6004::getStayZoneLifecycle() const { return _stayZoneLifecycle; }

// ── Frame parsers ─────────────────────────────────────────────────────────────

void MyLD6004::_parseTargetFrame()
{
    // DATA: int32 target_num, then per target: float x, y, z; int32 dop_idx, cluster_id
    if (_dataLen < 4) return;
    uint32_t count = readU32(_dataBuf);
    if (count > 3) count = 3;

    // Clear all target slots so the previous frame's data never leaks into the
    // current frame when the new count is lower (or parsing breaks early).
    memset(_targets, 0, sizeof(_targets));

    uint8_t parsed = 0;
    uint16_t offset = 4;
    for (uint8_t i = 0; i < count; i++) {
        if (offset + 20 > _dataLen) break;   // truncated payload
        _targets[i].x         = readF32(&_dataBuf[offset]);
        _targets[i].y         = readF32(&_dataBuf[offset + 4]);
        _targets[i].z         = readF32(&_dataBuf[offset + 8]);
        _targets[i].dopIdx    = (int32_t)readU32(&_dataBuf[offset + 12]);
        _targets[i].clusterId = (int32_t)readU32(&_dataBuf[offset + 16]);
        offset += 20;
        parsed++;
    }
    _targetCount    = parsed;
    _personDetected = (parsed > 0);
}

void MyLD6004::_parseZoneOccupancyFrame()
{
    // DATA: 4 × uint32 LE detection_state_area0..3 (1 = occupied)
    if (_dataLen < 16) return;
    for (uint8_t i = 0; i < 4; i++) {
        _zoneOccupancy[i] = (readU32(&_dataBuf[i * 4]) != 0);
    }
}

void MyLD6004::_parseZoneCoordsFrame(uint8_t cacheOffset)
{
    // DATA: 4 zones × 6 floats (96 bytes). The protocol table also shows an int32
    // area_id per zone; if the frame is 112 bytes, skip that 4-byte prefix.
    uint16_t stride, skip;
    if (_dataLen >= 112)     { stride = 28; skip = 4; }
    else if (_dataLen >= 96) { stride = 24; skip = 0; }
    else return;

    for (uint8_t i = 0; i < 4; i++) {
        const uint8_t* p = &_dataBuf[i * stride + skip];
        ZoneCoords& z = _zoneCache[cacheOffset + i];
        z.b.xMin = readF32(p);
        z.b.xMax = readF32(p + 4);
        z.b.yMin = readF32(p + 8);
        z.b.yMax = readF32(p + 12);
        z.b.zMin = readF32(p + 16);
        z.b.zMax = readF32(p + 20);
        z.valid  = true;
    }
}
