// Port of vgcs/viewpro/protocol.py. Keep the two in step: the tests in
// apk/custom/test compare these encoders with bytes made by the Python code.

#include "ViewproProtocol.h"

#include <cmath>

namespace viewpro {

namespace {

const Bytes kSerialHeader{0x55, 0xAA, 0xDC};
const Bytes kTcpHeader{0xEB, 0x90};

constexpr size_t kT1Len = 22;
constexpr size_t kF1Len = 1;
constexpr size_t kB1Len = 6;
constexpr size_t kD1Len = 12;
constexpr size_t kB1Offset = kT1Len + kF1Len;
constexpr size_t kD1Offset = kT1Len + kF1Len + kB1Len;

long long pyRound(double value)
{
    // nearbyint uses the current rounding mode, which is round-half-to-even
    // by default: the same rule as Python 3's round().
    return static_cast<long long>(std::nearbyint(value));
}

bool startsWith(const Bytes &payload, size_t at, const Bytes &head)
{
    if (payload.size() < at + head.size()) {
        return false;
    }
    for (size_t i = 0; i < head.size(); ++i) {
        if (payload[at + i] != head[i]) {
            return false;
        }
    }
    return true;
}

std::uint8_t serialChecksum(unsigned lengthByte, const Bytes &payload)
{
    unsigned cs = lengthByte & 0xFF;
    for (std::uint8_t b : payload) {
        cs ^= b;
    }
    return static_cast<std::uint8_t>(cs & 0xFF);
}

void putI16(Bytes &out, long long value)
{
    const long long v = value < -32768 ? -32768 : (value > 32767 ? 32767 : value);
    const unsigned u = static_cast<unsigned>(v) & 0xFFFF;
    out.push_back(static_cast<std::uint8_t>(u >> 8));
    out.push_back(static_cast<std::uint8_t>(u & 0xFF));
}

unsigned getU16(const Bytes &in, size_t at)
{
    return (static_cast<unsigned>(in[at]) << 8) | in[at + 1];
}

int getI16(const Bytes &in, size_t at)
{
    const int value = static_cast<int>(getU16(in, at));
    return value >= 0x8000 ? value - 0x10000 : value;
}

Bytes slice(const Bytes &in, size_t from, size_t count)
{
    if (from >= in.size()) {
        return {};
    }
    const size_t end = from + count < in.size() ? from + count : in.size();
    return Bytes(in.begin() + static_cast<long>(from), in.begin() + static_cast<long>(end));
}

} // namespace

Bytes buildSerialFrame(std::uint8_t cmdId, const Bytes &data)
{
    const unsigned bodyLen = (static_cast<unsigned>(data.size()) + 3) & 0x3F;
    Bytes payload{cmdId};
    payload.insert(payload.end(), data.begin(), data.end());
    Bytes out = kSerialHeader;
    out.push_back(static_cast<std::uint8_t>(bodyLen));
    out.insert(out.end(), payload.begin(), payload.end());
    out.push_back(serialChecksum(bodyLen, payload));
    return out;
}

std::optional<std::pair<std::uint8_t, Bytes>> parseSerialFrame(const Bytes &payload)
{
    if (payload.size() < 7 || !startsWith(payload, 0, kSerialHeader)) {
        return std::nullopt;
    }
    const unsigned bodyLen = payload[3] & 0x3F;
    if (bodyLen < 3 || payload.size() < 3 + bodyLen) {
        return std::nullopt;
    }
    const std::uint8_t cmdId = payload[4];
    const Bytes data = slice(payload, 5, bodyLen - 3);
    Bytes summed{cmdId};
    summed.insert(summed.end(), data.begin(), data.end());
    if (payload[3 + bodyLen - 1] != serialChecksum(bodyLen, summed)) {
        return std::nullopt;
    }
    return std::make_pair(cmdId, data);
}

Bytes wrapTcp(const Bytes &innerFrame)
{
    unsigned sum = 0;
    for (std::uint8_t b : innerFrame) {
        sum += b;
    }
    Bytes out = kTcpHeader;
    out.push_back(static_cast<std::uint8_t>(innerFrame.size() & 0xFF));
    out.insert(out.end(), innerFrame.begin(), innerFrame.end());
    out.push_back(static_cast<std::uint8_t>(sum & 0xFF));
    return out;
}

std::optional<Bytes> unwrapTcp(const Bytes &payload)
{
    if (payload.size() < 4 || !startsWith(payload, 0, kTcpHeader)) {
        return std::nullopt;
    }
    const size_t length = payload[2];
    if (payload.size() < 3 + length + 1) {
        return std::nullopt;
    }
    const Bytes inner = slice(payload, 3, length);
    unsigned sum = 0;
    for (std::uint8_t b : inner) {
        sum += b;
    }
    if ((sum & 0xFF) != payload[3 + length]) {
        return std::nullopt;
    }
    return inner;
}

Bytes encodeA1(int servo, long long p1, long long p2, long long p3, long long p4)
{
    Bytes out{static_cast<std::uint8_t>(servo & 0xFF)};
    putI16(out, p1);
    putI16(out, p2);
    putI16(out, p3);
    putI16(out, p4);
    return out;
}

Bytes encodeC1(int op, int zoomSpeed, int sensor, int lrf)
{
    const unsigned value = ((static_cast<unsigned>(lrf) & 0x7) << 13) | ((static_cast<unsigned>(op) & 0x7F) << 6)
        | ((static_cast<unsigned>(zoomSpeed) & 0x7) << 3) | (static_cast<unsigned>(sensor) & 0x7);
    return {static_cast<std::uint8_t>((value >> 8) & 0xFF), static_cast<std::uint8_t>(value & 0xFF)};
}

Bytes encodeGimbalCameraCommand(const GimbalCameraCommand &c)
{
    Bytes body = encodeA1(c.servo, c.servoP1, c.servoP2, c.servoP3, c.servoP4);
    const Bytes c1 = encodeC1(c.c1Op, c.c1ZoomSpeed, c.c1Sensor, c.c1Lrf);
    body.insert(body.end(), c1.begin(), c1.end());
    body.insert(body.end(), 3, 0);  // E1: the camera's tracker is left alone
    return wrapTcp(buildSerialFrame(kCmdA1C1E1, body));
}

Bytes encodeHeartbeat()
{
    return wrapTcp(buildSerialFrame(kCmdHeartbeat, Bytes{0x00}));
}

long long angleDegToRaw(double degrees)
{
    if (!std::isfinite(degrees)) {
        return 0;  // Python raises here; nothing sends such an angle
    }
    // Python's float % has the sign of the divisor.
    double wrapped = std::fmod(degrees + 180.0, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    wrapped -= 180.0;
    return pyRound(wrapped * 65536.0 / 360.0);
}

double angleRawToDeg(long long raw)
{
    return static_cast<double>(raw) * 360.0 / 65536.0;
}

long long speedDpsToRaw(double degreesPerSecond)
{
    if (!std::isfinite(degreesPerSecond)) {
        return 0;  // Python raises here
    }
    return pyRound(degreesPerSecond * 100.0);
}

std::optional<Attitude> decodeB1(const Bytes &data)
{
    if (data.size() < kB1Len) {
        return std::nullopt;
    }
    Attitude out;
    out.servoStatus = (data[0] >> 4) & 0x0F;
    const unsigned rollRaw12 = (static_cast<unsigned>(data[0] & 0x0F) << 8) | data[1];
    out.rollDeg = rollRaw12 * 180.0 / 4095.0 - 90.0;
    out.yawDeg = angleRawToDeg(getI16(data, 2));
    out.pitchDeg = angleRawToDeg(getI16(data, 4));
    return out;
}

std::optional<CameraStatus> decodeD1(const Bytes &data)
{
    if (data.size() < kD1Len) {
        return std::nullopt;
    }
    CameraStatus out;
    out.recordStatus = static_cast<int>(getU16(data, 2) & 0x3);
    const unsigned rangeRaw = (static_cast<unsigned>(data[1]) << 16) | (static_cast<unsigned>(data[4]) << 8) | data[5];
    if (rangeRaw > 0) {
        out.rangeM = rangeRaw * 0.1;
    }
    out.vfovDeg = getU16(data, 6) * 0.01;
    out.hfovDeg = getU16(data, 8) * 0.01;
    out.zoomX = getU16(data, 10) * 0.1;
    return out;
}

Tracking decodeF1(std::uint8_t byte)
{
    Tracking out;
    out.source = byte & 0x07;
    out.status = (byte >> 3) & 0x03;
    out.targetType = (byte >> 5) & 0x07;
    return out;
}

std::optional<Status> decodeStatusFrame(const Bytes &data)
{
    if (data.size() < kB1Offset + kB1Len) {
        return std::nullopt;
    }
    const std::optional<Attitude> attitude = decodeB1(slice(data, kB1Offset, kB1Len));
    if (!attitude) {
        return std::nullopt;
    }
    Status out;
    out.attitude = *attitude;
    out.tracking = decodeF1(data[kT1Len]);
    out.camera = decodeD1(slice(data, kD1Offset, kD1Len));
    return out;
}

std::optional<Status> findStatusFrame(const Bytes &payload)
{
    for (size_t pos = 0; pos + kSerialHeader.size() <= payload.size(); ++pos) {
        if (!startsWith(payload, pos, kSerialHeader)) {
            continue;
        }
        const auto parsed = parseSerialFrame(slice(payload, pos, payload.size() - pos));
        if (parsed && parsed->first == kCmdStatus) {
            if (auto decoded = decodeStatusFrame(parsed->second)) {
                return decoded;
            }
        }
    }
    return std::nullopt;
}

} // namespace viewpro
