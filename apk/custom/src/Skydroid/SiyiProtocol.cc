// Port of vgcs/siyi/protocol.py. Keep the two in step: the tests in
// apk/custom/test compare these encoders with bytes made by the Python code.

#include "SiyiProtocol.h"

#include <array>
#include <cmath>

namespace siyi {

namespace {

// The table of the SIYI manual is the CRC-CCITT table for G(X) = X^16 + X^12 + X^5 + 1
// (entry i is the CRC of the byte i). Made here instead of typed in, and checked
// against the Python code's typed-in table by the tests.
constexpr std::array<std::uint16_t, 256> makeTable()
{
    std::array<std::uint16_t, 256> table{};
    for (unsigned i = 0; i < 256; ++i) {
        unsigned crc = i << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
        table[i] = static_cast<std::uint16_t>(crc & 0xFFFF);
    }
    return table;
}

constexpr std::array<std::uint16_t, 256> kCrcTable = makeTable();

long long pyRound(double value)
{
    // nearbyint uses the current rounding mode, which is round-half-to-even
    // by default: the same rule as Python 3's round().
    return static_cast<long long>(std::nearbyint(value));
}

void putU16(Bytes &out, unsigned value)
{
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

unsigned getU16(const Bytes &in, size_t at)
{
    return static_cast<unsigned>(in[at]) | (static_cast<unsigned>(in[at + 1]) << 8);
}

int getI16(const Bytes &in, size_t at)
{
    const int value = static_cast<int>(getU16(in, at));
    return value >= 0x8000 ? value - 0x10000 : value;
}

/// struct.pack("<h", round(v)): false when it does not fit (Python raises),
/// also for NaN and infinity.
bool putRoundedI16(Bytes &out, double value)
{
    if (!std::isfinite(value) || value < -32768.5 || value > 32767.5) {
        return false;
    }
    const long long rounded = pyRound(value);
    if (rounded < -32768 || rounded > 32767) {
        return false;
    }
    putU16(out, static_cast<unsigned>(rounded) & 0xFFFF);
    return true;
}

std::uint8_t i8(long long value)
{
    return static_cast<std::uint8_t>(value & 0xFF);
}

std::uint8_t sign(int direction)
{
    return i8(direction > 0 ? 1 : (direction < 0 ? -1 : 0));
}

} // namespace

std::uint16_t crc16(const Bytes &data, std::uint16_t init)
{
    unsigned crc = init;
    for (std::uint8_t b : data) {
        const unsigned temp = (crc >> 8) & 0xFF;
        crc = ((crc << 8) ^ kCrcTable[(b ^ temp) & 0xFF]) & 0xFFFF;
    }
    return static_cast<std::uint16_t>(crc);
}

Bytes buildRequest(std::uint8_t cmdId, const Bytes &data, std::uint16_t seq)
{
    Bytes out;
    putU16(out, kStx);
    out.push_back(kCtrlNeedAck);
    putU16(out, static_cast<unsigned>(data.size()) & 0xFFFF);
    putU16(out, seq);
    out.push_back(cmdId);
    out.insert(out.end(), data.begin(), data.end());
    putU16(out, crc16(out));
    return out;
}

std::optional<Frame> parseFrame(const Bytes &payload)
{
    if (payload.size() < 10 || getU16(payload, 0) != kStx) {
        return std::nullopt;
    }
    const size_t end = 8 + getU16(payload, 3);
    if (payload.size() < end + 2) {
        return std::nullopt;
    }
    const Bytes body(payload.begin(), payload.begin() + static_cast<long>(end));
    if (crc16(body) != getU16(payload, end)) {
        return std::nullopt;
    }
    Frame frame;
    frame.ctrl = payload[2];
    frame.seq = static_cast<std::uint16_t>(getU16(payload, 5));
    frame.cmdId = payload[7];
    frame.data.assign(body.begin() + 8, body.end());
    return frame;
}

std::optional<Attitude> decodeAttitude(const Bytes &data)
{
    if (data.size() < 6) {
        return std::nullopt;
    }
    Attitude out;
    out.yawDeg = getI16(data, 0) / 10.0;
    out.pitchDeg = getI16(data, 2) / 10.0;
    out.rollDeg = getI16(data, 4) / 10.0;
    return out;
}

Bytes encodeAngle(double yawDeg, double pitchDeg)
{
    Bytes out;
    if (!putRoundedI16(out, yawDeg * 10.0) || !putRoundedI16(out, pitchDeg * 10.0)) {
        return {};
    }
    return out;
}

Bytes encodeRotationSpeed(double yawRate, double pitchRate)
{
    const double yaw = yawRate * 10.0;
    const double pitch = pitchRate * 10.0;
    if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
        return {};  // Python's round() raises on NaN and infinity
    }
    // Clamped before rounding: the same result, as -100 and 100 are whole numbers.
    auto axis = [](double v) { return i8(pyRound(v < -100.0 ? -100.0 : (v > 100.0 ? 100.0 : v))); };
    return {axis(yaw), axis(pitch)};
}

Bytes encodeManualZoom(int direction)
{
    return {sign(direction)};
}

Bytes encodeManualFocus(int direction)
{
    return {sign(direction)};
}

Bytes encodeAutoFocus(int touchX, int touchY)
{
    Bytes out{1};
    putU16(out, static_cast<unsigned>(touchX) & 0xFFFF);
    putU16(out, static_cast<unsigned>(touchY) & 0xFFFF);
    return out;
}

} // namespace siyi
