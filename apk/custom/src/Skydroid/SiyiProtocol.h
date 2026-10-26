#pragma once

// SIYI gimbal camera SDK (ZR10 and others): binary frames over UDP port 37260.
//
// A line-by-line port of vgcs/siyi/protocol.py, so the APK sends exactly the
// bytes VGCS sends (task AK). Plain C++17 with no Qt, so it can be unit tested
// on any compiler (see apk/custom/test/siyi_protocol_test.cc).
//
// Encoders return an empty vector where the Python code raises (a value that
// does not fit its field).

#include <cstdint>
#include <optional>
#include <vector>

namespace siyi {

using Bytes = std::vector<std::uint8_t>;

constexpr std::uint16_t kStx = 0x6655;
constexpr std::uint8_t kCtrlNeedAck = 0x01;
constexpr std::uint8_t kCtrlIsAck = 0x02;
constexpr int kDefaultPort = 37260;

constexpr std::uint8_t kCmdAutoFocus = 0x04;      // one-shot autofocus (ZR10, ZT6, ZR30, ZT30)
constexpr std::uint8_t kCmdManualZoom = 0x05;
constexpr std::uint8_t kCmdManualFocus = 0x06;
constexpr std::uint8_t kCmdGimbalRotation = 0x07;
constexpr std::uint8_t kCmdPhotoRecord = 0x0C;
constexpr std::uint8_t kCmdGimbalAttitude = 0x0D;
constexpr std::uint8_t kCmdGimbalAngle = 0x0E;

// The data byte of kCmdPhotoRecord, as VGCS sends it.
constexpr std::uint8_t kTakePhoto = 0;
constexpr std::uint8_t kRecordToggle = 2;

/// CRC16 of the SIYI manual (G(X) = X^16 + X^12 + X^5 + 1), table driven.
std::uint16_t crc16(const Bytes &data, std::uint16_t init = 0);

/// A request that asks for an answer: STX, ctrl 0x01, data length, seq, command,
/// data, CRC16 (all little-endian).
Bytes buildRequest(std::uint8_t cmdId, const Bytes &data = {}, std::uint16_t seq = 0);

struct Frame
{
    std::uint8_t ctrl = 0;
    std::uint16_t seq = 0;
    std::uint8_t cmdId = 0;
    Bytes data;
};

/// The frame at the start of payload, or nothing (too short, another start, a wrong CRC).
std::optional<Frame> parseFrame(const Bytes &payload);

struct Attitude
{
    double yawDeg = 0.0;
    double pitchDeg = 0.0;
    double rollDeg = 0.0;
};

/// The answer to kCmdGimbalAttitude: int16 yaw, pitch and roll in 0.1 degree.
std::optional<Attitude> decodeAttitude(const Bytes &data);

/// kCmdGimbalAngle: the target yaw and pitch as int16 in 0.1 degree.
Bytes encodeAngle(double yawDeg, double pitchDeg);
/// kCmdGimbalRotation: int8 yaw and pitch speed, -100 to 100 (about deg/s x 10).
Bytes encodeRotationSpeed(double yawRate, double pitchRate);
/// kCmdManualZoom: 1 zoom in, -1 zoom out, 0 stop.
Bytes encodeManualZoom(int direction);
/// kCmdManualFocus: 1 far, -1 near, 0 stop.
Bytes encodeManualFocus(int direction);
/// kCmdAutoFocus at a point of the stream picture, in pixels.
Bytes encodeAutoFocus(int touchX = 0, int touchY = 0);

} // namespace siyi
