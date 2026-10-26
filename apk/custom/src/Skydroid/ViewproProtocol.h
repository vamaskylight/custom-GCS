#pragma once

// Viewpro ViewLink gimbal protocol (A20KTR and others): serial frames in a TCP
// envelope, TCP port 2000.
//
// A line-by-line port of vgcs/viewpro/protocol.py, so the APK sends exactly
// the bytes VGCS sends (task AK). Plain C++17 with no Qt, so it can be unit
// tested on any compiler (see apk/custom/test/viewpro_protocol_test.cc).
// The camera's own tracker (E1/E2 frames) is left out on purpose.
//
// Serial frame: 55 AA DC | length (6 bits) + frame counter (2 bits) | command |
// data (big-endian) | XOR checksum. TCP envelope: EB 90 | inner length |
// inner frame | 8-bit sum of the inner frame.

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace viewpro {

using Bytes = std::vector<std::uint8_t>;

constexpr int kDefaultPort = 2000;

constexpr std::uint8_t kCmdA1C1E1 = 0x30;      // gimbal (A1, 9 bytes) + optics (C1, 2) + tracking (E1, 3)
constexpr std::uint8_t kCmdHeartbeat = 0x10;   // the camera answers with kCmdStatus
constexpr std::uint8_t kCmdStatus = 0x40;      // T1 (22) + F1 (1) + B1 (6) + D1 (12)

// A1 gimbal modes. Yaw positive is right, pitch positive is down.
constexpr std::uint8_t kServoMotorOnOff = 0x00;
constexpr std::uint8_t kServoManualSpeed = 0x01;          // p1/p2: yaw/pitch speed, 0.01 deg/s
constexpr std::uint8_t kServoHomePosition = 0x04;
constexpr std::uint8_t kServoManualRelativeAngle = 0x09;  // p1/p3 speed, p2/p4 angle from where it is
constexpr std::uint8_t kServoManualAbsoluteAngle = 0x0B;  // home is 0; not for fast repeats
constexpr std::uint8_t kServoNoChange = 0x0F;             // with an optics-only command
constexpr std::uint8_t kServoLookDown = 0x12;

// C1 optics: lrf (3 bits) | operation (7 bits) | zoom speed (3 bits) | sensor (3 bits).
constexpr int kOpStop = 0x01;
constexpr int kOpZoomOut = 0x08;   // FOV+
constexpr int kOpZoomIn = 0x09;    // FOV-
constexpr int kOpFocusPlus = 0x0A;
constexpr int kOpFocusMinus = 0x0B;
constexpr int kOpTakePicture = 0x13;
constexpr int kOpStartRecord = 0x14;
constexpr int kOpStopRecord = 0x15;
constexpr int kOpAutoFocus = 0x19;
constexpr int kOpManualFocusMode = 0x1A;

constexpr int kSensorNoAction = 0x00;
constexpr int kSensorEo1 = 0x01;
constexpr int kSensorIr = 0x02;
constexpr int kSensorEo1IrPip = 0x03;
constexpr int kSensorIrEo1Pip = 0x04;

constexpr int kLrfNone = 0x00;
constexpr int kLrfSingle = 0x01;
constexpr int kLrfContinuousStart = 0x02;
constexpr int kLrfStop = 0x05;

/// A serial frame with frame counter 0.
Bytes buildSerialFrame(std::uint8_t cmdId, const Bytes &data = {});
/// (command, data) of the serial frame at the very start of payload, or nothing.
std::optional<std::pair<std::uint8_t, Bytes>> parseSerialFrame(const Bytes &payload);

Bytes wrapTcp(const Bytes &innerFrame);
/// The inner frame of the TCP envelope at the start of payload, or nothing.
std::optional<Bytes> unwrapTcp(const Bytes &payload);

/// A1: the mode and four signed 16-bit values (clamped), big-endian.
Bytes encodeA1(int servo, long long p1 = 0, long long p2 = 0, long long p3 = 0, long long p4 = 0);
/// C1: the two optics bytes.
Bytes encodeC1(int op = 0, int zoomSpeed = 0, int sensor = 0, int lrf = 0);

struct GimbalCameraCommand
{
    int servo = kServoNoChange;
    long long servoP1 = 0;
    long long servoP2 = 0;
    long long servoP3 = 0;
    long long servoP4 = 0;
    int c1Op = 0;
    int c1ZoomSpeed = 0;
    int c1Sensor = 0;
    int c1Lrf = 0;
};

/// The ready-to-send TCP bytes of the A1+C1+E1 frame (E1 always "no action").
Bytes encodeGimbalCameraCommand(const GimbalCameraCommand &command);
/// The ready-to-send TCP bytes of a heartbeat.
Bytes encodeHeartbeat();

/// Degrees to the 16-bit angle field (360/65536 degree), wrapped into -180..180.
long long angleDegToRaw(double degrees);
double angleRawToDeg(long long raw);
/// deg/s to the 16-bit speed field (0.01 deg/s).
long long speedDpsToRaw(double degreesPerSecond);

struct Attitude
{
    double yawDeg = 0.0;
    double pitchDeg = 0.0;
    double rollDeg = 0.0;
    int servoStatus = 0;  // the gimbal mode it is in (top 4 bits of B1)
};

/// B1, 6 bytes.
std::optional<Attitude> decodeB1(const Bytes &data);

struct CameraStatus
{
    int recordStatus = 0;
    std::optional<double> rangeM;  // the laser range, nothing when 0
    double vfovDeg = 0.0;
    double hfovDeg = 0.0;
    double zoomX = 0.0;
};

/// D1, 12 bytes.
std::optional<CameraStatus> decodeD1(const Bytes &data);

struct Tracking
{
    int source = 0;
    int status = 0;  // 0 stopped, 1 searching, 2 tracking, 3 lost
    int targetType = 0;
};

/// F1, 1 byte.
Tracking decodeF1(std::uint8_t byte);

struct Status
{
    Attitude attitude;
    Tracking tracking;
    std::optional<CameraStatus> camera;  // when the frame is long enough for D1
};

/// The body of a kCmdStatus frame.
std::optional<Status> decodeStatusFrame(const Bytes &data);
/// The first good status frame anywhere in what a read returned.
std::optional<Status> findStatusFrame(const Bytes &payload);

} // namespace viewpro
