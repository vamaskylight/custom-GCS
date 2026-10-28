// Port of the camera rules in vgcs/viewpro/adapter.py (ViewproGimbalTcpAdapter) and
// vgcs/video/camera_control.py (ViewproCameraControl). Keep them in step: the tests
// in apk/custom/test replay scripts that ran through the Python code.

#include "ViewproCamera.h"

#include <algorithm>
#include <cctype>

namespace viewpro {

namespace {

constexpr size_t kMaxPending = 4096;  // more than this without a frame is noise

GimbalCameraCommand servo(int mode, long long p1 = 0, long long p2 = 0)
{
    GimbalCameraCommand c;
    c.servo = mode;
    c.servoP1 = p1;
    c.servoP2 = p2;
    return c;
}

GimbalCameraCommand optics(int op, int speed = 0)
{
    GimbalCameraCommand c;
    c.c1Op = op;
    c.c1ZoomSpeed = speed;
    return c;
}

std::string lowerTrimmed(const std::string &text)
{
    size_t from = 0;
    size_t to = text.size();
    while (from < to && std::isspace(static_cast<unsigned char>(text[from]))) {
        ++from;
    }
    while (to > from && std::isspace(static_cast<unsigned char>(text[to - 1]))) {
        --to;
    }
    std::string out = text.substr(from, to - from);
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

/// What starts at buf[i]: 0 nothing, 1 a TCP envelope (EB 90), 2 a bare serial
/// frame (55 AA DC), 3 maybe (the bytes end inside a header).
int frameStartAt(const Bytes &buf, size_t i)
{
    const size_t left = buf.size() - i;
    if (buf[i] == 0xEB) {
        return left < 2 ? 3 : (buf[i + 1] == 0x90 ? 1 : 0);
    }
    if (buf[i] == 0x55) {
        if (left < 2) {
            return 3;
        }
        if (buf[i + 1] != 0xAA) {
            return 0;
        }
        if (left < 3) {
            return 3;
        }
        return buf[i + 2] == 0xDC ? 2 : 0;
    }
    return 0;
}

} // namespace

Bytes Camera::gimbalSpeed(double yawDps, double pitchDps, long long nowMs)
{
    m_holdAt.reset();
    const long long yawRaw = speedDpsToRaw(yawDps);
    const long long pitchRaw = speedDpsToRaw(pitchDps);
    const Bytes out = encodeGimbalCameraCommand(servo(kServoManualSpeed, yawRaw, pitchRaw));
    if (yawRaw == 0 && pitchRaw == 0) {
        scheduleHold(nowMs);
    }
    return out;
}

Bytes Camera::ptz(const std::string &action, long long nowMs)
{
    m_holdAt.reset();  // any move of the operator's replaces a hold not yet sent
    const std::string a = lowerTrimmed(action);
    const long long raw = speedDpsToRaw(kPtzDps);
    if (a == "up" || a == "pitch_up") {
        return encodeGimbalCameraCommand(servo(kServoManualSpeed, 0, raw));
    }
    if (a == "down" || a == "pitch_down") {
        return encodeGimbalCameraCommand(servo(kServoManualSpeed, 0, -raw));
    }
    if (a == "left" || a == "yaw_left") {
        return encodeGimbalCameraCommand(servo(kServoManualSpeed, -raw, 0));
    }
    if (a == "right" || a == "yaw_right") {
        return encodeGimbalCameraCommand(servo(kServoManualSpeed, raw, 0));
    }
    if (a == "stop") {
        const Bytes out = encodeGimbalCameraCommand(servo(kServoManualSpeed, 0, 0));
        scheduleHold(nowMs);
        return out;
    }
    if (a == "center" || a == "home") {
        return encodeGimbalCameraCommand(servo(kServoHomePosition));
    }
    return {};
}

Bytes Camera::center()
{
    m_holdAt.reset();
    return encodeGimbalCameraCommand(servo(kServoHomePosition));
}

Bytes Camera::lookDown()
{
    m_holdAt.reset();
    return encodeGimbalCameraCommand(servo(kServoLookDown));
}

Bytes Camera::setAngle(double yawDeg, double pitchDeg)
{
    m_holdAt.reset();
    return encodeGimbalCameraCommand(
        servo(kServoManualAbsoluteAngle, angleDegToRaw(yawDeg), angleDegToRaw(pitchDeg)));
}

Bytes Camera::zoomStep(int direction, long long nowMs)
{
    m_zoomStopAt.reset();
    if (direction > 0) {
        m_zoomStopAt = nowMs + kZoomPulseMs;
        return encodeGimbalCameraCommand(optics(kOpZoomIn, kZoomSpeed));
    }
    if (direction < 0) {
        m_zoomStopAt = nowMs + kZoomPulseMs;
        return encodeGimbalCameraCommand(optics(kOpZoomOut, kZoomSpeed));
    }
    return encodeGimbalCameraCommand(optics(kOpStop));
}

Bytes Camera::focusStep(int direction)
{
    if (direction > 0) {
        return encodeGimbalCameraCommand(optics(kOpFocusPlus, kFocusSpeed));
    }
    if (direction < 0) {
        return encodeGimbalCameraCommand(optics(kOpFocusMinus, kFocusSpeed));
    }
    return encodeGimbalCameraCommand(optics(kOpStop));
}

Bytes Camera::autoFocus()
{
    return encodeGimbalCameraCommand(optics(kOpAutoFocus));
}

Bytes Camera::photo()
{
    return encodeGimbalCameraCommand(optics(kOpTakePicture));
}

Bytes Camera::setRecording(bool on)
{
    if (m_recording == on) {
        return {};
    }
    return toggleRecording();
}

Bytes Camera::toggleRecording()
{
    const Bytes out = encodeGimbalCameraCommand(optics(m_recording ? kOpStopRecord : kOpStartRecord));
    m_recording = !m_recording;
    return out;
}

Bytes Camera::setSensor(bool ir)
{
    GimbalCameraCommand c;
    c.c1Sensor = ir ? kSensorIr : kSensorEo1;
    m_sensorIr = ir;
    return encodeGimbalCameraCommand(c);
}

Bytes Camera::setLaser(bool on)
{
    GimbalCameraCommand c;
    c.c1Lrf = on ? kLrfContinuousStart : kLrfStop;
    m_laserOn = on;
    return encodeGimbalCameraCommand(c);
}

Bytes Camera::poll() const
{
    return encodeHeartbeat();
}

std::vector<Bytes> Camera::tick(long long nowMs)
{
    std::vector<Bytes> out;
    if (m_zoomStopAt && nowMs >= *m_zoomStopAt) {
        m_zoomStopAt.reset();
        out.push_back(encodeGimbalCameraCommand(optics(kOpStop)));
    }
    if (m_holdAt && nowMs >= *m_holdAt) {
        m_holdAt.reset();
        // Asked again when it is due: the camera's tracker may have taken the gimbal since.
        if (!cameraOwnsGimbal(nowMs)) {
            // Move by 0 from where it is: a position to hold, with no angle that could be wrong.
            out.push_back(encodeGimbalCameraCommand(servo(kServoManualRelativeAngle)));
        }
    }
    return out;
}

bool Camera::feed(const Bytes &received, long long nowMs)
{
    m_rx.insert(m_rx.end(), received.begin(), received.end());
    bool found = false;
    size_t i = 0;
    while (i < m_rx.size()) {
        const int kind = frameStartAt(m_rx, i);
        if (kind == 0) {
            ++i;
            continue;
        }
        if (kind == 3) {
            break;  // the rest of the header has not come yet
        }
        const Bytes rest(m_rx.begin() + static_cast<long>(i), m_rx.end());
        if (rest.size() < 4) {
            break;
        }
        Bytes frame;
        size_t used = 0;
        if (kind == 1) {
            used = 3 + static_cast<size_t>(rest[2]) + 1;
            if (rest.size() < used) {
                break;
            }
            const std::optional<Bytes> inner = unwrapTcp(rest);
            if (!inner) {
                ++i;  // not a good envelope after all: look further
                continue;
            }
            frame = *inner;
        } else {
            const size_t bodyLen = rest[3] & 0x3F;
            if (bodyLen < 3) {
                ++i;
                continue;
            }
            used = 3 + bodyLen;
            if (rest.size() < used) {
                break;
            }
            frame.assign(rest.begin(), rest.begin() + static_cast<long>(used));
        }
        const auto parsed = parseSerialFrame(frame);
        if (!parsed) {
            ++i;
            continue;
        }
        if (parsed->first == kCmdStatus) {
            if (const std::optional<Status> status = decodeStatusFrame(parsed->second)) {
                takeStatus(*status, nowMs);
                found = true;
            }
        }
        i += used;
    }
    m_rx.erase(m_rx.begin(), m_rx.begin() + static_cast<long>(i));
    if (m_rx.size() > kMaxPending) {
        m_rx.clear();
    }
    return found;
}

bool Camera::cameraOwnsGimbal(long long nowMs) const
{
    return m_servoMode == kServoModeTracking && m_servoModeMs >= 0 && nowMs - m_servoModeMs <= kServoModeStaleMs;
}

void Camera::scheduleHold(long long nowMs)
{
    if (cameraOwnsGimbal(nowMs)) {
        return;  // the camera's tracker drives the gimbal: a hold would pull it off the target
    }
    m_holdAt = nowMs + kPostJogHoldMs;
}

void Camera::takeStatus(const Status &status, long long nowMs)
{
    m_status = status;
    m_statusMs = nowMs;
    m_servoMode = status.attitude.servoStatus;
    m_servoModeMs = nowMs;
    if (!status.camera) {
        return;
    }
    const CameraStatus &cam = *status.camera;
    m_recording = cam.recordStatus == 1;
    // Only a real measurement: 0 means the laser has measured nothing.
    if (cam.rangeM) {
        m_rangeM = cam.rangeM;
        m_rangeMs = nowMs;
    }
    // 0 means "not reported", and a zero field of view would break the aim maths.
    if (cam.hfovDeg > 0.0 && cam.vfovDeg > 0.0) {
        m_hfovDeg = cam.hfovDeg;
        m_vfovDeg = cam.vfovDeg;
    }
    if (cam.zoomX > 0.0) {
        m_zoomX = cam.zoomX;
    }
}

} // namespace viewpro
