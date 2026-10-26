// Port of the camera rules in vgcs/siyi/adapter.py (SiyiGimbalUdpAdapter) and
// vgcs/video/camera_control.py (SiyiCameraControl). Keep them in step: the tests in
// apk/custom/test replay scripts that ran through the Python code.

#include "SiyiCamera.h"

#include <algorithm>
#include <cctype>

namespace siyi {

namespace {

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

} // namespace

Bytes Camera::request(std::uint8_t cmd, const Bytes &data)
{
    const Bytes out = buildRequest(cmd, data, m_seq);
    m_seq = static_cast<std::uint16_t>((m_seq + 1) & 0xFFFF);
    return out;
}

void Camera::later(long long at, Later what, std::uint8_t cmd, const Bytes &data)
{
    m_due.push_back(Due{at, m_order++, what, cmd, data});
}

void Camera::drop(Later what)
{
    m_due.erase(std::remove_if(m_due.begin(), m_due.end(), [what](const Due &d) { return d.what == what; }),
                m_due.end());
}

std::vector<Bytes> Camera::gimbalSpeed(double yawDps, double pitchDps)
{
    const Bytes data = encodeRotationSpeed(yawDps, pitchDps);
    if (data.empty()) {
        return {};  // NaN or infinity: VGCS raises before it sends anything
    }
    return {request(kCmdGimbalRotation, data)};
}

std::vector<Bytes> Camera::ptz(const std::string &action)
{
    const std::string a = lowerTrimmed(action);
    if (a == "up" || a == "pitch_up") {
        return gimbalSpeed(0.0, kPtzDps);
    }
    if (a == "down" || a == "pitch_down") {
        return gimbalSpeed(0.0, -kPtzDps);
    }
    if (a == "left" || a == "yaw_left") {
        return gimbalSpeed(-kPtzDps, 0.0);
    }
    if (a == "right" || a == "yaw_right") {
        return gimbalSpeed(kPtzDps, 0.0);
    }
    if (a == "stop") {
        return gimbalSpeed(0.0, 0.0);
    }
    if (a == "center" || a == "home") {
        return center();
    }
    return {};
}

std::vector<Bytes> Camera::center()
{
    std::vector<Bytes> out = setAngle(0.0, 0.0);
    const std::vector<Bytes> stop = gimbalSpeed(0.0, 0.0);
    out.insert(out.end(), stop.begin(), stop.end());
    return out;
}

std::vector<Bytes> Camera::pointDown(double nadirPitchDeg)
{
    return setAngle(m_attitude ? m_attitude->yawDeg : 0.0, nadirPitchDeg);
}

std::vector<Bytes> Camera::setAngle(double yawDeg, double pitchDeg)
{
    const Bytes data = encodeAngle(yawDeg, pitchDeg);
    if (data.empty()) {
        return {};  // does not fit: VGCS raises before it sends anything
    }
    return {request(kCmdGimbalAngle, data)};
}

std::vector<Bytes> Camera::zoomStep(int direction, long long nowMs)
{
    drop(Later::ZoomStop);
    const Bytes data = encodeManualZoom(direction);
    if (direction != 0) {
        later(nowMs + kZoomPulseMs, Later::ZoomStop, kCmdManualZoom, encodeManualZoom(0));
    }
    return {request(kCmdManualZoom, data)};
}

std::vector<Bytes> Camera::focusStep(int direction, long long nowMs)
{
    if (direction == 0) {
        return {request(kCmdManualFocus, encodeManualFocus(0))};
    }
    const Bytes press = encodeManualFocus(direction);
    std::vector<Bytes> out{request(kCmdManualFocus, press)};
    drop(Later::FocusStop);
    later(nowMs + kFocusResendMs, Later::FocusAgain, kCmdManualFocus, press);
    later(nowMs + kFocusStopMs, Later::FocusStop, kCmdManualFocus, encodeManualFocus(0));
    return out;
}

std::vector<Bytes> Camera::autoFocus(int x, int y)
{
    return {request(kCmdAutoFocus, encodeAutoFocus(x, y))};
}

std::vector<Bytes> Camera::photo()
{
    return {request(kCmdPhotoRecord, Bytes{kTakePhoto})};
}

std::vector<Bytes> Camera::toggleRecording()
{
    return {request(kCmdPhotoRecord, Bytes{kRecordToggle})};
}

Bytes Camera::poll()
{
    return request(kCmdGimbalAttitude, {});
}

std::vector<Bytes> Camera::tick(long long nowMs)
{
    std::vector<Due> due;
    for (auto it = m_due.begin(); it != m_due.end();) {
        if (it->at <= nowMs) {
            due.push_back(*it);
            it = m_due.erase(it);
        } else {
            ++it;
        }
    }
    std::sort(due.begin(), due.end(),
              [](const Due &a, const Due &b) { return a.at != b.at ? a.at < b.at : a.order < b.order; });
    std::vector<Bytes> out;
    for (const Due &d : due) {
        out.push_back(request(d.cmd, d.data));  // the sequence number of the moment it is sent
    }
    return out;
}

bool Camera::feed(const Bytes &datagram, long long nowMs)
{
    const std::optional<Frame> frame = parseFrame(datagram);
    if (!frame || frame->cmdId != kCmdGimbalAttitude) {
        return false;
    }
    const std::optional<Attitude> attitude = decodeAttitude(frame->data);
    if (!attitude) {
        return false;
    }
    m_attitude = attitude;
    m_attitudeMs = nowMs;
    return true;
}

} // namespace siyi
