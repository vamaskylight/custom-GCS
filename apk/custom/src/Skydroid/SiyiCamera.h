#pragma once

// SIYI camera logic (ZR10 and others, task AK): what VGCS's SiyiGimbalUdpAdapter and
// SiyiCameraControl do (vgcs/siyi/adapter.py, vgcs/video/camera_control.py), without
// sockets or timers, so the app's camera link can use it and host tests can check it.
//
// Each command returns the frames to send now (empty: nothing to send). A focus step
// and a zoom click leave frames to send later: tick() returns them when they are due.
// poll() is the attitude request, to send twice a second. Datagrams from the camera
// go to feed(). Times are milliseconds of a steady clock. Every request carries the
// next sequence number, as in VGCS.

#include "SiyiProtocol.h"

#include <optional>
#include <string>
#include <vector>

namespace siyi {

class Camera
{
public:
    static constexpr long long kPollMs = 500;         // VGCS asks for the attitude twice a second
    static constexpr long long kFocusResendMs = 70;   // a focus step: the "press" once more (UDP may lose one)
    static constexpr long long kFocusStopMs = 240;    // then the "release"
    // The app's own: VGCS's zoom buttons send no SIYI zoom. The camera zooms until it is
    // told to stop, so a click is a start and a stop this much later (not field-calibrated).
    static constexpr long long kZoomPulseMs = 150;
    static constexpr double kPtzDps = 5.0;            // the speed of the ptz() buttons
    static constexpr double kNadirPitchDeg = -90.0;   // VGCS's default for "point down"
    static constexpr int kAutoFocusX = 480;           // where VGCS asks for autofocus
    static constexpr int kAutoFocusY = 270;

    /// The gimbal at this speed in deg/s (about 10 units per deg/s, at most 100):
    /// pitch positive tilts UP, SIYI's own way.
    std::vector<Bytes> gimbalSpeed(double yawDps, double pitchDps);
    /// "up", "down", "left" or "right" at kPtzDps, "stop", "center" or "home". Nothing for anything else.
    std::vector<Bytes> ptz(const std::string &action);
    /// Angles 0 and 0, then speed 0.
    std::vector<Bytes> center();
    /// Straight down, keeping the yaw the camera last reported (0 before any).
    std::vector<Bytes> pointDown(double nadirPitchDeg = kNadirPitchDeg);
    /// Absolute angles. Nothing when an angle does not fit the frame.
    std::vector<Bytes> setAngle(double yawDeg, double pitchDeg);

    /// One zoom click: in (> 0) or out (< 0), the stop from tick(). 0 stops now.
    std::vector<Bytes> zoomStep(int direction, long long nowMs);
    /// Focus far (> 0) or near (< 0): the press now, once more and the release from tick().
    /// 0 sends the release now (as in VGCS, steps already given still finish).
    std::vector<Bytes> focusStep(int direction, long long nowMs);
    std::vector<Bytes> autoFocus(int x = kAutoFocusX, int y = kAutoFocusY);

    std::vector<Bytes> photo();
    /// SIYI has a toggle only, and does not say whether it records.
    std::vector<Bytes> toggleRecording();

    /// The attitude request.
    Bytes poll();
    /// Frames due by nowMs, in their order: the zoom stop, the focus press and release.
    std::vector<Bytes> tick(long long nowMs);
    /// A datagram from the camera. True when it held the attitude.
    bool feed(const Bytes &datagram, long long nowMs);

    const std::optional<Attitude> &attitude() const { return m_attitude; }
    long long attitudeMs() const { return m_attitudeMs; }

private:
    enum class Later
    {
        ZoomStop,     // replaced by the next zoom click
        FocusAgain,   // never taken back, as in VGCS
        FocusStop,    // replaced by the next focus step
    };

    struct Due
    {
        long long at;
        int order;  // the order they were asked in, for the same time
        Later what;
        std::uint8_t cmd;
        Bytes data;
    };

    Bytes request(std::uint8_t cmd, const Bytes &data);
    void later(long long at, Later what, std::uint8_t cmd, const Bytes &data);
    void drop(Later what);

    std::uint16_t m_seq = 0;
    int m_order = 0;
    std::vector<Due> m_due;
    std::optional<Attitude> m_attitude;
    long long m_attitudeMs = -1;
};

} // namespace siyi
