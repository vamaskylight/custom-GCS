#pragma once

// Viewpro camera logic (A20KTR and others, task AK): what VGCS's
// ViewproGimbalTcpAdapter and ViewproCameraControl do (vgcs/viewpro/adapter.py,
// vgcs/video/camera_control.py), without sockets or timers, so the app's camera
// link can use it and host tests can check it. The camera's own tracker is left
// out, as in VGCS.
//
// Each command returns the bytes to send now (empty: nothing to send). A zoom
// click and the end of a jog leave a frame to send later: tick() returns it when
// it is due. poll() is the status request, to send twice a second. What the camera
// sends goes to feed(), in any pieces. Times are milliseconds of a steady clock.

#include "ViewproProtocol.h"

#include <optional>
#include <string>
#include <vector>

namespace viewpro {

class Camera
{
public:
    static constexpr long long kPollMs = 500;          // VGCS asks for the status twice a second
    static constexpr long long kZoomPulseMs = 150;     // a zoom click: start, then stop (not field-calibrated)
    static constexpr long long kPostJogHoldMs = 350;   // after a jog stops: pin the gimbal where it came to rest
    static constexpr long long kServoModeStaleMs = 3000;
    static constexpr double kPtzDps = 10.0;            // the speed of the ptz() buttons
    static constexpr int kZoomSpeed = 7;               // 1 slowest to 7 fastest
    static constexpr int kFocusSpeed = 4;
    static constexpr int kServoModeTracking = 0x06;    // the camera's own tracker drives the gimbal

    /// The gimbal at this speed in deg/s: yaw positive turns right, pitch positive tilts UP
    /// (measured on a real unit in 2026-07; the vendor document's angle example says the
    /// opposite). Both 0 stops it, and kPostJogHoldMs later tick() pins it there: speed 0
    /// alone leaves it in a rate mode that drifts.
    Bytes gimbalSpeed(double yawDps, double pitchDps, long long nowMs);
    /// "up", "down", "left" or "right" at kPtzDps, "stop", "center" or "home". Empty for anything else.
    Bytes ptz(const std::string &action, long long nowMs);
    Bytes center();
    Bytes lookDown();
    /// Absolute angles, home position as 0. One shot, not for fast repeats.
    Bytes setAngle(double yawDeg, double pitchDeg);

    /// One zoom click: in (> 0) or out (< 0). The camera zooms until it is told to stop, so
    /// tick() sends the stop kZoomPulseMs later. 0 stops now.
    Bytes zoomStep(int direction, long long nowMs);
    /// Focus far (> 0) or near (< 0) until 0 stops it. Which way is which is not field-checked.
    Bytes focusStep(int direction);
    Bytes autoFocus();

    Bytes photo();
    /// Start or stop the camera's own recording. Empty when it already does what is asked
    /// (the camera says in its status whether it records).
    Bytes setRecording(bool on);
    Bytes toggleRecording();
    bool recording() const { return m_recording; }

    /// The thermal (true) or the day sensor in the one video stream. The camera's network
    /// is gone for seconds after a switch.
    Bytes setSensor(bool ir);
    bool sensorIsIr() const { return m_sensorIr; }

    /// The laser: continuous ranging (true), as VGCS ranges, or off. While it ranges, each status
    /// carries the last range it measured (rangeM()). VGCS's single shots came back with nothing
    /// in the field (2026-08-03), so there is none here. Off stops the emitter.
    Bytes setLaser(bool on);
    bool laserOn() const { return m_laserOn; }

    /// The status request (a heartbeat: the camera answers with its status).
    Bytes poll() const;
    /// Frames due by nowMs: the zoom stop, the hold after a jog.
    std::vector<Bytes> tick(long long nowMs);
    /// Bytes the camera sent. True when a whole status frame came with them.
    bool feed(const Bytes &received, long long nowMs);

    /// The last status, and when it came.
    const std::optional<Status> &status() const { return m_status; }
    long long statusMs() const { return m_statusMs; }
    /// The last laser range the camera measured, and when (nothing until it measured one).
    std::optional<double> rangeM() const { return m_rangeM; }
    long long rangeMs() const { return m_rangeMs; }
    /// The live field of view and zoom, once the camera reported them (not 0).
    std::optional<double> hfovDeg() const { return m_hfovDeg; }
    std::optional<double> vfovDeg() const { return m_vfovDeg; }
    std::optional<double> zoomX() const { return m_zoomX; }

private:
    bool cameraOwnsGimbal(long long nowMs) const;
    void scheduleHold(long long nowMs);
    void takeStatus(const Status &status, long long nowMs);

    std::optional<Status> m_status;
    long long m_statusMs = -1;
    int m_servoMode = -1;
    long long m_servoModeMs = -1;
    bool m_recording = false;
    bool m_sensorIr = false;
    bool m_laserOn = false;
    std::optional<double> m_rangeM;
    long long m_rangeMs = -1;
    std::optional<double> m_hfovDeg;
    std::optional<double> m_vfovDeg;
    std::optional<double> m_zoomX;
    std::optional<long long> m_zoomStopAt;
    std::optional<long long> m_holdAt;
    Bytes m_rx;  // what came that is not a whole frame yet
};

} // namespace viewpro
