#pragma once

// Skydroid TOP camera protocol (#TP / #tp frames over UDP port 5000).
//
// A line-by-line port of vgcs/skydroid/protocol.py, so the APK sends exactly
// the bytes VGCS sends. Plain C++17 with no Qt, so it can be unit tested on
// any compiler (see apk/custom/test).
//
// Builders return an empty string where the Python code raises ValueError or
// returns None.

#include <optional>
#include <string>
#include <vector>

namespace skydroid::top {

// Per-camera choices that the Python code keeps as module state.
struct Options
{
    // TOP V1.2.0 wants "#TP" on every gimbal (G-class) frame. C13 firmware
    // accepts the older lower-case "#tp" on variable-length gimbal frames,
    // so this is only on for cameras whose profile asks for it (C14 Pro).
    bool gClassUpperHeader = false;
    // Longest laser reading accepted, in decimeters. 0x2710 = 1000 m.
    int slrMaxDm = 0x2710;
};

constexpr int kSlrDmMin = 0x0032;      // 5 m
constexpr int kSlrDmMaxDefault = 0x2710; // 1000 m
constexpr int kLrfFrameW = 1280;
constexpr int kLrfFrameH = 720;

/// Python round(): halves go to the even neighbour (2.5 -> 2, 3.5 -> 4).
long long pyRound(double value);

/// ASCII sum of every byte before the checksum, as two upper-case hex digits.
std::string checksum(const std::string &body);

/// One TOP frame. variable: -1 = automatic (data longer than 2 chars), 0 = fixed "#TP" frame, 1 = variable frame.
std::string buildTpFrame(char dest, char control, const std::string &tag, const std::string &data,
                         char src = 'U', int variable = -1, const Options &options = {});

// --- Field encoders --------------------------------------------------------
std::string encodeSpeed2(double degPerSecond);    // signed byte, 0.5 deg/s units
std::string encodeAngle4(double degrees);          // int16, 0.01 deg units
std::string encodeAttitudeField4(double degrees);  // int16, 0.01 deg units
std::optional<double> decodeAttitudeField4(const std::string &field);

// --- Gimbal ------------------------------------------------------------------
std::string buildGaaEnable(int hz = 5, const Options &options = {});
std::string buildGacQuery(const Options &options = {});
/// stop, up, down, left, right, center, nadir (down_once, point_down). Empty for anything else.
std::string buildPtz(const std::string &action, const Options &options = {});
std::string buildGimbalSpeed(double yawDegPerSecond, double pitchDegPerSecond, const Options &options = {});
/// GSY, GSP or GSR: one axis speed.
std::string buildGimbalSpeedAxis(const std::string &tag, double degPerSecond, const Options &options = {});
/// GAY, GAP or GAR: one axis angle.
std::string buildGimbalAngleAxis(const std::string &axisTag, double degrees, double speed = 16.0,
                                 const Options &options = {});
/// GAM: yaw and pitch angles together.
std::string buildGimbalAngles(double yawDeg, double pitchDeg, double yawSpeed, double pitchSpeed,
                              const Options &options = {});

// --- Camera and system ---------------------------------------------------
std::string buildSystemCommand(const std::string &tag, const std::string &data, bool write = true,
                               const Options &options = {});
std::string buildPhoto(const Options &options = {});   // CAP 01
std::string buildRecord(const Options &options = {});  // REC 01

// --- Laser rangefinder (SLR) -------------------------------------------------
std::string buildSlrQuery(char dest = 'D', const Options &options = {});
std::string buildSlrTrigger(char dest = 'D', const Options &options = {});
/// The decimeter ceiling for a camera's laser range in metres (mirrors set_slr_max_range_m).
int slrMaxDmForRange(std::optional<double> maxMetres);
std::optional<int> decodeSlrDecimeters(const std::string &dataField, const Options &options = {});
std::optional<double> decodeSlrDistanceM(const std::string &dataField, const Options &options = {});

// --- Tracking ------------------------------------------------------------------
std::string buildGotTarget(int xPx, int yPx, int frameW = kLrfFrameW, int frameH = kLrfFrameH,
                           const Options &options = {});
std::string buildSumTrack(bool confirm, const Options &options = {});

// --- Zoom and focus ------------------------------------------------------------
std::string buildPodCameraCommand(const std::string &tag, const std::string &data, char dest = 'M',
                                  bool write = true, const Options &options = {});
std::string buildDzmAbsoluteZoom(double zoomX, int cameraX0 = 0, const Options &options = {});
std::string buildDzmAbsoluteZoomUd(double zoomX, int cameraX0 = 0, const Options &options = {});
std::string buildMulOpticalZoom(double zoomX, const Options &options = {});
std::vector<std::string> buildOpticalZoomFrames(double zoomX, const Options &options = {});
/// in, out, tele, wide, +, -
std::string buildDzmZoomStepV47(const std::string &action, const Options &options = {});
std::string buildDzmQuery(const Options &options = {});
/// long, tele, short, wide
std::string buildDzmLensSelect(const std::string &lens, const Options &options = {});
/// 1..4
std::string buildDzmPreset(int multiplier, const Options &options = {});
std::vector<std::string> buildDzmStepFrames(int direction, const Options &options = {});
std::vector<std::string> buildDzmHomeFrames(const Options &options = {});
std::optional<int> decodeDzmStep(const std::string &dataField);
std::vector<std::string> buildC13ZoomStepFrames(int direction, const Options &options = {});
/// stop, in, out, tele, wide
std::string buildDzmStepZoom(const std::string &action, const Options &options = {});
/// stop, out, in, wide, tele
std::string buildZmcZoom(const std::string &action, const Options &options = {});
/// stop, near, far, in, out, auto
std::string buildFccFocus(const std::string &action, const Options &options = {});

// --- Replies -----------------------------------------------------------------
struct DecodedFrame
{
    std::string address; // two letters, source then destination, upper case (e.g. "EU")
    std::string tag;   // upper case
    char ctrl = 'r';   // 'w' or 'r'
    std::string data;
    std::string raw;
    std::optional<double> yaw;     // GAC
    std::optional<double> pitch;   // GAC
    std::optional<double> roll;    // GAC
    std::optional<int> slrDm;      // SLR
    std::optional<int> dzmStep;    // DZM read reply only
};

/// Parses a #TP / #tp reply. Like the Python code, it does not check the checksum.
std::optional<DecodedFrame> parseTpFrame(const std::string &raw, const Options &options = {});

} // namespace skydroid::top
