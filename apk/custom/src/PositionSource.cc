#include "PositionSource.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <utility>

namespace vama {

namespace {

constexpr int k3dFix = 3;

/// Metres, or none: 0, less, or not a number (VGCS's _metres).
std::optional<double> metres(double value)
{
    if (std::isfinite(value) && value > 0.0) {
        return value;
    }
    return std::nullopt;
}

/// A setting's value as a whole number, as VGCS's int(float(value)): towards 0.
void readWhole(const std::map<std::string, double> &params, const char *name, std::optional<int> &out)
{
    const auto it = params.find(name);
    if (it != params.end() && std::isfinite(it->second) && std::fabs(it->second) < 2.0e9) {
        out = static_cast<int>(it->second);
    }
}

} // namespace

void PositionSource::reset()
{
    _gps1.reset();
    _gps2.reset();
    _seenGps2 = false;
    _armed = false;
    _gps2Type.reset();
    _autoSwitch.reset();
    _said.reset();
    _candidate.reset();
}

void PositionSource::updateGps1(int fixType, double nowS)
{
    _gps1 = Gps1Reading{fixType, nowS};
}

void PositionSource::updateGps2(int fixType, double hAccM, int satellites, double nowS)
{
    _gps2 = Gps2Reading{fixType, metres(hAccM), satellites, nowS};
    _seenGps2 = true;
}

void PositionSource::updateSettings(const std::map<std::string, double> &params)
{
    // GPS_TYPE2 is GPS2_TYPE's name before ArduPilot 4.6. With both, the second counts (as in VGCS).
    readWhole(params, "GPS2_TYPE", _gps2Type);
    readWhole(params, "GPS_TYPE2", _gps2Type);
    readWhole(params, "GPS_AUTO_SWITCH", _autoSwitch);
}

std::optional<int> PositionSource::_fresh1(double nowS) const
{
    if (_gps1 && nowS - _gps1->atS <= kStaleAfterS) {
        return _gps1->fix;
    }
    return std::nullopt;
}

std::optional<PositionSource::Gps2Reading> PositionSource::_fresh2(double nowS) const
{
    if (_gps2 && nowS - _gps2->atS <= kStaleAfterS) {
        return *_gps2;
    }
    return std::nullopt;
}

PositionSource::Source PositionSource::source(double nowS) const
{
    const std::optional<int> g1 = _fresh1(nowS);
    const std::optional<Gps2Reading> g2 = _fresh2(nowS);
    if (!g1 && !g2) {
        return Source::Unknown;
    }
    if (g1 && *g1 >= k3dFix) {
        return Source::Gps1;
    }
    if (g2 && g2->fix >= k3dFix) {
        return isCamera() ? Source::Camera : Source::SecondGps;
    }
    return Source::None;
}

std::string PositionSource::_gps2Name() const
{
    return isCamera() ? "the camera (GPS 2)" : "GPS 2";
}

std::string PositionSource::_gps2Words(const std::optional<Gps2Reading> &g2)
{
    if (!g2) {
        return "no report";
    }
    if (g2->fix < k3dFix) {
        return "no fix";
    }
    if (!g2->accM) {
        return "3D";
    }
    char text[64];
    std::snprintf(text, sizeof text, "3D, about %.1f m", *g2->accM);
    return text;
}

std::string PositionSource::fieldText(double nowS) const
{
    if (!_seenGps2) {
        return "N/A";
    }
    const std::optional<Gps2Reading> g2 = _fresh2(nowS);
    const Source src = source(nowS);
    if (src == Source::Unknown) {
        return "N/A";
    }
    if (src == Source::Gps1) {
        if (isCamera() && !_armed) {
            return "GPS 1 (GPS 2 repeats GPS 1 until the drone is armed)";
        }
        return "GPS 1 (" + _gps2Name() + ": " + _gps2Words(g2) + ")";
    }
    if (src == Source::Camera || src == Source::SecondGps) {
        std::string name = _gps2Name();
        name[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(name[0])));
        return name + ", " + _gps2Words(g2) + " (GPS 1 has no 3D fix)";
    }
    if (!_armed && g2 && g2->fix < k3dFix) {
        return "None on the ground (GPS 1 has no 3D fix)";
    }
    return "None: GPS 1 and " + _gps2Name() + " have no 3D fix";
}

std::string PositionSource::level(double nowS) const
{
    const Source src = source(nowS);
    if (!_seenGps2 || src == Source::Unknown) {
        return "na";
    }
    switch (src) {
    case Source::Gps1:
        return "ok";
    case Source::Camera:
    case Source::SecondGps:
        return "warn";
    default:
        return "bad";
    }
}

std::string PositionSource::tooltip() const
{
    std::string lines = "Where the drone's position comes from: GPS 1 while it has a 3D fix, else GPS 2.";
    if (isCamera()) {
        lines += "\nGPS 2 is the down camera's position (GPS-denied navigation, vama_nav on the Pi).";
    } else if (_gps2Type) {
        lines += "\nGPS2_TYPE is " + std::to_string(*_gps2Type) + ": GPS 2 is a receiver, not the camera (that is 14).";
    }
    if (_autoSwitch && isCamera() && *_autoSwitch != 4) {
        lines += "\nGPS_AUTO_SWITCH is " + std::to_string(*_autoSwitch) +
                 ": vama_nav holds the camera's positions back until it is 4.";
    }
    return lines;
}

std::string PositionSource::headerTag(double nowS) const
{
    const Source src = source(nowS);
    return src == Source::Camera ? "CAM" : (src == Source::SecondGps ? "GPS2" : "");
}

std::optional<PositionSource::Event> PositionSource::takeEvent(double nowS)
{
    if (!_seenGps2 || !_armed) {
        _candidate.reset();
        return std::nullopt;
    }
    const Source src = source(nowS);
    if (src == Source::Unknown) {
        return std::nullopt;
    }
    if (!_candidate || _candidate->first != src) {
        _candidate = std::make_pair(src, nowS);
    }
    if ((_said && *_said == src) || nowS - _candidate->second < kEventStableS) {
        return std::nullopt;
    }
    const bool first = !_said;
    _said = src;
    if (src == Source::Gps1) {
        if (first) {
            return std::nullopt;
        }
        return Event{"The position comes from GPS 1 again", false};
    }
    if (src == Source::Camera || src == Source::SecondGps) {
        return Event{"GPS 1 has no 3D fix: the position comes from " + _gps2Name(), true};
    }
    return Event{"No position: GPS 1 and " + _gps2Name() + " have no 3D fix", true};
}

} // namespace vama
