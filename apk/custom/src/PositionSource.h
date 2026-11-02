#pragma once

#include <map>
#include <optional>
#include <string>

namespace vama {

/// Where the drone's position comes from: GPS 1, or the down camera as GPS 2 (A8 step 4, 2026-10-30).
///
/// A port of VGCS's vgcs/app/position_source.py: the same rules and the same words, so the app and
/// VGCS say the same. test/gen_position_source_vectors.py runs VGCS's class over scripts, and
/// test/link/position_source_test.cc plays them here.
///
/// The GPS-denied navigation (drone/gps_denied/vama_nav) gives ArduPilot the camera's position as its
/// second GPS: GPS2_TYPE 14 (a MAVLink GPS), GPS_AUTO_SWITCH 4, so ArduPilot takes GPS 2 only while
/// GPS 1 has no 3D fix. ArduPilot reports GPS 1 in GPS_RAW_INT and GPS 2 in GPS2_RAW. So:
///
/// - GPS 1 with a 3D fix: the position is GPS 1's, whatever GPS 2 says.
/// - GPS 1 without one, GPS 2 with one: the camera's (or a second real GPS, when GPS2_TYPE is not 14:
///   then it is called "GPS 2").
/// - Neither: no position source. The EKF holds on its own sensors for a few seconds, then its
///   failsafe acts.
///
/// While the drone is disarmed vama_nav passes GPS 1's own fix on as GPS 2 (ArduPilot arms only when
/// every GPS has a 3D fix), so a GPS 2 fix then is not a camera position: the text says so.
///
/// A drone without GPS2_RAW shows nothing new. Every reading has a lifetime: a GPS that stopped
/// reporting is not shown as having a fix. A change of source is said once it has lasted
/// kEventStableS (a fix that flickers at the edge is not news every second).
///
/// Plain C++, no Qt. Times are seconds on any steady clock.
class PositionSource
{
public:
    enum class Source { Gps1, Camera, SecondGps, None, Unknown };

    struct Event
    {
        std::string text;
        bool warning = false;
    };

    static constexpr double kStaleAfterS = 5.0;
    static constexpr double kEventStableS = 2.0;
    static constexpr int kMavlinkGpsType = 14;

    PositionSource() { reset(); }

    /// A new link: nothing known.
    void reset();

    void updateGps1(int fixType, double nowS);
    /// hAccM: GPS 2's accuracy in metres. 0, less, or not a number: it gives none.
    void updateGps2(int fixType, double hAccM, int satellites, double nowS);
    /// GPS2_TYPE (GPS_TYPE2 before ArduPilot 4.6) and GPS_AUTO_SWITCH, as read from the drone.
    /// Other names are left alone.
    void updateSettings(const std::map<std::string, double> &params);
    void setArmed(bool armed) { _armed = armed; }

    bool armed() const { return _armed; }
    bool hasGps2() const { return _seenGps2; }
    bool isCamera() const { return _gps2Type && *_gps2Type == kMavlinkGpsType; }

    Source source(double nowS) const;
    /// The line of VGCS's Vehicle status ("Position source"), "N/A" without GPS 2.
    std::string fieldText(double nowS) const;
    /// VGCS's state colours: "na", "ok", "warn" or "bad".
    std::string level(double nowS) const;
    /// What GPS 2 is, and a wrong setting.
    std::string tooltip() const;
    /// Beside the satellite count: "CAM" while the camera gives the position, "GPS2" for a second
    /// real GPS, else nothing.
    std::string headerTag(double nowS) const;
    /// What to say once a new source has lasted kEventStableS. Only in the air and only for a drone
    /// with GPS 2: GPS 1's own loss is said elsewhere.
    std::optional<Event> takeEvent(double nowS);

private:
    struct Gps1Reading
    {
        int fix = 0;
        double atS = 0.0;
    };
    struct Gps2Reading
    {
        int fix = 0;
        std::optional<double> accM;
        int satellites = 0;
        double atS = 0.0;
    };

    std::optional<int> _fresh1(double nowS) const;
    std::optional<Gps2Reading> _fresh2(double nowS) const;
    std::string _gps2Name() const;
    static std::string _gps2Words(const std::optional<Gps2Reading> &g2);

    std::optional<Gps1Reading> _gps1;
    std::optional<Gps2Reading> _gps2;
    bool _seenGps2 = false;
    bool _armed = false;
    std::optional<int> _gps2Type;
    std::optional<int> _autoSwitch;
    std::optional<Source> _said;
    std::optional<std::pair<Source, double>> _candidate;
};

} // namespace vama
