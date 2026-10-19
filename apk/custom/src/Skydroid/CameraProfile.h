#pragma once

// Camera profiles (task AA, client 2026-10-16): what the app knows of a camera, so that an
// admin can add a camera without new code.
//
// The three Skydroid cameras the app was written for are built in (V12, V13, V14 Pro). Their
// numbers are the ones the app has always used, and they cannot be changed: the admin copies
// one as the start of a new camera. An added camera is a file, one per camera, in the app's
// folder "Cameras", so it can be copied to other remote controls (and VGCS can read it later).
//
// Every camera speaks Skydroid's TOP protocol (SkydroidTop). A camera with another protocol
// needs code once. VGCS keeps the same facts in its SkydroidCommandProfile
// (vgcs/skydroid/command_map.py): the names here follow it where they can.

#include <QtCore/QJsonObject>
#include <QtCore/QList>
#include <QtCore/QString>
#include <QtCore/QStringList>

#include <optional>
#include <vector>

namespace skydroid::camera {

/// How the app knows the camera's zoom (VGCS's zoom_model).
enum class Zoom
{
    None,      ///< no zoom: the zoom buttons are hidden
    Counted,   ///< C12, C13: the app counts the steps it sent, 0.1x each (VGCS "commanded_optical")
    Reported,  ///< C14 Pro: the camera reports its zoom step, which picks a lens and a crop (VGCS "dzm_step")
};

/// One lens of a camera that reports its zoom step: its view at its first step, and its steps.
struct Lens
{
    double fovHDeg = 0.0;
    double fovVDeg = 0.0;
    int firstStep = 0;
    int lastStep = 0;
};

/// A camera. The values without a note are the C13's.
struct Profile
{
    QString id;     ///< built in: "C12", "C13", "C14 Pro" (the saved setting); added: its file name without ".json"
    QString name;   ///< on screen
    bool builtIn = false;

    // Addresses. Choosing the camera sets them.
    QString host;
    int port = 5000;
    QString dayVideoUrl;
    QString thermalVideoUrl;  ///< empty: no thermal picture (no IR button)

    // The day picture
    double fovHDeg = 83.4;    ///< at the widest zoom (Zoom::None and Counted)
    double fovVDeg = 46.9;
    Zoom zoom = Zoom::Counted;
    double maxZoom = 30.0;    ///< Counted: the highest zoom
    std::vector<Lens> lenses; ///< Reported: in step order
    int sensorW = 3840;       ///< Reported: the picture is a crop of this many sensor points,
    int sensorH = 2160;
    int cropStepW = 36;       ///< and each step crops this many more (Skydroid, 2026-09-19)
    int cropStepH = 20;

    // The thermal picture
    double thermalFovHDeg = 48.7;
    double thermalFovVDeg = 38.6;
    /// False: the thermal lens is known only from a data sheet. Good enough to turn the camera
    /// and to follow an object, not to place a point from the thermal picture (only the cross).
    bool thermalFovSure = false;

    // How far the gimbal turns
    double pitchMinDeg = -90.0;
    double pitchMaxDeg = 10.0;
    double yawLimitDeg = 90.0;  ///< either way from the middle

    // The laser
    bool laser = true;
    double laserMaxM = 1000.0;  ///< longer readings are not taken as real

    // Other features
    bool photo = true;
    bool record = true;
    bool ownTracker = true;     ///< the camera's own tracker (object lock "The camera's own tracker")

    // A detail of the protocol: every gimbal frame with the upper-case "#TP" header (C14 Pro).
    // Else the long gimbal frames go out twice, upper-case first (see SkydroidLink::_sendGimbal).
    bool upperGimbalHeader = false;

    bool hasThermal() const { return !thermalVideoUrl.isEmpty(); }
};

/// The built-in cameras, in the order the list shows them.
QList<Profile> builtInProfiles();

/// The id of the camera a new app starts with (the V13).
QString defaultId();

/// The zoom of a counted step, and the steps a counted zoom has.
constexpr double kCountedStepZoom = 0.1;
int countedStepsMax(const Profile &p);

/// The field of view of the day picture, in degrees, at a zoom: the steps counted (Counted)
/// or the step the camera reported (Reported, -1 when not known yet: its first step).
void dayFov(const Profile &p, int countedSteps, int reportedStep, double &horizontalDeg, double &verticalDeg);

/// The zoom to show ("2.0x", "W2.9x", "T5.3x"), or nothing (no zoom, or no step reported yet).
QString zoomLabel(const Profile &p, int countedSteps, int reportedStep);

/// What is wrong with a camera's values, as a sentence for the editor, or nothing.
QString problem(const Profile &p);

/// The camera as its file holds it (and as the camera editor shows it, with "id" and "builtIn").
QJsonObject toJson(const Profile &p);
/// A camera from its file. Nothing when the file is not a camera file: why says what is wrong.
std::optional<Profile> fromJson(const QJsonObject &json, const QString &id, QString *why);
/// A camera from the editor's fields (toJson's, with "id"): the format and the protocol are
/// this app's. Nothing when a field is wrong: why says what.
std::optional<Profile> fromFields(const QJsonObject &fields, QString *why);

/// A file name (without ".json") for a camera's name: small letters, digits and "-".
QString idForName(const QString &name);

/// The cameras: the built-in ones, then the added ones (files in a folder), by name.
class Store
{
public:
    void setFolder(const QString &folder) { _folder = folder; }
    QString folder() const { return _folder; }

    /// Reads the folder again (and makes it, so that files can be copied into it).
    void load();
    const QList<Profile> &profiles() const { return _profiles; }
    const Profile *find(const QString &id) const;
    /// The files that were not read, each with why.
    QStringList notes() const { return _notes; }

    /// Saves an added camera, a new one when its id is empty or a built-in's. The problem, or
    /// nothing. savedId: its id.
    QString save(Profile profile, QString *savedId);
    /// Removes an added camera's file. The problem, or nothing.
    QString remove(const QString &id);

private:
    QString _nameProblem(const Profile &profile) const;

    QString _folder;
    QList<Profile> _profiles;
    QStringList _notes;
};

}  // namespace skydroid::camera
