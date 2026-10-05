// Port of vgcs/skydroid/protocol.py. Keep the two in step: the tests in
// apk/custom/test compare these builders with frames made by the Python code.

#include "SkydroidTop.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace skydroid::top {

namespace {

constexpr double kGimbalSpeedUnitDps = 0.5;

bool isSpace(char c)
{
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f';
}

std::string trim(const std::string &s)
{
    size_t b = 0;
    size_t e = s.size();
    while (b < e && isSpace(s[b])) {
        ++b;
    }
    while (e > b && isSpace(s[e - 1])) {
        --e;
    }
    return s.substr(b, e - b);
}

std::string upper(std::string s)
{
    for (char &c : s) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return s;
}

std::string lower(std::string s)
{
    for (char &c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

bool isHex(char c)
{
    return std::isxdigit(static_cast<unsigned char>(c)) != 0;
}

std::string onlyHex(const std::string &s)
{
    std::string out;
    for (char c : s) {
        if (isHex(c)) {
            out.push_back(c);
        }
    }
    return out;
}

std::string hexUpper(long long value, int width)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%0*llX", width, static_cast<unsigned long long>(value));
    return buf;
}

long long clampLL(long long v, long long lo, long long hi)
{
    return std::max(lo, std::min(hi, v));
}

double clampD(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

} // namespace

long long pyRound(double value)
{
    // nearbyint uses the current rounding mode, which is round-half-to-even
    // by default: the same rule as Python 3's round().
    return static_cast<long long>(std::nearbyint(value));
}

std::string checksum(const std::string &body)
{
    unsigned total = 0;
    for (unsigned char c : body) {
        total += c;
    }
    return hexUpper(total & 0xFF, 2);
}

std::string buildTpFrame(char dest, char control, const std::string &tag, const std::string &data,
                         char src, int variable, const Options &options)
{
    const char destC = static_cast<char>(std::toupper(static_cast<unsigned char>(dest ? dest : 'G')));
    const char srcC = static_cast<char>(std::toupper(static_cast<unsigned char>(src ? src : 'U')));
    const char ctrl = static_cast<char>(std::tolower(static_cast<unsigned char>(control ? control : 'w')));
    const std::string tagS = upper(trim(tag));
    if (tagS.size() != 3) {
        return {};
    }
    const bool useVar = (variable < 0) ? (data.size() > 2) : (variable != 0);
    std::string header;
    std::string lengthCh;
    if (useVar) {
        header = (destC == 'G' && options.gClassUpperHeader) ? "#TP" : "#tp";
        if (data.size() > 0x0F) {
            return {};
        }
        lengthCh = hexUpper(static_cast<long long>(data.size()), 1);
    } else {
        header = "#TP";
        if (data.size() != 2) {
            return {};
        }
        lengthCh = "2";
    }
    const std::string body = header + std::string(1, srcC) + std::string(1, destC) + lengthCh +
                             std::string(1, ctrl) + tagS + data;
    return body + checksum(body);
}

// --- Field encoders --------------------------------------------------------

std::string encodeSpeed2(double degPerSecond)
{
    long long units = pyRound(degPerSecond / kGimbalSpeedUnitDps);
    units = clampLL(units, -127, 127);
    return hexUpper(units & 0xFF, 2);
}

std::string encodeAttitudeField4(double degrees)
{
    long long units = pyRound(degrees * 100.0);
    units = clampLL(units, -32768, 32767);
    if (units < 0) {
        units = (units + 0x10000) & 0xFFFF;
    }
    return hexUpper(units, 4);
}

std::string encodeAngle4(double degrees)
{
    return encodeAttitudeField4(degrees);
}

std::optional<double> decodeAttitudeField4(const std::string &field)
{
    const std::string s = upper(trim(field));
    if (s.size() != 4 || !std::all_of(s.begin(), s.end(), isHex)) {
        return std::nullopt;
    }
    long raw = std::stol(s, nullptr, 16);
    if (raw >= 0x8000) {
        raw -= 0x10000;
    }
    return raw / 100.0;
}

// --- Gimbal ------------------------------------------------------------------

std::string buildGaaEnable(int hz, const Options &options)
{
    const int rate = std::max(1, std::min(100, hz));
    return buildTpFrame('G', 'w', "GAA", hexUpper(rate, 2), 'U', -1, options);
}

std::string buildGacQuery(const Options &options)
{
    return buildTpFrame('G', 'r', "GAC", "00", 'U', -1, options);
}

std::string buildPtz(const std::string &action, const Options &options)
{
    const std::string a = lower(trim(action));
    std::string code;
    if (a == "stop") {
        code = "00";
    } else if (a == "up") {
        code = "01";
    } else if (a == "down") {
        code = "02";
    } else if (a == "left") {
        code = "03";
    } else if (a == "right") {
        code = "04";
    } else if (a == "center") {
        code = "05";
    } else if (a == "nadir" || a == "down_once" || a == "point_down") {
        code = "0A";
    } else {
        return {};
    }
    return buildTpFrame('G', 'w', "PTZ", code, 'U', -1, options);
}

std::string buildGimbalSpeed(double yawDegPerSecond, double pitchDegPerSecond, const Options &options)
{
    return buildTpFrame('G', 'w', "GSM", encodeSpeed2(yawDegPerSecond) + encodeSpeed2(pitchDegPerSecond),
                        'U', -1, options);
}

std::string buildGimbalSpeedAxis(const std::string &tag, double degPerSecond, const Options &options)
{
    const std::string t = upper(trim(tag));
    if (t != "GSY" && t != "GSP" && t != "GSR") {
        return {};
    }
    return buildTpFrame('G', 'w', t, encodeSpeed2(degPerSecond), 'U', -1, options);
}

std::string buildGimbalAngleAxis(const std::string &axisTag, double degrees, double speed, const Options &options)
{
    const std::string t = upper(trim(axisTag));
    if (t != "GAY" && t != "GAP" && t != "GAR") {
        return {};
    }
    return buildTpFrame('G', 'w', t, encodeAngle4(degrees) + encodeSpeed2(speed), 'U', 1, options);
}

std::string buildGimbalAngles(double yawDeg, double pitchDeg, double yawSpeed, double pitchSpeed,
                              const Options &options)
{
    const std::string data =
        encodeAngle4(yawDeg) + encodeSpeed2(yawSpeed) + encodeAngle4(pitchDeg) + encodeSpeed2(pitchSpeed);
    return buildTpFrame('G', 'w', "GAM", data, 'U', 1, options);
}

// --- Camera and system ---------------------------------------------------

std::string buildSystemCommand(const std::string &tag, const std::string &data, bool write, const Options &options)
{
    const std::string t = upper(tag).substr(0, 3);
    return buildTpFrame('D', write ? 'w' : 'r', t, data, 'U', data.size() != 2 ? 1 : 0, options);
}

std::string buildPhoto(const Options &options)
{
    return buildSystemCommand("CAP", "01", true, options);
}

std::string buildRecord(const Options &options)
{
    return buildSystemCommand("REC", "01", true, options);
}

// --- Laser rangefinder (SLR) -------------------------------------------------

std::string buildSlrQuery(char dest, const Options &options)
{
    return buildTpFrame(dest ? dest : 'D', 'r', "SLR", "00", 'U', -1, options);
}

std::string buildSlrTrigger(char dest, const Options &options)
{
    return buildTpFrame(dest ? dest : 'D', 'w', "SLR", "01", 'U', -1, options);
}

int slrMaxDmForRange(std::optional<double> maxMetres)
{
    long long dm = kSlrDmMaxDefault;
    if (maxMetres && std::isfinite(*maxMetres)) {
        dm = pyRound(*maxMetres * 10.0);
    }
    // 0xFFFE keeps 0xFFFF (a common "no echo" filler) rejected.
    return static_cast<int>(std::max<long long>(kSlrDmMaxDefault, std::min<long long>(0xFFFE, dm)));
}

std::optional<int> decodeSlrDecimeters(const std::string &dataField, const Options &options)
{
    const std::string s = onlyHex(dataField);
    if (s.size() != 4) {
        return std::nullopt;
    }
    const int dm = static_cast<int>(std::stol(s, nullptr, 16));
    if (dm < kSlrDmMin || dm > options.slrMaxDm) {
        return std::nullopt;
    }
    return dm;
}

std::optional<double> decodeSlrDistanceM(const std::string &dataField, const Options &options)
{
    const auto dm = decodeSlrDecimeters(dataField, options);
    if (!dm) {
        return std::nullopt;
    }
    return *dm / 10.0;
}

// --- Tracking ------------------------------------------------------------------

std::string buildGotTarget(int xPx, int yPx, int frameW, int frameH, const Options &options)
{
    const int fw = std::max(1, frameW);
    const int fh = std::max(1, frameH);
    const int x = std::max(0, std::min(fw, xPx));
    const int y = std::max(0, std::min(fh, yPx));
    return buildTpFrame('G', 'w', "GOT", hexUpper(x, 4) + hexUpper(y, 4), 'U', 1, options);
}

std::string buildSumTrack(bool confirm, const Options &options)
{
    return buildTpFrame('G', 'w', "SUM", confirm ? "01" : "00", 'U', -1, options);
}

// --- Zoom and focus ------------------------------------------------------------

std::string buildPodCameraCommand(const std::string &tag, const std::string &data, char dest, bool write,
                                  const Options &options)
{
    return buildTpFrame(dest ? dest : 'M', write ? 'w' : 'r', upper(tag).substr(0, 3), data, 'P', 1, options);
}

namespace {

std::string dzmAbsoluteData(double zoomX, int cameraX0)
{
    const long long units = pyRound(clampD(zoomX * 10.0, 0.0, 300.0));
    return hexUpper(cameraX0 & 0xFF, 2) + "F" + hexUpper(units, 3);
}

} // namespace

std::string buildDzmAbsoluteZoom(double zoomX, int cameraX0, const Options &options)
{
    return buildPodCameraCommand("DZM", dzmAbsoluteData(zoomX, cameraX0), 'D', true, options);
}

std::string buildDzmAbsoluteZoomUd(double zoomX, int cameraX0, const Options &options)
{
    return buildTpFrame('D', 'w', "DZM", dzmAbsoluteData(zoomX, cameraX0), 'U', 1, options);
}

std::string buildMulOpticalZoom(double zoomX, const Options &options)
{
    const long long units = pyRound(clampD(zoomX * 10.0, 1.0, 300.0));
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04lld", units);
    return buildPodCameraCommand("MUL", buf, 'M', true, options);
}

std::vector<std::string> buildOpticalZoomFrames(double zoomX, const Options &options)
{
    const double lvl = clampD(zoomX, 1.0, 30.0);
    return {buildDzmAbsoluteZoom(lvl, 0, options), buildDzmAbsoluteZoomUd(lvl, 0, options),
            buildMulOpticalZoom(lvl, options)};
}

std::string buildDzmZoomStepV47(const std::string &action, const Options &options)
{
    const std::string a = lower(trim(action));
    std::string code;
    if (a == "in" || a == "tele" || a == "+") {
        code = "0A";
    } else if (a == "out" || a == "wide" || a == "-") {
        code = "0B";
    } else {
        return {};
    }
    return buildTpFrame('D', 'w', "DZM", code, 'U', 0, options);
}

std::string buildDzmQuery(const Options &options)
{
    return buildTpFrame('D', 'r', "DZM", "00", 'U', 0, options);
}

std::string buildDzmLensSelect(const std::string &lens, const Options &options)
{
    const std::string l = lower(trim(lens));
    std::string code;
    if (l == "long" || l == "tele") {
        code = "0C";
    } else if (l == "short" || l == "wide") {
        code = "0D";
    } else {
        return {};
    }
    return buildTpFrame('D', 'w', "DZM", code, 'U', 0, options);
}

std::string buildDzmPreset(int multiplier, const Options &options)
{
    if (multiplier < 1 || multiplier > 4) {
        return {};
    }
    return buildTpFrame('D', 'w', "DZM", hexUpper(multiplier, 2), 'U', 0, options);
}

std::vector<std::string> buildDzmStepFrames(int direction, const Options &options)
{
    if (direction == 0) {
        return {};
    }
    return {buildDzmZoomStepV47(direction > 0 ? "in" : "out", options)};
}

std::vector<std::string> buildDzmHomeFrames(const Options &options)
{
    return {buildDzmLensSelect("short", options), buildDzmPreset(1, options)};
}

std::optional<int> decodeDzmStep(const std::string &dataField)
{
    const std::string s = onlyHex(dataField);
    if (s.size() != 2) {
        return std::nullopt;
    }
    return static_cast<int>(std::stol(s, nullptr, 16));
}

std::vector<std::string> buildC13ZoomStepFrames(int direction, const Options &options)
{
    // Same as the Python code: direction 0 counts as "out".
    const std::string act = direction > 0 ? "in" : "out";
    return {buildZmcZoom(act, options), buildDzmZoomStepV47(act, options), buildZmcZoom("stop", options)};
}

std::string buildDzmStepZoom(const std::string &action, const Options &options)
{
    const std::string a = lower(trim(action));
    std::string code;
    if (a == "stop") {
        code = "0E";
    } else if (a == "in" || a == "tele") {
        code = "0C";
    } else if (a == "out" || a == "wide") {
        code = "0D";
    } else {
        return {};
    }
    return buildPodCameraCommand("DZM", "00" + code, 'D', true, options);
}

std::string buildZmcZoom(const std::string &action, const Options &options)
{
    const std::string a = lower(trim(action));
    std::string code;
    if (a == "stop") {
        code = "00";
    } else if (a == "out" || a == "wide") {
        code = "01";
    } else if (a == "in" || a == "tele") {
        code = "02";
    } else {
        return {};
    }
    return buildPodCameraCommand("ZMC", code, 'M', true, options);
}

std::string buildFccFocus(const std::string &action, const Options &options)
{
    const std::string a = lower(trim(action));
    std::string code;
    if (a == "stop") {
        code = "00";
    } else if (a == "near" || a == "in") {
        code = "02";
    } else if (a == "far" || a == "out") {
        code = "01";
    } else if (a == "auto") {
        code = "10";
    } else {
        return {};
    }
    return buildPodCameraCommand("FCC", code, 'M', true, options);
}

// --- Replies -----------------------------------------------------------------

std::optional<DecodedFrame> parseTpFrame(const std::string &raw, const Options &options)
{
    // Python: raw.decode("ascii", errors="ignore").strip()
    std::string ascii;
    ascii.reserve(raw.size());
    for (unsigned char c : raw) {
        if (c < 0x80) {
            ascii.push_back(static_cast<char>(c));
        }
    }
    const std::string text = trim(ascii);

    // Python: ^#tp([UMDEG]{2})([0-9A-F])([wr])([A-Z]{3})(.*?)([0-9A-F]{2})$, ignoring case.
    if (text.size() < 12 || upper(text.substr(0, 3)) != "#TP") {
        return std::nullopt;
    }
    auto isAddr = [](char c) {
        const char u = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        return u == 'U' || u == 'M' || u == 'D' || u == 'E' || u == 'G';
    };
    if (!isAddr(text[3]) || !isAddr(text[4]) || !isHex(text[5])) {
        return std::nullopt;
    }
    const char ctrl = static_cast<char>(std::tolower(static_cast<unsigned char>(text[6])));
    if (ctrl != 'w' && ctrl != 'r') {
        return std::nullopt;
    }
    for (size_t i = 7; i < 10; ++i) {
        if (!std::isalpha(static_cast<unsigned char>(text[i]))) {
            return std::nullopt;
        }
    }
    if (!isHex(text[text.size() - 2]) || !isHex(text[text.size() - 1])) {
        return std::nullopt;
    }

    DecodedFrame frame;
    frame.address = upper(text.substr(3, 2));
    frame.tag = upper(text.substr(7, 3));
    frame.ctrl = ctrl;
    frame.data = text.substr(10, text.size() - 12);
    frame.raw = text;

    const std::string &data = frame.data;
    if (frame.tag == "GAC" && data.size() >= 12) {
        frame.yaw = decodeAttitudeField4(data.substr(0, 4));
        frame.pitch = decodeAttitudeField4(data.substr(4, 4));
        frame.roll = decodeAttitudeField4(data.substr(8, 4));
    } else if (frame.tag == "SLR" && !data.empty()) {
        frame.slrDm = decodeSlrDecimeters(data, options);
    } else if (frame.tag == "DZM" && !data.empty() && ctrl == 'r') {
        // Read replies only: a write echo carries an action code, not a step.
        frame.dzmStep = decodeDzmStep(data);
    }
    return frame;
}

} // namespace skydroid::top
