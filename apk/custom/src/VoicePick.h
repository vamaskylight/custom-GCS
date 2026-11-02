#pragma once

// Which of the speech engine's voices the app speaks with (the client, Nish 2026-10-30: Indian
// English, female). No Qt in it, so the host tests can check it (test/voice_pick_test.cc);
// VamaVoice.h feeds it the engine's voices and applies the one it picks.

#include <string>
#include <vector>

namespace vama::voice {

enum class Gender { Unknown, Male, Female };

struct Voice {
    std::string name;       // the engine's own name, as Android gives it ("en-in-x-ena-local")
    std::string language;   // ISO 639: "en"
    std::string territory;  // ISO 3166: "IN"
    Gender gender = Gender::Unknown;  // Android tells it only in the name ("#female"), so often unknown
};

/// An Android voice that needs the internet (its name ends in "-network"). The remote control has
/// none in the field, and such a voice then speaks with another one.
bool needsNetwork(const std::string &name);

/// English as spoken in India (language "en", territory "IN", in any case).
bool isIndianEnglish(const Voice &voice);

/// The voices the menu offers, as indexes into @p voices: English ones only, Indian English first
/// (offline before online, then female, unknown, male), then the other English ones (offline first).
std::vector<int> listOrder(const std::vector<Voice> &voices);

/// The voice to speak with: the one chosen by name when it is there, else the first Indian English one
/// of listOrder, else -1 (QGC's own choice stays: American English, the engine's first voice).
int pick(const std::vector<Voice> &voices, const std::string &chosen);

} // namespace vama::voice
