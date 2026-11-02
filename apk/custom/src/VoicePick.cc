#include "VoicePick.h"

#include <algorithm>
#include <cctype>

namespace vama::voice {

namespace {

std::string lower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

bool isEnglish(const Voice &voice)
{
    return lower(voice.language) == "en";
}

int genderRank(Gender gender)
{
    switch (gender) {
    case Gender::Female:
        return 0;
    case Gender::Unknown:
        return 1;
    case Gender::Male:
        return 2;
    }
    return 1;
}

} // namespace

bool needsNetwork(const std::string &name)
{
    return lower(name).find("network") != std::string::npos;
}

bool isIndianEnglish(const Voice &voice)
{
    return isEnglish(voice) && lower(voice.territory) == "in";
}

std::vector<int> listOrder(const std::vector<Voice> &voices)
{
    std::vector<int> order;
    for (int i = 0; i < static_cast<int>(voices.size()); ++i) {
        if (isEnglish(voices[static_cast<size_t>(i)])) {
            order.push_back(i);
        }
    }
    std::stable_sort(order.begin(), order.end(), [&voices](int a, int b) {
        const Voice &x = voices[static_cast<size_t>(a)];
        const Voice &y = voices[static_cast<size_t>(b)];
        if (isIndianEnglish(x) != isIndianEnglish(y)) {
            return isIndianEnglish(x);
        }
        if (needsNetwork(x.name) != needsNetwork(y.name)) {
            return !needsNetwork(x.name);
        }
        if (isIndianEnglish(x) && genderRank(x.gender) != genderRank(y.gender)) {
            return genderRank(x.gender) < genderRank(y.gender);
        }
        return false;  // as the engine lists them
    });
    return order;
}

int pick(const std::vector<Voice> &voices, const std::string &chosen)
{
    if (!chosen.empty()) {
        for (int i = 0; i < static_cast<int>(voices.size()); ++i) {
            if (voices[static_cast<size_t>(i)].name == chosen) {
                return i;
            }
        }
    }
    const std::vector<int> order = listOrder(voices);
    if (!order.empty() && isIndianEnglish(voices[static_cast<size_t>(order.front())])) {
        return order.front();
    }
    return -1;
}

} // namespace vama::voice
