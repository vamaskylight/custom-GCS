// Host unit test for the app's choice of voice (VoicePick.h, the client 2026-10-30: Indian English,
// female): made-up voice lists in the forms Android's speech engines give them.

#include "../VoicePick.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace vama::voice;

namespace {

int g_checks = 0;
int g_failures = 0;

void check(bool ok, const char *what)
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL %s\n", what);
    }
}

Voice v(const char *name, const char *language, const char *territory, Gender gender = Gender::Unknown)
{
    return Voice{name, language, territory, gender};
}

std::string nameAt(const std::vector<Voice> &voices, int index)
{
    return index < 0 ? std::string("(none)") : voices[static_cast<size_t>(index)].name;
}

} // namespace

int main()
{
    // An older Google engine: the gender is in the name, so Qt tells it.
    const std::vector<Voice> older = {
        v("en-gb-x-rjs#male_1-local", "en", "GB", Gender::Male),
        v("en-us-x-sfg#female_1-local", "en", "US", Gender::Female),
        v("en-in-x-cxx#male_1-local", "en", "IN", Gender::Male),
        v("hi-in-x-hia#female_1-local", "hi", "IN", Gender::Female),
        v("en-in-x-cxx#female_2-network", "en", "IN", Gender::Female),
        v("en-in-x-cxx#female_1-local", "en", "IN", Gender::Female),
    };
    check(nameAt(older, pick(older, "")) == "en-in-x-cxx#female_1-local",
          "the offline Indian English female voice is picked");
    const std::vector<int> order = listOrder(older);
    check(order.size() == 5, "only English voices are offered (no Hindi)");
    check(order.size() == 5 && nameAt(older, order[0]) == "en-in-x-cxx#female_1-local" &&
              nameAt(older, order[1]) == "en-in-x-cxx#male_1-local" &&
              nameAt(older, order[2]) == "en-in-x-cxx#female_2-network",
          "Indian English first: offline before online, then female before male");
    check(order.size() == 5 && nameAt(older, order[3]) == "en-gb-x-rjs#male_1-local" &&
              nameAt(older, order[4]) == "en-us-x-sfg#female_1-local",
          "then the other English voices, as the engine lists them (female first only among the Indian ones)");

    // A newer Google engine: no gender in the names.
    const std::vector<Voice> newer = {
        v("en-us-x-iol-local", "en", "US"),
        v("en-in-x-ena-network", "en", "IN"),
        v("en-in-x-enc-local", "en", "IN"),
        v("en-in-x-ene-local", "en", "IN"),
    };
    check(nameAt(newer, pick(newer, "")) == "en-in-x-enc-local",
          "without genders, the first offline Indian English voice");
    check(nameAt(newer, pick(newer, "en-in-x-ene-local")) == "en-in-x-ene-local",
          "a voice chosen in the menu wins");
    check(nameAt(newer, pick(newer, "en-in-x-ena-network")) == "en-in-x-ena-network",
          "even an online one, when it is chosen");
    check(nameAt(newer, pick(newer, "gone-voice")) == "en-in-x-enc-local",
          "a chosen voice that is no longer there: the automatic choice");

    // No Indian English voice at all (an engine without the Indian voice data).
    const std::vector<Voice> plain = {v("en-us-x-sfg-local", "en", "US"), v("de-de-x-nfh-local", "de", "DE")};
    check(pick(plain, "") == -1, "no Indian English voice: QGC's own choice stays");
    check(listOrder(plain).size() == 1, "the English voice is still offered");
    check(pick({}, "") == -1 && listOrder({}).empty(), "no voices at all");

    // Upper and lower case as engines give them.
    const std::vector<Voice> cases = {v("Microsoft Heera - English (India)", "EN", "in", Gender::Female)};
    check(pick(cases, "") == 0, "language and territory in any case");

    check(needsNetwork("en-in-x-ena-network") && !needsNetwork("en-in-x-ena-local"), "online voices are known");

    std::printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
