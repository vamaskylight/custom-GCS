// Host test for the app's voice on the speech engine (VamaVoiceEngine.h, B21, B23): the real VamaVoice
// and VamaVoiceEngine against a stand-in for Qt Speech that counts what goes to Android
// (voice_stubs/QtTextToSpeech/qtexttospeech.h), with QGC's AudioOutput as it sets its own voice.
// Test build 18 listed the voices after every spoken message, through every locale of the engine,
// on the main thread: the client's buttons answered 3 to 8 s late (2026-10-31).
// Run with apk/custom/test/run_link_test.ps1.

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QSettings>
#include <QtTest/QSignalSpy>
#include <QtTest/QTest>
#include <QtTextToSpeech/QTextToSpeech>

#include <memory>

#include "VamaVoice.h"
#include "VamaVoiceEngine.h"

namespace {

const QLocale kUs(QLocale::English, QLocale::UnitedStates);
const QLocale kUk(QLocale::English, QLocale::UnitedKingdom);
const QLocale kAustralia(QLocale::English, QLocale::Australia);
const QLocale kIndia(QLocale::English, QLocale::India);
const QLocale kHindi(QLocale::Hindi, QLocale::India);
const QLocale kGerman(QLocale::German, QLocale::Germany);
const QLocale kJapanese(QLocale::Japanese, QLocale::Japan);

const char *const kBestIndianVoice = "en-in-x-cxx#female_1-local";

QVoice voiceOf(const char *name, const QLocale &locale, QVoice::Gender gender = QVoice::Unknown)
{
    return QVoice(QString::fromLatin1(name), locale, gender);
}

// American and British voices, as an RC without the Indian voice data offers them.
QList<QVoice> plainVoices()
{
    return {voiceOf("de-de-x-nfh-local", kGerman),
            voiceOf("en-us-x-iob-local", kUs),
            voiceOf("en-us-x-sfg#female_1-local", kUs, QVoice::Female),
            voiceOf("ja-jp-x-htm-local", kJapanese),
            voiceOf("en-gb-x-rjs#male_1-local", kUk, QVoice::Male),
            voiceOf("en-gb-x-gba-local", kUk)};
}

QList<QVoice> indianVoices()
{
    return {voiceOf("en-in-x-cxx#male_1-local", kIndia, QVoice::Male),
            voiceOf("en-in-x-cxx#female_2-network", kIndia, QVoice::Female),
            voiceOf(kBestIndianVoice, kIndia, QVoice::Female)};
}

// A Google engine: many languages, each several voices.
QList<QVoice> googleVoices()
{
    QList<QVoice> voices = plainVoices();
    voices << voiceOf("hi-in-x-hia#female_1-local", kHindi, QVoice::Female)
           << voiceOf("en-au-x-afa-local", kAustralia);
    voices << indianVoices();
    return voices;
}

// QGC's AudioOutput (init, _finishInit, _applyEngineSettings): its own locale and voice, in direct slots,
// at the engine's first Ready and when the engine changes.
void applyQgcSettings(QTextToSpeech &engine)
{
    if (engine.state() != QTextToSpeech::Ready) {
        return;
    }
    if (engine.availableLocales().contains(kUs)) {
        engine.setLocale(kUs);
    }
    const QList<QVoice> voices = engine.availableVoices();
    if (!voices.isEmpty()) {
        engine.setVoice(voices.constFirst());
    }
}

void startQgcAudioOutput(QTextToSpeech &engine)
{
    auto initialized = std::make_shared<bool>(false);
    QObject::connect(&engine, &QTextToSpeech::stateChanged, &engine, [&engine, initialized](QTextToSpeech::State state) {
        if (state == QTextToSpeech::Ready && !*initialized) {
            *initialized = true;
            applyQgcSettings(engine);
        }
    });
    QObject::connect(&engine, &QTextToSpeech::engineChanged, &engine, [&engine]() { applyQgcSettings(engine); });
    if (engine.state() == QTextToSpeech::Ready) {
        *initialized = true;
        applyQgcSettings(engine);
    }
}

// What is queued runs: the engine's signals reach VamaVoiceEngine by the event loop.
void settle()
{
    QCoreApplication::sendPostedEvents(nullptr, QEvent::MetaCall);
}

// One spoken message: the engine says Speaking, then Ready.
void speakOne(QTextToSpeech &engine)
{
    engine.setState(QTextToSpeech::Speaking);
    settle();
    engine.setState(QTextToSpeech::Ready);
    settle();
}

// The app's order: QGC first, VamaVoiceEngine when the event loop runs, then the engine becomes ready.
struct App {
    explicit App(const QList<QVoice> &voices)
        : voice(nullptr)
        , hook(&voice, nullptr)
    {
        engine.setAllVoices(voices);
        startQgcAudioOutput(engine);
        hook.attach(&engine);
    }

    void engineReady()
    {
        engine.setState(QTextToSpeech::Ready);
        settle();
    }

    QString speaking() const { return engine.voice().name(); }

    QTextToSpeech engine;
    VamaVoice voice;
    VamaVoiceEngine hook;
};

// The line of a voice in the menu's list.
int lineOf(const VamaVoice &voice, const QString &name)
{
    const QStringList lines = voice.choices();
    for (int line = 1; line < lines.size(); ++line) {  // line 0 is "Automatic: ..." with a voice's name
        if (lines.at(line).endsWith(name)) {
            return line;
        }
    }
    return -1;
}

} // namespace

class VoiceEngineTest : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void init();
    void theFirstReadyListsOnlyTheEnglishLocales();
    void spokenMessagesAskTheEngineNothing();
    void theIndianVoiceKeepsSpeaking();
    void theVoiceIsTheOneTheWholeSearchWouldFind();
    void theListHasEveryEnglishVoiceOnce();
    void noVoiceIsPutBeforeTheEngineIsReady();
    void anEngineAlreadyReadyIsListedAtOnce();
    void speakingAtTheFirstReadyWaitsForTheNext();
    void aSavedChoiceSpeaksFromTheStart();
    void ourVoiceComesAfterQgcsWhoeverConnectedFirst();
    void aChoiceBeforeTheFirstReadyAddsNoQuestionLater();
    void qgcsOwnVoiceComesBackAfterMessages();
    void noIndianVoiceLeavesQgcsVoice();
    void aFirstListWithoutAnIndianVoiceGetsOneMore();
    void aVoiceChosenWhileSpeakingIsPutAtTheNextReady();
    void aChoiceWaitingIsPutOnce();
    void aVoiceChosenWhenReadyIsPutAtOnce();
    void thePanelAsksAgainWithoutAnIndianVoice();
    void thePanelAsksNothingWithAnIndianVoice();
    void thePanelWaitsWhileTheEngineSpeaks();
    void anEngineWithoutEnglishVoicesIsAskedAgain();
    void anotherEngineIsListedAgain();
    void noEngineAtAll();
};

void VoiceEngineTest::initTestCase()
{
    // Never the user's settings: an INI file in a temporary folder.
    QCoreApplication::setOrganizationName("VamaVoiceTest");
    QCoreApplication::setApplicationName("VamaVoiceTest");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, QDir::tempPath() + "/vama-voice-test");
}

void VoiceEngineTest::init()
{
    QSettings().remove("VamaVoice");
}

void VoiceEngineTest::theFirstReadyListsOnlyTheEnglishLocales()
{
    App app(googleVoices());
    app.engine.setState(QTextToSpeech::Ready);  // QGC's slots run here
    app.engine.calls = {};
    settle();
    QCOMPARE(app.engine.calls.localeLists, 1);
    // en_US, en_GB, en_AU, en_IN: no round for German, Japanese or Hindi.
    QCOMPARE(app.engine.calls.voiceLists, 4);
    for (const QLocale &locale : std::as_const(app.engine.calls.localesTried)) {
        QCOMPARE(locale.language(), QLocale::English);
    }
    QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));
}

void VoiceEngineTest::spokenMessagesAskTheEngineNothing()
{
    App app(googleVoices());
    app.engineReady();
    app.engine.calls = {};
    QSignalSpy changes(&app.voice, &VamaVoice::changed);
    for (int message = 0; message < 50; ++message) {
        speakOne(app.engine);
    }
    QCOMPARE(app.engine.calls.total(), 0);
    QCOMPARE(changes.count(), 0);
}

void VoiceEngineTest::theIndianVoiceKeepsSpeaking()
{
    App app(googleVoices());
    app.engineReady();
    for (int message = 0; message < 5; ++message) {
        speakOne(app.engine);
        QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));
        QCOMPARE(app.voice.current(), QString::fromLatin1(kBestIndianVoice));
    }
}

void VoiceEngineTest::theVoiceIsTheOneTheWholeSearchWouldFind()
{
    // The client accepted the voice of test build 18, which searched with findVoices(QLocale::English).
    App app(googleVoices());
    app.engineReady();
    const QVoice speaking = app.engine.voice();
    const QList<QVoice> whole = app.engine.findVoices(QLocale::English);
    std::vector<vama::voice::Voice> voices;
    for (const QVoice &voice : whole) {
        const auto gender = voice.gender() == QVoice::Female ? vama::voice::Gender::Female
                            : voice.gender() == QVoice::Male ? vama::voice::Gender::Male
                                                             : vama::voice::Gender::Unknown;
        voices.push_back({voice.name().toStdString(), QLocale::languageToCode(voice.locale().language()).toStdString(),
                          QLocale::territoryToCode(voice.locale().territory()).toStdString(), gender});
    }
    const int picked = vama::voice::pick(voices, std::string());
    QVERIFY(picked >= 0);
    QCOMPARE(speaking.name(), whole.at(picked).name());
}

void VoiceEngineTest::theListHasEveryEnglishVoiceOnce()
{
    App app(googleVoices());
    app.engineReady();
    const QStringList lines = app.voice.choices();
    // Automatic, 2 American, 2 British, 1 Australian, 3 Indian. The Indian ones first.
    QCOMPARE(lines.size(), 1 + 8);
    QVERIFY(lines.at(1).endsWith(QString::fromLatin1(kBestIndianVoice)));
    for (const QVoice &voice : googleVoices()) {
        const int expected = voice.locale().language() == QLocale::English ? 1 : 0;
        int found = 0;
        for (int line = 1; line < lines.size(); ++line) {  // line 0 is "Automatic: ..." with the same name
            found += lines.at(line).endsWith(voice.name()) ? 1 : 0;
        }
        QCOMPARE(found, expected);
    }
}

void VoiceEngineTest::noVoiceIsPutBeforeTheEngineIsReady()
{
    App app(googleVoices());
    settle();
    QCOMPARE(app.engine.calls.total(), 0);
    QCOMPARE(app.voice.current(), QString());
    QVERIFY(app.voice.choices().size() == 1);
}

void VoiceEngineTest::anEngineAlreadyReadyIsListedAtOnce()
{
    QTextToSpeech engine;
    engine.setAllVoices(googleVoices());
    engine.setState(QTextToSpeech::Ready);
    startQgcAudioOutput(engine);
    VamaVoice voice(nullptr);
    VamaVoiceEngine hook(&voice, nullptr);
    hook.attach(&engine);
    QCOMPARE(engine.voice().name(), QString::fromLatin1(kBestIndianVoice));
}

void VoiceEngineTest::speakingAtTheFirstReadyWaitsForTheNext()
{
    // QGC's first message can start before the queued slot runs.
    App app(googleVoices());
    app.engine.setState(QTextToSpeech::Ready);
    const QVoice qgcVoice = app.engine.voice();
    app.engine.setState(QTextToSpeech::Speaking);
    app.engine.calls = {};
    settle();
    QCOMPARE(app.engine.calls.total(), 0);
    QCOMPARE(app.speaking(), qgcVoice.name());
    app.engine.setState(QTextToSpeech::Ready);
    settle();
    QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));
}

void VoiceEngineTest::aSavedChoiceSpeaksFromTheStart()
{
    QSettings().setValue(QLatin1String(VamaVoice::kSettingsKey), "en-gb-x-gba-local");
    App app(googleVoices());
    app.engineReady();
    QCOMPARE(app.speaking(), QString("en-gb-x-gba-local"));
    speakOne(app.engine);
    QCOMPARE(app.speaking(), QString("en-gb-x-gba-local"));
}

void VoiceEngineTest::ourVoiceComesAfterQgcsWhoeverConnectedFirst()
{
    // Queued: even with VamaVoiceEngine's connection first, QGC's direct slots run before it.
    QTextToSpeech engine;
    engine.setAllVoices(googleVoices());
    VamaVoice voice(nullptr);
    VamaVoiceEngine hook(&voice, nullptr);
    hook.attach(&engine);
    startQgcAudioOutput(engine);
    engine.setState(QTextToSpeech::Ready);
    settle();
    QCOMPARE(engine.voice().name(), QString::fromLatin1(kBestIndianVoice));

    // And it read QGC's choice, not what the engine had before QGC set it.
    QTextToSpeech plainEngine;
    plainEngine.setAllVoices(plainVoices());
    VamaVoice plainVoice(nullptr);
    VamaVoiceEngine plainHook(&plainVoice, nullptr);
    plainHook.attach(&plainEngine);
    startQgcAudioOutput(plainEngine);
    plainEngine.setState(QTextToSpeech::Ready);
    settle();
    plainVoice.chooseIndex(lineOf(plainVoice, "en-gb-x-gba-local"));
    plainVoice.chooseIndex(0);
    QCOMPARE(plainEngine.voice().name(), QString("en-us-x-iob-local"));
}

void VoiceEngineTest::aChoiceBeforeTheFirstReadyAddsNoQuestionLater()
{
    App app(googleVoices());
    app.voice.chooseIndex(0);  // the engine is not ready: _put waits, and the first list puts the voice
    app.engineReady();
    QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));
    app.engine.calls = {};
    speakOne(app.engine);
    QCOMPARE(app.engine.calls.total(), 0);
}

void VoiceEngineTest::qgcsOwnVoiceComesBackAfterMessages()
{
    App app(plainVoices());
    app.engineReady();
    QCOMPARE(app.speaking(), QString("en-us-x-iob-local"));  // QGC's: en_US, its first voice
    app.voice.chooseIndex(lineOf(app.voice, "en-gb-x-gba-local"));
    QCOMPARE(app.speaking(), QString("en-gb-x-gba-local"));
    // Messages in the other voice: that is not QGC's choice.
    speakOne(app.engine);
    speakOne(app.engine);
    app.voice.chooseIndex(0);
    QCOMPARE(app.speaking(), QString("en-us-x-iob-local"));
    QCOMPARE(app.engine.locale(), kUs);
}

void VoiceEngineTest::noIndianVoiceLeavesQgcsVoice()
{
    App app(plainVoices());
    app.engineReady();
    QCOMPARE(app.speaking(), QString("en-us-x-iob-local"));
    QVERIFY(app.voice.note().startsWith("This remote control has no Indian English voice"));
    // One more list at the next Ready (aFirstListWithoutAnIndianVoiceGetsOneMore), then no question.
    speakOne(app.engine);
    QCOMPARE(app.speaking(), QString("en-us-x-iob-local"));
    app.engine.calls = {};
    speakOne(app.engine);
    speakOne(app.engine);
    QCOMPARE(app.engine.calls.total(), 0);
    QCOMPARE(app.speaking(), QString("en-us-x-iob-local"));
}

void VoiceEngineTest::aFirstListWithoutAnIndianVoiceGetsOneMore()
{
    // Build 18 listed at every Ready, so an engine whose Indian voice came late got it at the next message.
    App app(plainVoices());
    app.engineReady();
    QCOMPARE(app.speaking(), QString("en-us-x-iob-local"));
    app.engine.setAllVoices(googleVoices());
    app.engine.calls = {};
    speakOne(app.engine);
    QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));
    QCOMPARE(app.voice.note(), QString());
    QCOMPARE(app.engine.calls.localeLists, 1);
    QCOMPARE(app.engine.calls.voiceLists, 4);
    for (const QLocale &locale : std::as_const(app.engine.calls.localesTried)) {
        QCOMPARE(locale.language(), QLocale::English);
    }
    app.engine.calls = {};
    speakOne(app.engine);
    QCOMPARE(app.engine.calls.total(), 0);
}

void VoiceEngineTest::aVoiceChosenWhileSpeakingIsPutAtTheNextReady()
{
    App app(googleVoices());
    app.engineReady();
    app.engine.setState(QTextToSpeech::Speaking);
    settle();
    app.voice.chooseIndex(lineOf(app.voice, "en-gb-x-gba-local"));
    QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));  // a message is going out
    app.engine.setState(QTextToSpeech::Ready);
    settle();
    QCOMPARE(app.speaking(), QString("en-gb-x-gba-local"));
    QCOMPARE(app.voice.current(), QString("en-gb-x-gba-local"));
    QCOMPARE(QSettings().value(QLatin1String(VamaVoice::kSettingsKey)).toString(), QString("en-gb-x-gba-local"));
}

void VoiceEngineTest::aChoiceWaitingIsPutOnce()
{
    App app(googleVoices());
    app.engineReady();
    app.engine.setState(QTextToSpeech::Speaking);
    settle();
    app.voice.chooseIndex(lineOf(app.voice, "en-gb-x-gba-local"));
    app.engine.setState(QTextToSpeech::Ready);
    settle();
    app.engine.calls = {};
    speakOne(app.engine);
    QCOMPARE(app.engine.calls.total(), 0);
}

void VoiceEngineTest::aVoiceChosenWhenReadyIsPutAtOnce()
{
    App app(googleVoices());
    app.engineReady();
    QSignalSpy changes(&app.voice, &VamaVoice::changed);
    app.voice.chooseIndex(lineOf(app.voice, "en-in-x-cxx#male_1-local"));
    QCOMPARE(app.speaking(), QString("en-in-x-cxx#male_1-local"));
    QCOMPARE(app.voice.current(), QString("en-in-x-cxx#male_1-local"));
    QCOMPARE(changes.count(), 1);
}

void VoiceEngineTest::thePanelAsksAgainWithoutAnIndianVoice()
{
    // The panel's note: add a voice in Android's settings, then choose it here.
    App app(plainVoices());
    app.engineReady();
    speakOne(app.engine);  // its one more list
    QCOMPARE(app.voice.choices().size(), 1 + 4);
    app.engine.setAllVoices(googleVoices());
    speakOne(app.engine);
    QCOMPARE(app.voice.choices().size(), 1 + 4);  // not by the messages
    app.voice.panelOpened();
    QCOMPARE(app.voice.choices().size(), 1 + 8);
    QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));
    QCOMPARE(app.voice.note(), QString());
    // QGC's own choice is still QGC's.
    app.voice.chooseIndex(lineOf(app.voice, "en-gb-x-gba-local"));
    QCOMPARE(app.speaking(), QString("en-gb-x-gba-local"));
    app.engine.calls = {};
    speakOne(app.engine);
    QCOMPARE(app.engine.calls.total(), 0);
}

void VoiceEngineTest::thePanelAsksNothingWithAnIndianVoice()
{
    App app(googleVoices());
    app.engineReady();
    app.engine.calls = {};
    QSignalSpy changes(&app.voice, &VamaVoice::changed);
    app.voice.panelOpened();
    QCOMPARE(app.engine.calls.total(), 0);
    QCOMPARE(changes.count(), 0);
}

void VoiceEngineTest::thePanelWaitsWhileTheEngineSpeaks()
{
    App app(plainVoices());
    app.engineReady();
    app.engine.setAllVoices(googleVoices());
    app.engine.setState(QTextToSpeech::Speaking);
    settle();
    app.engine.calls = {};
    app.voice.panelOpened();
    QCOMPARE(app.engine.calls.total(), 0);
    app.engine.setState(QTextToSpeech::Ready);
    settle();
    QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));
}

void VoiceEngineTest::anEngineWithoutEnglishVoicesIsAskedAgain()
{
    App app({voiceOf("de-de-x-nfh-local", kGerman)});
    app.engineReady();
    QVERIFY(app.voice.note().startsWith("The speech engine of this remote control offers no English voice"));
    // Its question is only the list of locales, no round through a locale that is not English.
    app.engine.calls = {};
    speakOne(app.engine);
    QCOMPARE(app.engine.calls.localeLists, 1);
    QCOMPARE(app.engine.calls.voiceLists, 0);
    QCOMPARE(app.engine.calls.localeSets, 0);
    // Android has the voices now.
    app.engine.setAllVoices(googleVoices());
    speakOne(app.engine);
    QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));
    QCOMPARE(app.voice.note(), QString());
    app.engine.calls = {};
    speakOne(app.engine);
    QCOMPARE(app.engine.calls.total(), 0);
}

void VoiceEngineTest::anotherEngineIsListedAgain()
{
    App app(googleVoices());
    app.engineReady();
    QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));
    // No American voice on it: QGC's slot leaves the locale, and takes that locale's first voice.
    app.engine.changeEngine({voiceOf("en-gb-x-new-local", kUk), voiceOf("en-au-x-afa-local", kAustralia)}, kUk);
    QCOMPARE(app.speaking(), QString("en-gb-x-new-local"));
    settle();
    QCOMPARE(app.voice.choices().size(), 1 + 2);
    QVERIFY(app.voice.note().startsWith("This remote control has no Indian English voice"));
    // QGC's choice on the new engine, not the locale and voice of the old one.
    app.voice.chooseIndex(lineOf(app.voice, "en-au-x-afa-local"));
    QCOMPARE(app.speaking(), QString("en-au-x-afa-local"));
    app.voice.chooseIndex(0);
    QCOMPARE(app.speaking(), QString("en-gb-x-new-local"));
    QCOMPARE(app.engine.locale(), kUk);
    // It gets its one more list too: its Indian voice comes by the next message.
    app.engine.setAllVoices(googleVoices());
    speakOne(app.engine);
    QCOMPARE(app.speaking(), QString::fromLatin1(kBestIndianVoice));
}

void VoiceEngineTest::noEngineAtAll()
{
    VamaVoice voice(nullptr);
    VamaVoiceEngine hook(&voice, nullptr);
    hook.attach(nullptr);
    voice.panelOpened();
    voice.chooseIndex(0);
    QCOMPARE(voice.current(), QString());
}

QTEST_GUILESS_MAIN(VoiceEngineTest)
#include "voice_engine_test.moc"
