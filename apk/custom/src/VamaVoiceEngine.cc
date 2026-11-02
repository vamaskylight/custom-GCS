#include "VamaVoiceEngine.h"

#include "VamaVoice.h"

#include <QtTextToSpeech/QTextToSpeech>

#include <string>
#include <utility>
#include <vector>

namespace {

vama::voice::Gender genderOf(QVoice::Gender gender)
{
    switch (gender) {
    case QVoice::Female:
        return vama::voice::Gender::Female;
    case QVoice::Male:
        return vama::voice::Gender::Male;
    default:
        return vama::voice::Gender::Unknown;
    }
}

} // namespace

VamaVoiceEngine::VamaVoiceEngine(VamaVoice *voice, QObject *parent)
    : QObject(parent)
    , _voice(voice)
{
}

void VamaVoiceEngine::attach(QTextToSpeech *engine)
{
    if (_engine) {
        disconnect(_engine, nullptr, this, nullptr);
    }
    _engine = engine;
    if (!engine || !_voice) {
        return;
    }
    _voice->setApplier([this](int index) { return _put(index); });
    _voice->setLister([this]() {
        _voicesListed = false;
        _engineReady();
    });
    // Queued: AudioOutput sets its own voice in direct slots of the same signals, so this comes after it.
    (void) connect(engine, &QTextToSpeech::stateChanged, this, [this](QTextToSpeech::State state) {
        if (state == QTextToSpeech::State::Ready) {
            _engineReady();
        }
    }, Qt::QueuedConnection);
    (void) connect(engine, &QTextToSpeech::engineChanged, this, [this]() {
        _qgcChoiceKnown = false;
        _voicesListed = false;
        _listedBefore = false;
        _engineReady();
    }, Qt::QueuedConnection);
    _engineReady();
}

void VamaVoiceEngine::_engineReady()
{
    if (!_engine || _engine->state() != QTextToSpeech::State::Ready) {
        return;
    }
    // This runs on the main thread after every spoken message: only what is due, and normally nothing
    // that goes to Android (B23).
    if (!_qgcChoiceKnown) {
        // QGC has just set its own locale and voice, once for this engine: they speak when no Indian
        // English voice is there. Read later, they would be our own voice.
        _qgcLocale = _engine->locale();
        _qgcVoice = _engine->voice();
        _qgcChoiceKnown = true;
    }
    if (!_voicesListed) {
        _listVoices();
    } else if (_putWaiting) {
        _putWaiting = false;
        _voice->applyAgain();
    }
}

void VamaVoiceEngine::_listVoices()
{
    // By locale: findVoices(QLocale::English) has Qt go through every locale of the engine, each with
    // setLocale and all the RC's voices by JNI (QTextToSpeech::allVoices, Android engine).
    _engineVoices.clear();
    const QList<QLocale> locales = _engine->availableLocales();
    for (const QLocale &locale : locales) {
        if (locale.language() == QLocale::English) {
            _engineVoices += _engine->findVoices(locale);
        }
    }
    std::vector<vama::voice::Voice> voices;
    voices.reserve(static_cast<size_t>(_engineVoices.size()));
    for (const QVoice &voice : std::as_const(_engineVoices)) {
        voices.push_back(vama::voice::Voice{voice.name().toStdString(),
                                            QLocale::languageToCode(voice.locale().language()).toStdString(),
                                            QLocale::territoryToCode(voice.locale().territory()).toStdString(),
                                            genderOf(voice.gender())});
    }
    // An engine that offers none now is asked again at its next Ready: this is not the long search.
    // One without an Indian English voice once more, in case its voices were not all there yet.
    const bool indian = vama::voice::pick(voices, std::string()) >= 0;
    _voicesListed = !voices.empty() && (indian || _listedBefore);
    _listedBefore = true;
    _putWaiting = false;
    _voice->setVoices(voices, voices.empty());
}

QString VamaVoiceEngine::_put(int index)
{
    if (!_engine || _engine->state() != QTextToSpeech::State::Ready) {
        _putWaiting = true;  // speaking: _engineReady puts it when the engine is ready again
        return QString();
    }
    const bool ours = index >= 0 && index < _engineVoices.size();
    if (!ours && _qgcVoice.name().isEmpty()) {
        return _engine->voice().name();  // QGC's own choice is not known yet: the engine stays as it is
    }
    const QLocale locale = ours ? _engineVoices.at(index).locale() : _qgcLocale;
    const QVoice voice = ours ? _engineVoices.at(index) : _qgcVoice;
    if (_engine->locale() != locale) {
        _engine->setLocale(locale);
    }
    if (!voice.name().isEmpty() && _engine->voice() != voice) {
        _engine->setVoice(voice);
    }
    return _engine->voice().name();
}
