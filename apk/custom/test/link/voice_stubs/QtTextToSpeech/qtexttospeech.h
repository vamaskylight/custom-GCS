#pragma once

// Stand-in for Qt's QTextToSpeech with its Android engine: the desktop Qt kit of the host tests has no
// Qt Speech. The real VamaVoiceEngine.cc compiles against this, and the APK's compile check against Qt's.
//
// findVoices and allVoices are Qt's 6.11.1 code (qtexttospeech.h, qtexttospeech.cpp), as those decide
// how often the engine is asked. The engine's part is what the Android engine does
// (qtexttospeech_android.cpp, QtTextToSpeech.java): setLocale fails for a locale without voices, and
// availableVoices gives only the voices of the current locale. `calls` counts every question that goes
// to Android by JNI. state() is not one: Qt keeps it here.

#include <QtCore/QList>
#include <QtCore/QLocale>
#include <QtCore/QObject>
#include <QtCore/QSignalBlocker>
#include <QtCore/QString>

#include <tuple>
#include <type_traits>

#include "qvoice.h"

class QTextToSpeech : public QObject
{
    Q_OBJECT

public:
    enum State { Ready, Speaking, Paused, Error, Synthesizing };
    Q_ENUM(State)

    struct Calls {
        int localeLists = 0;
        int voiceLists = 0;
        int localeReads = 0;
        int voiceReads = 0;
        int localeSets = 0;
        int voiceSets = 0;
        QList<QLocale> localesTried;

        int total() const { return localeLists + voiceLists + localeReads + voiceReads + localeSets + voiceSets; }
    };

    explicit QTextToSpeech(QObject *parent = nullptr)
        : QObject(parent)
    {
    }

    State state() const { return _state; }

    QList<QLocale> availableLocales() const
    {
        ++calls.localeLists;
        QList<QLocale> locales;
        for (const QVoice &voice : _all) {
            if (!locales.contains(voice.locale())) {
                locales << voice.locale();
            }
        }
        return locales;
    }

    QList<QVoice> availableVoices() const
    {
        ++calls.voiceLists;
        QList<QVoice> voices;
        for (const QVoice &voice : _all) {
            if (voice.locale() == _locale) {
                voices << voice;
            }
        }
        return voices;
    }

    QLocale locale() const
    {
        ++calls.localeReads;
        return _locale;
    }

    QVoice voice() const
    {
        ++calls.voiceReads;
        return _voice;
    }

    void setLocale(const QLocale &locale)
    {
        if (this->locale() != locale && _engineSetLocale(locale) && !signalsBlocked()) {
            emit localeChanged(locale);
        }
    }

    void setVoice(const QVoice &voice)
    {
        ++calls.voiceSets;
        if (_all.contains(voice)) {
            _voice = voice;
            _locale = voice.locale();  // Android's setVoice takes the voice's language with it
        }
    }

    template <typename... Args>
    QList<QVoice> findVoices(Args &&...args) const
    {
        QLocale locale;
        QLocale *plocale = nullptr;
        if constexpr (std::disjunction_v<std::is_same<std::remove_cv_t<std::remove_reference_t<Args>>, QLocale>...>) {
            locale = std::get<QLocale>(std::make_tuple(args...));
            plocale = &locale;
        }
        auto voices = allVoices(plocale);
        if constexpr (sizeof...(args) > 0) {
            voices.removeIf([&](const QVoice &voice) -> bool { return !voiceMatches(voice, args...); });
        }
        return voices;
    }

    // For the test.
    mutable Calls calls;

    void setAllVoices(const QList<QVoice> &voices) { _all = voices; }

    void setState(State state)
    {
        if (_state != state) {
            _state = state;
            emit stateChanged(state);
        }
    }

    void changeEngine(const QList<QVoice> &voices, const QLocale &locale)
    {
        _all = voices;
        _locale = locale;
        _voice = QVoice();
        emit engineChanged(QStringLiteral("another"));
    }

signals:
    void stateChanged(QTextToSpeech::State state);
    void engineChanged(const QString &engine);
    void localeChanged(const QLocale &locale);

private:
    QList<QVoice> allVoices(const QLocale *locale) const
    {
        const QVoice oldVoice = this->voice();

        QList<QVoice> voices;
        QSignalBlocker blockSignals(const_cast<QTextToSpeech *>(this));
        const QList<QLocale> allLocales = locale ? QList<QLocale>{*locale} : availableLocales();
        for (const auto &l : allLocales) {
            if (this->locale() != l) {
                const_cast<QTextToSpeech *>(this)->_engineSetLocale(l);
            }
            voices << availableVoices();
        }

        // reset back to old voice, which will have changed when we changed the engine's locale.
        if (this->voice() != oldVoice) {
            const_cast<QTextToSpeech *>(this)->setVoice(oldVoice);
        }
        return voices;
    }

    template <typename Arg>
    bool voiceMatches(const QVoice &voice, Arg &&arg) const
    {
        using ArgType = std::remove_cv_t<std::remove_reference_t<Arg>>;
        if constexpr (std::is_same_v<ArgType, QLocale>) {
            return voice.locale() == arg;
        } else {
            static_assert(std::is_same_v<ArgType, QLocale::Language>, "not in this stand-in");
            return voice.locale().language() == arg;
        }
    }

    bool _engineSetLocale(const QLocale &locale)
    {
        ++calls.localeSets;
        calls.localesTried << locale;
        for (const QVoice &voice : std::as_const(_all)) {
            if (voice.locale() == locale) {
                _locale = locale;
                _voice = voice;  // the voice follows the locale
                return true;
            }
        }
        return false;
    }

    State _state = Error;  // Android: until the service is bound
    QList<QVoice> _all;
    QLocale _locale = QLocale(QLocale::English, QLocale::UnitedStates);
    QVoice _voice;
};
