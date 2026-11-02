#pragma once

#include <QtCore/QObject>
#include <QtCore/QStringList>
#include <QtQml/qqmlregistration.h>

#include <functional>
#include <vector>

#include "VoicePick.h"

class QJSEngine;
class QQmlEngine;

/// The voice the app speaks with (the client, Nish 2026-10-30: Indian English, female).
///
/// QGC's AudioOutput speaks with a QTextToSpeech of its own: once that engine is ready it sets
/// American English and the engine's first voice (AudioOutput::_applyEngineSettings), and leaves it.
/// VamaVoiceEngine.h, in the app, hands this the engine's English voices after that and puts on the
/// engine the voice picked here (VoicePick.h), or the one chosen in the app menu
/// (VamaVoicePanel.qml), saved for the next start. Android tells a voice's gender only in its name
/// ("#female", QTextToSpeech's Android engine), which newer engines leave out, so the menu offers the
/// English voices to choose from, and QGC's Test there speaks with the one in use. No Qt Speech here:
/// the screen preview's Qt has none, and checks this with made-up voices.
class VamaVoice : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

    /// For the menu's list: "Automatic: ..." first, then the voices of VoicePick's listOrder.
    Q_PROPERTY(QStringList choices READ choices NOTIFY changed)
    /// The line of the list in use: 0 automatic, else the chosen voice's line.
    Q_PROPERTY(int choiceIndex READ choiceIndex NOTIFY changed)
    /// The name of the voice speaking now ("" before the engine is ready).
    Q_PROPERTY(QString current READ current NOTIFY changed)
    /// A line for the menu when something is missing ("" when all is well).
    Q_PROPERTY(QString note READ note NOTIFY changed)

public:
    /// No default for the parent, on purpose: QML makes a singleton with a constructor that needs no
    /// argument when there is one, instead of using create() (see SkydroidLink.h, tasks X and V).
    explicit VamaVoice(QObject *parent);

    /// The app's one VamaVoice. QML gets it too (create()).
    static VamaVoice *instance();
    static VamaVoice *create(QQmlEngine *qmlEngine, QJSEngine *jsEngine);

    QStringList choices() const;
    int choiceIndex() const;
    QString current() const { return _current; }
    QString note() const;

    /// A line of choices: 0 automatic, else that voice. Saved for the next start.
    Q_INVOKABLE void chooseIndex(int index);
    /// The menu's panel opens. Without an Indian English voice the speech engine is asked again, as the
    /// panel's note says to add one in Android's settings and choose it here. With one, it is not asked:
    /// that takes a moment on the main thread (VamaVoiceEngine.h).
    Q_INVOKABLE void panelOpened();

    /// The speech engine's English voices (VamaVoiceEngine in the app, made-up ones in tests), and
    /// whether the engine has none at all. Picks again.
    void setVoices(const std::vector<vama::voice::Voice> &voices, bool engineHasNoVoices = false);
    /// Puts a voice on the speech engine: an index into the voices, or -1 for QGC's own choice; returns
    /// the name of the voice speaking then. Without one (tests) the picked voice counts as speaking.
    void setApplier(std::function<QString(int)> applier) { _applier = std::move(applier); }
    /// Asks the speech engine for its voices again, for panelOpened.
    void setLister(std::function<void()> lister) { _lister = std::move(lister); }
    /// Puts the voice on the speech engine again, after it was busy speaking.
    void applyAgain() { _apply(); }

    static constexpr const char *kSettingsKey = "VamaVoice/chosen";

signals:
    void changed();

private:
    void _apply();
    QString _describe(int index) const;

    std::vector<vama::voice::Voice> _voices;
    std::vector<int> _order;  // VoicePick's listOrder of _voices
    QString _chosen;          // the voice chosen in the menu, "" for automatic
    QString _current;
    bool _engineHasNoVoices = false;
    std::function<QString(int)> _applier;
    std::function<void()> _lister;
};
