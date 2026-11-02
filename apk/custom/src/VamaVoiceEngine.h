#pragma once

#include <QtCore/QList>
#include <QtCore/QLocale>
#include <QtCore/QObject>
#include <QtCore/QPointer>
#include <QtTextToSpeech/QVoice>

class QTextToSpeech;
class VamaVoice;

/// The speech engine's side of VamaVoice, in the app: the English voices of QGC's own QTextToSpeech
/// (AudioOutput's), handed to VamaVoice, and the voice VamaVoice picks put on that engine.
///
/// QGC sets its own locale and voice once for each engine, in direct slots of the engine's first Ready
/// and of engineChanged (AudioOutput::init, _finishInit): this listens queued, so it comes after QGC.
/// Without an Indian English voice, QGC's own choice speaks.
///
/// The engine says Ready after every spoken message, on the main thread, and every list of voices is
/// slow on Android (JNI calls for all the RC's voices). Test build 18 listed them on every Ready, the
/// likely cause of its buttons answering 3 to 8 s late (B23, not measured on an RC). So they are listed
/// once for each engine, a second time at the next Ready if that found no Indian English voice, and
/// again only when the menu's panel asks (VamaVoice::panelOpened).
class VamaVoiceEngine : public QObject
{
    Q_OBJECT

public:
    VamaVoiceEngine(VamaVoice *voice, QObject *parent);

    /// QGC's engine (AudioOutput's QTextToSpeech child). CustomPlugin attaches it once the app runs,
    /// when QGC has set the engine up.
    void attach(QTextToSpeech *engine);

private:
    void _engineReady();
    void _listVoices();
    QString _put(int index);

    VamaVoice *_voice;
    QPointer<QTextToSpeech> _engine;
    QList<QVoice> _engineVoices;  // in the order of the voices handed to VamaVoice
    QLocale _qgcLocale;           // QGC's own choice on this engine
    QVoice _qgcVoice;
    bool _qgcChoiceKnown = false;
    bool _voicesListed = false;
    bool _listedBefore = false;   // for this engine
    bool _putWaiting = false;     // a voice came while the engine was speaking
};
