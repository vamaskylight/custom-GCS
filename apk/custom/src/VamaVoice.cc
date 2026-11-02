#include "VamaVoice.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QLocale>
#include <QtCore/QSettings>
#include <QtQml/QJSEngine>

#include <type_traits>

VamaVoice::VamaVoice(QObject *parent)
    : QObject(parent)
    , _chosen(QSettings().value(QLatin1String(kSettingsKey)).toString())
{
}

VamaVoice *VamaVoice::instance()
{
    static VamaVoice *voice = new VamaVoice(QCoreApplication::instance());
    return voice;
}

VamaVoice *VamaVoice::create(QQmlEngine *qmlEngine, QJSEngine *jsEngine)
{
    Q_UNUSED(qmlEngine);
    Q_UNUSED(jsEngine);
    VamaVoice *voice = instance();
    QJSEngine::setObjectOwnership(voice, QJSEngine::CppOwnership);
    return voice;
}

// QML calls create() only for a singleton that cannot be made without an argument (see SkydroidLink.cc).
static_assert(!std::is_default_constructible_v<VamaVoice>,
              "QML would make its own VamaVoice instead of using VamaVoice::create()");

QStringList VamaVoice::choices() const
{
    QStringList lines;
    const int automatic = vama::voice::pick(_voices, std::string());
    if (automatic >= 0) {
        lines << tr("Automatic: %1").arg(_describe(automatic));
    } else if (!_current.isEmpty()) {
        lines << tr("Automatic: %1 (no Indian English voice)").arg(_current);
    } else {
        lines << tr("Automatic");
    }
    for (const int index : _order) {
        lines << _describe(index);
    }
    return lines;
}

int VamaVoice::choiceIndex() const
{
    if (_chosen.isEmpty()) {
        return 0;
    }
    for (size_t line = 0; line < _order.size(); ++line) {
        if (QString::fromStdString(_voices[static_cast<size_t>(_order[line])].name) == _chosen) {
            return static_cast<int>(line) + 1;
        }
    }
    return 0;  // the chosen voice is not on this engine: the automatic choice speaks
}

QString VamaVoice::note() const
{
    if (_engineHasNoVoices) {
        return tr("The speech engine of this remote control offers no English voice.");
    }
    if (_voices.empty()) {
        return QString();  // the engine is not ready yet
    }
    bool indian = false;
    bool genderKnown = false;
    for (const vama::voice::Voice &voice : _voices) {
        if (vama::voice::isIndianEnglish(voice)) {
            indian = true;
            genderKnown = genderKnown || voice.gender != vama::voice::Gender::Unknown;
        }
    }
    if (!indian) {
        return tr("This remote control has no Indian English voice. Add one in Android's text-to-speech "
                  "settings (English, India), then choose it here.");
    }
    if (!genderKnown) {
        return tr("Android does not say which of these voices are female: choose one, then press Test to hear it.");
    }
    return QString();
}

void VamaVoice::chooseIndex(int index)
{
    QString chosen;
    if (index > 0 && index <= static_cast<int>(_order.size())) {
        chosen = QString::fromStdString(_voices[static_cast<size_t>(_order[static_cast<size_t>(index - 1)])].name);
    }
    _chosen = chosen;
    QSettings settings;
    if (chosen.isEmpty()) {
        settings.remove(QLatin1String(kSettingsKey));
    } else {
        settings.setValue(QLatin1String(kSettingsKey), chosen);
    }
    _apply();
}

void VamaVoice::panelOpened()
{
    if (_lister && vama::voice::pick(_voices, std::string()) < 0) {
        _lister();
    }
}

void VamaVoice::setVoices(const std::vector<vama::voice::Voice> &voices, bool engineHasNoVoices)
{
    _voices = voices;
    _engineHasNoVoices = engineHasNoVoices;
    _order = vama::voice::listOrder(_voices);
    _apply();
}

void VamaVoice::_apply()
{
    const int index = vama::voice::pick(_voices, _chosen.toStdString());
    if (_applier) {
        _current = _applier(index);
    } else {
        _current = index >= 0 ? QString::fromStdString(_voices[static_cast<size_t>(index)].name) : QString();
    }
    emit changed();
}

QString VamaVoice::_describe(int index) const
{
    const vama::voice::Voice &voice = _voices[static_cast<size_t>(index)];
    const QLocale locale(QStringLiteral("%1_%2").arg(QString::fromStdString(voice.language).toLower(),
                                                     QString::fromStdString(voice.territory).toUpper()));
    QString text = QStringLiteral("%1 (%2)").arg(QLocale::languageToString(locale.language()),
                                                 QLocale::territoryToString(locale.territory()));
    if (voice.gender == vama::voice::Gender::Female) {
        text += tr(", female");
    } else if (voice.gender == vama::voice::Gender::Male) {
        text += tr(", male");
    }
    if (vama::voice::needsNetwork(voice.name)) {
        text += tr(", needs the internet");
    }
    return text + QStringLiteral(", ") + QString::fromStdString(voice.name);
}
