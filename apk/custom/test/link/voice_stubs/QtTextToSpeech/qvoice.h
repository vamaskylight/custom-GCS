#pragma once

// Stand-in for Qt's QVoice: the desktop Qt kit of the host tests has no Qt Speech. See qtexttospeech.h.

#include <QtCore/QLocale>
#include <QtCore/QString>

class QVoice
{
public:
    enum Gender { Male, Female, Unknown };

    QVoice() = default;
    QVoice(const QString &name, const QLocale &locale, Gender gender)
        : _name(name)
        , _locale(locale)
        , _gender(gender)
    {
    }

    QString name() const { return _name; }
    QLocale locale() const { return _locale; }
    Gender gender() const { return _gender; }

    friend bool operator==(const QVoice &a, const QVoice &b)
    {
        return a._name == b._name && a._locale == b._locale && a._gender == b._gender;
    }
    friend bool operator!=(const QVoice &a, const QVoice &b) { return !(a == b); }

private:
    QString _name;
    QLocale _locale;
    Gender _gender = Unknown;
};
