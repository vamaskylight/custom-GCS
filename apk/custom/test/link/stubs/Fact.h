#pragma once

// Host-test stand-in for QGC's Fact: only what SkydroidLink uses.

#include <QtCore/QVariant>

class Fact
{
public:
    explicit Fact(const QVariant &value = {}) : _value(value) {}
    QVariant rawValue() const { return _value; }
    void setRawValue(const QVariant &value) { _value = value; }

private:
    QVariant _value;
};
