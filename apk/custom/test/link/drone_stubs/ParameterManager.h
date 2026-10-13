#pragma once

// Host-test stand-in for QGC's ParameterManager: the drone's parameters by name.

#include <QtCore/QMap>
#include <QtCore/QString>

#include "Fact.h"

class ParameterManager
{
public:
    static constexpr int defaultComponentId = -1;

    bool parameterExists(int componentId, const QString &name) const
    {
        return componentId == defaultComponentId && _facts.contains(name);
    }
    Fact *getParameter(int componentId, const QString &name)
    {
        Q_UNUSED(componentId);
        return &_facts[name];
    }

    void set(const QString &name, const QVariant &value) { _facts[name].setRawValue(value); }
    void remove(const QString &name) { _facts.remove(name); }

private:
    QMap<QString, Fact> _facts;
};
