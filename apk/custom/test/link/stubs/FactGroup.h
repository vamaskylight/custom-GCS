#pragma once

// Host-test stand-in for QGC's FactGroup: facts looked up by name.

#include <QtCore/QMap>
#include <QtCore/QString>

#include <memory>

#include "Fact.h"

class FactGroup
{
public:
    bool factExists(const QString &name) const { return _facts.contains(name); }
    Fact *getFact(const QString &name) const { return _facts.value(name).get(); }

    void set(const QString &name, const QVariant &value)
    {
        if (!_facts.contains(name)) {
            _facts.insert(name, std::make_shared<Fact>());
        }
        _facts.value(name)->setRawValue(value);
    }
    void remove(const QString &name) { _facts.remove(name); }

private:
    QMap<QString, std::shared_ptr<Fact>> _facts;
};
