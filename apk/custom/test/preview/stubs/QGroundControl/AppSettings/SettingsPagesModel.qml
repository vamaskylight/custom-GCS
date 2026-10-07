// Stand-in for QGC's SettingsPagesModel.qml, which QGC's build makes from
// src/AppSettings/pages/SettingsPages.json (the made file is in the APK build
// folder, qml/QGroundControl/AppSettings/). Same roles and page order, with
// what the release APK shows: no PX4 page, no debug pages. Only General has
// sections, so its button gets the arrow, like QGC's.

import QtQml.Models
import QtQml

ListModel {
    ListElement {
        name: qsTranslate("SettingsPages.json", "General")
        nameKey: "General"
        url: "qrc:/qml/QGroundControl/AppSettings/GeneralSettings.qml"
        iconUrl: "qrc:/res/QGCLogoWhite.svg"
        sections: function() {
            return [
                { index: 0, name: "General", searchTerms: ["general language"], visible: true },
                { index: 1, name: "Vehicle Preferences", searchTerms: ["vehicle preferences firmware"], visible: true },
                { index: 2, name: "Units", searchTerms: ["units metric imperial"], visible: true }
            ]
        }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "Fly View")
        nameKey: "Fly View"
        url: "qrc:/qml/QGroundControl/AppSettings/FlyViewSettings.qml"
        iconUrl: "qrc:/qmlimages/PaperPlane.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "3D View")
        nameKey: "3D View"
        url: "qrc:/qml/QGroundControl/AppSettings/Viewer3DSettings.qml"
        iconUrl: "qrc:/qml/QGroundControl/Viewer3D/City3DMapIcon.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "Plan View")
        nameKey: "Plan View"
        url: "qrc:/qml/QGroundControl/AppSettings/PlanViewSettings.qml"
        iconUrl: "qrc:/qmlimages/Plan.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: "Divider"
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "ADSB Server")
        nameKey: "ADSB Server"
        url: "qrc:/qml/QGroundControl/AppSettings/ADSBServerSettings.qml"
        iconUrl: "qrc:/InstrumentValueIcons/airplane.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "Comm Links")
        nameKey: "Comm Links"
        url: "qrc:/qml/QGroundControl/AppSettings/CommLinksSettings.qml"
        iconUrl: "qrc:/InstrumentValueIcons/usb.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "App Logging")
        nameKey: "App Logging"
        url: "qrc:/qml/QGroundControl/AppSettings/LoggingSettings.qml"
        iconUrl: "qrc:/InstrumentValueIcons/conversation.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "App Log Viewer")
        nameKey: "App Log Viewer"
        url: "qrc:/qml/QGroundControl/AppSettings/AppLogging.qml"
        iconUrl: "qrc:/InstrumentValueIcons/conversation.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "Maps")
        nameKey: "Maps"
        url: "qrc:/qml/QGroundControl/AppSettings/MapsSettings.qml"
        iconUrl: "qrc:/InstrumentValueIcons/globe.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "NTRIP/RTK")
        nameKey: "NTRIP/RTK"
        url: "qrc:/qml/QGroundControl/AppSettings/NTRIPSettings.qml"
        iconUrl: "qrc:/InstrumentValueIcons/globe.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "PX4 Log Transfer")
        nameKey: "PX4 Log Transfer"
        url: "qrc:/qml/QGroundControl/AppSettings/PX4LogTransferSettings.qml"
        iconUrl: "qrc:/InstrumentValueIcons/inbox-download.svg"
        sections: function() { return [] }
        pageVisible: function() { return false }  // PX4 is off in the VAMA build
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "Remote ID")
        nameKey: "Remote ID"
        url: "qrc:/qml/QGroundControl/AppSettings/RemoteIDSettings.qml"
        iconUrl: "qrc:/qmlimages/RidIconManNoID.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "Telemetry")
        nameKey: "Telemetry"
        url: "qrc:/qml/QGroundControl/AppSettings/TelemetrySettings.qml"
        iconUrl: "qrc:/InstrumentValueIcons/drone.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "Video")
        nameKey: "Video"
        url: "qrc:/qml/QGroundControl/AppSettings/VideoSettings.qml"
        iconUrl: "qrc:/InstrumentValueIcons/camera.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: "Divider"
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "Help")
        nameKey: "Help"
        url: "qrc:/qml/QGroundControl/AppSettings/HelpSettings.qml"
        iconUrl: "qrc:/InstrumentValueIcons/question.svg"
        sections: function() { return [] }
        pageVisible: function() { return true }
    }

    ListElement {
        name: "Divider"
    }

    // The debug pages: ScreenTools.isDebug, false in the release APK.
    ListElement {
        name: qsTranslate("SettingsPages.json", "Mock Link")
        nameKey: "Mock Link"
        url: "qrc:/qml/QGroundControl/AppSettings/MockLink.qml"
        iconUrl: "qrc:/InstrumentValueIcons/drone.svg"
        sections: function() { return [] }
        pageVisible: function() { return false }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "Debug")
        nameKey: "Debug"
        url: "qrc:/qml/QGroundControl/AppSettings/DebugWindow.qml"
        iconUrl: "qrc:/InstrumentValueIcons/bug.svg"
        sections: function() { return [] }
        pageVisible: function() { return false }
    }

    ListElement {
        name: qsTranslate("SettingsPages.json", "Palette Test")
        nameKey: "Palette Test"
        url: "qrc:/qml/QGroundControl/AppSettings/QmlTest.qml"
        iconUrl: "qrc:/InstrumentValueIcons/photo.svg"
        sections: function() { return [] }
        pageVisible: function() { return false }
    }
}
