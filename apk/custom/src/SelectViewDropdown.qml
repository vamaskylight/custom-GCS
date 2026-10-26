// VAMA GCS: replaces QGC's Toolbar/SelectViewDropdown.qml (the app menu), see custom.qrc.
// Copied from QGC v5.1.5. Changes, marked "VAMA":
//  - The Settings button shows the VAMA mark, not QGC's logo. QGC tints menu
//    icons through its "coloredsvg" image provider, which reads the file
//    directly, so the /Custom/res override that changes QGC's other logos
//    does not reach this one.
//  - The version lines name VAMA GCS and our test build, not "VAMA-GCS Daily
//    Version" and QGC's commit date. The hidden Advanced Mode tap stays.
//  - Advanced Mode asks for the admin password (task Z): the tap shows
//    VamaAdminPasswordPanel.qml in place of the buttons. QGC's own question
//    (advancedModeOnConfirmation) is not asked. The page's content is in a
//    ColumnLayout for that, around QGC's GridLayout.
//  - "Camera settings", and in Advanced Mode "Cameras" (the camera editor),
//    task AG: the client looked for them here. The fly view opens them.
// When moving to a new QGC release, compare with QGC's new file.

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

ToolIndicatorPage {
    id: root

    property real _toolButtonHeight: ScreenTools.defaultFontPixelHeight * 3

    contentComponent: Component {
        // VAMA (task Z): the menu, or in its place the admin password for Advanced Mode.
        ColumnLayout {
            id:      menuContent
            spacing: 0

            property bool askAdminPassword: false

            GridLayout {
                visible: !menuContent.askAdminPassword  // VAMA (task Z)
                columns: 2
                columnSpacing: ScreenTools.defaultFontPixelWidth
                rowSpacing: columnSpacing

                SubMenuButton {
                    objectName: "toolbar_viewFly"
                    implicitHeight: root._toolButtonHeight
                    Layout.fillWidth: true
                    text: qsTr("Fly")
                    imageResource: "/res/FlyingPaperPlane.svg"
                    onClicked: {
                        if (mainWindow.allowViewSwitch()) {
                            mainWindow.closeIndicatorDrawer()
                            mainWindow.showFlyView()
                        }
                    }
                }

                SubMenuButton {
                    objectName: "toolbar_viewPlan"
                    implicitHeight: root._toolButtonHeight
                    Layout.fillWidth: true
                    text: qsTr("Plan")
                    imageResource: "/qmlimages/Plan.svg"
                    onClicked: {
                        if (mainWindow.allowViewSwitch()) {
                            mainWindow.closeIndicatorDrawer()
                            mainWindow.showPlanView()
                        }
                    }
                }

                SubMenuButton {
                    objectName: "toolbar_viewAnalyze"
                    implicitHeight: root._toolButtonHeight
                    Layout.fillWidth: true
                    text: qsTr("Analyze")
                    imageResource: "/qmlimages/Analyze.svg"
                    visible: QGroundControl.corePlugin.showAdvancedUI
                    onClicked: {
                        if (mainWindow.allowViewSwitch()) {
                            mainWindow.closeIndicatorDrawer()
                            mainWindow.showAnalyzeTool()
                        }
                    }
                }

                SubMenuButton {
                    id: setupButton
                    objectName: "toolbar_viewConfigure"
                    implicitHeight: root._toolButtonHeight
                    Layout.fillWidth: true
                    text: qsTr("Configure")
                    imageResource: "/res/GearWithPaperPlane.svg"
                    onClicked: {
                        if (mainWindow.allowViewSwitch()) {
                            mainWindow.closeIndicatorDrawer()
                            mainWindow.showVehicleConfig()
                        }
                    }
                }

                SubMenuButton {
                    id: settingsButton
                    objectName: "toolbar_viewSettings"
                    implicitHeight: root._toolButtonHeight
                    Layout.fillWidth: true
                    text: qsTr("Settings")
                    imageResource: "/custom/img/vama_logo_white.svg" // VAMA
                    visible: !QGroundControl.corePlugin.options.combineSettingsAndSetup
                    onClicked: {
                        if (mainWindow.allowViewSwitch()) {
                            mainWindow.closeIndicatorDrawer()
                            mainWindow.showSettingsTool()
                        }
                    }
                }

                // VAMA (task AG): the camera windows belong to the fly view, which lives as long as
                // the app. This menu only asks for them (CustomPlugin::requestCameraWindow) before it
                // closes: a window made here would stop working with the menu.
                SubMenuButton {
                    objectName: "vamaMenuCameraSettings"
                    implicitHeight: root._toolButtonHeight
                    Layout.fillWidth: true
                    text: qsTr("Camera settings")
                    imageResource: "/custom/img/vama_settings.svg"
                    onClicked: {
                        if (mainWindow.allowViewSwitch()) {
                            QGroundControl.corePlugin.requestCameraWindow("settings")
                            mainWindow.closeIndicatorDrawer()
                            mainWindow.showFlyView()
                        }
                    }
                }

                SubMenuButton {
                    objectName: "vamaMenuCameras"
                    implicitHeight: root._toolButtonHeight
                    Layout.fillWidth: true
                    text: qsTr("Cameras")
                    imageResource: "/custom/img/vama_photo.svg"
                    visible: QGroundControl.corePlugin.showAdvancedUI
                    onClicked: {
                        if (mainWindow.allowViewSwitch()) {
                            QGroundControl.corePlugin.requestCameraWindow("editor")
                            mainWindow.closeIndicatorDrawer()
                            mainWindow.showFlyView()
                        }
                    }
                }

                SubMenuButton {
                    id: closeButton
                    objectName: "toolbar_viewClose"
                    implicitHeight: root._toolButtonHeight
                    Layout.fillWidth: true
                    text: qsTr("Close")
                    imageResource: "/res/OpenDoor.svg"
                    onClicked: {
                        if (mainWindow.allowViewSwitch()) {
                            mainWindow.closeIndicatorDrawer()
                            // Route through the window close handler so the unsaved
                            // mission / pending parameter / active connection checks
                            // run, matching the desktop window-close behavior.
                            mainWindow.close()
                        }
                    }
                }

                ColumnLayout {
                    id: versionColumnLayout
                    Layout.fillWidth: true
                    Layout.columnSpan: 2
                    spacing: 0

                    QGCLabel {
                        id: versionLabel
                        Layout.fillWidth: true
                        horizontalAlignment: Text.AlignHCenter
                        text: qsTr("VAMA GCS") // VAMA
                        font.pointSize: ScreenTools.smallFontPointSize
                        wrapMode: QGCLabel.WordWrap
                    }

                    // VAMA: our build. Change it for every APK sent out.
                    QGCLabel {
                        Layout.fillWidth: true
                        horizontalAlignment: Text.AlignHCenter
                        text: qsTr("Test build 14, 26 October 2026")
                        font.pointSize: ScreenTools.smallFontPointSize
                        wrapMode: QGCLabel.WordWrap
                    }

                    // VAMA: the QGC release it is built on, in place of QGC's commit date.
                    // Always shown, as it carries the hidden Advanced Mode tap.
                    QGCLabel {
                        Layout.fillWidth: true
                        horizontalAlignment: Text.AlignHCenter
                        text: QGroundControl.qgcVersion
                        font.pointSize: ScreenTools.smallFontPointSize
                        wrapMode: QGCLabel.WrapAnywhere

                        QGCMouseArea {
                            objectName: "vamaAdvancedModeTap" // VAMA: for the screen preview
                            anchors.topMargin: -(parent.y - versionLabel.y)
                            anchors.fill: parent

                            onClicked: (mouse) => {
                                if (mouse.modifiers & Qt.ControlModifier) {
                                    QGroundControl.corePlugin.showTouchAreas = !QGroundControl.corePlugin.showTouchAreas
                                    showTouchAreasNotification.open()
                                } else if (ScreenTools.isMobile || mouse.modifiers & Qt.ShiftModifier) {
                                    if (!QGroundControl.corePlugin.showAdvancedUI) {
                                        // VAMA (task Z): the admin password, here in the menu.
                                        menuContent.askAdminPassword = true
                                    } else {
                                        mainWindow.closeIndicatorDrawer()
                                        advancedModeOffConfirmation.open()
                                    }
                                }
                            }

                            // This allows you to change this on mobile
                            onPressAndHold: {
                                QGroundControl.corePlugin.showTouchAreas = !QGroundControl.corePlugin.showTouchAreas
                                showTouchAreasNotification.open()
                            }
                        }
                    }
                }
            }

            // VAMA (task Z): made when asked, gone when the menu closes.
            Loader {
                id:      adminPasswordLoader
                active:  menuContent.askAdminPassword
                visible: active
                source:  "qrc:/Custom/qml/QGroundControl/Toolbar/VamaAdminPasswordPanel.qml"

                Connections {
                    target: adminPasswordLoader.item
                    function onFinished() { mainWindow.closeIndicatorDrawer() }
                }
            }
        }
    }
}
