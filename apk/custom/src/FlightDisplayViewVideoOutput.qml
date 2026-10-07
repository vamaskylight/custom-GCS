// VAMA copy of QGC 5.1.5's src/FlyView/FlightDisplayViewVideoOutput.qml (the
// video picture). custom.qrc serves it in place of QGC's file. Compare it with
// QGC's file again after every QGC update. The changes are marked "VAMA": the
// thermal colour modes of VGCS (client request 2026-10-07). QGC's comment on
// orientation has a colon in place of its long dash.

import QtQuick
import QtMultimedia

import QGroundControl

VideoOutput {
    id:         videoOutput  // VAMA
    objectName: "videoContent"

    // Do NOT set `orientation` here: VideoOutput composes orientation on top of the
    // QVideoFrame's own rotation()/mirrored() metadata that qgcqvideosink forwards from
    // GstVideoOrientationMeta. Setting it would double-rotate any stream with orientation tags.

    // videoFit enum: 0=Fit Width, 1=Fit Height, 2=Fill, 3=No Crop. The container
    // handles fit-width/fit-height sizing; only Fill needs the cropping fillMode.
    fillMode: QGroundControl.settingsManager.videoSettings.videoFit.rawValue === 2
              ? VideoOutput.PreserveAspectCrop
              : VideoOutput.PreserveAspectFit

    Connections {
        target: QGroundControl.videoManager
        function onImageFileChanged(filename) {
            grabToImage(function(result) {
                if (!result.saveToFile(filename)) {
                    console.error('Error capturing video frame');
                }
            });
        }
    }

    // VAMA: thermal colour modes, the same as VGCS (vgcs/video/thermal_palette.py).
    // While the IR button shows the thermal stream (QGC's video address is the
    // thermal address, the same test as _thermalOn in FlyViewCustomLayer.qml),
    // the picture is drawn through the chosen mode's row of the colour table
    // (thermal_palettes.png, made from VGCS by apk/make_thermal_palettes.py).
    // "White hot (as received)" leaves the picture alone and costs nothing.
    // Photos and recordings on the camera keep the camera's own picture.
    readonly property bool _vamaThermal:    QGroundControl.settingsManager.videoSettings.rtspUrl.rawValue === SkydroidLink.thermalVideoUrl
    readonly property int  _vamaPaletteRow: _vamaThermal ? Math.max(0, SkydroidLink.thermalPaletteIndex) : 0

    layer.enabled: _vamaPaletteRow > 0
    layer.effect: ShaderEffect {
        // The mode's row of the table, at the row's centre.
        readonly property real     row:     (videoOutput._vamaPaletteRow + 0.5) / SkydroidLink.thermalPalettes.length
        // Where the picture is, so black bars beside it stay black.
        readonly property vector4d picture: {
            const r = videoOutput.contentRect
            if (r.width <= 0 || r.height <= 0 || videoOutput.width <= 0 || videoOutput.height <= 0) {
                return Qt.vector4d(0, 0, 1, 1)
            }
            return Qt.vector4d(r.x / videoOutput.width, r.y / videoOutput.height,
                               r.width / videoOutput.width, r.height / videoOutput.height)
        }
        readonly property var      table:   tableImage

        fragmentShader: "qrc:/custom/shaders/thermal_palette.frag.qsb"

        Image {
            id:      tableImage
            source:  "qrc:/custom/img/thermal_palettes.png"
            visible: false
        }
    }
}
