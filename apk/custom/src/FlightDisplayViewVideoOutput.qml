// VAMA copy of QGC 5.1.5's src/FlyView/FlightDisplayViewVideoOutput.qml (the
// video picture). custom.qrc serves it in place of QGC's file. Compare it with
// QGC's file again after every QGC update. The changes are marked "VAMA": the
// thermal colour modes of VGCS (client request 2026-10-07), the pictures for
// the object lock (2026-10-09), and the pictures for the object finder
// (2026-10-13). QGC's comment on orientation has a colon in place of its long dash.

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

    // VAMA: pictures for the object lock. While the app follows an object itself
    // (SkydroidLink.lockWantsPictures), a small copy of the video picture goes to
    // it about 12 times a second, with where the picture is inside this item
    // (black bars beside a 5:4 thermal picture). grabToImage is what QGC uses
    // above for its own video snapshot. One grab at a time: the next one starts
    // when the last has come back.
    Timer {
        id:       lockPictures
        objectName: "vamaLockPictures"
        interval: 70
        repeat:   true
        running:  SkydroidLink.lockWantsPictures && videoOutput.visible && videoOutput.width > 0 && videoOutput.height > 0

        property bool waiting: false
        property int  skipped: 0
        // How wide the copy is, in points: enough to find the object, quick to copy.
        readonly property int copyWidth: 480

        onRunningChanged: {
            waiting = false
            skipped = 0
        }
        onTriggered: {
            // A grab that never comes back must not stop the pictures for good.
            if (waiting && skipped < 8) {
                skipped++
                return
            }
            skipped = 0
            const r = videoOutput.contentRect
            if (r.width < 32 || r.height < 32) {
                return
            }
            const w = Math.min(copyWidth, Math.round(videoOutput.width))
            const h = Math.max(32, Math.round(w * videoOutput.height / videoOutput.width))
            const px = r.x / videoOutput.width
            const py = r.y / videoOutput.height
            const pw = r.width / videoOutput.width
            const ph = r.height / videoOutput.height
            waiting = videoOutput.grabToImage(function(result) {
                lockPictures.waiting = false
                SkydroidLink.lockPicture(result.image, px, py, pw, ph)
            }, Qt.size(w, h))
        }
    }

    // VAMA: pictures for the object finder. While the app marks people and
    // vehicles (ObjectMarks.running), a picture of the video at the camera's own
    // size goes to it whenever it asks for one: after each look, a few times a
    // second at most. One grab at a time, as above.
    Timer {
        id:       markPictures
        objectName: "vamaMarkPictures"
        interval: 40
        repeat:   true
        running:  ObjectMarks.running && videoOutput.visible && videoOutput.width > 0 && videoOutput.height > 0

        property bool waiting: false
        property int  skipped: 0

        onRunningChanged: {
            waiting = false
            skipped = 0
        }
        onTriggered: {
            // A grab that never comes back must not stop the pictures for good.
            if (waiting && skipped < 25) {
                skipped++
                return
            }
            skipped = 0
            if (!ObjectMarks.wantsPicture()) {
                return
            }
            const r = videoOutput.contentRect
            if (r.width < 32 || r.height < 32) {
                return
            }
            // The camera's own size, also when the video is shown smaller: what is
            // small in the picture must not get smaller still.
            const w = ObjectMarks.pictureWidth
            const h = Math.max(32, Math.round(w * videoOutput.height / videoOutput.width))
            const px = r.x / videoOutput.width
            const py = r.y / videoOutput.height
            const pw = r.width / videoOutput.width
            const ph = r.height / videoOutput.height
            const asked = Date.now()
            waiting = videoOutput.grabToImage(function(result) {
                markPictures.waiting = false
                // How long the picture took to get is told on the screen, for a field video.
                ObjectMarks.picture(result.image, px, py, pw, ph, Date.now() - asked)
            }, Qt.size(w, h))
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
