#pragma once

// The object finder at work for the marks (ObjectMarks.h): MarksWorker looks at
// one picture of the video at a time, on a thread of its own, and answers with
// what it found.
//
// The finder is behind PictureFinder, so tests run the marks with a stand-in.
// The app's is the network (ObjectMarksNet.cc).

#include <QtCore/QMetaType>
#include <QtCore/QObject>
#include <QtCore/QRectF>
#include <QtCore/QString>
#include <QtGui/QImage>

#include <memory>
#include <vector>

#include "ObjectFinder.h"

/// What finds the objects in a picture.
class PictureFinder
{
public:
    virtual ~PictureFinder() = default;
    /// Makes ready (the app's loads its network). Called on the finder's thread
    /// before the first find, and again after rest(). False, with the reason,
    /// when this finder cannot work at all.
    virtual bool load(QString &whyNot) = 0;
    /// As ObjectFinder::find. Two threads may call it at the same time.
    virtual std::vector<skydroid::find::Found> find(const skydroid::find::Picture &picture,
                                                    const skydroid::find::Ask &ask, QString &whyNot) const = 0;
    /// Gives its memory back until the next load().
    virtual void rest() {}
};

/// The app's finder. (Tests and the preview have their own.)
PictureFinder *vamaMakePictureFinder();
/// The app's finder with two other files in place of its own. It refuses them:
/// for the test of that.
PictureFinder *vamaMakePictureFinderFrom(const QString &layersFile, const QString &weightsFile);
/// The licence notices that go with that finder, as text.
QString vamaFinderNotices();
/// What that finder is, for the settings: the library with its version, and the network.
/// Empty when the build has none.
QString vamaFinderName();

/// What was found in one picture.
struct MarksAnswer
{
    std::vector<skydroid::find::Found> found;  // boxes in parts of the video picture (0 to 1), the surest first
    qint64 atMs = 0;                           // the picture's time
    double aspect = 16.0 / 9.0;                // the video picture's width over its height
    QRectF closer;                             // the part looked at at its own size, in parts of the picture (empty: none)
    double lookMs = 0.0;                       // how long the looks took, all in all
    double unitMs = 0.0;                       // of that: for an input of 640 x 384 points (0: not measured)
    bool blank = false;                        // the picture is one flat colour: there is no video in it
    QString whyNot;                            // not empty: the finder cannot work at all
    QString trouble;                           // not empty: a look at this picture failed, and why
};
Q_DECLARE_METATYPE(MarksAnswer)

/// Looks at the pictures, on the finder's thread.
class MarksWorker : public QObject
{
    Q_OBJECT

public:
    /// Takes the finder over. sideBySide: the two looks at a picture run at the same time.
    MarksWorker(PictureFinder *finder, bool sideBySide, QObject *parent = nullptr);
    ~MarksWorker() override;

public slots:
    /// image: the picture as QML grabbed it. pictureRect: where the video picture is
    /// inside it, in parts of the image. unitMs: how fast this device is (see planLooks).
    void look(const QImage &image, const QRectF &pictureRect, double unitMs, qint64 atMs);
    void rest();

signals:
    void looked(const MarksAnswer &answer);

private:
    std::unique_ptr<PictureFinder> _finder;
    bool _sideBySide = false;
    bool _loaded = false;
    bool _broken = false;
    QString _whyNot;
};
