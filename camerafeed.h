#ifndef CAMERAFEED_H
#define CAMERAFEED_H

#include <QWidget>
#include <QImage>
#include <QThread>
#include "cameraworker.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class CameraFeed;
}
QT_END_NAMESPACE

class CameraFeed : public QWidget
{
    Q_OBJECT

public:
    explicit CameraFeed(int camIndex = 0, int camIndex2 = 1, QWidget *parent = nullptr);
    ~CameraFeed();

signals:
    void frameCaptured(const QImage &frame);  // Forward to MainWindow if needed

public slots:
    void saveSnapshot(QString filename); // You can call this from MainWindow
    void saveSnapshot2(QString filename);
    void onNewFrame(const QImage &frame);
    void onNewFrame2(const QImage &frame);
    QImage returnCurrentFrame();


private:
    Ui::CameraFeed *ui;

    QThread *cameraThread;
    QThread *cameraThread2;
    CameraWorker *worker;
    CameraWorker *worker2;

    QImage currentFrame; // Save last frame for snapshot
    QImage currentFrame2;
    QMutex frameMutex;
    QMutex frameMutex2;
};

#endif // CAMERAFEED_H
