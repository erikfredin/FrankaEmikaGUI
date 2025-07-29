#ifndef CAMERAWORKER_H
#define CAMERAWORKER_H

#include <QObject>
#include <QImage>
#include <QMutex>
#include <opencv2/opencv.hpp>

class CameraWorker : public QObject
{
    Q_OBJECT

public:
    explicit CameraWorker(int cameraIndex, QObject *parent = nullptr);
    ~CameraWorker();

public slots:
    void process();   // Main loop: capture frames and emit
    void stop();      // Stop signal for graceful thread exit

signals:
    void frameReady(const QImage &image);  // Emitted when a new frame is available

private:
    int cameraIndex;
    bool running;
    QMutex mutex;
};

#endif // CAMERAWORKER_H
