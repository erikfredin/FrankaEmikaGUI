#include "cameraworker.h"
#include <QThread>

CameraWorker::CameraWorker(int cameraIndex, QObject *parent)
    : QObject(parent), cameraIndex(cameraIndex), running(true) {}

CameraWorker::~CameraWorker() {}

void CameraWorker::process() {
    cv::VideoCapture cap(cameraIndex);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    if (!cap.isOpened()) return;

    cv::Mat frame;
    while (true) {
        {
            QMutexLocker locker(&mutex);
            if (!running) break;
        }

        cap >> frame;
        if (frame.empty()) continue;

        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        QImage image((const uchar*)frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        emit frameReady(image.copy());

        //QThread::msleep(30); // Adjust for ~30 FPS
    }
}

void CameraWorker::stop() {
    QMutexLocker locker(&mutex);
    running = false;
}
