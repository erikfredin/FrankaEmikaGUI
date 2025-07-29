#include "camerafeed.h"
#include "ui_camerafeed.h"
#include <QFileDialog>
#include <QDateTime>

CameraFeed::CameraFeed(int camIndex, int camIndex2, QWidget *parent) :
    QWidget(parent), ui(new Ui::CameraFeed), cameraThread(new QThread),
    worker(new CameraWorker(camIndex)), cameraThread2(new QThread),
    worker2(new CameraWorker(camIndex2))
    {
    ui->setupUi(this);

    // Camera 1
    worker->moveToThread(cameraThread);
    connect(cameraThread, &QThread::started, worker, &CameraWorker::process);
    connect(worker, &CameraWorker::frameReady, this, &CameraFeed::onNewFrame);
    connect(worker, &CameraWorker::frameReady, this, &CameraFeed::frameCaptured);
    connect(this, &CameraFeed::destroyed, worker, &CameraWorker::stop);
    connect(cameraThread, &QThread::finished, worker, &QObject::deleteLater);
    connect(cameraThread, &QThread::finished, cameraThread, &QObject::deleteLater);
    cameraThread->start();

    // Camera 2
    worker2->moveToThread(cameraThread2);
    connect(cameraThread2, &QThread::started, worker2, &CameraWorker::process);
    connect(worker2, &CameraWorker::frameReady, this, &CameraFeed::onNewFrame2);
    connect(this, &CameraFeed::destroyed, worker2, &CameraWorker::stop);
    connect(cameraThread2, &QThread::finished, worker2, &QObject::deleteLater);
    connect(cameraThread2, &QThread::finished, cameraThread2, &QObject::deleteLater);
    cameraThread2->start();

    setWindowFlags(Qt::Window);  // Makes it a standalone top-level window
    setAttribute(Qt::WA_DeleteOnClose);  // Ensures it deletes itself when closed


}

CameraFeed::~CameraFeed()
{
    if (cameraThread->isRunning()) {
        worker->stop();         // Tell the worker to stop its loop
        cameraThread->quit();   // Quit the thread event loop
        cameraThread->wait();   // Wait for the thread to finish (safe now)
    }
    if (cameraThread2->isRunning()) {
            worker2->stop();
            cameraThread2->quit();
            cameraThread2->wait();
    }
    delete ui;  // UI and other child widgets get deleted as normal
}

void CameraFeed::onNewFrame(const QImage &frame) {
    QMutexLocker locker(&frameMutex);
    currentFrame = frame;
    ui->labelVideo->setPixmap(QPixmap::fromImage(frame).scaled(
    ui->labelVideo->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void CameraFeed::onNewFrame2(const QImage &frame) {
    QMutexLocker locker(&frameMutex2);
    currentFrame2 = frame;
    ui->labelVideo2->setPixmap(QPixmap::fromImage(frame).scaled(
    ui->labelVideo2->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void CameraFeed::saveSnapshot(QString filename) {
    QMutexLocker locker(&frameMutex);
    if (currentFrame.isNull()) return;

    //QString filename = QFileDialog::getSaveFileName(this, "Save Snapshot",
    //    QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".png",
    //    "Images (*.png *.jpg)");

    if (!filename.isEmpty()) {
        currentFrame.save(filename);
    }
}

void CameraFeed::saveSnapshot2(QString filename) {
    QMutexLocker locker(&frameMutex2);
    if (currentFrame2.isNull()) return;

    if (!filename.isEmpty()) {
        currentFrame2.save(filename);
    }
}

QImage CameraFeed::returnCurrentFrame(){
    if (!currentFrame.isNull()){
        return currentFrame;
    } else {
        QImage dummy;
        return dummy;
    }
}




