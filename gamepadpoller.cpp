#include "gamepadpoller.h"
#include "gamepadmonitor.h"   // <- your provided header
#include <QElapsedTimer>
#include <cmath>

GamepadPoller::GamepadPoller(GamepadMonitor* monitor,
                             std::atomic<TwistCmd>& outCmd,
                             double threshold,
                             double linearSpeed,
                             double angularSpeed,
                             int periodMs,
                             QObject* parent)
  : QThread(parent)
  , monitor_(monitor)
  , outCmd_(outCmd)
  , TH_(threshold)
  , V_(linearSpeed)
  , W_(angularSpeed)
  , period_ms_(periodMs)
{}

void GamepadPoller::stop() { running_ = false; }

void GamepadPoller::run() {
  running_ = true;
  QElapsedTimer timer; timer.start();

  while (running_) {
    TwistCmd cmd{};  // zeros by default (=> robot stops if no axis is active)

    // Only publish motion when your GamepadMonitor is enabled
    if (monitor_ && monitor_->enabled) {
      // NB: GamepadMonitor defines:
      // 0: Left X, 1: Left Y, 2: Right X, 3: Right Y, 4: L2, 5: R2
      const double* a = monitor_->joystickValues;

      // ---- Your mapping exactly as requested ----
      // if (a[0] > 0.8)  -> +EE X
      // if (a[0] < -0.8) -> -EE X
      if (a[0] >  TH_) cmd.vx =  +V_;
      else if (a[0] < -TH_) cmd.vx =  -V_;

      // if (a[1] > 0.8)  -> +EE Z
      // if (a[1] < -0.8) -> -EE Z
      if (a[1] >  TH_) cmd.vz =  +V_;
      else if (a[1] < -TH_) cmd.vz =  -V_;

      // if (a[2] > 0.8)  -> -EE Y
      // if (a[2] < -0.8) -> +EE Y
      if (a[2] >  TH_) cmd.vy =  -V_;
      else if (a[2] < -TH_) cmd.vy =  +V_;

      // if (a[3] > 0.8)  -> +rot around EE Z
      // if (a[3] < -0.8) -> -rot around EE Z
      if (a[3] >  TH_) cmd.wz =  +W_;
      else if (a[3] < -TH_) cmd.wz =  -W_;

      // if (a[4] == 1)   -> +rot around EE X
      // if (a[5] == 1)   -> -rot around EE X
      // (treat "==1" with a small tolerance in case the driver gives 0..1 floats)
      if (std::abs(a[4] - 1.0) < 1e-6) cmd.wx = +W_;
      if (std::abs(a[5] - 1.0) < 1e-6) cmd.wx = -W_;
    }

    outCmd_.store(cmd, std::memory_order_relaxed);

    // pacing
    qint64 elapsed = timer.elapsed();
    qint64 sleep_for = period_ms_ - (elapsed % period_ms_);
    msleep(static_cast<unsigned long>(sleep_for));
  }
}
