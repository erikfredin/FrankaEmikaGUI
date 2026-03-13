#pragma once

#include <QThread>
#include <atomic>

class GamepadMonitor;  // comes from your GamepadMonitor.h

// EE-frame velocity command
struct TwistCmd {
  double vx{0.0}, vy{0.0}, vz{0.0};   // m/s
  double wx{0.0}, wy{0.0}, wz{0.0};   // rad/s
};


class GamepadPoller : public QThread {
  Q_OBJECT
public:
  // thresholds/speeds can be tuned via ctor
  GamepadPoller(GamepadMonitor* monitor,
                std::atomic<TwistCmd>& outCmd,
                double threshold = 0.8,
                double linearSpeed = 0.05,   // m/s when active
                double angularSpeed = 0.4,   // rad/s when active
                int periodMs = 8,            // ~125 Hz
                QObject* parent = nullptr);

  // request a graceful stop
  void stop();

protected:
  void run() override;

private:
  GamepadMonitor* monitor_{nullptr};
  std::atomic<TwistCmd>& outCmd_;
  std::atomic<bool> running_{false};

  const double TH_;
  const double V_;
  const double W_;
  const int period_ms_;
};
