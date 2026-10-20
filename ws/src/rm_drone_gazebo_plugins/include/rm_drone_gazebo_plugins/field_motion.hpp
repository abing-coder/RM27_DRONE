#pragma once
// 仿真道具的运动时序。输入时间均为 Gazebo 仿真秒；不依赖墙钟或飞机状态。
#include <algorithm>
#include <cmath>
#include <random>
#include <string>

namespace rm_drone
{
class DartMotion
{
public:
  explicit DartMotion(unsigned seed = 2027) : rng_(seed), seed_(seed) {}
  bool setMode(const std::string & value)
  {
    if (value != "fixed" && value != "random_fixed" && value != "random_moving" && value != "end_moving") {return false;}
    mode = value;
    return true;
  }
  void reset()
  {
    rng_.seed(seed_);
    y = from_ = goal_ = 0.0;
    opened_ = next_ = shot_ = delayed_ = -1.0;
    last_hit_ = -100.0;
    moving = false;
  }
  void open(double t)
  {
    opened_ = t;
    shot_ = delayed_ = -1.0;
    last_hit_ = -100.0;
    if (mode == "fixed") {move(t, 0.0);}
    else {move(t, choose(y));}
    next_ = t + 1.0;
  }
  void close(double t)
  {
    opened_ = shot_ = delayed_ = -1.0;
    next_ = -1.0;
    move(t, 0.0);
  }
  bool window(double t) const {return opened_ >= 0.0 && t >= opened_ + 7.0 && t < opened_ + 47.0;}
  bool canHit(double t) const {return window(t) && !moving && t - last_hit_ >= 2.0;}
  void fire(double t)
  {
    if (!window(t) || (mode != "random_moving" && mode != "end_moving")) {return;}
    if (shot_ < 0.0) {shot_ = t;}
    if (mode == "end_moving") {
      // 连续发射时保留最先待执行的末端移动，不能把它无限推迟。
      if (delayed_ < 0.0) {delayed_ = t + 1.2;}
    } else {
      move(t, choose(y));
    }
  }
  bool hit(double t)
  {
    if (!canHit(t)) {return false;}
    last_hit_ = t;
    if (mode == "random_fixed") {move(t, choose(y));}
    if (mode == "random_moving" || mode == "end_moving") {
      shot_ = delayed_ = -1.0;
      next_ = t;
    }
    return true;
  }
  void update(double t)
  {
    if (opened_ >= 0.0 && t >= opened_ + 47.0) {close(t);}
    if (delayed_ >= 0.0 && t >= delayed_) {
      move(delayed_, choose(y));
      delayed_ = -1.0;
    }
    // 第一发发出十秒后恢复自由移动；新一发发出才开启下一轮等待。
    if (shot_ >= 0.0 && t >= shot_ + 10.0) {shot_ = delayed_ = -1.0; next_ = t;}
    if (opened_ >= 0.0 && mode != "fixed" && mode != "random_fixed" && shot_ < 0.0 && t >= next_) {
      move(t, choose(y));
      next_ = t + 1.0;
    }
    const double elapsed = t - move_t_;
    const double u = elapsed >= 0.6 - 1e-9 ? 1.0 : std::clamp(elapsed / 0.6, 0.0, 1.0);
    y = from_ + (goal_ - from_) * (u * u * (3.0 - 2.0 * u));
    moving = u < 1.0 && std::abs(goal_ - from_) > 1e-9;
  }
  std::string mode{"random_moving"};
  double y{0.0};
  bool moving{false};

private:
  void move(double t, double target)
  {
    from_ = y;
    goal_ = std::clamp(target, -0.280, 0.280);
    move_t_ = t;
    moving = std::abs(goal_ - from_) > 1e-9;
  }
  double choose(double origin)
  {
    // 规则未规定随机分布/最低位移；均匀抽样及至少 80 mm 的变化是可复现仿真假设。
    std::uniform_real_distribution<double> d(-0.280, 0.280);
    double value = d(rng_);
    for (int i = 0; i < 20 && std::abs(value - origin) < 0.08; ++i) {value = d(rng_);}
    if (std::abs(value - origin) < 0.08) {value = origin > 0.0 ? -0.280 : 0.280;}
    return value;
  }
  std::mt19937 rng_;
  unsigned seed_;
  double from_{0.0}, goal_{0.0}, move_t_{0.0};
  double opened_{-1.0}, next_{-1.0}, shot_{-1.0}, delayed_{-1.0}, last_hit_{-100.0};
};

class EnergyMotion
{
public:
  explicit EnergyMotion(unsigned seed = 2027) : rng_(seed), seed_(seed) {reset();}
  void reset()
  {
    rng_.seed(seed_);
    phase = 0.0;
    direction = (rng_() % 2) ? 1.0 : -1.0;
    restartProfile();
  }
  void restartProfile()
  {
    a = std::uniform_real_distribution<double>(0.780, 1.045)(rng_);
    omega = std::uniform_real_distribution<double>(1.884, 2.000)(rng_);
  }
  double step(double dt, double active_time, bool large_active)
  {
    // 精确积分，减速运行仿真时仍保持同一条角度轨迹。
    const double b = 2.090 - a;
    const double angle = large_active ?
      a / omega * (std::cos(omega * active_time) - std::cos(omega * (active_time + dt))) + b * dt :
      3.14159265358979323846 / 3.0 * dt;
    phase = std::remainder(phase + direction * angle, 2.0 * 3.14159265358979323846);
    return phase;
  }
  double phase{0.0}, direction{1.0}, a{0.9}, omega{1.9};
private:
  std::mt19937 rng_;
  unsigned seed_;
};
}  // namespace rm_drone
