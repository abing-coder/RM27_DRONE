#pragma once
// 断流、急停和仿真暂停的判定。实机接收超时用单调时间；仿真按仿真时钟计龄。
#include <algorithm>
#include <cmath>
#include <string>

namespace rm_drone
{

enum class FaultAct
{
  None,
  Descend,    // 视觉或设定值失效：保持水平并减小推力，不是空中零推力
  CutMotors   // 明确急停：停桨
};

// 时钟未推进时视为仿真暂停/锁步等待。控制定时器通常每20ms执行，不能要求
// 单次墙钟间隔超过50ms，否则永远无法识别锁步等待，反而反复触发接收超时。
// 只要ROS时间向前推进，就恢复断流检查；慢速仿真不等于可以忽略旧采样。
inline bool simPaused(double ros_dt, double steady_dt)
{
  return steady_dt > 0.0 && ros_dt >= 0.0 && ros_dt < 1e-9;
}

// 锁步渲染可能让墙钟经过数秒、物理只经过几毫秒。仿真数据的新鲜度必须与
// 物理使用同一时间轴；实机仍保留单调时钟看门狗，避免系统时间调整掩盖断流。
inline double receiveAge(bool simulation_clock, double ros_age, double steady_age)
{
  return simulation_clock ? std::max(0.0, ros_age) : steady_age;
}

// sample/ros_now/prev 都是 ROS 时间（秒）。finite/quat_ok 为假、超前、过期或乱序都拒绝。
inline bool stampUsable(double sample, double ros_now, double prev, bool finite, bool quat_ok)
{
  if (!finite || !quat_ok) {return false;}
  if (!std::isfinite(sample) || !std::isfinite(ros_now)) {return false;}
  if (sample > ros_now + 0.05) {return false;}
  if (ros_now - sample > 0.5) {return false;}
  if (prev >= 0.0 && sample + 1e-4 < prev) {return false;}
  return true;
}

// odom_age_ros 用采样时间；接收超时通过 receiveAge 选择仿真/实机的时间轴。
inline FaultAct positionFault(
  bool estop, bool paused, bool odom_valid, double odom_age_ros, double odom_receive_age,
  bool sp_valid, double sp_receive_age, double sp_timeout)
{
  if (estop) {return FaultAct::CutMotors;}
  if (paused) {return FaultAct::None;}
  if (!odom_valid || odom_age_ros > 0.3 || odom_receive_age > 0.3) {return FaultAct::Descend;}
  if (!sp_valid || sp_receive_age > sp_timeout) {return FaultAct::Descend;}
  return FaultAct::None;
}

// 急停锁存。arm=true 不能解除；必须显式复位，且当前不能处于解锁。
inline bool estopRelease(bool latched, bool reset, bool armed)
{
  return latched && reset && !armed;
}

struct EstimateHealth
{
  const char * source;
  bool vio_ok;
};

// use_vio=false 时不得报告 VIO 健康。真值模式单独标明，不伪装成视觉。
inline EstimateHealth classifyEstimate(
  bool ground_truth, bool use_vio, bool initialized, bool aligned, double age, double timeout, int reject)
{
  if (ground_truth) {return {"ground_truth", false};}
  if (!use_vio) {return {initialized ? "disabled" : "waiting", false};}
  if (!initialized) {return {"waiting", false};}
  if (aligned && age >= 0.0 && age < timeout && reject < 5) {return {"healthy", true};}
  return {"degraded", false};
}

inline const char * rejectTruthConfig(const std::string & runtime, bool use_sim_truth, const std::string & mode)
{
  if (use_sim_truth) {return "use_sim_truth 不能作为控制输出来源；真值调试请用 mode=gt";}
  if (runtime == "real" && mode == "gt") {return "实机配置不能使用仿真真值";}
  return nullptr;
}

}  // namespace rm_drone
