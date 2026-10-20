#pragma once
// 断流、急停和仿真暂停的判定。接收超时用单调时间，轨迹时间用 ROS 时间，二者不要混用。
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

// 墙钟在走、仿真时间几乎不动，视为暂停，不当成断流。
inline bool simPaused(double ros_dt, double steady_dt)
{
  return steady_dt > 0.05 && ros_dt >= 0.0 && ros_dt < 0.25 * steady_dt;
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

// odom_age_ros 用采样时间，odom_age_steady / sp_age_steady 用单调时间。
inline FaultAct positionFault(
  bool estop, bool paused, bool odom_valid, double odom_age_ros, double odom_age_steady,
  bool sp_valid, double sp_age_steady, double sp_timeout)
{
  if (estop) {return FaultAct::CutMotors;}
  if (paused) {return FaultAct::None;}
  if (!odom_valid || odom_age_ros > 0.3 || odom_age_steady > 0.3) {return FaultAct::Descend;}
  if (!sp_valid || sp_age_steady > sp_timeout) {return FaultAct::Descend;}
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
