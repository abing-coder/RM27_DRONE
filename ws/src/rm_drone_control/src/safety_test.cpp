// 四类故障：里程计断流、设定值断流、非法输入、急停后 arm=true 不能解锁。
#include <iostream>
#include <string>

#include "rm_drone_control/fault_policy.hpp"

namespace
{
int fail(const std::string & msg)
{
  std::cerr << msg << "\n";
  return 1;
}
}  // namespace

int main()
{
  using rm_drone::FaultAct;
  if (rm_drone::positionFault(false, false, false, 1.0, 1.0, true, 0.0, 0.5) != FaultAct::Descend) {
    return fail("odom 断流应受控下降，而不是继续跟踪");
  }
  if (rm_drone::positionFault(false, false, true, 0.0, 0.0, false, 1.0, 0.5) != FaultAct::Descend) {
    return fail("设定值断流不应变成空中零推力");
  }
  if (rm_drone::positionFault(false, true, false, 5.0, 5.0, false, 5.0, 0.5) != FaultAct::None) {
    return fail("仿真暂停不应被当成断流");
  }
  if (rm_drone::stampUsable(1.0, 1.0, -1.0, false, true)) {return fail("NaN/非法向量应拒绝");}
  if (rm_drone::stampUsable(1.0, 1.0, -1.0, true, false)) {return fail("非法四元数应拒绝");}
  if (rm_drone::stampUsable(0.5, 1.0, 0.8, true, true)) {return fail("乱序时间戳应拒绝");}
  if (rm_drone::stampUsable(0.1, 1.0, -1.0, true, true)) {return fail("过期采样应拒绝");}
  if (!rm_drone::stampUsable(0.95, 1.0, 0.9, true, true)) {return fail("正常采样被拒绝");}

  if (rm_drone::estopRelease(true, false, false)) {return fail("没有复位不能解除急停");}
  if (rm_drone::estopRelease(true, true, true)) {return fail("arm=true 不能解除急停");}
  if (!rm_drone::estopRelease(true, true, false)) {return fail("复位且已上锁时应允许解除");}
  if (rm_drone::positionFault(true, false, true, 0.0, 0.0, true, 0.0, 0.5) != FaultAct::CutMotors) {
    return fail("急停应停桨");
  }

  const auto off = rm_drone::classifyEstimate(false, false, true, true, 0.0, 0.3, 0);
  if (off.vio_ok || std::string(off.source) == "healthy") {return fail("未启用 VIO 不能报告为正常");}
  const auto bad = rm_drone::classifyEstimate(false, true, true, false, 2.0, 0.3, 12);
  if (bad.vio_ok || std::string(bad.source) != "degraded") {return fail("VIO 失效应降级且不能回退真值");}
  if (rm_drone::rejectTruthConfig("real", false, "gt") == nullptr) {return fail("实机真值配置应被拒绝");}
  if (rm_drone::rejectTruthConfig("sim", true, "eskf") == nullptr) {return fail("真值替代应被拒绝");}
  if (rm_drone::rejectTruthConfig("sim", false, "gt") != nullptr) {return fail("显式真值调试应允许");}

  std::cout << "safety_test ok\n";
  return 0;
}
