#include <cmath>
#include <iostream>
#include <string>
#include <utility>

#include "rm_drone_planner/planning.hpp"

namespace
{
int fail(const std::string & msg)
{
  std::cerr << msg << "\n";
  return 1;
}

bool limitsOk(const rm_drone::MinSnapTrajectory & traj, double v_max, double a_max)
{
  double pv = 0, pa = 0;
  const double dt = std::min(0.01, traj.duration() / 200.0);
  for (double t = 0; t <= traj.duration(); t += dt) {
    const auto s = traj.sample(std::min(t, traj.duration()));
    pv = std::max(pv, s.v.norm());
    pa = std::max(pa, s.a.norm());
  }
  std::cout << "  duration=" << traj.duration() << " peak_v=" << pv << " peak_a=" << pa << "\n";
  return pv <= v_max * 1.02 && pa <= a_max * 1.02;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {return fail("usage: plan_test <field_map.yaml>");}
  rm_drone::FieldMap map;
  if (!map.load(argv[1])) {return fail(std::string("failed to load ") + argv[1]);}

  // B. 两端静止的 10 m 直线，限制必须作用在采样结果上。
  {
    rm_drone::MinSnapTrajectory traj;
    const std::vector<Eigen::Vector3d> wps{Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(10, 0, 1)};
    if (!traj.generate(wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 3.0, 3.0)) {
      return fail("B: 10 m 直线在 v=3 a=3 下无解");
    }
    std::cout << "B";
    if (!limitsOk(traj, 3.0, 3.0)) {return fail("B: 轨迹超过速度或加速度限制");}
    const auto s0 = traj.sample(0.0);
    if (s0.v.norm() > 0.05 || s0.a.norm() > 0.05) {return fail("B: 起点速度/加速度被清零约束未保持");}
  }

  // 起点速度已经超过限制时必须失败，且不能留下可采样的候选。
  {
    rm_drone::MinSnapTrajectory traj;
    const bool ok = traj.generate(
      {Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 0, 0)}, Eigen::Vector3d(10, 0, 0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), 3.0, 3.0);
    if (ok || traj.valid()) {return fail("超限起点速度不应生成可执行轨迹");}
  }

  rm_drone::OccupancyGrid grid;
  rm_drone::GridConfig cfg;
  cfg.ceiling = 1.55;
  double takeoff_height = 0.35;
  const auto navigation = YAML::LoadFile(argv[1])["navigation"];
  if (navigation) {
    cfg.ceiling = navigation["ceiling"].as<double>(cfg.ceiling);
    cfg.obstacle_margin = navigation["obstacle_margin"].as<double>(cfg.obstacle_margin);
    takeoff_height = navigation["takeoff_height"].as<double>(takeoff_height);
  }
  grid.build(map, cfg);

  const Eigen::Vector3d start = map.hangar_pos[0] + Eigen::Vector3d(0, 0, 0.023 + takeoff_height);
  const rm_drone::Target * armor = map.findTarget("blue_base_armor_0");
  if (!armor) {return fail("地图里没有 blue_base_armor_0");}
  const Eigen::Vector3d n(std::cos(armor->normal_yaw), std::sin(armor->normal_yaw), 0.0);
  const Eigen::Vector3d goal = armor->position + n * 2.0;

  std::vector<Eigen::Vector3d> path;
  const bool path_ok = rm_drone::astar(grid, start, goal, path);
  rm_drone::MinSnapTrajectory approved;
  bool planned = false;
  if (path_ok) {
    path = rm_drone::simplifyPath(grid, path);
    planned = approved.generateSafe(grid, path, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), 3.0, 3.0);
    if (!planned) {
      rm_drone::MinSnapTrajectory retry;
      planned = retry.generateSafe(grid, path, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), 1.8, 3.0);
      if (planned) {approved = std::move(retry);}
    }
  }
  std::cout << "A path=" << (path_ok ? "ok" : "fail") << " plan=" << (planned ? "ok" : "fail");
  if (planned) {
    std::cout << "\n";
    if (!approved.valid() || !limitsOk(approved, 3.0, 3.0)) {return fail("A: 成功轨迹未满足限制");}
    const double step = 0.05;
    Eigen::Vector3d prev = approved.sample(0).p;
    for (double t = 0.02; t <= approved.duration(); t += 0.02) {
      const auto p = approved.sample(t).p;
      if (!grid.lineFree(prev, p, step)) {return fail("A: 成功轨迹碰到障碍");}
      prev = p;
    }
  } else {
    std::cout << "\n";
    if (approved.valid()) {return fail("A: 规划失败后仍保留了可执行轨迹");}
  }

  // 飞镖发射站禁区必须挡住其上空，而不是只挡住有限高度的实体。
  bool saw_nofly = false;
  for (const auto & c : map.nofly) {
    if (c.name.find("dart") == std::string::npos) {continue;}
    saw_nofly = true;
    const Eigen::Vector3d above(c.x, c.y, 2.0);
    if (!map.inNoFly(above) || !grid.occupied(above)) {return fail("飞镖禁区上空仍可规划: " + c.name);}
  }
  if (!saw_nofly) {return fail("地图没有飞镖发射站禁区柱");}

  std::cout << "plan_test ok\n";
  return 0;
}
