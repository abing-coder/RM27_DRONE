#pragma once
// 规划基础库：3D 占据栅格（由场地地图膨胀生成）、A*、视线简化、minimum-snap 轨迹
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "rm_drone_control/field_map.hpp"

namespace rm_drone
{

struct GridConfig
{
  double resolution{0.1};
  double obstacle_margin{0.30};
  double wall_margin{0.45};
  double floor_margin{0.30};
  double ceiling{2.1};
};

class OccupancyGrid
{
public:
  void build(const FieldMap & map, const GridConfig & cfg);
  bool occupied(const Eigen::Vector3d & p) const;
  bool occupiedIdx(int x, int y, int z) const;
  bool lineFree(const Eigen::Vector3d & a, const Eigen::Vector3d & b, double step = 0.05) const;
  // 起点可以落在障碍里（例如贴着碰撞体），只检查离开起点之后的连线。
  bool exitFree(const Eigen::Vector3d & from, const Eigen::Vector3d & to, double step = 0.05) const;
  Eigen::Vector3i toIdx(const Eigen::Vector3d & p) const;
  Eigen::Vector3d toPos(const Eigen::Vector3i & i) const;
  bool valid(const Eigen::Vector3i & i) const;
  bool nearestFree(const Eigen::Vector3d & p, Eigen::Vector3d & out, double max_r = 1.0) const;
  const GridConfig & config() const {return cfg_;}
  Eigen::Vector3i size() const {return size_;}

private:
  GridConfig cfg_;
  Eigen::Vector3d origin_;
  Eigen::Vector3i size_;
  std::vector<uint8_t> data_;
};

bool astar(const OccupancyGrid & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
  std::vector<Eigen::Vector3d> & path);

std::vector<Eigen::Vector3d> simplifyPath(const OccupancyGrid & grid, const std::vector<Eigen::Vector3d> & path);

struct TrajState
{
  Eigen::Vector3d p, v, a;
};

class MinSnapTrajectory
{
public:
  // 只在求解、边界残差和速度/加速度限制都通过后写入本对象。失败时保持原轨迹。
  bool generate(const std::vector<Eigen::Vector3d> & waypoints, const Eigen::Vector3d & v0, const Eigen::Vector3d & a0,
    const Eigen::Vector3d & vf, double v_max, double a_max);
  // 在临时对象上生成并做碰撞检查。成功才替换本对象。
  bool generateSafe(const OccupancyGrid & grid, std::vector<Eigen::Vector3d> waypoints, const Eigen::Vector3d & v0,
    const Eigen::Vector3d & a0, const Eigen::Vector3d & vf, double v_max, double a_max);
  TrajState sample(double t) const;
  double duration() const {return total_;}
  bool empty() const {return !approved_;}
  bool valid() const {return approved_;}
  double peakSpeed() const {return peak_v_;}
  double peakAcc() const {return peak_a_;}
  const std::vector<Eigen::Vector3d> & waypoints() const {return wps_;}

private:
  struct Solved
  {
    std::vector<double> T;
    std::vector<Eigen::Matrix<double, 8, 3>> coef;
    double total{0};
    double residual{0};
    double peak_v{0};
    double peak_a{0};
  };
  static TrajState sampleSolved(const Solved & sol, double t);
  static bool solve(const std::vector<Eigen::Vector3d> & wps, const Eigen::Vector3d & v0, const Eigen::Vector3d & a0,
    const Eigen::Vector3d & vf, const std::vector<double> & T, Solved & out);
  static bool withinLimits(const Solved & s, double v_max, double a_max);
  int firstCollidingSegment(const OccupancyGrid & grid) const;
  std::vector<double> T_;
  std::vector<Eigen::Matrix<double, 8, 3>> coef_;
  std::vector<Eigen::Vector3d> wps_;
  double total_{0.0};
  double peak_v_{0.0};
  double peak_a_{0.0};
  bool approved_{false};
};

}  // namespace rm_drone
