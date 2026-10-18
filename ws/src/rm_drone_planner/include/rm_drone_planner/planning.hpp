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
  // waypoints 至少两个；v0/a0 为起点速度/加速度，vf 为终点速度
  bool generate(const std::vector<Eigen::Vector3d> & waypoints, const Eigen::Vector3d & v0, const Eigen::Vector3d & a0,
    const Eigen::Vector3d & vf, double v_max, double a_max);
  // 生成后做碰撞检查，碰撞段插入中点重新求解
  bool generateSafe(const OccupancyGrid & grid, std::vector<Eigen::Vector3d> waypoints, const Eigen::Vector3d & v0,
    const Eigen::Vector3d & a0, const Eigen::Vector3d & vf, double v_max, double a_max);
  TrajState sample(double t) const;
  double duration() const {return total_;}
  bool empty() const {return T_.empty();}
  const std::vector<Eigen::Vector3d> & waypoints() const {return wps_;}

private:
  int firstCollidingSegment(const OccupancyGrid & grid) const;
  std::vector<double> T_;
  std::vector<Eigen::Matrix<double, 8, 3>> coef_;
  std::vector<Eigen::Vector3d> wps_;
  double total_{0.0};
};

}  // namespace rm_drone
