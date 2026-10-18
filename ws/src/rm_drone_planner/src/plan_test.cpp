#include <iostream>

#include "rm_drone_planner/planning.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: plan_test <field_map.yaml>\n";
    return 1;
  }
  rm_drone::FieldMap map;
  if (!map.load(argv[1])) {
    std::cerr << "failed to load " << argv[1] << "\n";
    return 1;
  }
  rm_drone::OccupancyGrid grid;
  rm_drone::GridConfig cfg;
  grid.build(map, cfg);

  const Eigen::Vector3d start = map.hangar_pos[0] + Eigen::Vector3d(0, 0, 0.6);
  const Eigen::Vector3d goal(10.0, 0.0, 1.5);
  std::vector<Eigen::Vector3d> path;
  if (!rm_drone::astar(grid, start, goal, path)) {
    std::cerr << "A* failed\n";
    return 2;
  }
  path = rm_drone::simplifyPath(grid, path);
  rm_drone::MinSnapTrajectory traj;
  if (!traj.generateSafe(grid, path, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), 3.0, 3.0)) {
    std::cerr << "min-snap failed\n";
    return 3;
  }
  const auto s = traj.sample(0.5 * traj.duration());
  std::cout << "waypoints=" << traj.waypoints().size()
            << " duration=" << traj.duration()
            << " mid=(" << s.p.transpose() << ")\n";
  return 0;
}
