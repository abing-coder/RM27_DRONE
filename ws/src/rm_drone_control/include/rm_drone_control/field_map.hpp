#pragma once
// 场地地图（由 rm_field_gazebo/scripts/gen_field.py 生成的 field_map.yaml）：
// 轴对齐障碍物盒、可攻击目标、机库位姿。规划、估计（地形高度）、任务、裁判模拟共用。
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace rm_drone
{

struct Box
{
  std::string name;
  Eigen::Vector3d center, size;
  Eigen::Vector3d min() const {return center - 0.5 * size;}
  Eigen::Vector3d max() const {return center + 0.5 * size;}
  bool contains(const Eigen::Vector3d & p, double margin = 0.0) const
  {
    return ((p - center).cwiseAbs() - (0.5 * size + Eigen::Vector3d::Constant(margin))).maxCoeff() <= 0.0;
  }
};

struct Target
{
  std::string name, team, kind, armor_type;
  Eigen::Vector3d position;
  double normal_yaw{0.0};   // 基地装甲法向（朝外）
  double radius{0.0};       // 前哨站旋转半径
  double spin_speed{0.0};
};

struct FieldMap
{
  double length{28.0}, width{15.0}, wall_height{2.4};
  std::vector<Box> obstacles;
  std::vector<Target> targets;
  Eigen::Vector3d hangar_pos[2];
  double hangar_yaw[2]{0.0, 0.0};

  static Eigen::Vector3d vec3(const YAML::Node & n)
  {
    return Eigen::Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
  }

  bool load(const std::string & path)
  {
    YAML::Node root = YAML::LoadFile(path);
    length = root["field"]["length"].as<double>();
    width = root["field"]["width"].as<double>();
    wall_height = root["field"]["wall_height"].as<double>();
    for (const auto & o : root["obstacles"]) {
      obstacles.push_back({o["name"].as<std::string>(), vec3(o["center"]), vec3(o["size"])});
    }
    for (const auto & t : root["targets"]) {
      Target tg;
      tg.name = t["name"].as<std::string>();
      tg.team = t["team"].as<std::string>();
      tg.kind = t["kind"].as<std::string>();
      tg.armor_type = t["armor_type"].as<std::string>();
      tg.position = vec3(t["position"]);
      if (t["normal_yaw"]) {tg.normal_yaw = t["normal_yaw"].as<double>();}
      if (t["radius"]) {tg.radius = t["radius"].as<double>();}
      if (t["spin_speed"]) {tg.spin_speed = t["spin_speed"].as<double>();}
      targets.push_back(tg);
    }
    const char * teams[2] = {"red", "blue"};
    for (int i = 0; i < 2; ++i) {
      hangar_pos[i] = vec3(root["hangars"][teams[i]]["position"]);
      hangar_yaw[i] = root["hangars"][teams[i]]["yaw"].as<double>();
    }
    return true;
  }

  // (x,y) 处的地形/结构顶面高度（只考虑低于 z_query 的结构顶面，用于 ToF 测高）
  double terrainHeight(double x, double y, double z_query = 1e9) const
  {
    double h = 0.0;
    for (const auto & b : obstacles) {
      const auto mn = b.min(), mx = b.max();
      if (x >= mn.x() && x <= mx.x() && y >= mn.y() && y <= mx.y() && mx.z() < z_query) {
        h = std::max(h, mx.z());
      }
    }
    return h;
  }

  bool inCollision(const Eigen::Vector3d & p, double margin) const
  {
    for (const auto & b : obstacles) {
      if (b.contains(p, margin)) {return true;}
    }
    return false;
  }

  bool insideField(const Eigen::Vector3d & p, double margin, double ceiling) const
  {
    return std::abs(p.x()) <= length / 2 - margin && std::abs(p.y()) <= width / 2 - margin &&
           p.z() <= ceiling && p.z() >= 0.0;
  }

  const Target * findTarget(const std::string & name) const
  {
    for (const auto & t : targets) {
      if (t.name == name) {return &t;}
    }
    return nullptr;
  }
};

}  // namespace rm_drone
