#pragma once
// 场地地图（由 rm_field_gazebo/scripts/gen_field.py 生成的 field_map.yaml）：
// 轴对齐障碍物盒、可攻击目标、机库位姿。规划、估计（地形高度）、任务、裁判模拟共用。
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <algorithm>
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
  Eigen::Vector3d normal{Eigen::Vector3d::UnitX()};  // 含装甲倾角的朝外法向
  double radius{0.0};       // 前哨站旋转半径
  double spin_speed{0.0};
};

// 规则禁区在水平面上的足迹。竖直方向覆盖地面及其上空，不是有限高度盒体。
struct NoFlyColumn
{
  std::string name;
  double x{0}, y{0}, hx{0}, hy{0};
  bool contains(double px, double py) const
  {
    return std::abs(px - x) <= hx && std::abs(py - y) <= hy;
  }
};

struct TerrainTriangle
{
  Eigen::Vector3d a, b, c;
  double xmin, xmax, ymin, ymax;
  double height(double x, double y) const
  {
    if (x < xmin || x > xmax || y < ymin || y > ymax) {return -1e9;}
    const double det = (b.y() - c.y()) * (a.x() - c.x()) + (c.x() - b.x()) * (a.y() - c.y());
    if (std::abs(det) < 1e-12) {return -1e9;}
    const double u = ((b.y() - c.y()) * (x - c.x()) + (c.x() - b.x()) * (y - c.y())) / det;
    const double v = ((c.y() - a.y()) * (x - c.x()) + (a.x() - c.x()) * (y - c.y())) / det;
    if (u < -1e-8 || v < -1e-8 || u + v > 1.0 + 1e-8) {return -1e9;}
    return u * a.z() + v * b.z() + (1.0 - u - v) * c.z();
  }
};

struct FieldMap
{
  double length{28.0}, width{15.0}, wall_height{2.4};
  std::vector<Box> obstacles;
  std::vector<NoFlyColumn> nofly;
  std::vector<Target> targets;
  std::vector<TerrainTriangle> terrain;
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
    obstacles.clear();
    nofly.clear();
    targets.clear();
    terrain.clear();
    for (const auto & o : root["obstacles"]) {
      obstacles.push_back({o["name"].as<std::string>(), vec3(o["center"]), vec3(o["size"])});
    }
    if (root["nofly"]) {
      for (const auto & n : root["nofly"]) {
        const auto c = vec3(n["center"]);
        const auto s = vec3(n["size"]);
        nofly.push_back({n["name"].as<std::string>(), c.x(), c.y(), 0.5 * s.x(), 0.5 * s.y()});
      }
    }
    if (root["terrain_triangles"]) {
      for (const auto & row : root["terrain_triangles"]) {
        TerrainTriangle triangle;
        triangle.a = vec3(row[0]);
        triangle.b = vec3(row[1]);
        triangle.c = vec3(row[2]);
        triangle.xmin = std::min({triangle.a.x(), triangle.b.x(), triangle.c.x()});
        triangle.xmax = std::max({triangle.a.x(), triangle.b.x(), triangle.c.x()});
        triangle.ymin = std::min({triangle.a.y(), triangle.b.y(), triangle.c.y()});
        triangle.ymax = std::max({triangle.a.y(), triangle.b.y(), triangle.c.y()});
        terrain.push_back(triangle);
      }
    }
    for (const auto & t : root["targets"]) {
      Target tg;
      tg.name = t["name"].as<std::string>();
      tg.team = t["team"].as<std::string>();
      tg.kind = t["kind"].as<std::string>();
      tg.armor_type = t["armor_type"].as<std::string>();
      tg.position = vec3(t["position"]);
      if (t["normal_yaw"]) {tg.normal_yaw = t["normal_yaw"].as<double>();}
      tg.normal = Eigen::Vector3d(std::cos(tg.normal_yaw), std::sin(tg.normal_yaw), 0.0);
      if (t["normal"]) {
        const auto n = vec3(t["normal"]);
        if (n.allFinite() && n.norm() > 1e-6) {tg.normal = n.normalized();}
      }
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
    if (!terrain.empty()) {
      for (const auto & triangle : terrain) {
        const double z = triangle.height(x, y);
        if (z < z_query) {h = std::max(h, z);}
      }
      return h;
    }
    for (const auto & b : obstacles) {
      const auto mn = b.min(), mx = b.max();
      if (x >= mn.x() && x <= mx.x() && y >= mn.y() && y <= mx.y() && mx.z() < z_query) {
        h = std::max(h, mx.z());
      }
    }
    return h;
  }

  bool inNoFly(const Eigen::Vector3d & p, double margin = 0.0) const
  {
    if (p.z() < -margin) {return false;}
    for (const auto & c : nofly) {
      if (std::abs(p.x() - c.x) <= c.hx + margin && std::abs(p.y() - c.y) <= c.hy + margin) {return true;}
    }
    return false;
  }

  // allow_prefix 非空时，名称以此开头的实体障碍不拦截（仅用于指定目标的接触段）。
  // 能量机关、机库和禁区柱不受该前缀豁免。
  bool inCollision(const Eigen::Vector3d & p, double margin, const std::string & allow_prefix = "") const
  {
    if (inNoFly(p, margin)) {return true;}
    for (const auto & b : obstacles) {
      const bool sensitive = b.name.find("energy") != std::string::npos || b.name.find("hangar") != std::string::npos;
      if (!sensitive && !allow_prefix.empty() && b.name.rfind(allow_prefix, 0) == 0) {continue;}
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
