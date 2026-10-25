#pragma once
// SPDX-License-Identifier: GPL-3.0-only
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include <Eigen/Dense>
#include "rm_drone_planner/planning.hpp"

namespace rm_drone
{
struct DepthRay {Eigen::Vector3d endpoint; bool hit;};

// Bounded voxel grid, hit/miss evidence once per voxel per sensor frame.
// Free-space rays clear moving-object trails; unobserved obstacles are retained.
class LocalMap
{
public:
  LocalMap(const FieldMap & field, GridConfig cfg, bool use_prior, bool unknown_occupied, double inflation)
  : resolution_(cfg.resolution), unknown_occupied_(unknown_occupied)
  {
    FieldMap rules = field;
    if (!use_prior) {
      for (const auto & home : field.hangar_pos) {launches_.push_back(home);}
      rules.obstacles.erase(std::remove_if(rules.obstacles.begin(), rules.obstacles.end(),
        [&field](const Box & b) {return !field.launchObstacle(b);}), rules.obstacles.end());
    }
    prior_.build(rules, cfg);
    // Deduplication uses voxel centres, including half-cell rounding error.
    cfg.obstacle_margin = cfg.resolution / 2 + 1e-6;
    cfg.wall_margin = 0;
    cfg.floor_margin = 0;
    raw_prior_.build(rules, cfg);
    size_ = prior_.size();
    count_ = static_cast<size_t>(size_.x()) * size_.y() * size_.z();
    odds_.assign(count_, 0);
    seen_.assign(count_, false);
    marks_.assign(count_, 0);
    inflated_.assign(count_, 0);
    // Include the endpoint voxel's half diagonal in the Minkowski expansion.
    const double radius = inflation + std::sqrt(3.) * resolution_ / 2;
    const int reach = std::ceil(radius / resolution_);
    for (int x = -reach; x <= reach; ++x) {
      for (int y = -reach; y <= reach; ++y) {
        for (int z = -reach; z <= reach; ++z) {
          Eigen::Vector3i offset(x, y, z);
          if (offset.cast<double>().norm() * resolution_ <= radius + 1e-9) {offsets_.push_back(offset);}
        }
      }
    }
  }
  void clear()
  {
    std::fill(odds_.begin(), odds_.end(), 0);
    std::fill(seen_.begin(), seen_.end(), false);
    std::fill(inflated_.begin(), inflated_.end(), 0);
    occupied_.clear();
    ++revision_;
  }
  bool fuse(const Eigen::Vector3d & origin, const std::vector<DepthRay> & rays)
  {
    if (!origin.allFinite() || !prior_.valid(prior_.toIdx(origin)) || rays.empty()) {return false;}
    std::vector<size_t> touched;
    auto mark = [&](const Eigen::Vector3i & index, bool hit) {
        const size_t key = address(index);
        if (!marks_[key]) {touched.push_back(key);}
        if (hit || marks_[key] != 1) {marks_[key] = hit ? 1 : -1;}
      };
    for (const auto & ray : rays) {
      if (!ray.endpoint.allFinite()) {continue;}
      const auto end = prior_.toIdx(ray.endpoint);
      auto cell = prior_.toIdx(origin);
      const Eigen::Vector3d delta = ray.endpoint - origin;
      if (delta.norm() < resolution_) {continue;}
      Eigen::Vector3i step;
      Eigen::Vector3d tmax, tdelta;
      for (int axis = 0; axis < 3; ++axis) {
        step[axis] = delta[axis] > 0 ? 1 : -1;
        if (std::abs(delta[axis]) < 1e-12) {
          tmax[axis] = tdelta[axis] = std::numeric_limits<double>::infinity();
        } else {
          const double edge = prior_.toPos(cell)[axis] + step[axis] * resolution_ / 2;
          tmax[axis] = (edge - origin[axis]) / delta[axis];
          tdelta[axis] = resolution_ / std::abs(delta[axis]);
        }
      }
      for (int iteration = 0; iteration < size_.sum() + 3 && prior_.valid(cell); ++iteration) {
        if (cell == end) {
          // Static CAD surfaces already have their own calibrated inflation.
          if (ray.hit && !raw_prior_.occupied(ray.endpoint)) {mark(cell, true);}
          else if (!ray.hit) {mark(cell, false);}
          break;
        }
        mark(cell, false);
        Eigen::Index axis;
        tmax.minCoeff(&axis);
        cell[axis] += step[axis];
        tmax[axis] += tdelta[axis];
      }
    }
    if (touched.empty()) {return false;}
    for (size_t key : touched) {
      odds_[key] = std::clamp(odds_[key] + (marks_[key] == 1 ? .9f : -.7f), -2.f, 3.f);
      seen_[key] = true;
      marks_[key] = 0;
    }
    std::fill(inflated_.begin(), inflated_.end(), 0);
    occupied_.clear();
    for (size_t key = 0; key < count_; ++key) {
      if (odds_[key] < .7f) {continue;}
      const auto center = index(key);
      occupied_.push_back(prior_.toPos(center));
      for (const auto & offset : offsets_) {
        const Eigen::Vector3i at = center + offset;
        if (prior_.valid(at)) {inflated_[address(at)] = 1;}
      }
    }
    ++revision_;
    return true;
  }
  bool dynamicOccupied(const Eigen::Vector3d & p) const
  {
    if (!p.allFinite()) {return true;}
    const auto i = prior_.toIdx(p);
    return prior_.valid(i) && inflated_[address(i)];
  }
  bool occupied(const Eigen::Vector3d & p) const
  {
    if (!p.allFinite() || prior_.occupied(p)) {return true;}
    const size_t key = address(prior_.toIdx(p));
    return inflated_[key] || (unknown_occupied_ && !seen_[key] && !knownLaunchOrBody(p));
  }
  void setBodyPosition(const Eigen::Vector3d & p) {body_ = p; have_body_ = p.allFinite();}
  OccupancyGrid routeGrid() const
  {
    auto grid = prior_;
    for (size_t key = 0; key < count_; ++key) {
      if (inflated_[key]) {grid.markOccupied(prior_.toPos(index(key)));}
    }
    return grid;
  }
  const std::vector<Eigen::Vector3d> & observedObstacles() const {return occupied_;}
  uint64_t revision() const {return revision_;}
  double resolution() const {return resolution_;}
private:
  bool knownLaunchOrBody(const Eigen::Vector3d & p) const
  {
    if (have_body_ && (p - body_).norm() <= .12) {return true;}
    for (const auto & home : launches_) {
      if ((p - home).head<2>().norm() <= .35 && p.z() >= home.z() && p.z() <= home.z() + .65) {return true;}
    }
    return false;
  }
  size_t address(const Eigen::Vector3i & i) const {return (static_cast<size_t>(i.x()) * size_.y() + i.y()) * size_.z() + i.z();}
  Eigen::Vector3i index(size_t key) const
  {
    const int z = key % size_.z(); key /= size_.z();
    const int y = key % size_.y();
    return {static_cast<int>(key / size_.y()), y, z};
  }
  double resolution_;
  bool unknown_occupied_;
  OccupancyGrid prior_, raw_prior_;
  Eigen::Vector3i size_;
  size_t count_;
  uint64_t revision_{0};
  std::vector<float> odds_;
  std::vector<bool> seen_;
  std::vector<int8_t> marks_, inflated_;
  std::vector<Eigen::Vector3i> offsets_;
  std::vector<Eigen::Vector3d> occupied_;
  std::vector<Eigen::Vector3d> launches_;
  Eigen::Vector3d body_{Eigen::Vector3d::Zero()};
  bool have_body_{false};
};
}
