#pragma once
// RM integration interface: EGO's collision optimizer queries our timestamped
// PointCloud2 fusion map. No second, unsynchronized mapping pipeline is started.
#include <functional>
#include <memory>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

class GridMap
{
public:
  using Ptr = std::shared_ptr<GridMap>;
  void initMap(rclcpp::Node::SharedPtr node) {node->get_parameter("resolution", resolution_);}
  double getResolution() const {return resolution_;}
  void setQuery(std::function<bool(const Eigen::Vector3d &)> query) {query_ = std::move(query);}
  int getInflateOccupancy(const Eigen::Vector3d & point) const
  {
    return !point.allFinite() || !query_ || query_(point);
  }
private:
  double resolution_{0.1};
  std::function<bool(const Eigen::Vector3d &)> query_;
};
