// SPDX-License-Identifier: GPL-3.0-only
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rm_drone_msgs/msg/planner_goal.hpp>
#include <rm_drone_msgs/msg/local_trajectory.hpp>
#include <rm_drone_msgs/msg/planner_status.hpp>
#include <ego_planner/planner_manager.h>
#include "rm_drone_planner/spline_trajectory.hpp"
#include "rm_ego_planner/local_map.hpp"

namespace rm_drone
{
using Vec = Eigen::Vector3d;
class EgoLocalPlanner : public rclcpp::Node
{
public:
  EgoLocalPlanner() : Node("ego_local_planner")
  {
    FieldMap field;
    field.load(declare_parameter<std::string>("field_map", ""));
    GridConfig cfg;
    cfg.resolution = declare_parameter("resolution", .1);
    cfg.obstacle_margin = declare_parameter("obstacle_margin", .14);
    cfg.wall_margin = declare_parameter("wall_margin", .45);
    cfg.floor_margin = declare_parameter("floor_margin", .3);
    cfg.ceiling = declare_parameter("ceiling", 2.2);
    const bool prior = declare_parameter("use_static_prior", true);
    const bool unknown = declare_parameter("unknown_is_occupied", !prior);
    for (double value : {cfg.resolution, cfg.obstacle_margin, cfg.wall_margin, cfg.floor_margin, cfg.ceiling}) {
      if (!std::isfinite(value)) {throw std::invalid_argument("non-finite local grid parameter");}
    }
    if (cfg.resolution < .05 || cfg.resolution > .3 || cfg.obstacle_margin < .1 || cfg.ceiling <= .5) {
      throw std::invalid_argument("invalid local grid resolution/inflation/ceiling");
    }
    map_ = std::make_unique<LocalMap>(field, cfg, prior, unknown, cfg.obstacle_margin);
    speed_ = declare_parameter("max_speed", 1.2);
    acceleration_ = declare_parameter("max_acceleration", 2.0);
    braking_ = declare_parameter("braking_acceleration", 2.0);
    horizon_ = declare_parameter("local_horizon", 2.2);
    period_ = declare_parameter("replan_period", .25);
    budget_ = declare_parameter("planning_budget", .2);
    sensor_timeout_ = declare_parameter("sensor_timeout", .5);
    odom_timeout_ = declare_parameter("odom_timeout", .3);
    goal_timeout_ = declare_parameter("goal_timeout", .6);
    range_ = declare_parameter("cloud_max_range", 6.0);
    min_range_ = declare_parameter("cloud_min_range", .15);
    stride_ = declare_parameter("pixel_stride", 3);
    max_rays_ = declare_parameter("max_rays", 6000);
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "depth_optical");
    for (double value : {speed_, acceleration_, braking_, horizon_, period_, budget_, sensor_timeout_,
      odom_timeout_, goal_timeout_, range_, min_range_}) {
      if (!std::isfinite(value)) {throw std::invalid_argument("non-finite local planner parameter");}
    }
    if (speed_ <= 0 || acceleration_ <= 0 || braking_ <= 0 || horizon_ <= .3 ||
      period_ < .05 || budget_ <= 0 || sensor_timeout_ <= 0 || odom_timeout_ <= 0 ||
      goal_timeout_ <= 0 || min_range_ < 0 || range_ <= min_range_ || stride_ < 1 ||
      max_rays_ < 100 || max_rays_ > 100000) {throw std::invalid_argument("invalid local planner limits");}
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("depth/points", qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {pending_cloud_ = std::move(m);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", qos,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        const auto & p = m->pose.pose.position;
        const auto & q = m->pose.pose.orientation;
        const auto & v = m->twist.twist.linear;
        Vec position(p.x, p.y, p.z), velocity(v.x, v.y, v.z);
        Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
        const auto stamp = rclcpp::Time(m->header.stamp);
        const double age = (now() - stamp).seconds();
        if (m->header.frame_id != "map" || stamp.nanoseconds() <= 0 || age < -.05 || age > odom_timeout_ ||
          !position.allFinite() || !velocity.allFinite() || !orientation.coeffs().allFinite() ||
          orientation.norm() < 1e-6 || (have_odom_ && stamp < odom_stamp_)) {have_odom_ = false; return;}
        p_ = position;
        map_->setBodyPosition(p_);
        v_ = orientation.normalized() * velocity;  // Odometry twist is in child frame.
        odom_stamp_ = stamp;
        have_odom_ = true;
      });
    goal_sub_ = create_subscription<rm_drone_msgs::msg::PlannerGoal>("ego/goal", 1,
      [this](rm_drone_msgs::msg::PlannerGoal::SharedPtr m) {
        const Vec p(m->position.x, m->position.y, m->position.z);
        const double age = (now() - rclcpp::Time(m->header.stamp)).seconds();
        if (m->header.frame_id != "map" || !p.allFinite() || age < -.05 || age > goal_timeout_) {return;}
        if (!m->active || m->request_id != request_.request_id) {active_ = false; last_plan_t_ = -1;}
        request_ = *m;
      });
    trajectory_pub_ = create_publisher<rm_drone_msgs::msg::LocalTrajectory>("ego/trajectory", 1);
    status_pub_ = create_publisher<rm_drone_msgs::msg::PlannerStatus>("ego/status", 1);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("ego/occupancy", qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("planner/markers", 1);
  }
  void initialize()
  {
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, shared_from_this(), true);
    auto node = shared_from_this();
    auto vis = std::make_shared<ego_planner::PlanningVisualization>(node);
    planner_.initPlanModules(node, vis);
    planner_.deliverTrajToOptimizer();
    planner_.setDroneIdtoOpt();
    planner_.grid_map_->setQuery([this](const Vec & p) {return map_->occupied(p);});
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "EGO local planner: timestamped PointCloud2 fusion, %.2f m grid, single setpoint owner=mission", map_->resolution());
  }
private:
  static Eigen::Isometry3d transform(const geometry_msgs::msg::Transform & t)
  {
    Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6) {throw std::runtime_error("invalid sensor TF quaternion");}
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = q.normalized().toRotationMatrix();
    result.translation() = Vec(t.translation.x, t.translation.y, t.translation.z);
    if (!result.matrix().allFinite()) {throw std::runtime_error("invalid sensor TF translation");}
    return result;
  }
  void fuseCloud()
  {
    if (!pending_cloud_) {return;}
    const auto & msg = *pending_cloud_;
    const rclcpp::Time stamp(msg.header.stamp);
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || age < -.05 || age > sensor_timeout_ || msg.header.frame_id.empty() ||
      (have_cloud_ && stamp <= cloud_stamp_)) {pending_cloud_.reset(); cloud_error_ = "depth timestamp rejected"; return;}
    if (msg.is_bigendian || !msg.width || !msg.height || msg.point_step < 12 ||
      uint64_t(msg.width) * msg.height > 4000000 || uint64_t(msg.row_step) < uint64_t(msg.width) * msg.point_step ||
      msg.data.size() < uint64_t(msg.height) * msg.row_step) {
      pending_cloud_.reset(); cloud_error_ = "malformed PointCloud2"; return;
    }
    int offsets[3] = {-1, -1, -1};
    for (const auto & field : msg.fields) {
      for (int i = 0; i < 3; ++i) {
        if (field.name == std::string(1, "xyz"[i]) && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
          field.count == 1 && uint64_t(field.offset) + 4 <= msg.point_step) {offsets[i] = field.offset;}
      }
    }
    if (offsets[0] < 0 || offsets[1] < 0 || offsets[2] < 0) {pending_cloud_.reset(); cloud_error_ = "PointCloud2 requires float32 XYZ"; return;}
    Eigen::Isometry3d cloud_to_map, sensor_to_map;
    try {
      cloud_to_map = transform(buffer_->lookupTransform("map", msg.header.frame_id, stamp).transform);
      const std::string source = sensor_frame_.empty() ? msg.header.frame_id : sensor_frame_;
      if (source == "map") {throw std::runtime_error("sensor_frame must identify the physical depth sensor");}
      sensor_to_map = source == msg.header.frame_id ? cloud_to_map : transform(buffer_->lookupTransform("map", source, stamp).transform);
    } catch (const std::exception & e) {cloud_error_ = std::string("waiting for image-time TF: ") + e.what(); return;}
    const Vec origin = sensor_to_map.translation();
    std::vector<DepthRay> rays;
    const double sampling = double(msg.width) * msg.height / max_rays_;
    const int step = std::max(stride_, static_cast<int>(std::ceil(msg.height == 1 ? sampling : std::sqrt(sampling))));
    rays.reserve(max_rays_);
    for (uint32_t row = 0; row < msg.height && rays.size() < size_t(max_rays_); row += step) {
      for (uint32_t col = 0; col < msg.width && rays.size() < size_t(max_rays_); col += step) {
        const uint8_t * bytes = msg.data.data() + size_t(row) * msg.row_step + size_t(col) * msg.point_step;
        float xyz[3];
        for (int axis = 0; axis < 3; ++axis) {std::memcpy(&xyz[axis], bytes + offsets[axis], 4);}
        const Vec local(xyz[0], xyz[1], xyz[2]);
        if (!local.allFinite()) {continue;}
        Vec end = cloud_to_map * local;
        const Vec delta = end - origin;
        const double distance = delta.norm();
        if (distance < min_range_) {continue;}
        const bool hit = distance < range_;
        if (!hit) {end = origin + delta * (range_ / distance);}
        rays.push_back({end, hit});
      }
    }
    if (map_->fuse(origin, rays)) {
      cloud_stamp_ = stamp;
      have_cloud_ = true;
      cloud_error_.clear();
    } else {cloud_error_ = "no usable depth rays";}
    pending_cloud_.reset();
  }
  bool healthy() const
  {
    return have_odom_ && have_cloud_ && (now() - odom_stamp_).seconds() <= odom_timeout_ &&
      (now() - cloud_stamp_).seconds() <= sensor_timeout_;
  }
  bool motionSafe() const
  {
    const double speed = v_.norm();
    const double delay = have_cloud_ ? std::max(0., (now() - cloud_stamp_).seconds()) : sensor_timeout_;
    const double time = std::min(2., speed / braking_ + delay + .15);
    const int samples = std::max(1, static_cast<int>(std::ceil(speed * time / (map_->resolution() * .5))));
    for (int i = 0; i <= samples; ++i) {
      if (map_->dynamicOccupied(p_ + v_ * (time * i / samples))) {return false;}
    }
    return true;
  }
  double firstCollision(double from)
  {
    if (!active_) {return 0;}
    const double start = std::max(0., from), end = active_spline_.duration();
    if (start > end) {return std::numeric_limits<double>::infinity();}
    const int samples = std::max(1, static_cast<int>(std::ceil((end - start) / .02)));
    for (int i = 0; i <= samples; ++i) {
      const double t = start + (end - start) * i / samples;
      if (map_->occupied(active_spline_.position(t))) {return t - from;}
    }
    return std::numeric_limits<double>::infinity();
  }
  bool makePlan()
  {
    const auto start_wall = std::chrono::steady_clock::now();
    auto grid = map_->routeGrid();
    const Vec final_goal(request_.position.x, request_.position.y, request_.position.z);
    std::vector<Vec> route;
    if (!astar(grid, p_, final_goal, route)) {reason_ = "no free route to mission goal"; return false;}
    Vec target = p_;
    double arc = 0;
    for (size_t i = 1; i < route.size(); ++i) {
      if (map_->occupied(route[i])) {break;}
      arc += (route[i] - route[i - 1]).norm();
      target = route[i];
      if (arc >= horizon_) {break;}
    }
    if ((target - p_).norm() < .25) {reason_ = "waiting for an observed free corridor"; return false;}
    bool ok = planner_.reboundReplan(p_, v_, Vec::Zero(), target, Vec::Zero(), true, false);
    if (!ok) {reason_ = "EGO rebound optimization failed"; return false;}
    auto & trajectory = planner_.local_data_;
    rm_drone_msgs::msg::LocalTrajectory message;
    message.header.frame_id = "map";
    message.header.stamp = now();
    message.request_id = request_.request_id;
    message.revision = ++trajectory_revision_;
    message.map_revision = map_->revision();
    message.degree = 3;
    const auto knots = trajectory.position_traj_.getKnot();
    for (int i = 0; i < knots.size(); ++i) {message.knots.push_back(knots[i] - knots[3]);}
    const auto points = trajectory.position_traj_.getControlPoint();
    for (int i = 0; i < points.cols(); ++i) {
      geometry_msgs::msg::Point point;
      point.x = points(0, i); point.y = points(1, i); point.z = points(2, i);
      message.control_points.push_back(point);
    }
    SplineTrajectory candidate;
    if (!candidate.assign(message)) {reason_ = "invalid EGO spline"; return false;}
    const int samples = std::max(1, static_cast<int>(std::ceil(candidate.duration() / .02)));
    for (int i = 0; i <= samples; ++i) {
      const double t = candidate.duration() * i / samples;
      const Vec p = candidate.position(t), v = candidate.velocity(t), a = candidate.acceleration(t);
      if (!p.allFinite() || !v.allFinite() || !a.allFinite() || map_->occupied(p) ||
        v.norm() > speed_ * 1.1 || a.norm() > acceleration_ * 1.1) {
        reason_ = "EGO spline failed full collision/velocity/acceleration validation"; return false;
      }
    }
    planning_ms_ = 1000 * std::chrono::duration<double>(std::chrono::steady_clock::now() - start_wall).count();
    if (planning_ms_ > 1000 * budget_ || !healthy()) {reason_ = "planning budget or sensor freshness exceeded"; return false;}
    active_spline_ = std::move(candidate);
    active_message_ = message;
    active_ = true;
    blocked_ = false;
    reason_ = "EGO trajectory validated";
    trajectory_pub_->publish(message);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "EGO plan #%lu map=%lu %.1fms horizon=(%.2f %.2f %.2f)",
      message.revision, message.map_revision, planning_ms_, target.x(), target.y(), target.z());
    return true;
  }
  void publishStatus(bool ready)
  {
    rm_drone_msgs::msg::PlannerStatus status;
    status.header.frame_id = "map"; status.header.stamp = now();
    status.request_id = request_.request_id;
    status.map_revision = map_->revision();
    status.ready = ready;
    status.blocked = blocked_;
    status.motion_safe = ready && motionSafe();
    status.cloud_age = have_cloud_ ? std::max(0., (now() - cloud_stamp_).seconds()) : 999;
    status.planning_ms = planning_ms_;
    status.reason = !ready ? (cloud_error_.empty() ? "waiting for fresh depth/odometry" : cloud_error_) : reason_;
    status_pub_->publish(status);
  }
  void publishMapAndTrajectory()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map"; cloud.header.stamp = now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    const auto & obstacles = map_->observedObstacles();
    modifier.resize(obstacles.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto & p : obstacles) {*x = p.x(); *y = p.y(); *z = p.z(); ++x; ++y; ++z;}
    if (cloud_pub_->get_subscription_count()) {cloud_pub_->publish(cloud);}
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker trajectory;
    trajectory.header = cloud.header; trajectory.ns = "ego_local_trajectory"; trajectory.id = 0;
    trajectory.type = trajectory.LINE_STRIP; trajectory.action = trajectory.ADD;
    trajectory.pose.orientation.w = 1; trajectory.scale.x = .035;
    trajectory.color.g = 1; trajectory.color.b = .8; trajectory.color.a = 1;
    trajectory.lifetime = rclcpp::Duration::from_seconds(.5);
    if (active_) {
      for (double t = 0; t <= active_spline_.duration(); t += .05) {
        const auto p = active_spline_.position(t);
        geometry_msgs::msg::Point point; point.x = p.x(); point.y = p.y(); point.z = p.z();
        trajectory.points.push_back(point);
      }
    }
    markers.markers.push_back(trajectory);
    auto occupied = trajectory;
    occupied.ns = "ego_observed_obstacles"; occupied.type = occupied.CUBE_LIST;
    occupied.scale.x = occupied.scale.y = occupied.scale.z = map_->resolution();
    occupied.color.r = 1; occupied.color.g = .25; occupied.color.b = .1; occupied.color.a = .65;
    occupied.points.clear();
    const size_t step = std::max<size_t>(1, obstacles.size() / 6000);
    for (size_t i = 0; i < obstacles.size(); i += step) {
      geometry_msgs::msg::Point point; point.x = obstacles[i].x(); point.y = obstacles[i].y(); point.z = obstacles[i].z();
      occupied.points.push_back(point);
    }
    markers.markers.push_back(occupied);
    marker_pub_->publish(markers);
  }
  void tick()
  {
    const double time = now().seconds();
    if (last_tick_ > time + .001) {
      map_->clear(); have_cloud_ = have_odom_ = active_ = false;
      pending_cloud_.reset(); request_.active = false; last_plan_t_ = -1;
    }
    const bool paused = last_tick_ == time;
    last_tick_ = time;
    fuseCloud();
    if (paused) {return;}
    const bool ready = healthy();
    if (!request_.active || (now() - rclcpp::Time(request_.header.stamp)).seconds() > goal_timeout_) {
      active_ = false; blocked_ = false; reason_ = "navigation inactive";
    } else if (!ready) {
      active_ = false; blocked_ = true;
    } else {
      const Vec goal(request_.position.x, request_.position.y, request_.position.z);
      const double elapsed = active_ ? (now() - rclcpp::Time(active_message_.header.stamp)).seconds() : 0;
      const double collision = firstCollision(elapsed);
      if (!motionSafe() || (active_ && collision < .6)) {
        blocked_ = true; reason_ = "obstacle inside braking horizon";
        publishStatus(ready);  // Do not wait for optimization before requesting braking.
      }
      const bool reached = (p_ - goal).norm() < .2 && v_.norm() < .2;
      const bool need = !active_ || collision < 1.5 || time - last_plan_t_ > period_;
      if (!reached && need) {
        last_plan_t_ = time;
        if (!makePlan() && (!active_ || collision < .8 || elapsed >= active_spline_.duration())) {blocked_ = true;}
      }
    }
    publishStatus(ready);
    if (time - last_vis_ > .2) {last_vis_ = time; publishMapAndTrajectory();}
  }
  std::unique_ptr<LocalMap> map_;
  ego_planner::EGOPlannerManager planner_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  Vec p_{Vec::Zero()}, v_{Vec::Zero()};
  bool have_odom_{false}, have_cloud_{false}, active_{false}, blocked_{false};
  double speed_, acceleration_, braking_, horizon_, period_, budget_, sensor_timeout_, odom_timeout_, goal_timeout_, range_, min_range_;
  double last_tick_{-1}, last_plan_t_{-1}, last_vis_{-1}, planning_ms_{0};
  int stride_, max_rays_;
  uint64_t trajectory_revision_{0};
  rclcpp::Time cloud_stamp_{0,0,RCL_ROS_TIME}, odom_stamp_{0,0,RCL_ROS_TIME};
  std::string sensor_frame_, reason_, cloud_error_;
  SplineTrajectory active_spline_;
  rm_drone_msgs::msg::PlannerGoal request_;
  rm_drone_msgs::msg::LocalTrajectory active_message_;
  sensor_msgs::msg::PointCloud2::SharedPtr pending_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rm_drone_msgs::msg::PlannerGoal>::SharedPtr goal_sub_;
  rclcpp::Publisher<rm_drone_msgs::msg::LocalTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<rm_drone_msgs::msg::PlannerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rm_drone::EgoLocalPlanner>();
    node->initialize();
    rclcpp::spin(node);
  } catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("ego_local_planner"), "%s", e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown();
  return 0;
}
