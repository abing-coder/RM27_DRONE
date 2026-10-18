// 任务状态机：
//   IDLE -> ARMING -> TAKEOFF -> PLAN -> CRUISE -> SEARCH -> (OUTPOST_WAIT) -> ATTACK -> EGRESS -> ... -> LAND -> DONE
// 全程监控：估计器健康（VIO 丢失则原地降落）、电子围栏（软围栏悬停、硬围栏停桨，对应规则 R48）。
#include <deque>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/armor_detection_array.hpp>
#include <rm_drone_msgs/msg/estimator_status.hpp>
#include <rm_drone_msgs/msg/impact_event.hpp>
#include <rm_drone_msgs/msg/mission_state.hpp>
#include <rm_drone_msgs/msg/trajectory_setpoint.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_drone_control/math_utils.hpp"
#include "rm_drone_planner/planning.hpp"

namespace rm_drone
{

using Vec3 = Eigen::Vector3d;

enum class State { IDLE, ARMING, TAKEOFF, PLAN, CRUISE, SEARCH, OUTPOST_WAIT, ATTACK, EGRESS, LAND, DONE, ESTOP };

static const char * Name(State s)
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::ARMING: return "ARMING";
    case State::TAKEOFF: return "TAKEOFF";
    case State::PLAN: return "PLAN";
    case State::CRUISE: return "CRUISE";
    case State::SEARCH: return "SEARCH";
    case State::OUTPOST_WAIT: return "OUTPOST_WAIT";
    case State::ATTACK: return "ATTACK";
    case State::EGRESS: return "EGRESS";
    case State::LAND: return "LAND";
    case State::DONE: return "DONE";
    case State::ESTOP: return "ESTOP";
  }
  return "?";
}

class MissionNode : public rclcpp::Node
{
public:
  MissionNode()
  : Node("mission")
  {
    team_ = declare_parameter<std::string>("team", "red");
    target_name_ = declare_parameter<std::string>("target", "auto");
    auto_start_ = declare_parameter("auto_start", true);
    start_delay_ = declare_parameter("start_delay", 3.0);
    require_vio_ = declare_parameter("require_vio", true);
    takeoff_h_ = declare_parameter("takeoff_height", 0.6);
    cruise_v_ = declare_parameter("cruise_speed", 3.0);
    cruise_a_ = declare_parameter("cruise_acc", 3.0);
    standoff_ = declare_parameter("standoff_distance", 2.0);
    attack_v_ = declare_parameter("attack_speed", 2.5);
    max_attacks_ = declare_parameter("max_attacks", 0);
    search_timeout_ = declare_parameter("search_timeout", 4.0);
    blind_attack_ = declare_parameter("allow_blind_attack", true);
    impact_acc_ = declare_parameter("impact_acc_threshold", 30.0);
    soft_fence_ = declare_parameter("soft_fence_margin", 0.25);
    hard_fence_ = declare_parameter("hard_fence_margin", 0.05);
    ceiling_ = declare_parameter("ceiling", 2.3);
    drone_entity_ = declare_parameter<std::string>("drone_entity_name", "rm_drone");
    phase_tol_ = declare_parameter("outpost_phase_tolerance", 0.12);
    latency_ = declare_parameter("guidance_latency", 0.12);

    GridConfig gc;
    gc.resolution = declare_parameter("grid_resolution", 0.1);
    gc.obstacle_margin = declare_parameter("obstacle_margin", 0.30);
    gc.wall_margin = declare_parameter("wall_margin", 0.45);
    gc.floor_margin = declare_parameter("floor_margin", 0.30);
    gc.ceiling = declare_parameter("plan_ceiling", 2.1);
    const auto map_path = declare_parameter<std::string>("field_map", "");
    field_.load(map_path);
    const auto t0 = std::chrono::steady_clock::now();
    grid_.build(field_, gc);
    RCLCPP_INFO(get_logger(), "栅格地图 %dx%dx%d 构建耗时 %.2fs", grid_.size().x(), grid_.size().y(), grid_.size().z(),
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());

    auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", qos, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        p_ = Vec3(m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z);
        q_ = toEigen(m->pose.pose.orientation).normalized();
        v_ = q_ * toEigen(m->twist.twist.linear);
        have_odom_ = true;
      });
    status_sub_ = create_subscription<rm_drone_msgs::msg::EstimatorStatus>("estimator_status", 10,
      [this](rm_drone_msgs::msg::EstimatorStatus::SharedPtr m) {est_ = *m; have_est_ = true;});
    armor_sub_ = create_subscription<rm_drone_msgs::msg::ArmorDetectionArray>("armors", 10,
      std::bind(&MissionNode::OnArmors, this, std::placeholders::_1));
    impact_sub_ = create_subscription<rm_drone_msgs::msg::ImpactEvent>("/referee/impact", 10,
      [this](rm_drone_msgs::msg::ImpactEvent::SharedPtr m) {
        if (m->striker_name == drone_entity_) {impact_flag_ = true;}
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu", qos, [this](sensor_msgs::msg::Imu::SharedPtr m) {
        const Vec3 a(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
        if (state_ == State::ATTACK && (a - q_.conjugate() * Vec3(0, 0, 9.81)).norm() > impact_acc_) {impact_flag_ = true;}
      });
    start_sub_ = create_subscription<std_msgs::msg::Empty>("mission/start", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {start_requested_ = true;});

    sp_pub_ = create_publisher<rm_drone_msgs::msg::TrajectorySetpoint>("setpoint", 10);
    arm_pub_ = create_publisher<std_msgs::msg::Bool>("arm", 10);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("estop", 10);
    state_pub_ = create_publisher<rm_drone_msgs::msg::MissionState>("mission_state", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("planner/markers", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&MissionNode::Step, this));
  }

private:
  // ------------------------------------------------------------ 目标
  bool SelectTarget()
  {
    const std::string enemy = team_ == "red" ? "blue" : "red";
    const Target * best = nullptr;
    if (target_name_ != "auto") {
      best = field_.findTarget(target_name_);
    } else {
      double best_score = -1e9;
      for (const auto & t : field_.targets) {
        if (t.team != enemy || t.kind != "base") {continue;}
        const Vec3 n(std::cos(t.normal_yaw), std::sin(t.normal_yaw), 0.0);
        const Vec3 s = t.position + n * standoff_;
        if (grid_.occupied(s)) {continue;}
        const double score = n.dot((p_ - t.position).normalized()) - 0.02 * (p_ - s).norm();
        if (score > best_score) {
          best_score = score;
          best = &t;
        }
      }
    }
    if (!best) {return false;}
    target_ = *best;
    if (target_.kind == "outpost") {
      Vec3 d = p_ - target_.position;
      d.z() = 0.0;
      d.normalize();
      normal_ = d;
      aim_ = target_.position + d * target_.radius;
    } else {
      normal_ = Vec3(std::cos(target_.normal_yaw), std::sin(target_.normal_yaw), 0.0);
      aim_ = target_.position;
    }
    standoff_pt_ = aim_ + normal_ * standoff_;
    Vec3 free_pt;
    if (grid_.nearestFree(standoff_pt_, free_pt, 0.6)) {standoff_pt_ = free_pt;}
    RCLCPP_INFO(get_logger(), "目标 %s：瞄准点 (%.2f %.2f %.2f)，待机点 (%.2f %.2f %.2f)", target_.name.c_str(),
      aim_.x(), aim_.y(), aim_.z(), standoff_pt_.x(), standoff_pt_.y(), standoff_pt_.z());
    return true;
  }

  void OnArmors(rm_drone_msgs::msg::ArmorDetectionArray::SharedPtr m)
  {
    const double t = now().seconds();
    for (const auto & a : m->armors) {
      if (!a.has_map_pose) {continue;}
      const Vec3 p(a.pose_map.position.x, a.pose_map.position.y, a.pose_map.position.z);
      if (target_.kind == "outpost") {
        Vec3 r = p - target_.position;
        r.z() = 0.0;
        if (std::abs(r.norm() - target_.radius) > 0.25 || std::abs(p.z() - target_.position.z()) > 0.3) {continue;}
        UpdateOutpostPhase(t, std::atan2(r.y(), r.x()));
        last_match_ = t;
        ++match_count_;
      } else if ((p - target_.position).norm() < 0.7) {
        const double alpha = match_count_ == 0 ? 1.0 : 0.3;
        aim_filtered_ = alpha * p + (1 - alpha) * aim_filtered_;
        last_match_ = t;
        ++match_count_;
      }
    }
  }

  // 前哨站相位估计：三块装甲 120° 均布，用装甲中心绕前哨站中心的方位角估计相位与角速度
  void UpdateOutpostPhase(double t, double psi)
  {
    const double sector = 2.0 * M_PI / 3.0;
    auto wrapSector = [&](double a) {
        a = std::fmod(a + sector / 2, sector);
        if (a < 0) {a += sector;}
        return a - sector / 2;
      };
    if (phase_t_ > 0 && t - phase_t_ < 0.3) {
      const double pred = phase_ + omega_ * (t - phase_t_);
      const double dpsi = wrapSector(psi - pred);
      const double dt = t - phase_t_;
      const double w_meas = omega_ + dpsi / std::max(dt, 1e-3);
      omega_ = 0.85 * omega_ + 0.15 * w_meas;
      phase_ = pred + 0.5 * dpsi;
    } else {
      phase_ = psi;
    }
    phase_t_ = t;
  }

  bool OutpostReady(double t)
  {
    if (t - phase_t_ > 0.3) {return false;}
    const double d_yaw = std::atan2(normal_.y(), normal_.x());
    if (std::abs(omega_) < 0.15) {return std::abs(std::remainder(d_yaw - phase_, 2 * M_PI / 3)) < 0.3;}  // 已停转
    const double t_go = (p_ - aim_).norm() / attack_v_ + latency_;
    const double pred = phase_ + omega_ * (t + t_go - phase_t_);
    return std::abs(std::remainder(d_yaw - pred, 2.0 * M_PI / 3.0)) < phase_tol_;
  }

  // ------------------------------------------------------------ 主循环
  void Step()
  {
    if (!have_odom_) {return;}
    const double t = now().seconds();
    if (state_ != prev_state_) {
      RCLCPP_INFO(get_logger(), "状态 %s -> %s", Name(prev_state_), Name(state_));
      prev_state_ = state_;
      state_t0_ = t;
    }
    const double ts = t - state_t0_;
    const bool flying = state_ != State::IDLE && state_ != State::DONE && state_ != State::ESTOP;

    // 安全监控
    if (flying && state_ != State::ARMING) {
      if (!field_.insideField(p_, hard_fence_, 3.5)) {
        RCLCPP_ERROR(get_logger(), "越过硬围栏 (%.2f %.2f %.2f)，紧急停桨", p_.x(), p_.y(), p_.z());
        Estop();
        return;
      }
      if (require_vio_ && have_est_ && !est_.vio_ok && state_ != State::LAND && state_ != State::TAKEOFF) {
        if (++vio_bad_ > 50) {
          RCLCPP_WARN(get_logger(), "VIO 失效超过 1s，原地降落");
          land_xy_ = p_;
          state_ = State::LAND;
        }
      } else {
        vio_bad_ = 0;
      }
    }

    rm_drone_msgs::msg::TrajectorySetpoint sp;
    sp.header.stamp = now();
    sp.header.frame_id = "map";
    Vec3 pos = hold_, vel = Vec3::Zero(), acc = Vec3::Zero();
    double yaw = yaw_sp_;

    switch (state_) {
      case State::IDLE: {
          hold_ = p_;
          pos = p_;
          yaw_sp_ = yaw = yawOf(q_);
          const bool ready = have_est_ && est_.initialized && (!require_vio_ || est_.vio_ok);
          if (ready && ready_since_ < 0) {ready_since_ = t;}
          if (!ready) {ready_since_ = -1;}
          if (ready && (start_requested_ || (auto_start_ && t - ready_since_ > start_delay_))) {
            home_ = p_;
            state_ = State::ARMING;
          }
          PublishArm(false);
          break;
        }
      case State::ARMING:
        PublishArm(true);
        pos = home_;
        if (ts > 1.0) {
          traj_.generate({home_, home_ + Vec3(0, 0, takeoff_h_)}, Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), 1.0, 2.0);
          traj_t0_ = t;
          state_ = State::TAKEOFF;
        }
        break;
      case State::TAKEOFF: {
          const auto st = traj_.sample(t - traj_t0_);
          pos = st.p;
          vel = st.v;
          acc = st.a;
          const double z_cap = home_.z() + takeoff_h_ + 0.15;
          if (p_.z() > z_cap) {
            pos.z() = z_cap;
            vel.z() = std::min(vel.z(), 0.0);
            acc.z() = std::min(acc.z(), 0.0);
          }
          if (t - traj_t0_ > traj_.duration() && (p_ - st.p).norm() < 0.15) {
            hold_ = st.p;
            state_ = State::PLAN;
          }
          break;
        }
      case State::PLAN: {
          pos = hold_;
          if (ts < 0.2) {break;}
          if (!SelectTarget()) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "没有可用目标");
            break;
          }
          std::vector<Vec3> raw;
          if (!astar(grid_, p_, standoff_pt_, raw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "A* 失败，重试");
            break;
          }
          path_ = simplifyPath(grid_, raw);
          if (!traj_.generateSafe(grid_, path_, Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), cruise_v_, cruise_a_)) {
            RCLCPP_WARN(get_logger(), "min-snap 轨迹有碰撞，退化为降速重规划");
            traj_.generateSafe(grid_, path_, Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), 0.6 * cruise_v_, cruise_a_);
          }
          RCLCPP_INFO(get_logger(), "规划完成：A* %zu 点 -> 简化 %zu 点，轨迹 %.2fs", raw.size(), path_.size(), traj_.duration());
          traj_t0_ = t;
          PublishPlanMarkers();
          state_ = State::CRUISE;
          break;
        }
      case State::CRUISE: {
          const auto st = traj_.sample(t - traj_t0_);
          pos = st.p;
          vel = st.v;
          acc = st.a;
          yaw = YawToward(aim_, t);
          if (t - traj_t0_ > traj_.duration() && (p_ - standoff_pt_).norm() < 0.3) {
            hold_ = standoff_pt_;
            match_count_ = 0;
            state_ = target_.kind == "outpost" ? State::OUTPOST_WAIT : State::SEARCH;
          }
          break;
        }
      case State::SEARCH: {
          pos = hold_;
          yaw = YawToward(aim_, t);
          const bool seen = match_count_ >= 5 && t - last_match_ < 0.3;
          if (seen && ts > 0.5) {
            aim_ = aim_filtered_;
            BeginAttack(t);
          } else if (ts > search_timeout_ && blind_attack_) {
            RCLCPP_WARN(get_logger(), "未识别到装甲，按场地地图先验位置盲打");
            BeginAttack(t);
          }
          break;
        }
      case State::OUTPOST_WAIT:
        pos = hold_;
        yaw = YawToward(target_.position, t);
        if (match_count_ >= 5 && OutpostReady(t)) {
          RCLCPP_INFO(get_logger(), "前哨站相位就绪 omega=%.2f rad/s，出击", omega_);
          BeginAttack(t);
        } else if (ts > 15.0) {
          RCLCPP_WARN(get_logger(), "前哨站相位等待超时，直接出击");
          BeginAttack(t);
        }
        break;
      case State::ATTACK: {
          // 末端导引：沿装甲法向直线逼近，侧向误差由位置环消除，速度前馈 = -n * v_attack
          if (target_.kind != "outpost" && t - last_match_ < 0.2 && (p_ - aim_).norm() > 0.6) {
            aim_ = 0.7 * aim_ + 0.3 * aim_filtered_;
          }
          const Vec3 rel = p_ - aim_;
          const double s = rel.dot(normal_);
          const double closing = -v_.dot(normal_);
          // 先退到 1.2 m 外，并且本次接近速度超过 1 m/s，才承认撞击。
          // 否则贴在装甲上的残余接触会被记成下一次攻击，三次后就降落。
          if (s > 1.2) {approach_ready_ = true;}
          if (approach_ready_ && closing > peak_closing_) {peak_closing_ = closing;}
          if (approach_ready_ && closing > 1.0) {had_speed_ = true;}
          const double carrot = std::max(s - attack_v_ * 0.3, -0.15);
          pos = aim_ + normal_ * carrot;
          vel = -normal_ * attack_v_;
          yaw = std::atan2(-normal_.y(), -normal_.x());
          yaw_sp_ = yaw;
          const bool hit = had_speed_ && (impact_flag_ || s < -0.05);
          if (hit || ts > 6.0) {
            if (hit) {
              ++attacks_;
              RCLCPP_INFO(get_logger(), "第 %d 次撞击 %s：法向距离 %.2f m，接近速度 %.2f m/s",
                attacks_, target_.name.c_str(), s, peak_closing_);
            } else {
              RCLCPP_WARN(get_logger(), "未形成有效撞击（法向 %.2f m，接近速度 %.2f m/s），退回重来", s, closing);
            }
            impact_flag_ = false;
            approach_ready_ = false;
            had_speed_ = false;
            peak_closing_ = 0.0;
            state_ = State::EGRESS;
          }
          break;
        }
      case State::EGRESS:
        // 沿法向退回待机点，退出接触后再加速撞下一次。max_attacks<=0 时不降落。
        pos = standoff_pt_;
        vel = normal_ * std::min(1.5, attack_v_);
        yaw = std::atan2(-normal_.y(), -normal_.x());
        if ((p_ - aim_).dot(normal_) > std::max(1.4, 0.7 * standoff_)) {
          if (max_attacks_ > 0 && attacks_ >= max_attacks_) {
            land_xy_ = p_;
            state_ = State::LAND;
          } else {
            match_count_ = 0;
            impact_flag_ = false;
            BeginAttack(t);
          }
        }
        break;
      case State::LAND: {
          const double terrain = field_.terrainHeight(land_xy_.x(), land_xy_.y(), p_.z() - 0.05);
          const double z = std::max(terrain - 0.1, p_.z() - 0.5);
          pos = Vec3(land_xy_.x(), land_xy_.y(), z);
          vel = Vec3(0, 0, -0.4);
          if (p_.z() - terrain < 0.06 || (ts > 2.0 && std::abs(v_.z()) < 0.03 && p_.z() - terrain < 0.15)) {
            RCLCPP_INFO(get_logger(), "降落完成，停桨");
            PublishArm(false);
            state_ = State::DONE;
          }
          break;
        }
      case State::DONE:
      case State::ESTOP:
        PublishArm(false);
        pos = p_;
        break;
    }

    // 软围栏：把设定点夹在场地内
    if (flying) {
      const double lx = field_.length / 2 - soft_fence_ - 0.2, ly = field_.width / 2 - soft_fence_ - 0.2;
      pos.x() = std::clamp(pos.x(), -lx, lx);
      pos.y() = std::clamp(pos.y(), -ly, ly);
      pos.z() = std::min(pos.z(), ceiling_);
    }

    sp.position = toMsg(pos);
    sp.velocity = toMsg(vel);
    sp.acceleration = toMsg(acc);
    sp.yaw = yaw;
    sp_pub_->publish(sp);
    if (flying && state_ != State::ARMING) {PublishArm(true);}

    rm_drone_msgs::msg::MissionState ms;
    ms.header.stamp = now();
    ms.state = Name(state_);
    ms.target = target_.name;
    ms.distance_to_target = static_cast<float>((p_ - aim_).norm());
    ms.info = "attacks=" + std::to_string(attacks_);
    state_pub_->publish(ms);
    if (++marker_count_ % 10 == 0) {PublishStateMarkers(pos);}
  }

  void BeginAttack(double t)
  {
    impact_flag_ = false;
    approach_ready_ = false;
    had_speed_ = false;
    peak_closing_ = 0.0;
    (void)t;
    state_ = State::ATTACK;
  }

  double YawToward(const Vec3 & target, double t)
  {
    const double desired = std::atan2(target.y() - p_.y(), target.x() - p_.x());
    const double dt = last_yaw_t_ > 0 ? std::clamp(t - last_yaw_t_, 0.0, 0.1) : 0.02;
    last_yaw_t_ = t;
    const double err = wrapPi(desired - yaw_sp_);
    yaw_sp_ = wrapPi(yaw_sp_ + std::clamp(err, -1.5 * dt, 1.5 * dt));
    return yaw_sp_;
  }

  void PublishArm(bool a)
  {
    std_msgs::msg::Bool m;
    m.data = a;
    arm_pub_->publish(m);
  }

  void Estop()
  {
    std_msgs::msg::Bool m;
    m.data = true;
    estop_pub_->publish(m);
    PublishArm(false);
    state_ = State::ESTOP;
  }

  // ------------------------------------------------------------ 可视化
  visualization_msgs::msg::Marker MakeMarker(const std::string & ns, int id, int type)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = m.ADD;
    m.pose.orientation.w = 1.0;
    m.color.a = 1.0;
    return m;
  }

  void PublishPlanMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    auto traj = MakeMarker("trajectory", 0, visualization_msgs::msg::Marker::LINE_STRIP);
    traj.scale.x = 0.04;
    traj.color.r = 1.0;
    traj.color.g = 0.6;
    for (double t = 0; t <= traj_.duration(); t += 0.05) {
      const auto st = traj_.sample(t);
      geometry_msgs::msg::Point p;
      p.x = st.p.x();
      p.y = st.p.y();
      p.z = st.p.z();
      traj.points.push_back(p);
    }
    arr.markers.push_back(traj);
    auto wp = MakeMarker("waypoints", 0, visualization_msgs::msg::Marker::SPHERE_LIST);
    wp.scale.x = wp.scale.y = wp.scale.z = 0.12;
    wp.color.b = 1.0;
    wp.color.g = 0.8;
    for (const auto & w : path_) {
      geometry_msgs::msg::Point p;
      p.x = w.x();
      p.y = w.y();
      p.z = w.z();
      wp.points.push_back(p);
    }
    arr.markers.push_back(wp);
    marker_pub_->publish(arr);
  }

  void PublishStateMarkers(const Vec3 & sp)
  {
    visualization_msgs::msg::MarkerArray arr;
    auto aim = MakeMarker("aim", 0, visualization_msgs::msg::Marker::SPHERE);
    aim.pose.position.x = aim_.x();
    aim.pose.position.y = aim_.y();
    aim.pose.position.z = aim_.z();
    aim.scale.x = aim.scale.y = aim.scale.z = 0.15;
    aim.color.r = 1.0;
    arr.markers.push_back(aim);
    auto spm = MakeMarker("setpoint", 0, visualization_msgs::msg::Marker::SPHERE);
    spm.pose.position.x = sp.x();
    spm.pose.position.y = sp.y();
    spm.pose.position.z = sp.z();
    spm.scale.x = spm.scale.y = spm.scale.z = 0.08;
    spm.color.g = 1.0;
    arr.markers.push_back(spm);
    auto txt = MakeMarker("state", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    txt.pose.position.x = p_.x();
    txt.pose.position.y = p_.y();
    txt.pose.position.z = p_.z() + 0.35;
    txt.scale.z = 0.2;
    txt.color.r = txt.color.g = txt.color.b = 1.0;
    txt.text = std::string(Name(state_)) + "  atk=" + std::to_string(attacks_);
    arr.markers.push_back(txt);
    marker_pub_->publish(arr);
  }

  std::string team_, target_name_, drone_entity_;
  bool auto_start_, require_vio_, blind_attack_;
  double start_delay_, takeoff_h_, cruise_v_, cruise_a_, standoff_, attack_v_, search_timeout_, impact_acc_;
  double soft_fence_, hard_fence_, ceiling_, phase_tol_, latency_;
  int max_attacks_;

  FieldMap field_;
  OccupancyGrid grid_;
  MinSnapTrajectory traj_;
  std::vector<Vec3> path_;
  Target target_;
  Vec3 aim_{Vec3::Zero()}, aim_filtered_{Vec3::Zero()}, normal_{Vec3::UnitX()}, standoff_pt_{Vec3::Zero()};

  State state_{State::IDLE}, prev_state_{State::IDLE};
  double state_t0_{0}, traj_t0_{0}, ready_since_{-1}, last_match_{-1e9}, last_yaw_t_{-1}, yaw_sp_{0};
  double phase_{0}, phase_t_{-1}, omega_{0};
  int match_count_{0}, attacks_{0}, vio_bad_{0};
  uint64_t marker_count_{0};
  bool have_odom_{false}, have_est_{false}, start_requested_{false}, impact_flag_{false};
  bool approach_ready_{false}, had_speed_{false};
  double peak_closing_{0.0};
  Vec3 p_{Vec3::Zero()}, v_{Vec3::Zero()}, hold_{Vec3::Zero()}, home_{Vec3::Zero()}, land_xy_{Vec3::Zero()};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  rm_drone_msgs::msg::EstimatorStatus est_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rm_drone_msgs::msg::EstimatorStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<rm_drone_msgs::msg::ArmorDetectionArray>::SharedPtr armor_sub_;
  rclcpp::Subscription<rm_drone_msgs::msg::ImpactEvent>::SharedPtr impact_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Publisher<rm_drone_msgs::msg::TrajectorySetpoint>::SharedPtr sp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_, estop_pub_;
  rclcpp::Publisher<rm_drone_msgs::msg::MissionState>::SharedPtr state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rm_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_drone::MissionNode>());
  rclcpp::shutdown();
  return 0;
}
