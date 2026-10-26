// 任务状态机：
//   IDLE -> ARMING -> TAKEOFF -> PLAN -> CRUISE -> SEARCH -> (OUTPOST_WAIT) -> ATTACK -> EGRESS -> ... -> LAND -> DONE
//   规划失败且仍有速度时进入 BRAKE，不把失败轨迹交给 CRUISE。
// 起飞需要仿真适配的比赛阶段和机库放行（R47：准备阶段不得转桨）。
// 硬围栏对应 R48；飞镖发射站禁区含上空（R64）。急停锁存对应 S79，arm=true 不能解除。
#include <chrono>
#include <deque>
#include <utility>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/armor_detection_array.hpp>
#include <rm_drone_msgs/msg/estimator_status.hpp>
#include <rm_drone_msgs/msg/impact_event.hpp>
#include <rm_drone_msgs/msg/mission_state.hpp>
#include <rm_drone_msgs/msg/referee_status.hpp>
#include <rm_drone_msgs/msg/trajectory_setpoint.hpp>
#include <rm_drone_msgs/msg/planner_goal.hpp>
#include <rm_drone_msgs/msg/local_trajectory.hpp>
#include <rm_drone_msgs/msg/planner_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_drone_control/fault_policy.hpp"
#include "rm_drone_control/math_utils.hpp"
#include "rm_drone_planner/planning.hpp"
#include "rm_drone_planner/spline_trajectory.hpp"

namespace rm_drone
{

using Vec3 = Eigen::Vector3d;

enum class State { IDLE, ARMING, TAKEOFF, PLAN, BRAKE, RECOVER, CRUISE, SEARCH, OUTPOST_WAIT, ATTACK, EGRESS, LAND_ALIGN, LAND, DONE, ESTOP };

static const char * Name(State s)
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::ARMING: return "ARMING";
    case State::TAKEOFF: return "TAKEOFF";
    case State::PLAN: return "PLAN";
    case State::BRAKE: return "BRAKE";
    case State::RECOVER: return "RECOVER";
    case State::CRUISE: return "CRUISE";
    case State::SEARCH: return "SEARCH";
    case State::OUTPOST_WAIT: return "OUTPOST_WAIT";
    case State::ATTACK: return "ATTACK";
    case State::EGRESS: return "EGRESS";
    case State::LAND_ALIGN: return "LAND_ALIGN";
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
    planner_backend_ = declare_parameter<std::string>("planner_backend", "ego");
    if (planner_backend_ != "ego" && planner_backend_ != "astar") {throw std::invalid_argument("planner_backend must be ego or astar");}
    use_static_prior_ = declare_parameter("use_static_prior", true);
    team_ = declare_parameter<std::string>("team", "red");
    if (team_!="red" && team_!="blue") {throw std::invalid_argument("team must be red or blue");}
    target_name_ = declare_parameter<std::string>("target", "auto");
    auto_start_ = declare_parameter("auto_start", false);
    start_delay_ = declare_parameter("start_delay", 3.0);
    require_vio_ = declare_parameter("require_vio", true);
    takeoff_h_ = declare_parameter("takeoff_height", 0.6);
    cruise_v_ = declare_parameter("cruise_speed", 3.0);
    cruise_a_ = declare_parameter("cruise_acc", 3.0);
    standoff_ = declare_parameter("standoff_distance", 3.0);
    base_standoff_ = declare_parameter("base_standoff_distance", 1.5);
    base_view_height_ = declare_parameter("base_observation_height", .8);
    attack_v_ = declare_parameter("attack_speed", 1.7);
    attack_acc_ = declare_parameter("attack_acceleration", 2.0);
    body_half_h_ = declare_parameter("body_half_height", 0.0225);
    attack_z_tol_ = declare_parameter("attack_height_tolerance", 0.008);
    attack_vz_tol_ = declare_parameter("attack_vertical_speed_tolerance", 0.03);
    max_attacks_ = declare_parameter("max_attacks", 0);
    max_attempts_ = declare_parameter("max_attempts", 3);
    match_mode_ = declare_parameter("match_mode", true);
    match_duration_ = declare_parameter("match_duration", 420.0);
    return_reserve_ = declare_parameter("return_reserve", 60.0);
    mission_timeout_ = declare_parameter("mission_timeout", 300.0);
    return_timeout_ = declare_parameter("return_timeout", 150.0);
    landing_vision_ = declare_parameter("require_landing_vision", true);
    if (max_attacks_ < 0 || max_attempts_ < 1 || !std::isfinite(mission_timeout_) ||
      !std::isfinite(return_timeout_) || mission_timeout_ <= 0 || return_timeout_ <= 0 ||
      !std::isfinite(match_duration_) || !std::isfinite(return_reserve_) ||
      return_reserve_<=0 || match_duration_<=return_reserve_) {
      throw std::invalid_argument("Mission requires finite positive attack/attempt/time budgets");
    }
    search_timeout_ = declare_parameter("search_timeout", 4.0);
    blind_attack_ = declare_parameter("allow_blind_attack", false);
    impact_acc_ = declare_parameter("impact_acc_threshold", 30.0);
    soft_fence_ = declare_parameter("soft_fence_margin", 0.25);
    hard_fence_ = declare_parameter("hard_fence_margin", 0.05);
    ceiling_ = declare_parameter("ceiling", 1.55);
    body_r_ = declare_parameter("body_radius", 0.07);
    brake_acc_ = declare_parameter("brake_acc", 3.0);
    odom_timeout_ = declare_parameter("odom_timeout", 0.3);
    drone_entity_ = declare_parameter<std::string>("drone_entity_name", "rm_drone");
    phase_tol_ = declare_parameter("outpost_phase_tolerance", 0.12);
    latency_ = declare_parameter("guidance_latency", 0.12);

    GridConfig gc;
    gc.resolution = declare_parameter("grid_resolution", 0.1);
    gc.obstacle_margin = declare_parameter("obstacle_margin", 0.30);
    gc.wall_margin = declare_parameter("wall_margin", 0.45);
    gc.floor_margin = declare_parameter("floor_margin", 0.30);
    const double plan_ceiling = declare_parameter("plan_ceiling", ceiling_);
    if (std::abs(plan_ceiling - ceiling_) > 1e-3) {
      RCLCPP_WARN(get_logger(), "plan_ceiling %.2f 与 ceiling %.2f 不一致，统一为较低的那个，避免先验证再裁剪",
        plan_ceiling, ceiling_);
    }
    ceiling_ = std::min(ceiling_, plan_ceiling);
    gc.ceiling = ceiling_;
    const auto map_path = declare_parameter<std::string>("field_map", "");
    field_.load(map_path);
    if (target_name_!="auto") {
      const auto * selected=field_.findTarget(target_name_);
      if (!selected || selected->team==team_ || (selected->kind!="base" && selected->kind!="outpost")) {
        throw std::invalid_argument("target must be an enemy base/outpost ordinary armor: "+target_name_);
      }
    }
    const auto t0 = std::chrono::steady_clock::now();
    FieldMap planning_field = field_;
    if (planner_backend_ == "ego" && !use_static_prior_) {
      planning_field.obstacles.erase(std::remove_if(planning_field.obstacles.begin(), planning_field.obstacles.end(),
        [this](const Box & b) {return !field_.launchObstacle(b);}), planning_field.obstacles.end());
    }
    grid_.build(planning_field, gc);
    RCLCPP_INFO(get_logger(), "栅格地图 %dx%dx%d 构建耗时 %.2fs", grid_.size().x(), grid_.size().y(), grid_.size().z(),
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());

    auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", qos, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        odom_rx_ = std::chrono::steady_clock::now();
        const rclcpp::Time stamp(m->header.stamp);
        const Vec3 p(m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z);
        const auto q = toEigen(m->pose.pose.orientation);
        const Vec3 vb = toEigen(m->twist.twist.linear);
        const double prev = last_odom_stamp_.nanoseconds() > 0 ? last_odom_stamp_.seconds() : -1.0;
        if (!stampUsable(stamp.seconds(), now().seconds(), prev, finiteVec(p) && finiteVec(vb), finiteQuat(q))) {
          odom_valid_ = false;
          return;
        }
        p_ = p;
        q_ = q.normalized();
        v_ = q_ * vb;
        last_odom_stamp_ = stamp;
        have_odom_ = true;
        odom_valid_ = true;
      });
    status_sub_ = create_subscription<rm_drone_msgs::msg::EstimatorStatus>("estimator_status", 10,
      [this](rm_drone_msgs::msg::EstimatorStatus::SharedPtr m) {
        est_rx_ = std::chrono::steady_clock::now();
        est_ = *m;
        have_est_ = finiteVec(Vec3(m->position_std, 0, 0));
      });
    ref_sub_ = create_subscription<rm_drone_msgs::msg::RefereeStatus>("/referee/status", 10,
      [this](rm_drone_msgs::msg::RefereeStatus::SharedPtr m) {ref_ = *m; have_ref_ = true;});
    armor_sub_ = create_subscription<rm_drone_msgs::msg::ArmorDetectionArray>("armors", 10,
      std::bind(&MissionNode::OnArmors, this, std::placeholders::_1));
    impact_sub_ = create_subscription<rm_drone_msgs::msg::ImpactEvent>("/referee/impact", 10,
      [this](rm_drone_msgs::msg::ImpactEvent::SharedPtr m) {
        if (m->striker_name == drone_entity_ && m->armor_name == target_.name &&
          rclcpp::Time(m->header.stamp).seconds() >= attack_started_at_) {impact_flag_ = true;}
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu", qos, [this](sensor_msgs::msg::Imu::SharedPtr m) {
        const Vec3 a(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
        const Vec3 w(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z);
        landing_imu_still_=a.allFinite() && w.allFinite() && w.norm()<.2 &&
          (a-q_.conjugate()*Vec3(0,0,9.81)).norm()<1.5;
        landing_imu_stamp_=m->header.stamp;
        // 只用于判断已经接触并撤离，不作为裁判有效得分。
        if (state_ == State::ATTACK && a.allFinite() && (a - q_.conjugate() * Vec3(0, 0, 9.81)).norm() > impact_acc_) {
          contact_guess_ = true;
        }
      });
    start_sub_ = create_subscription<std_msgs::msg::Empty>("mission/start", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {start_requested_ = true;});
    return_sub_ = create_subscription<std_msgs::msg::Empty>("mission/return", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {return_requested_ = true;});
    ground_sub_ = create_subscription<std_msgs::msg::Bool>("grounded", 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {grounded_ = m->data; ground_stamp_ = now();});
    landing_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("landing/error", qos,
      [this](geometry_msgs::msg::PointStamped::SharedPtr m) {
        const double age = (now() - rclcpp::Time(m->header.stamp)).seconds();
        if (m->header.frame_id == "map" && age >= -.05 && age < .3 &&
          std::isfinite(m->point.x) && std::isfinite(m->point.y) && std::isfinite(m->point.z) &&
          std::hypot(m->point.x, m->point.y) < .25 && m->point.z > .05 && m->point.z < 1.0) {
          const Vec3 observed(m->point.x,m->point.y,m->point.z);
          const double dt=(rclcpp::Time(m->header.stamp)-landing_stamp_).seconds();
          if (have_landing_ && dt>0 && dt<.3) {
            const Vec3 change=observed-landing_error_;
            Vec3 velocity(-change.x()/dt,-change.y()/dt,change.z()/dt);
            if (velocity.norm()<2.) {
              const double alpha=dt/(.15+dt);
              landing_velocity_=have_landing_velocity_?(1-alpha)*landing_velocity_+alpha*velocity:velocity;
              have_landing_velocity_=true;
              const double bias_alpha=dt/(1.+dt);
              landing_velocity_bias_=(1-bias_alpha)*landing_velocity_bias_+bias_alpha*(v_-landing_velocity_);
            }
          } else {have_landing_velocity_=false;}
          landing_error_ = observed;
          landing_stamp_ = m->header.stamp;
          have_landing_ = true;
        }
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>("estop", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
        if (m->data) {estop_latched_ = true;}
      });
    estop_reset_sub_ = create_subscription<std_msgs::msg::Empty>("estop_reset", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {estop_reset_ = true;});

    sp_pub_ = create_publisher<rm_drone_msgs::msg::TrajectorySetpoint>("setpoint", 10);
    arm_pub_ = create_publisher<std_msgs::msg::Bool>("arm", 10);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("estop", 10);
    state_pub_ = create_publisher<rm_drone_msgs::msg::MissionState>("mission_state", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("planner/markers", 1);
    ego_goal_pub_ = create_publisher<rm_drone_msgs::msg::PlannerGoal>("ego/goal", 1);
    contact_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ego/contact_goal", 1);
    ego_status_sub_ = create_subscription<rm_drone_msgs::msg::PlannerStatus>("ego/status", 1,
      [this](rm_drone_msgs::msg::PlannerStatus::SharedPtr m) {
        const double age = (now() - rclcpp::Time(m->header.stamp)).seconds();
        if (m->header.frame_id == "map" && age >= -.05 && age < .35) {ego_status_ = *m; have_ego_status_ = true;}
      });
    ego_traj_sub_ = create_subscription<rm_drone_msgs::msg::LocalTrajectory>("ego/trajectory", 1,
      [this](rm_drone_msgs::msg::LocalTrajectory::SharedPtr m) {OnEgoTrajectory(*m);});
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&MissionNode::Step, this));
  }

private:
  bool EgoReady() const
  {
    if (!have_ego_status_) {return false;}
    const double age = (now() - rclcpp::Time(ego_status_.header.stamp)).seconds();
    return ego_status_.ready && age >= -.05 && age < .35;
  }

  void PublishEgoGoal()
  {
    if (planner_backend_ != "ego") {return;}
    rm_drone_msgs::msg::PlannerGoal goal;
    goal.header.frame_id = "map"; goal.header.stamp = now();
    goal.request_id = ego_request_;
    goal.active = ego_navigation_ && (state_ == State::PLAN || state_ == State::CRUISE);
    goal.position.x = standoff_pt_.x(); goal.position.y = standoff_pt_.y(); goal.position.z = standoff_pt_.z();
    goal.tolerance = returning_ ? .40 : .20;
    ego_goal_pub_->publish(goal);
    geometry_msgs::msg::PoseStamped contact;
    contact.header.frame_id = "map";
    // A brake during egress must not turn the authorized contact surface back
    // into an inescapable dynamic obstacle. Keep the same spatially bounded
    // permission while waiting to resume egress; all other obstacles still gate.
    if (state_ == State::ATTACK || state_ == State::EGRESS ||
      (state_ == State::BRAKE && resume_after_brake_ == State::EGRESS)) {
      contact.header.stamp = now();
      const Vec3 center = BodyAim(), axis = (standoff_pt_ - center).normalized();
      contact.pose.position.x = center.x(); contact.pose.position.y = center.y(); contact.pose.position.z = center.z();
      contact.pose.orientation = toMsg(Eigen::Quaterniond::FromTwoVectors(Vec3::UnitX(), axis));
    }
    contact_goal_pub_->publish(contact);
  }

  void OnEgoTrajectory(const rm_drone_msgs::msg::LocalTrajectory & message)
  {
    if (planner_backend_ != "ego" || !ego_navigation_ || message.request_id != ego_request_ ||
      message.revision <= ego_revision_ || message.header.frame_id != "map") {return;}
    const double age = (now() - rclcpp::Time(message.header.stamp)).seconds();
    if (age < -.05 || age > .3) {return;}
    SplineTrajectory candidate;
    if (!candidate.assign(message)) {return;}
    Vec3 previous = candidate.position(0);
    if ((previous - p_).norm() > .5) {return;}
    const int samples = std::max(1, static_cast<int>(std::ceil(candidate.duration() / .02)));
    for (int i = 0; i <= samples; ++i) {
      const double t = candidate.duration() * i / samples;
      const Vec3 p = candidate.position(t), v = candidate.velocity(t), a = candidate.acceleration(t);
      if (!p.allFinite() || !v.allFinite() || !a.allFinite() || v.norm() > cruise_v_ * 1.15 ||
        a.norm() > cruise_a_ * 1.15 || !grid_.lineFree(previous, p, .025)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "EGO 轨迹未通过任务层静态碰撞/运动约束校验");
        return;
      }
      previous = p;
    }
    ego_spline_ = std::move(candidate);
    ego_start_ = message.header.stamp;
    ego_revision_ = message.revision;
    have_ego_trajectory_ = true;
  }
  void ReportStartupWait(const std::string & reason, bool publish = false)
  {
    startup_info_ = reason;
    const auto wall = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(wall - startup_log_).count() >= 5.0) {
      RCLCPP_INFO(get_logger(), "等待起飞：%s（仿真时间 %.2fs）", reason.c_str(), now().seconds());
      startup_log_ = wall;
    }
    if (publish) {
      rm_drone_msgs::msg::MissionState ms;
      ms.header.stamp = now();
      ms.state = "IDLE";
      ms.info = reason;
      state_pub_->publish(ms);
    }
  }

  // ------------------------------------------------------------ 目标
  Vec3 BodyAim() const
  {
    // 地图/PnP给的是板面中心，控制器跟踪的是机身中心。用桨保盒在末端
    // 近水平姿态下的支撑点补偿。不能跟随加速时的实时pitch抬高目标：
    // 位置环会追着抬高，姿态恢复后高度来不及下降，先碰到上方飞镖模块。
    const Eigen::Quaterniond impact_q(Eigen::AngleAxisd(
      std::atan2(-normal_.y(), -normal_.x()), Vec3::UnitZ()));
    const Vec3 n = impact_q.conjugate() * target_.normal;
    const Vec3 half(body_r_, body_r_, body_half_h_);
    Vec3 support = Vec3::Zero();
    for (int i = 0; i < 3; ++i) {
      if (std::abs(n[i]) > 0.10) {support[i] = std::copysign(half[i], n[i]);}
    }
    return aim_ + impact_q * support;
  }

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
        const Vec3 s = t.position + n * base_standoff_ + Vec3(0, 0, base_view_height_);
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
    if (target_.kind == "outpost" && std::abs(target_.spin_speed) > 1e-3) {
      Vec3 d = p_ - target_.position;
      d.z() = 0.0;
      if (d.norm() < 1e-3) {d = Vec3::UnitX();}
      d.normalize();
      normal_ = d;
      aim_ = target_.position + d * target_.radius;
    } else {
      // 固定装甲，包括停转的前哨站：瞄准这块板自己的中心和法向，不从塔心外推。
      normal_ = Vec3(std::cos(target_.normal_yaw), std::sin(target_.normal_yaw), 0.0);
      aim_ = target_.position;
    }
    standoff_pt_ = BodyAim() + normal_ * (target_.kind == "base" ? base_standoff_ : standoff_);
    if (target_.kind == "base") {standoff_pt_.z() += base_view_height_;}
    // Observe upward-facing base armor from an elevated standoff. The terminal
    // axis connects that viewpoint to the compensated contact point in 3-D.
    Vec3 free_pt;
    if (grid_.nearestFree(standoff_pt_, free_pt, 0.6)) {standoff_pt_ = free_pt;}
    RCLCPP_INFO(get_logger(), "目标 %s：瞄准点 (%.2f %.2f %.2f)，待机点 (%.2f %.2f %.2f)", target_.name.c_str(),
      aim_.x(), aim_.y(), aim_.z(), standoff_pt_.x(), standoff_pt_.y(), standoff_pt_.z());
    return true;
  }

  void OnArmors(rm_drone_msgs::msg::ArmorDetectionArray::SharedPtr m)
  {
    const rclcpp::Time stamp(m->header.stamp);
    if (stamp.nanoseconds() <= 0) {return;}
    const double t = stamp.seconds();
    const double age = now().seconds() - t;
    if (age < -.05 || age > .3 || t <= last_match_) {return;}
    if (t - last_match_ > .3) {match_count_ = 0;}
    for (const auto & a : m->armors) {
      if (!a.has_map_pose) {continue;}
      // 场景中的飞镖模块有同色灯条，但不属于无人机冲击目标。
      // 基地大装甲不能关联成小灯条板；颜色与已知类别也必须匹配目标。
      if (a.color != target_.team || (!a.kind.empty() && a.kind != target_.kind) ||
        a.armor_type != (target_.armor_type == "large" ? a.LARGE : a.SMALL)) {continue;}
      const Vec3 p(a.pose_map.position.x, a.pose_map.position.y, a.pose_map.position.z);
      if (!p.allFinite()) {continue;}
      const auto orientation = toEigen(a.pose_map.orientation);
      if (state_ == State::SEARCH && finiteQuat(orientation)) {
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
          "装甲关联：地图残差 %.3fm，法向点积 %.2f，置信度 %.2f",
          (p-target_.position).norm(),(orientation.normalized()*Vec3::UnitX()).dot(target_.normal),a.confidence);
      }
      if (!finiteQuat(orientation) ||
        (orientation.normalized() * Vec3::UnitX()).dot(target_.normal) < .8) {continue;}
      if (target_.kind == "outpost" && std::abs(target_.spin_speed) > 1e-3) {
        Vec3 r = p - target_.position;
        r.z() = 0.0;
        if (std::abs(r.norm() - target_.radius) > 0.25 || std::abs(p.z() - target_.position.z()) > 0.3) {continue;}
        UpdateOutpostPhase(t, std::atan2(r.y(), r.x()));
        last_match_ = t;
        ++match_count_;
        break;  // confirmation counts distinct image timestamps, not boxes
      } else if ((p - target_.position).norm() < .25) {
        // 固定前哨站与基地都使用实际板中心；SEARCH 会把该滤波值设为 aim_。
        // 仅增加 match_count 而不更新 aim_filtered_ 会在确认后把目标错误地改为原点。
        const double alpha = match_count_ == 0 ? 1.0 : 0.3;
        aim_filtered_ = alpha * p + (1 - alpha) * aim_filtered_;
        last_match_ = t;
        ++match_count_;
        break;
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

  bool LocOk() const
  {
    if (!require_vio_) {return have_est_ && est_.initialized;}
    return have_est_ && est_.initialized && est_.vio_ok && est_.source == "healthy";
  }

  std::string AllowPrefix() const
  {
    if (state_ != State::ATTACK && state_ != State::EGRESS) {return "";}
    if (target_.kind == "outpost") {return target_.team + "_outpost";}
    if (target_.kind == "base") {return target_.team + "_base";}
    return "";
  }

  bool CommandCollision(const Vec3 & position) const
  {
    const double floor = home_.z() - body_half_h_;
    const bool docking = state_ == State::LAND && returning_ && landing_aligned_ &&
      (position.head<2>() - home_.head<2>()).norm() < .12 &&
      position.z() >= floor + body_half_h_ - .016;
    if (!docking) {return field_.inCollision(position, 0.0, AllowPrefix());}
    if (field_.inNoFly(position)) {return true;}
    const std::string support = team_ + "_hangar";
    for (const auto & box : field_.obstacles) {
      // The 15 cm CAD occupancy voxels overestimate the deck top by 5 cm.
      // Permit only this robot's certified landing support, within the small
      // landing cylinder; energy structures and all other obstacles stay solid.
      if (box.name.rfind(support, 0) == 0 && box.max().z() <= floor + .1) {continue;}
      if (box.contains(position, 0.0)) {return true;}
    }
    return false;
  }

  // Leave only a static safety buffer at low speed, with physical geometry
  // clearance increasing along the entire segment. Online obstacles still gate.
  bool BeginBufferRecovery(double t)
  {
    if (!grid_.occupied(p_) || v_.norm() > .15 || !EgoReady() || !ego_status_.motion_safe ||
      (q_*Vec3::UnitZ()).z()<.995) {return false;}
    // Use the yawed box footprint, including corners and a centimetre of
    // extra clearance. Low-speed acceleration keeps recovery tilt below 3deg.
    const Vec3 half=Eigen::AngleAxisd(yawOf(q_),Vec3::UnitZ()).toRotationMatrix().cwiseAbs()*
      Vec3(body_r_,body_r_,body_half_h_)+Vec3::Constant(.01);
    auto clearance = [&](const Vec3 & p) {
      if (!field_.insideField(p, .45, ceiling_) || field_.inNoFly(p,half.head<2>().maxCoeff())) {return -1.;}
      double distance = 1e3;
      for (const auto & box : field_.obstacles) {
        const Vec3 delta = (box.min()-half-p).cwiseMax(p-box.max()-half).cwiseMax(Vec3::Zero());
        distance = std::min(distance, delta.norm());
      }
      return distance;
    };
    const double initial = clearance(p_);
    if (initial < .001) {return false;}
    Vec3 best;
    double score = 1e3;
    for (int x=-4;x<=4;++x) for (int y=-4;y<=4;++y) for (int z=-2;z<=2;++z) {
      const Vec3 end = p_ + Vec3(x,y,z)*.1;
      const double length = (end-p_).norm();
      if (length < .1 || length > .45 || length >= score || grid_.occupied(end)) {continue;}
      bool safe = true, left_buffer = false;
      for (double u=0;u<=1.001;u+=.04) {
        const Vec3 at=p_+(end-p_)*u;
        const double space=clearance(at);
        if (space < initial-.0001 || space < .001 || (left_buffer && grid_.occupied(at))) {safe=false;break;}
        if (!grid_.occupied(at)) {left_buffer=true;}
      }
      if (safe) {best=end;score=length;}
    }
    if (score > .45) {return false;}
    MinSnapTrajectory recovery;
    if (!recovery.generate({p_,best},Vec3::Zero(),Vec3::Zero(),Vec3::Zero(),.2,.5)) {return false;}
    traj_=std::move(recovery);traj_t0_=t;hold_=best;
    ego_navigation_=have_ego_trajectory_=false;state_=State::RECOVER;
    RCLCPP_WARN(get_logger(), "静态缓冲区恢复：%.2fm，速度上限0.2m/s，实体净空%.2fm",score,initial);
    return true;
  }

  // ------------------------------------------------------------ 主循环
  void Step()
  {
    const rclcpp::Time ros_now = now();
    const auto steady_now = std::chrono::steady_clock::now();
    const double ros_dt = last_ros_.nanoseconds() > 0 ? (ros_now - last_ros_).seconds() : 0.02;
    const double steady_dt = std::chrono::duration<double>(steady_now - last_steady_).count();
    if (ros_dt < -1e-3) {
      last_ros_ = ros_now;
      last_steady_ = steady_now;
      last_odom_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      if (planner_backend_ == "ego") {
        ego_navigation_ = have_ego_trajectory_ = have_ego_status_ = false;
        ++ego_request_;
        state_ = State::IDLE;
        returning_ = return_requested_ = landing_aligned_ = have_landing_ = have_landing_velocity_ = false;
        landing_velocity_bias_.setZero();
        attempts_ = attacks_ = consecutive_misses_ = 0; touchdown_since_ = -1; outcome_ = "world_reset";
        ready_since_ = -1;
        PublishArm(false);
        PublishEgoGoal();
      }
      return;
    }
    const bool paused = last_ros_.nanoseconds() > 0 && simPaused(std::max(0.0, ros_dt), steady_dt);
    last_ros_ = ros_now;
    last_steady_ = steady_now;
    if (estop_latched_ && state_ != State::ESTOP) {Estop();}
    if (!have_odom_) {
      PublishArm(false);
      ReportStartupWait("等待有效里程计", true);
      return;
    }
    const double t = ros_now.seconds();
    const double odom_age_ros = (ros_now - last_odom_stamp_).seconds();
    const double odom_receive_age = receiveAge(get_clock()->ros_time_is_active(), odom_age_ros,
      std::chrono::duration<double>(steady_now - odom_rx_).count());
    const bool odom_ok = odom_valid_ && (paused || (odom_age_ros < odom_timeout_ && odom_receive_age < odom_timeout_));
    if (state_ != prev_state_) {
      RCLCPP_INFO(get_logger(), "状态 %s -> %s", Name(prev_state_), Name(state_));
      prev_state_ = state_;
      state_t0_ = t;
      if (state_ == State::PLAN || state_ == State::BRAKE || state_ == State::IDLE || state_ == State::LAND_ALIGN) {
        ego_navigation_ = have_ego_trajectory_ = false;
        brake_hold_set_ = false;
      }
    }
    const double ts = t - state_t0_;
    const bool flying = state_ != State::IDLE && state_ != State::DONE && state_ != State::ESTOP;

    if (estop_latched_) {
      if (estopRelease(true, estop_reset_, false) && state_ == State::ESTOP) {
        estop_latched_ = false;
        estop_reset_ = false;
        state_ = State::IDLE;
      } else if (state_ != State::ESTOP) {
        Estop();
      }
    } else {
      estop_reset_ = false;
    }

    // 安全监控。odom 断流时不能直接 return，否则保护逻辑也不再跑。
    if (flying && state_ != State::ARMING && state_ != State::ESTOP) {
      const double hard_xy = std::max(hard_fence_, body_r_ + (have_est_ ? est_.position_std : 0.05));
      if (!field_.insideField(p_, hard_xy, ceiling_ + 0.15) || field_.inNoFly(p_)) {
        RCLCPP_ERROR(get_logger(), "越过硬围栏或禁区 (%.2f %.2f %.2f)，紧急停桨", p_.x(), p_.y(), p_.z());
        Estop();
      } else if (!odom_ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "里程计断流，按最后位置受控降落");
        outcome_="odometry_timeout";
        land_xy_ = p_;
        state_ = State::LAND;
      } else if (require_vio_ && !LocOk() && state_ != State::LAND && state_ != State::TAKEOFF) {
        if (++vio_bad_ > 50) {
          RCLCPP_WARN(get_logger(), "VIO 失效超过 1s，原地降落，不回退真值");
          outcome_="localization_lost";
          land_xy_ = p_;
          state_ = State::LAND;
        }
      } else {
        vio_bad_ = 0;
      }
    }

    if (planner_backend_ == "ego" && flying && odom_ok && state_ != State::ARMING &&
      state_ != State::LAND && state_ != State::LAND_ALIGN && state_ != State::ESTOP && state_ != State::BRAKE) {
      const bool blocked = state_ == State::CRUISE && ego_status_.request_id == ego_request_ && ego_status_.blocked;
      if (!EgoReady() || !ego_status_.motion_safe || blocked) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "局部导航保护，刹停：%s", ego_status_.reason.c_str());
        resume_after_brake_ = state_==State::TAKEOFF ? State::TAKEOFF :
          (state_ == State::ATTACK || state_ == State::EGRESS) ? State::EGRESS : State::PLAN;
        hold_ = p_; state_ = State::BRAKE;
        ego_navigation_ = have_ego_trajectory_ = false;
        brake_hold_set_ = false;
      }
    }

    if (flying && state_ != State::ARMING && state_ != State::TAKEOFF &&
      state_ != State::LAND && state_ != State::LAND_ALIGN && state_ != State::ESTOP) {
      if (match_mode_ && !returning_ && !return_requested_) {
        const double ref_age=have_ref_?(ros_now-rclcpp::Time(ref_.header.stamp)).seconds():1e3;
        if (ref_age<-.05 || ref_age>.5) {return_requested_=true;outcome_="referee_timeout";}
        else if (ref_.phase!="match" || !ref_.flight_permitted) {return_requested_=true;outcome_="match_stopped";}
        else if (ref_.match_time>=match_duration_-return_reserve_) {return_requested_=true;outcome_="match_return_reserve";}
        else if ((target_.kind=="base" && (target_.team=="red"?ref_.red_base_hp:ref_.blue_base_hp)<=0) ||
          (target_.kind=="outpost" && (target_.team=="red"?ref_.red_outpost_hp:ref_.blue_outpost_hp)<=0)) {
          return_requested_=true;outcome_="target_destroyed";
        }
      }
      if (!returning_ && (return_requested_ || t - mission_started_ > mission_timeout_)) {
        if (!return_requested_) {outcome_="mission_timeout";}
        else if (outcome_=="running") {outcome_="operator_return";}
        return_requested_ = true;
        if (state_ == State::ATTACK || state_ == State::EGRESS) {state_ = State::EGRESS;}
        else {BeginReturn(t, outcome_);}
      }
      if (returning_ && t - return_started_ > return_timeout_) {
        outcome_ = "return_timeout"; land_xy_ = p_; state_ = State::LAND;
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
          const bool phase_ok = have_ref_ && ref_.flight_permitted && ref_.hangar_released;
          const bool local_ready = planner_backend_ != "ego" || (EgoReady() && ego_status_.motion_safe);
          const bool ready = phase_ok && LocOk() && odom_ok && local_ready;
          if (ready && ready_since_ < 0) {ready_since_ = t;}
          if (!ready) {ready_since_ = -1;}
          if (ready && (start_requested_ || (auto_start_ && t - ready_since_ > start_delay_))) {
            home_ = p_; home_yaw_ = yawOf(q_); mission_started_ = t;
            attacks_ = attempts_ = consecutive_misses_ = 0; returning_ = false; return_requested_ = false; outcome_ = "running";
            state_ = State::ARMING;
          }
          if (state_ == State::IDLE) {
            if (!have_ref_) {ReportStartupWait("等待裁判状态");}
            else if (!phase_ok) {ReportStartupWait("等待裁判飞行许可和机库放行");}
            else if (!LocOk()) {ReportStartupWait("等待定位初始化/健康估计");}
            else if (!odom_ok) {ReportStartupWait("等待新鲜有效的里程计");}
            else if (!local_ready) {ReportStartupWait("等待 EGO 新鲜深度地图与采样时刻 TF");}
            else if (!auto_start_ && !start_requested_) {ReportStartupWait("自动起飞关闭，等待 mission/start");}
            else {
              ReportStartupWait("条件已满足，倒计时 " +
                std::to_string(std::max(0.0, start_delay_ - (t - ready_since_))) + " 仿真秒");
            }
          }
          break;
        }
      case State::ARMING:
        pos = home_;
        if (planner_backend_ == "ego" && !EgoReady()) {state_ = State::IDLE; break;}
        if (ts > 1.0) {
          MinSnapTrajectory cand;
          if (!cand.generate({home_, home_ + Vec3(0, 0, takeoff_h_)}, Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), 1.0, 2.0)) {
            RCLCPP_ERROR(get_logger(), "起飞轨迹生成失败，保持锁定");
            state_ = State::IDLE;
          } else {
            traj_ = std::move(cand);
            traj_t0_ = t;
            state_ = State::TAKEOFF;
          }
        }
        break;
      case State::TAKEOFF: {
          if (!traj_.valid()) {state_ = State::IDLE; break;}
          const auto st = traj_.sample(t - traj_t0_);
          pos = st.p;
          vel = st.v;
          acc = st.a;
          // 起飞位置误差15cm时仍可能处在机库的膨胀避障区；锁步下不能
          // 按轨迹时间到期就提前规划，否则几次快速失败会直接触发降落。
          if (t - traj_t0_ > traj_.duration() && (p_ - st.p).norm() < 0.05 &&
            v_.norm() < 0.05 && !grid_.occupied(p_)) {
            hold_ = st.p;
            state_ = State::PLAN;
          }
          break;
        }
      case State::PLAN: {
          pos = hold_;
          vel.setZero();
          acc.setZero();
          if (ts < 0.2) {break;}
          if (planner_backend_ == "ego" && BeginBufferRecovery(t)) {break;}
          if (!returning_ && QuotaReached()) {BeginReturn(t, "contact_completed"); break;}
          // A return command can arrive while still above the departure pad.
          // EGO correctly refuses to generate a zero-length spline; recognize
          // an already reached goal here rather than waiting for that spline.
          if (returning_ && (p_ - standoff_pt_).norm() < .45 && v_.norm() < .2 &&
            (planner_backend_ != "ego" || (EgoReady() && ego_status_.motion_safe))) {
            hold_ = standoff_pt_; state_ = State::LAND_ALIGN; break;
          }
          if (planner_backend_ == "ego") {
            if (!ego_navigation_) {
              if (!returning_ && !SelectTarget()) {break;}
              ego_navigation_ = true;
              have_ego_trajectory_ = false;
              ego_revision_ = 0;
              ++ego_request_;
            }
            if ((p_ - standoff_pt_).norm() < (returning_ ? .45 : .25) && v_.norm() < .2 &&
                grid_.lineFree(p_, standoff_pt_, .025) && EgoReady() && ego_status_.motion_safe) {
                hold_ = standoff_pt_;
                if (returning_) {state_ = State::LAND_ALIGN;}
                else {
                  match_count_ = 0;
                  state_ = (target_.kind == "outpost" && std::abs(target_.spin_speed) > 1e-3)
                    ? State::OUTPOST_WAIT : State::SEARCH;
                }
                break;
              }
            if (have_ego_trajectory_ && EgoReady() && ego_status_.request_id == ego_request_ && !ego_status_.blocked) {
              state_ = State::CRUISE;
              RCLCPP_INFO(get_logger(), "EGO 轨迹通过校验，进入巡航；在线地图版本 %lu", ego_status_.map_revision);
            }
            break;
          }
          if (!returning_ && !SelectTarget()) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "没有可用目标");
            break;
          }
          std::vector<Vec3> raw;
          MinSnapTrajectory cand;
          const bool path_ok = astar(grid_, p_, standoff_pt_, raw);
          bool ok = path_ok;
          if (ok) {
            path_ = simplifyPath(grid_, raw);
            const Vec3 v0 = v_;
            const Vec3 a0 = Vec3::Zero();
            ok = cand.generateSafe(grid_, path_, v0, a0, Vec3::Zero(), cruise_v_, cruise_a_);
            if (!ok) {
              ok = cand.generateSafe(grid_, path_, v0, a0, Vec3::Zero(), 0.6 * cruise_v_, cruise_a_);
            }
          }
          if (!ok || !cand.valid()) {
            ++plan_fail_;
            RCLCPP_WARN(get_logger(), "规划失败（第 %d 次，%s），位置 (%.2f %.2f %.2f)，速度 %.2f m/s，不进入巡航",
              plan_fail_, path_ok ? "轨迹校验未通过" : "无可用路径", p_.x(), p_.y(), p_.z(), v_.norm());
            if (v_.norm() > 0.2) {state_ = State::BRAKE;}
            else if (plan_fail_ > 8) {
              land_xy_ = p_;
              state_ = State::LAND;
            }
            break;
          }
          standoff_pt_ = cand.waypoints().back();
          traj_ = std::move(cand);
          plan_fail_ = 0;
          RCLCPP_INFO(get_logger(), "规划完成：轨迹 %.2fs，峰值速度 %.2f", traj_.duration(), traj_.peakSpeed());
          traj_t0_ = t;
          PublishPlanMarkers();
          state_ = State::CRUISE;
          break;
        }
      case State::BRAKE: {
          const double sp = v_.norm();
          // Hold one spatial anchor throughout braking. A setpoint recomputed
          // from p+v*dt each tick follows vertical drift indefinitely, especially
          // after repeated contacts. Keep position feedback and taper inertial
          // deceleration near zero speed rather than applying a constant step.
          if (!brake_hold_set_) {hold_ = p_; brake_hold_set_ = true;}
          pos = hold_;
          vel.setZero();
          acc.setZero();
          if (sp < 0.15) {
            if (planner_backend_ != "ego" || (EgoReady() && ego_status_.motion_safe)) {
              if (resume_after_brake_==State::TAKEOFF) {
                const Vec3 goal=home_+Vec3(0,0,takeoff_h_);
                if ((p_-goal).norm()<.05 && traj_.valid()) {
                  traj_t0_=t-traj_.duration();state_=State::TAKEOFF;
                } else {
                  MinSnapTrajectory resumed;
                  if (resumed.generate({p_,goal},v_,Vec3::Zero(),Vec3::Zero(),1.,2.)) {
                    traj_=std::move(resumed);traj_t0_=t;state_=State::TAKEOFF;
                  } else {
                    outcome_="takeoff_recovery_failed";land_xy_=p_;state_=State::LAND;
                  }
                }
              } else {state_=resume_after_brake_;}
              resume_after_brake_ = State::PLAN;
            }
          } else {
            acc = -v_ * std::min(2., brake_acc_ / sp);
          }
          break;
        }
      case State::RECOVER: {
          const auto sample=traj_.sample(t-traj_t0_);
          pos=sample.p;vel=sample.v;acc=sample.a;
          if (t-traj_t0_>traj_.duration() && (p_-hold_).norm()<.05 && v_.norm()<.1) {state_=State::PLAN;}
          else if (ts>8) {hold_=p_;state_=State::BRAKE;}
          break;
        }
      case State::CRUISE: {
          if (planner_backend_ == "ego") {
            const double elapsed = (ros_now - ego_start_).seconds();
            if (!have_ego_trajectory_ || !ego_spline_.valid() || elapsed < -.05 || elapsed > ego_spline_.duration() + .25) {
              hold_ = p_; state_ = State::BRAKE; break;
            }
            pos = ego_spline_.position(elapsed);
            vel = ego_spline_.velocity(elapsed);
            acc = ego_spline_.acceleration(elapsed);
            // Face the travel direction so the forward depth camera observes the corridor.
            const Vec3 direction(vel.x(), vel.y(), 0);
            yaw = YawToward(direction.norm() > .15 ? p_ + direction : aim_, t);
            if ((p_ - standoff_pt_).norm() < (returning_ ? .45 : .25) && v_.norm() < .2) {
              hold_ = standoff_pt_; match_count_ = 0;
              state_ = returning_ ? State::LAND_ALIGN : (target_.kind == "outpost" && std::abs(target_.spin_speed) > 1e-3)
                ? State::OUTPOST_WAIT : State::SEARCH;
            }
            break;
          }
          if (!traj_.valid()) {state_ = State::BRAKE; break;}
          const auto st = traj_.sample(t - traj_t0_);
          pos = st.p;
          vel = st.v;
          acc = st.a;
          yaw = YawToward(aim_, t);
          if (t - traj_t0_ > traj_.duration() && (p_ - standoff_pt_).norm() < 0.3) {
            hold_ = standoff_pt_;
            match_count_ = 0;
            state_ = returning_ ? State::LAND_ALIGN : (target_.kind == "outpost" && std::abs(target_.spin_speed) > 1e-3)
              ? State::OUTPOST_WAIT : State::SEARCH;
          }
          break;
        }
      case State::SEARCH: {
          pos = standoff_pt_;
          yaw = YawToward(aim_, t);
          const bool fresh = match_count_ >= 5 && t - last_match_ < 0.3;
          const bool aligned = (p_ - standoff_pt_).norm() < 0.12 && v_.norm() < 0.25 &&
            std::abs(p_.z() - standoff_pt_.z()) < attack_z_tol_ && std::abs(v_.z()) < attack_vz_tol_;
          RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
            "等待接近：视觉=%d/5 新鲜=%d 对中=%d 位置误差=%.3fm 高度误差=%.3fm 垂速=%.3fm/s",
            match_count_,fresh,aligned,(p_-standoff_pt_).norm(),std::abs(p_.z()-standoff_pt_.z()),v_.z());
          if (fresh && LocOk() && aligned && ts > 0.5) {
            aim_ = aim_filtered_;
            BeginAttack(t);
          } else if (ts > search_timeout_) {
            if (blind_attack_ && LocOk() && aligned) {
              RCLCPP_WARN(get_logger(), "未识别到装甲，按场地地图先验位置盲打");
              BeginAttack(t);
            } else if (!blind_attack_) {
              BeginReturn(t, "target_not_confirmed");
            }
          }
          break;
        }
      case State::OUTPOST_WAIT:
        pos = hold_;
        yaw = YawToward(target_.position, t);
        if (match_count_ >= 5 && LocOk() && OutpostReady(t) && t - last_match_ < 0.3) {
          RCLCPP_INFO(get_logger(), "前哨站相位就绪 omega=%.2f rad/s，出击", omega_);
          BeginAttack(t);
        } else if (ts > 15.0) {
          RCLCPP_WARN(get_logger(), "前哨站相位等待超时，不出击");
          BeginReturn(t, "outpost_phase_timeout");
        }
        break;
      case State::ATTACK: {
          // 末端导引：沿装甲法向直线逼近，侧向误差由位置环消除，速度前馈 = -n * v_attack
          if ((target_.kind != "outpost" || std::abs(target_.spin_speed) < 1e-3) &&
            t - last_match_ < 0.2 && (p_ - aim_).norm() > 0.6) {
            aim_ = 0.7 * aim_ + 0.3 * aim_filtered_;
          }
          const Vec3 center = BodyAim();
          const Vec3 axis = (standoff_pt_ - center).normalized();
          const double s = (p_ - center).dot(axis);
          const double closing = -v_.dot(axis);
          // 先退到 1.2 m 外，并且本次接近速度超过 1 m/s，才承认撞击。
          // 否则贴在装甲上的残余接触会被记成下一次攻击，三次后就降落。
          if (s > 1.2) {approach_ready_ = true;}
          if (approach_ready_ && closing > peak_closing_) {peak_closing_ = closing;}
          if (approach_ready_ && closing > 1.0) {had_speed_ = true;}
          const double run = std::max(0.0, (p_ - center).dot(axis));
          const double speed = std::min(attack_v_, attack_acc_ * std::max(0.0, ts));
          // 沿接近方向只使用速度环；位置环只消除横向/高度误差。
          // 前移carrot再叠加同速前馈会额外加速，随后猛抬头并掉高。
          pos = center + axis * run;
          vel = -axis * speed;
          if (speed < attack_v_) {acc = -axis * attack_acc_;}
          yaw = std::atan2(-normal_.y(), -normal_.x());
          yaw_sp_ = yaw;
          const bool hit = had_speed_ && impact_flag_;
          const bool obstructed = had_speed_ && contact_guess_;
          if (hit || obstructed || s < -0.12 || ts > 6.0) {
            if (hit) {
              ++attacks_;
              consecutive_misses_=0;
              attack_counted_ = true;
              RCLCPP_INFO(get_logger(), "第 %d 次目标装甲接触 %s（非裁判得分）：法向距离 %.2f m，接近速度 %.2f m/s",
                attacks_, target_.name.c_str(), s, peak_closing_);
            } else {
              RCLCPP_WARN(get_logger(), "结束本次接近（法向 %.2f m，接近速度 %.2f m/s），撤离并等待目标装甲接触事件", s, closing);
            }
            contact_guess_ = false;
            state_ = State::EGRESS;
          }
          break;
        }
      case State::EGRESS: {
        // 接触传感器在接触结束时才发事件；IMU先触发撤离时，仍接收本次匹配装甲的迟到事件。
        if (had_speed_ && impact_flag_ && !attack_counted_) {
          ++attacks_;
          consecutive_misses_=0;
          attack_counted_ = true;
          RCLCPP_INFO(get_logger(), "第 %d 次目标装甲接触 %s（撤离后收到接触事件，非裁判得分）", attacks_, target_.name.c_str());
        }
        // 撤离也沿线做速度导引，避免把数米的位置阶跃交给位置环而猛倾斜掉高。
        const Vec3 axis = (standoff_pt_ - BodyAim()).normalized();
        const double retreat = std::max(0.0, (standoff_pt_ - p_).dot(axis));
        pos = standoff_pt_ - axis * retreat;
        const double retreat_speed = std::min({1.2, attack_acc_ * std::max(0.0, ts),
          std::sqrt(2.0 * attack_acc_ * retreat)});
        vel = axis * retreat_speed;
        if (retreat < 0.15) {pos = standoff_pt_; vel.setZero();}
        yaw = std::atan2(-normal_.y(), -normal_.x());
        if ((p_ - standoff_pt_).norm() < 0.15 && v_.norm() < 0.3) {
          if (!attempt_resolved_) {if (!attack_counted_) {++consecutive_misses_;}attempt_resolved_=true;}
          if (QuotaReached() || consecutive_misses_ >= max_attempts_ || return_requested_) {
            BeginReturn(t, QuotaReached() ? "contact_completed" :
              (return_requested_ ? outcome_ : "attempts_exhausted"));
          } else {
            hold_ = standoff_pt_;
            match_count_=0;last_match_=-1e9;
            state_ = State::SEARCH;
          }
        }
        break;
      }
      case State::LAND_ALIGN: {
          pos = standoff_pt_; yaw = YawToward(p_ + Vec3(std::cos(home_yaw_), std::sin(home_yaw_), 0), t);
          const bool visual = LandingFresh(t);
          if (visual) {
            pos.x() = p_.x() + std::clamp(landing_error_.x(), -.12, .12);
            pos.y() = p_.y() + std::clamp(landing_error_.y(), -.12, .12);
            pos.z() = p_.z()+std::clamp(takeoff_h_+body_half_h_-landing_error_.z(),-.12,.12);
            if (have_landing_velocity_) {vel=landing_velocity_bias_;}
            land_xy_ = Vec3(p_.x()+landing_error_.x(), p_.y()+landing_error_.y(), home_.z());
          }
          const double align_speed=visual && have_landing_velocity_?(v_-landing_velocity_bias_).norm():v_.norm();
          if ((visual || (!landing_vision_ && (p_.head<2>() - home_.head<2>()).norm() < .12)) &&
            align_speed < .1 && (!visual || (have_landing_velocity_ && landing_error_.head<2>().norm() < .025))) {
            land_xy_ = visual ? land_xy_ : home_;
            landing_aligned_ = true; state_ = State::LAND;
          } else if (ts > 15) {
            // Loss of a landing target is not permission to descend blindly onto
            // the small hangar. Hold above it and report the failed alignment.
            outcome_ = "landing_target_unavailable";
          }
          break;
        }
      case State::LAND: {
          const bool home_landing = returning_ && landing_aligned_;
          const bool visual = LandingFresh(t);
          pos = land_xy_;
          pos.z() = p_.z();
          const double terrain = home_landing ? home_.z() - body_half_h_ :
            field_.terrainHeight(land_xy_.x(), land_xy_.y(), p_.z() - .05);
          const double height = home_landing && visual ? landing_error_.z()-body_half_h_ :
            p_.z() - terrain - body_half_h_;
          // Keep relative visual centering whenever visible, including after
          // map-frame corrections. Only the final 8 cm may use the held point.
          const double observation_age=t-landing_stamp_.seconds();
          const bool relative_velocity=have_landing_velocity_ && observation_age>=0 && observation_age<1.;
          const double lateral_speed=relative_velocity?(v_-landing_velocity_bias_).head<2>().norm():v_.head<2>().norm();
          const bool centered = landing_error_.head<2>().norm()<.02 && lateral_speed<.06;
          const bool descend = !home_landing || !landing_vision_ ||
            (centered && (visual || (height < .08 && relative_velocity)));
          if (home_landing && relative_velocity) {
            // Cancel global VIO velocity bias in the image-servo loop. Pad
            // relative motion supplies damping; fresh odom is still required.
            vel=landing_velocity_bias_;
            if (!visual) {
              pos.x()=p_.x()+landing_error_.x();pos.y()=p_.y()+landing_error_.y();
            }
          }
          if (home_landing && visual) {
            land_xy_.x() = p_.x()+std::clamp(landing_error_.x(), -.08, .08);
            land_xy_.y() = p_.y()+std::clamp(landing_error_.y(), -.08, .08);
            pos.x() = land_xy_.x(); pos.y() = land_xy_.y();
          }
          if (descend) {
            const double speed = height > .18 ? .15 : .08;
            // With a pad in view, control relative height; a biased global
            // height must not command a climb during the last descent.
            // A moving relative setpoint may overshoot below the deck before
            // the body reaches it. Bound it to the same 15 mm contact allowance
            // used by CommandCollision; otherwise that guard substitutes the
            // old hover point and repeatedly commands a climb during landing.
            const double descent_z = home_landing && visual ?
              p_.z()-std::clamp(height+.015,.015,.12) : p_.z()-.12;
            pos.z() = std::max(terrain + body_half_h_ - .015, descent_z);
            vel.z() = home_landing && relative_velocity ? landing_velocity_bias_.z()-speed : -speed;
          }
          const double imu_age=(ros_now-landing_imu_stamp_).seconds();
          const bool pad_settled=home_landing && have_landing_velocity_ && observation_age>=0 && observation_age<2 &&
            (v_-landing_velocity_bias_).head<2>().norm()<.08 && landing_imu_still_ && imu_age>=0 && imu_age<.1;
          const bool supported = grounded_ && (ros_now - ground_stamp_).seconds() < .3 &&
            (pad_settled || (std::abs(v_.z()) < .08 && v_.head<2>().norm() < .08));
          if (supported) {if (touchdown_since_ < 0) {touchdown_since_ = t;}}
          else {touchdown_since_ = -1;}
          if (touchdown_since_ >= 0 && t - touchdown_since_ > .4) {
            RCLCPP_INFO(get_logger(), "物理接地确认，停桨；任务结果：%s", outcome_.c_str());
            state_ = State::DONE;
          }
          break;
        }
      case State::DONE:
      case State::ESTOP:
        pos = p_;
        vel.setZero();
        acc.setZero();
        break;
    }

    const double pos_std = have_est_ ? static_cast<double>(est_.position_std) : 0.1;
    const double predict = body_r_ + pos_std + v_.norm() * latency_ + v_.squaredNorm() / (2.0 * std::max(brake_acc_, 0.5));
    const double lx = field_.length / 2 - std::max(soft_fence_, predict);
    const double ly = field_.width / 2 - std::max(soft_fence_, predict);
    if (flying) {
      if (pos.x() > lx) {pos.x() = lx; vel.x() = std::min(vel.x(), 0.0); acc.x() = std::min(acc.x(), 0.0);}
      if (pos.x() < -lx) {pos.x() = -lx; vel.x() = std::max(vel.x(), 0.0); acc.x() = std::max(acc.x(), 0.0);}
      if (pos.y() > ly) {pos.y() = ly; vel.y() = std::min(vel.y(), 0.0); acc.y() = std::min(acc.y(), 0.0);}
      if (pos.y() < -ly) {pos.y() = -ly; vel.y() = std::max(vel.y(), 0.0); acc.y() = std::max(acc.y(), 0.0);}
      if (pos.z() > ceiling_) {pos.z() = ceiling_; vel.z() = std::min(vel.z(), 0.0); acc.z() = std::min(acc.z(), 0.0);}
      const bool prior_collision = planner_backend_ != "ego" || use_static_prior_;
      if (prior_collision ? CommandCollision(pos) : field_.inNoFly(pos)) {
        pos = hold_;
        vel.setZero();
        acc.setZero();
      }
    }

    const bool want_arm = !estop_latched_ && state_ != State::IDLE && state_ != State::DONE && state_ != State::ESTOP;
    PublishArm(want_arm);

    sp.position = toMsg(pos);
    sp.velocity = toMsg(vel);
    sp.acceleration = toMsg(acc);
    sp.yaw = yaw;
    sp_pub_->publish(sp);
    PublishEgoGoal();

    rm_drone_msgs::msg::MissionState ms;
    ms.header.stamp = now();
    ms.state = Name(state_);
    ms.target = target_.name;
    ms.distance_to_target = static_cast<float>((p_ - aim_).norm());
    ms.info = state_ == State::IDLE ? startup_info_ : "attacks=" + std::to_string(attacks_) +
      " attempts=" + std::to_string(attempts_) + " misses="+std::to_string(consecutive_misses_)+
      " mode="+(match_mode_?"match":"single")+" returning=" + (returning_ ? "true" : "false") + " result=" + outcome_;
    if (planner_backend_ == "ego" && (state_ == State::BRAKE || state_ == State::PLAN)) {ms.info += " | " + ego_status_.reason;}
    state_pub_->publish(ms);
    if (++marker_count_ % 10 == 0) {PublishStateMarkers(pos);}
  }

  bool LandingFresh(double t) const
  {
    const double age = t - landing_stamp_.seconds();
    return have_landing_ && age >= -.05 && age < .3;
  }

  bool QuotaReached() const {return max_attacks_>0 && attacks_>=max_attacks_;}

  void BeginReturn(double t, const std::string & outcome)
  {
    if (returning_) {return;}
    outcome_ = outcome; returning_ = true; return_started_ = t;
    landing_aligned_ = false; touchdown_since_ = -1;
    aim_ = home_; standoff_pt_ = home_ + Vec3(0, 0, takeoff_h_);
    hold_ = p_; ego_navigation_ = have_ego_trajectory_ = false;
    resume_after_brake_ = State::PLAN;
    state_ = v_.norm() > .15 ? State::BRAKE : State::PLAN;
    RCLCPP_INFO(get_logger(), "返航：%s，使用同一局部规划器回机库", outcome.c_str());
  }

  void BeginAttack(double t)
  {
    ++attempts_;
    impact_flag_ = false;
    contact_guess_ = false;
    approach_ready_ = false;
    had_speed_ = false;
    peak_closing_ = 0.0;
    attack_started_at_ = t;
    attack_counted_ = false;
    attempt_resolved_ = false;
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
    estop_latched_ = true;
    start_requested_ = false;
    std_msgs::msg::Bool m;
    m.data = true;
    estop_pub_->publish(m);
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
  double base_standoff_, base_view_height_;
  double attack_acc_, body_half_h_, attack_started_at_{0.0};
  double attack_z_tol_, attack_vz_tol_;
  bool attack_counted_{false};
  double soft_fence_, hard_fence_, ceiling_, phase_tol_, latency_, body_r_, brake_acc_, odom_timeout_;
  int max_attacks_, max_attempts_, attempts_{0}, plan_fail_{0};
  int consecutive_misses_{0};
  bool match_mode_{true},attempt_resolved_{false};
  double match_duration_,return_reserve_;
  double mission_timeout_, return_timeout_, mission_started_{0}, return_started_{0}, home_yaw_{0};
  bool returning_{false}, return_requested_{false}, landing_vision_{true}, landing_aligned_{false};
  bool grounded_{false}, have_landing_{false};
  bool have_landing_velocity_{false},landing_imu_still_{false};
  Vec3 landing_velocity_{Vec3::Zero()};
  Vec3 landing_velocity_bias_{Vec3::Zero()};
  rclcpp::Time landing_imu_stamp_{0,0,RCL_ROS_TIME};
  double touchdown_since_{-1};
  Vec3 landing_error_{Vec3::Zero()};
  std::string outcome_{"idle"};
  rclcpp::Time landing_stamp_{0, 0, RCL_ROS_TIME}, ground_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr return_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ground_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr landing_sub_;

  FieldMap field_;
  OccupancyGrid grid_;
  MinSnapTrajectory traj_;
  std::vector<Vec3> path_;
  Target target_;
  Vec3 aim_{Vec3::Zero()}, aim_filtered_{Vec3::Zero()}, normal_{Vec3::UnitX()}, standoff_pt_{Vec3::Zero()};

  State state_{State::IDLE}, prev_state_{State::IDLE}, resume_after_brake_{State::PLAN};
  double state_t0_{0}, traj_t0_{0}, ready_since_{-1}, last_match_{-1e9}, last_yaw_t_{-1}, yaw_sp_{0};
  double phase_{0}, phase_t_{-1}, omega_{0};
  int match_count_{0}, attacks_{0}, vio_bad_{0};
  uint64_t marker_count_{0};
  std::string planner_backend_;
  bool use_static_prior_{true};
  bool ego_navigation_{false}, have_ego_trajectory_{false}, have_ego_status_{false}, brake_hold_set_{false};
  uint64_t ego_request_{0}, ego_revision_{0};
  rclcpp::Time ego_start_{0, 0, RCL_ROS_TIME};
  SplineTrajectory ego_spline_;
  rm_drone_msgs::msg::PlannerStatus ego_status_;
  rclcpp::Publisher<rm_drone_msgs::msg::PlannerGoal>::SharedPtr ego_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr contact_goal_pub_;
  rclcpp::Subscription<rm_drone_msgs::msg::LocalTrajectory>::SharedPtr ego_traj_sub_;
  rclcpp::Subscription<rm_drone_msgs::msg::PlannerStatus>::SharedPtr ego_status_sub_;
  bool have_odom_{false}, odom_valid_{false}, have_est_{false}, have_ref_{false}, start_requested_{false};
  bool impact_flag_{false}, contact_guess_{false}, estop_latched_{false}, estop_reset_{false};
  bool approach_ready_{false}, had_speed_{false};
  double peak_closing_{0.0};
  Vec3 p_{Vec3::Zero()}, v_{Vec3::Zero()}, hold_{Vec3::Zero()}, home_{Vec3::Zero()}, land_xy_{Vec3::Zero()};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  rm_drone_msgs::msg::EstimatorStatus est_;
  rm_drone_msgs::msg::RefereeStatus ref_;
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME}, last_ros_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point odom_rx_{}, est_rx_{}, last_steady_{};
  std::chrono::steady_clock::time_point startup_log_{};
  std::string startup_info_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rm_drone_msgs::msg::EstimatorStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<rm_drone_msgs::msg::RefereeStatus>::SharedPtr ref_sub_;
  rclcpp::Subscription<rm_drone_msgs::msg::ArmorDetectionArray>::SharedPtr armor_sub_;
  rclcpp::Subscription<rm_drone_msgs::msg::ImpactEvent>::SharedPtr impact_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_, estop_reset_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
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
