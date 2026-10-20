// 机载位置控制器（仿真与实机共用）：
// 输入：估计器里程计 odom（map/ENU）+ 轨迹设定点 setpoint；输出：AttitudeTarget（姿态+归一化推力）。
// 控制律：a_des = a_ff + Kp*e_p + Kv*e_v + Ki*∫e_p + g，机体 z 轴对齐 a_des（带倾角限制），推力投影到当前机体 z 轴。
#include <chrono>

#include <Eigen/Dense>
#include <std_msgs/msg/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/attitude_target.hpp>
#include <rm_drone_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rm_drone_control/fault_policy.hpp"
#include "rm_drone_control/math_utils.hpp"

namespace rm_drone
{

class PositionControllerNode : public rclcpp::Node
{
public:
  PositionControllerNode()
  : Node("position_controller")
  {
    mass_ = declare_parameter("mass", 0.150);
    max_thrust_ = declare_parameter("max_thrust", 6.0);  // N，4 * kf * wmax^2
    const double kp_xy = declare_parameter("kp_xy", 6.0);
    const double kv_xy = declare_parameter("kv_xy", 4.0);
    const double ki_xy = declare_parameter("ki_xy", 0.5);
    kp_ = Eigen::Vector3d(kp_xy, kp_xy, declare_parameter("kp_z", 8.0));
    kv_ = Eigen::Vector3d(kv_xy, kv_xy, declare_parameter("kv_z", 5.0));
    ki_ = Eigen::Vector3d(ki_xy, ki_xy, declare_parameter("ki_z", 1.5));
    max_tilt_ = declare_parameter("max_tilt_deg", 40.0) * M_PI / 180.0;
    max_acc_ = declare_parameter("max_acc", 12.0);
    sp_timeout_ = declare_parameter("setpoint_timeout", 0.5);
    hover_thrust_ = declare_parameter("hover_thrust", 0.25);
    thrust_limit_ = declare_parameter("thrust_limit", 0.35);

    sp_sub_ = create_subscription<rm_drone_msgs::msg::TrajectorySetpoint>(
      "setpoint", 10, [this](rm_drone_msgs::msg::TrajectorySetpoint::SharedPtr m) {
        const Vec3 p = toEigen(m->position), v = toEigen(m->velocity), a = toEigen(m->acceleration);
        sp_rx_ = std::chrono::steady_clock::now();
        if (!finiteVec(p) || !finiteVec(v) || !finiteVec(a) || !std::isfinite(m->yaw)) {sp_valid_ = false; return;}
        sp_ = *m;
        sp_stamp_ = m->header.stamp;
        have_sp_ = true;
        sp_valid_ = true;
      });
    arm_sub_ = create_subscription<std_msgs::msg::Bool>(
      "arm", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
        if (m->data && !armed_ && !estop_latched_) {integ_.setZero();}
        if (m->data != armed_) {RCLCPP_INFO(get_logger(), m->data ? "ARM" : "DISARM");}
        armed_ = m->data && !estop_latched_;
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>("estop", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
        if (m->data) {estop_latched_ = true; armed_ = false; integ_.setZero();}
      });
    estop_reset_sub_ = create_subscription<std_msgs::msg::Empty>("estop_reset", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        if (estopRelease(estop_latched_, true, armed_)) {estop_latched_ = false;}
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), std::bind(&PositionControllerNode::OnOdom, this, std::placeholders::_1));
    pub_ = create_publisher<rm_drone_msgs::msg::AttitudeTarget>("attitude_target", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&PositionControllerNode::Step, this));
  }

private:
  using Vec3 = Eigen::Vector3d;

  void OnOdom(nav_msgs::msg::Odometry::SharedPtr odom)
  {
    odom_rx_ = std::chrono::steady_clock::now();
    const rclcpp::Time stamp(odom->header.stamp);
    const Vec3 p(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    const Eigen::Quaterniond q = toEigen(odom->pose.pose.orientation);
    const Vec3 vb = toEigen(odom->twist.twist.linear);
    const double prev = odom_stamp_.nanoseconds() > 0 ? odom_stamp_.seconds() : -1.0;
    if (!stampUsable(stamp.seconds(), now().seconds(), prev, finiteVec(p) && finiteVec(vb), finiteQuat(q))) {
      odom_valid_ = false;
      return;
    }
    p_ = p;
    q_ = q.normalized();
    v_ = q_ * vb;
    odom_stamp_ = stamp;
    have_odom_ = true;
    odom_valid_ = true;
  }

  void Step()
  {
    const rclcpp::Time ros_now = now();
    const auto steady_now = std::chrono::steady_clock::now();
    const double ros_dt = last_ros_.nanoseconds() > 0 ? (ros_now - last_ros_).seconds() : 0.02;
    const double steady_dt = std::chrono::duration<double>(steady_now - last_wall_).count();
    if (ros_dt < -1e-3) {
      last_ros_ = ros_now;
      last_wall_ = steady_now;
      return;
    }
    const bool paused = last_ros_.nanoseconds() > 0 && simPaused(std::max(0.0, ros_dt), steady_dt);
    last_ros_ = ros_now;
    last_wall_ = steady_now;
    if (!have_odom_) {return;}

    const double odom_age_ros = (ros_now - odom_stamp_).seconds();
    const double odom_age_steady = std::chrono::duration<double>(steady_now - odom_rx_).count();
    const double sp_age_steady = std::chrono::duration<double>(steady_now - sp_rx_).count();
    const auto act = positionFault(estop_latched_, paused, odom_valid_, odom_age_ros, odom_age_steady,
      sp_valid_, sp_age_steady, sp_timeout_);

    const double dt = std::clamp(paused ? 0.0 : ros_dt, 0.0, 0.05);
    rm_drone_msgs::msg::AttitudeTarget out;
    out.header.stamp = ros_now;
    out.header.frame_id = "map";
    out.mode = rm_drone_msgs::msg::AttitudeTarget::MODE_ATTITUDE;
    out.armed = armed_ && act != FaultAct::CutMotors;
    if (act == FaultAct::CutMotors || !armed_) {
      out.orientation = toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(yawOf(q_), Eigen::Vector3d::UnitZ())));
      out.thrust = 0.0f;
      out.armed = false;
      pub_->publish(out);
      integ_.setZero();
      return;
    }
    if (act == FaultAct::Descend) {
      out.orientation = toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(yawOf(q_), Eigen::Vector3d::UnitZ())));
      out.thrust = static_cast<float>(std::clamp(0.8 * hover_thrust_, 0.05, thrust_limit_));
      pub_->publish(out);
      return;
    }

    const Eigen::Vector3d p_sp = toEigen(sp_.position), v_sp = toEigen(sp_.velocity), a_sp = toEigen(sp_.acceleration);
    Eigen::Vector3d ep = p_sp - p_, ev = v_sp - v_;
    ep.z() = std::clamp(ep.z(), -0.35, 0.35);
    ev.z() = std::clamp(ev.z(), -0.8, 0.8);
    Eigen::Vector3d a_unsat = a_sp + kp_.cwiseProduct(ep) + kv_.cwiseProduct(ev) + ki_.cwiseProduct(integ_);
    Eigen::Vector3d a_lim = a_unsat;
    if (a_lim.norm() > max_acc_) {a_lim *= max_acc_ / a_lim.norm();}
    integ_ += (ep - 0.5 * (a_unsat - a_lim)) * dt;
    integ_ = integ_.cwiseMax(-1.0).cwiseMin(1.0);
    Eigen::Vector3d a_des = a_sp + kp_.cwiseProduct(ep) + kv_.cwiseProduct(ev) + ki_.cwiseProduct(integ_);
    if (a_des.norm() > max_acc_) {a_des *= max_acc_ / a_des.norm();}
    a_des.z() += 9.81;
    a_des.z() = std::max(a_des.z(), 0.5);

    // 倾角限制：保持竖直分量，缩小水平分量
    const double h = a_des.head<2>().norm();
    const double h_max = a_des.z() * std::tan(max_tilt_);
    if (h > h_max) {a_des.head<2>() *= h_max / h;}

    const Eigen::Vector3d zb = a_des.normalized();
    const Eigen::Vector3d xc(std::cos(sp_.yaw), std::sin(sp_.yaw), 0.0);
    Eigen::Vector3d yb = zb.cross(xc);
    if (yb.norm() < 1e-3) {yb = Eigen::Vector3d::UnitY();}
    yb.normalize();
    const Eigen::Vector3d xb = yb.cross(zb);
    Eigen::Matrix3d R;
    R << xb, yb, zb;
    const Eigen::Quaterniond q_des(R);

    const double thrust_n = mass_ * a_des.dot(q_ * Eigen::Vector3d::UnitZ());
    out.orientation = toMsg(q_des);
    out.body_rate = toMsg(Eigen::Vector3d(0, 0, sp_.yaw_rate));
    // thrust_limit 是仿真侧的保护上限，没有台架数据，不代表实机推力已经标定。
    out.thrust = static_cast<float>(std::clamp(thrust_n / max_thrust_, 0.0, thrust_limit_));
    pub_->publish(out);
  }

  double mass_, max_thrust_, max_tilt_, max_acc_, sp_timeout_, hover_thrust_, thrust_limit_;
  Eigen::Vector3d kp_, kv_, ki_, integ_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p_{Eigen::Vector3d::Zero()}, v_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  rm_drone_msgs::msg::TrajectorySetpoint sp_;
  bool have_sp_{false}, sp_valid_{false}, armed_{false}, have_odom_{false}, odom_valid_{false}, estop_latched_{false};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME}, sp_stamp_{0, 0, RCL_ROS_TIME}, last_ros_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point odom_rx_{}, sp_rx_{}, last_wall_{};

  rclcpp::Subscription<rm_drone_msgs::msg::TrajectorySetpoint>::SharedPtr sp_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_, estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr estop_reset_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<rm_drone_msgs::msg::AttitudeTarget>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rm_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_drone::PositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
