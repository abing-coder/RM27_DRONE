// 机载位置控制器（仿真与实机共用）：
// 输入：估计器里程计 odom（map/ENU）+ 轨迹设定点 setpoint；输出：AttitudeTarget（姿态+归一化推力）。
// 控制律：a_des = a_ff + Kp*e_p + Kv*e_v + Ki*∫e_p + g，机体 z 轴对齐 a_des（带倾角限制），推力投影到当前机体 z 轴。
#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/attitude_target.hpp>
#include <rm_drone_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/bool.hpp>

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

    sp_sub_ = create_subscription<rm_drone_msgs::msg::TrajectorySetpoint>(
      "setpoint", 10, [this](rm_drone_msgs::msg::TrajectorySetpoint::SharedPtr m) {
        sp_ = *m;
        last_sp_ = now();
        have_sp_ = true;
      });
    arm_sub_ = create_subscription<std_msgs::msg::Bool>(
      "arm", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
        if (m->data != armed_) {RCLCPP_INFO(get_logger(), m->data ? "ARM" : "DISARM");}
        armed_ = m->data;
        integ_.setZero();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), std::bind(&PositionControllerNode::OnOdom, this, std::placeholders::_1));
    pub_ = create_publisher<rm_drone_msgs::msg::AttitudeTarget>("attitude_target", 10);
  }

private:
  void OnOdom(nav_msgs::msg::Odometry::SharedPtr odom)
  {
    const rclcpp::Time t = odom->header.stamp;
    const double dt = last_t_.nanoseconds() > 0 ? std::clamp((t - last_t_).seconds(), 1e-4, 0.05) : 0.01;
    last_t_ = t;

    const Eigen::Vector3d p(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    const Eigen::Quaterniond q = toEigen(odom->pose.pose.orientation).normalized();
    // nav_msgs/Odometry 的 twist 在 child_frame（机体系）中表达
    const Eigen::Vector3d v = q * toEigen(odom->twist.twist.linear);

    rm_drone_msgs::msg::AttitudeTarget out;
    out.header.stamp = odom->header.stamp;
    out.header.frame_id = "map";
    out.mode = rm_drone_msgs::msg::AttitudeTarget::MODE_ATTITUDE;
    out.armed = armed_;

    if (!armed_ || !have_sp_ || (now() - last_sp_).seconds() > sp_timeout_) {
      out.orientation = toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(yawOf(q), Eigen::Vector3d::UnitZ())));
      out.thrust = 0.0f;
      out.armed = armed_ && have_sp_;
      pub_->publish(out);
      integ_.setZero();
      return;
    }

    const Eigen::Vector3d p_sp = toEigen(sp_.position), v_sp = toEigen(sp_.velocity), a_sp = toEigen(sp_.acceleration);
    Eigen::Vector3d ep = p_sp - p, ev = v_sp - v;
    ep.z() = std::clamp(ep.z(), -0.35, 0.35);
    ev.z() = std::clamp(ev.z(), -0.8, 0.8);
    integ_ = (integ_ + ep * dt).cwiseMax(-1.0).cwiseMin(1.0);

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

    const double thrust_n = mass_ * a_des.dot(q * Eigen::Vector3d::UnitZ());
    out.orientation = toMsg(q_des);
    out.body_rate = toMsg(Eigen::Vector3d(0, 0, sp_.yaw_rate));
    // 悬停约 0.25。限制在 0.45，避免估计瞬变时满推力把飞机打上天
    out.thrust = static_cast<float>(std::clamp(thrust_n / max_thrust_, 0.0, 0.35));
    pub_->publish(out);
  }

  double mass_, max_thrust_, max_tilt_, max_acc_, sp_timeout_;
  Eigen::Vector3d kp_, kv_, ki_, integ_{Eigen::Vector3d::Zero()};
  rm_drone_msgs::msg::TrajectorySetpoint sp_;
  bool have_sp_{false}, armed_{false};
  rclcpp::Time last_sp_{0, 0, RCL_ROS_TIME}, last_t_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<rm_drone_msgs::msg::TrajectorySetpoint>::SharedPtr sp_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<rm_drone_msgs::msg::AttitudeTarget>::SharedPtr pub_;
};

}  // namespace rm_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_drone::PositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
