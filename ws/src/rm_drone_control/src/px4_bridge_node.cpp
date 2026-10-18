// 实机 PX4 桥（uXRCE-DDS + px4_msgs）：
//  - AttitudeTarget(ENU/FLU)  -> /fmu/in/vehicle_attitude_setpoint (NED/FRD) + offboard_control_mode 心跳
//  - 估计器 odom(ENU/FLU)     -> /fmu/in/vehicle_visual_odometry (NED/FRD)，供 PX4 EKF2 融合外部视觉
//  - arm / offboard 模式切换通过 /fmu/in/vehicle_command
// 话题名做成参数，适配不同 PX4 版本（如 v1.16 的 *_v1 后缀）。
#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/attitude_target.hpp>

#include "rm_drone_control/math_utils.hpp"

namespace rm_drone
{

namespace
{
// ENU->NED 世界系旋转，FLU->FRD 机体系旋转
const Eigen::Quaterniond Q_NED_ENU(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 1, 0).normalized()));
const Eigen::Quaterniond Q_FLU_FRD(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

Eigen::Vector3d enuToNed(const Eigen::Vector3d & v) {return Eigen::Vector3d(v.y(), v.x(), -v.z());}
Eigen::Vector3d fluToFrd(const Eigen::Vector3d & v) {return Eigen::Vector3d(v.x(), -v.y(), -v.z());}
Eigen::Quaterniond enuFluToNedFrd(const Eigen::Quaterniond & q) {return Q_NED_ENU * q * Q_FLU_FRD;}
}  // namespace

class Px4BridgeNode : public rclcpp::Node
{
public:
  Px4BridgeNode()
  : Node("px4_bridge")
  {
    const auto t_att = declare_parameter<std::string>("topic_attitude_sp", "/fmu/in/vehicle_attitude_setpoint");
    const auto t_ocm = declare_parameter<std::string>("topic_offboard_mode", "/fmu/in/offboard_control_mode");
    const auto t_cmd = declare_parameter<std::string>("topic_vehicle_command", "/fmu/in/vehicle_command");
    const auto t_odo = declare_parameter<std::string>("topic_visual_odometry", "/fmu/in/vehicle_visual_odometry");
    send_odom_ = declare_parameter("send_visual_odometry", true);
    auto_offboard_ = declare_parameter("auto_offboard", true);

    att_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(t_att, 10);
    ocm_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(t_ocm, 10);
    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(t_cmd, 10);
    odo_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(t_odo, 10);

    att_sub_ = create_subscription<rm_drone_msgs::msg::AttitudeTarget>(
      "attitude_target", 10, std::bind(&Px4BridgeNode::OnAttitude, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), std::bind(&Px4BridgeNode::OnOdom, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&Px4BridgeNode::Heartbeat, this));
  }

private:
  uint64_t Micros() {return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);}

  void Heartbeat()
  {
    px4_msgs::msg::OffboardControlMode m{};
    m.timestamp = Micros();
    m.attitude = true;
    ocm_pub_->publish(m);
  }

  void Command(uint16_t cmd, float p1 = 0.f, float p2 = 0.f)
  {
    px4_msgs::msg::VehicleCommand c{};
    c.timestamp = Micros();
    c.command = cmd;
    c.param1 = p1;
    c.param2 = p2;
    c.target_system = 1;
    c.target_component = 1;
    c.source_system = 1;
    c.source_component = 1;
    c.from_external = true;
    cmd_pub_->publish(c);
  }

  void OnAttitude(rm_drone_msgs::msg::AttitudeTarget::SharedPtr m)
  {
    if (m->armed != armed_) {
      if (m->armed && auto_offboard_) {
        Command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);  // PX4 custom main mode 6 = OFFBOARD
      }
      Command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, m->armed ? 1.f : 0.f);
      armed_ = m->armed;
    }
    const Eigen::Quaterniond q = enuFluToNedFrd(toEigen(m->orientation).normalized());
    px4_msgs::msg::VehicleAttitudeSetpoint sp{};
    sp.timestamp = Micros();
    sp.q_d[0] = q.w();
    sp.q_d[1] = q.x();
    sp.q_d[2] = q.y();
    sp.q_d[3] = q.z();
    sp.thrust_body[0] = 0.f;
    sp.thrust_body[1] = 0.f;
    sp.thrust_body[2] = -m->thrust;
    sp.yaw_sp_move_rate = -m->body_rate.z;
    att_pub_->publish(sp);
  }

  void OnOdom(nav_msgs::msg::Odometry::SharedPtr m)
  {
    if (!send_odom_) {return;}
    const Eigen::Quaterniond q_enu = toEigen(m->pose.pose.orientation).normalized();
    const Eigen::Vector3d p = enuToNed(Eigen::Vector3d(m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z));
    const Eigen::Vector3d v = enuToNed(q_enu * toEigen(m->twist.twist.linear));
    const Eigen::Vector3d w = fluToFrd(toEigen(m->twist.twist.angular));
    const Eigen::Quaterniond q = enuFluToNedFrd(q_enu);

    px4_msgs::msg::VehicleOdometry o{};
    o.timestamp = Micros();
    o.timestamp_sample = static_cast<uint64_t>(rclcpp::Time(m->header.stamp).nanoseconds() / 1000);
    o.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    o.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    for (int i = 0; i < 3; ++i) {
      o.position[i] = p[i];
      o.velocity[i] = v[i];
      o.angular_velocity[i] = w[i];
      o.position_variance[i] = static_cast<float>(std::max(1e-4, m->pose.covariance[i * 7]));
      o.orientation_variance[i] = static_cast<float>(std::max(1e-4, m->pose.covariance[21 + i * 7]));
      o.velocity_variance[i] = static_cast<float>(std::max(1e-4, m->twist.covariance[i * 7]));
    }
    o.q[0] = q.w();
    o.q[1] = q.x();
    o.q[2] = q.y();
    o.q[3] = q.z();
    odo_pub_->publish(o);
  }

  bool send_odom_, auto_offboard_, armed_{false};
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr att_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odo_pub_;
  rclcpp::Subscription<rm_drone_msgs::msg::AttitudeTarget>::SharedPtr att_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rm_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_drone::Px4BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
