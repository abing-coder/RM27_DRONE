// 仿真飞控：模拟 PX4 在 offboard 姿态模式下的内环（姿态 P -> 角速度 PID -> 混控）。
// 实机上这一层由 PX4 本身完成，上层通过 px4_bridge_node 发送同样的 AttitudeTarget。
// 姿态来源：仿真真值（代表 PX4 自身 EKF 的姿态估计），角速度来源：带噪声的仿真 IMU。
#include <array>
#include <chrono>

#include <std_msgs/msg/empty.hpp>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/attitude_target.hpp>
#include <rm_drone_msgs/msg/motor_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rm_drone_control/fault_policy.hpp"
#include "rm_drone_control/math_utils.hpp"

namespace rm_drone
{

class FcSimNode : public rclcpp::Node
{
public:
  FcSimNode()
  : Node("fc_sim")
  {
    kf_ = declare_parameter("thrust_coefficient", 1.667e-7);
    km_ = declare_parameter("moment_coefficient", 1.667e-9);
    w_max_ = declare_parameter("max_rot_velocity", 3000.0);
    arm_ = declare_parameter("arm_length", 0.048);
    J_ = Eigen::Vector3d(
      declare_parameter("ixx", 1.2e-4), declare_parameter("iyy", 1.2e-4), declare_parameter("izz", 2.2e-4)).asDiagonal();
    const double att_kp_rp = declare_parameter("att_kp_rp", 9.0);
    const double rate_kp_rp = declare_parameter("rate_kp_rp", 30.0);
    const double rate_ki_rp = declare_parameter("rate_ki_rp", 20.0);
    const double rate_kd_rp = declare_parameter("rate_kd_rp", 0.4);
    att_kp_ = Eigen::Vector3d(att_kp_rp, att_kp_rp, declare_parameter("att_kp_yaw", 4.0));
    rate_kp_ = Eigen::Vector3d(rate_kp_rp, rate_kp_rp, declare_parameter("rate_kp_yaw", 12.0));
    rate_ki_ = Eigen::Vector3d(rate_ki_rp, rate_ki_rp, declare_parameter("rate_ki_yaw", 5.0));
    rate_kd_ = Eigen::Vector3d(rate_kd_rp, rate_kd_rp, declare_parameter("rate_kd_yaw", 0.0));
    max_rate_ = declare_parameter("max_rate", 12.0);
    sp_timeout_ = declare_parameter("setpoint_timeout", 0.3);

    f_max_ = kf_ * w_max_ * w_max_;
    BuildMixer();

    att_sub_ = create_subscription<rm_drone_msgs::msg::AttitudeTarget>(
      "attitude_target", 10, [this](rm_drone_msgs::msg::AttitudeTarget::SharedPtr m) {
        sp_ = *m;
        last_sp_ = now();
        have_sp_ = true;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "ground_truth/odom", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::SharedPtr m) {
        const auto q = toEigen(m->pose.pose.orientation);
        if (!finiteQuat(q)) {return;}
        q_ = q.normalized();
        att_rx_ = std::chrono::steady_clock::now();
        have_att_ = true;
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "estop", 10, [this](std_msgs::msg::Bool::SharedPtr m) {
        if (m->data && !killed_) {RCLCPP_WARN(get_logger(), "紧急停桨锁存。arm=true 不能解除，需要 estop_reset 且已上锁。");}
        if (m->data) {killed_ = true;}
      });
    estop_reset_sub_ = create_subscription<std_msgs::msg::Empty>(
      "estop_reset", 10, [this](std_msgs::msg::Empty::SharedPtr) {
        if (estopRelease(killed_, true, have_sp_ && sp_.armed)) {killed_ = false;}
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(), std::bind(&FcSimNode::OnImu, this, std::placeholders::_1));
    motor_pub_ = create_publisher<rm_drone_msgs::msg::MotorCommand>("motor_cmd", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
        const rclcpp::Time ros_now = now();
        const double ros_dt = last_ros_.nanoseconds() > 0 ? (ros_now - last_ros_).seconds() : 0.02;
        const double wall_dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_wall_).count();
        const bool paused = last_ros_.nanoseconds() > 0 && simPaused(std::max(0.0, ros_dt), wall_dt);
        last_ros_ = ros_now;
        last_wall_ = std::chrono::steady_clock::now();
        if (paused || ros_dt < 0.0) {return;}
        const double age = receiveAge(get_clock()->ros_time_is_active(), (ros_now - last_imu_).seconds(),
          std::chrono::duration<double>(std::chrono::steady_clock::now() - imu_rx_).count());
        if (have_imu_rx_ && age > 0.1) {
          rm_drone_msgs::msg::MotorCommand out;
          out.header.stamp = ros_now;
          out.motor_speed.assign(4, 0.0f);
          motor_pub_->publish(out);
        }
      });
    RCLCPP_INFO(get_logger(), "fc_sim ready, max thrust %.2f N。遥控器失联停桨需实机接收机实现，这里只锁存 ROS 急停。", 4 * f_max_);
  }

private:
  void BuildMixer()
  {
    const double d = arm_ / std::sqrt(2.0);
    const std::array<double, 4> x{{d, -d, d, -d}}, y{{-d, d, d, -d}}, dir{{1, 1, -1, -1}};
    Eigen::Matrix4d A;
    for (int i = 0; i < 4; ++i) {
      A(0, i) = 1.0;
      A(1, i) = y[i];
      A(2, i) = -x[i];
      A(3, i) = -dir[i] * km_ / kf_;
    }
    mix_ = A.inverse();
  }

  void OnImu(sensor_msgs::msg::Imu::SharedPtr imu)
  {
    imu_rx_ = std::chrono::steady_clock::now();
    have_imu_rx_ = true;
    const rclcpp::Time t = imu->header.stamp;
    const double raw_dt = last_imu_.nanoseconds() > 0 ? (t - last_imu_).seconds() : 0.0025;
    if (!std::isfinite(raw_dt) || raw_dt < 0.0 || raw_dt > 0.05) {
      last_imu_ = t;
      return;
    }
    const double dt = raw_dt;
    last_imu_ = t;

    rm_drone_msgs::msg::MotorCommand out;
    out.header.stamp = imu->header.stamp;
    out.motor_speed.assign(4, 0.0f);

    const bool sp_fresh = have_sp_ && (now() - last_sp_).seconds() < sp_timeout_;
    if (killed_ || !have_att_ || !have_sp_ || !sp_.armed) {
      integ_.setZero();
      motor_pub_->publish(out);
      return;
    }

    Eigen::Quaterniond q_sp = toEigen(sp_.orientation).normalized();
    double thrust = sp_.thrust;
    Eigen::Vector3d rate_ff = toEigen(sp_.body_rate);
    if (!sp_fresh) {
      // 失去 offboard 设定值：保持水平、缓慢下降（类似 PX4 failsafe land）
      q_sp = Eigen::AngleAxisd(yawOf(q_), Eigen::Vector3d::UnitZ()) * Eigen::Quaterniond::Identity();
      thrust = 0.85 * hover_thrust_guess_;
      rate_ff.setZero();
    } else if (sp_.mode == rm_drone_msgs::msg::AttitudeTarget::MODE_ATTITUDE) {
      hover_thrust_guess_ = 0.98 * hover_thrust_guess_ + 0.02 * std::clamp(thrust, 0.1, 0.6);
    }

    const Eigen::Vector3d omega(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    Eigen::Vector3d rate_sp;
    if (sp_fresh && sp_.mode == rm_drone_msgs::msg::AttitudeTarget::MODE_RATES) {
      rate_sp = rate_ff;
    } else {
      Eigen::Quaterniond qe = q_.conjugate() * q_sp;
      if (qe.w() < 0) {qe.coeffs() *= -1.0;}
      rate_sp = 2.0 * att_kp_.cwiseProduct(qe.vec()) + rate_ff;
    }
    rate_sp = rate_sp.cwiseMax(-max_rate_).cwiseMin(max_rate_);

    const Eigen::Vector3d err = rate_sp - omega;
    const Eigen::Vector3d d_omega = (omega - last_omega_) / std::max(dt, 1e-4);
    last_omega_ = omega;
    if (thrust > 0.1) {integ_ = (integ_ + err * dt).cwiseMax(-0.3).cwiseMin(0.3);}
    const Eigen::Vector3d alpha = rate_kp_.cwiseProduct(err) + rate_ki_.cwiseProduct(integ_) - rate_kd_.cwiseProduct(d_omega);
    Eigen::Vector3d torque = J_ * alpha + omega.cross(J_ * omega);

    const double T = std::clamp(thrust, 0.0, 1.0) * 4.0 * f_max_;
    Eigen::Vector4d f;
    // 饱和处理：优先保证推力和横滚/俯仰，必要时逐步缩小力矩
    for (double s = 1.0; s >= 0.0; s -= 0.1) {
      Eigen::Vector4d u(T, s * torque.x(), s * torque.y(), (s < 1.0 ? 0.3 * s : 1.0) * torque.z());
      f = mix_ * u;
      if (f.minCoeff() >= 0.0 && f.maxCoeff() <= f_max_) {break;}
    }
    for (int i = 0; i < 4; ++i) {
      out.motor_speed[i] = static_cast<float>(std::sqrt(std::clamp(f[i], 0.0, f_max_) / kf_));
    }
    motor_pub_->publish(out);
  }

  double kf_, km_, w_max_, arm_, f_max_, max_rate_, sp_timeout_;
  double hover_thrust_guess_{0.25};
  Eigen::Matrix3d J_;
  Eigen::Vector3d att_kp_, rate_kp_, rate_ki_, rate_kd_;
  Eigen::Matrix4d mix_;
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d integ_{Eigen::Vector3d::Zero()}, last_omega_{Eigen::Vector3d::Zero()};
  rm_drone_msgs::msg::AttitudeTarget sp_;
  rclcpp::Time last_sp_{0, 0, RCL_ROS_TIME}, last_imu_{0, 0, RCL_ROS_TIME};
  bool have_sp_{false}, have_att_{false}, killed_{false}, have_imu_rx_{false};
  std::chrono::steady_clock::time_point att_rx_{}, imu_rx_{}, last_wall_{};
  rclcpp::Time last_ros_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<rm_drone_msgs::msg::AttitudeTarget>::SharedPtr att_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr estop_reset_sub_;
  rclcpp::Publisher<rm_drone_msgs::msg::MotorCommand>::SharedPtr motor_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rm_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_drone::FcSimNode>());
  rclcpp::shutdown();
  return 0;
}
