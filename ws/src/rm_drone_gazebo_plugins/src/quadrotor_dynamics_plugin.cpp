// 四旋翼动力学插件：电机一阶延迟、推力/反扭矩、机体空气阻力。
// 电机顺序与 PX4 Quad-X 一致（0 起编号）：
//   m0 右前 CCW, m1 左后 CCW, m2 左前 CW, m3 右后 CW
// 订阅 rm_drone_msgs/MotorCommand（rad/s），超时未收到指令则电机转速指令归零（模拟失控停桨）。
#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/motor_command.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace rm_drone
{

class QuadrotorDynamicsPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    auto get = [&](const std::string & k, double def) {
        return sdf->HasElement(k) ? sdf->Get<double>(k) : def;
      };
    std::string link_name = sdf->HasElement("link_name") ? sdf->Get<std::string>("link_name") : "base_link";
    link_ = model_->GetLink(link_name);
    if (!link_) {
      RCLCPP_FATAL(node_->get_logger(), "quadrotor_dynamics: link %s not found", link_name.c_str());
      return;
    }
    arm_ = get("arm_length", 0.048);
    kf_ = get("thrust_coefficient", 1.667e-7);
    km_ = get("moment_coefficient", 1.667e-9);
    tau_up_ = get("motor_time_constant_up", 0.020);
    tau_down_ = get("motor_time_constant_down", 0.035);
    max_w_ = get("max_rot_velocity", 3000.0);
    drag_lin_ = get("linear_drag", 0.06);
    drag_quad_ = get("quadratic_drag", 0.012);
    rotor_drag_ = get("rotor_drag", 1.2e-5);
    timeout_ = get("command_timeout", 0.25);

    const double d = arm_ / std::sqrt(2.0);
    pos_ = {{{d, -d}, {-d, d}, {d, d}, {-d, -d}}};
    dir_ = {{1, 1, -1, -1}};  // +1 = CCW

    sub_ = node_->create_subscription<rm_drone_msgs::msg::MotorCommand>(
      "motor_cmd", rclcpp::SensorDataQoS(),
      [this](rm_drone_msgs::msg::MotorCommand::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        for (size_t i = 0; i < 4 && i < msg->motor_speed.size(); ++i) {
          cmd_[i] = std::clamp<double>(msg->motor_speed[i], 0.0, max_w_);
        }
        last_cmd_ = model_->GetWorld()->SimTime().Double();
      });
    pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("motor_speed", 10);

    conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&QuadrotorDynamicsPlugin::OnUpdate, this));
    RCLCPP_INFO(node_->get_logger(), "quadrotor_dynamics loaded: arm=%.3f kf=%.3g km=%.3g", arm_, kf_, km_);
  }

private:
  void OnUpdate()
  {
    const double t = model_->GetWorld()->SimTime().Double();
    const double dt = model_->GetWorld()->Physics()->GetMaxStepSize();
    std::array<double, 4> cmd;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      cmd = cmd_;
      if (t - last_cmd_ > timeout_) {cmd.fill(0.0);}
    }

    const auto v_world = link_->WorldLinearVel();
    const auto rot = link_->WorldPose().Rot();
    const auto v_body = rot.RotateVectorReverse(v_world);

    double yaw_torque = 0.0;
    double w_sum = 0.0;
    for (int i = 0; i < 4; ++i) {
      const double tau = cmd[i] > w_[i] ? tau_up_ : tau_down_;
      w_[i] += (cmd[i] - w_[i]) * std::min(1.0, dt / tau);
      const double thrust = kf_ * w_[i] * w_[i];
      link_->AddLinkForce({0.0, 0.0, thrust}, {pos_[i][0], pos_[i][1], 0.0});
      yaw_torque += -dir_[i] * km_ * w_[i] * w_[i];
      w_sum += w_[i];
    }
    link_->AddRelativeTorque({0.0, 0.0, yaw_torque});

    // 机体阻力（线性 + 二次）以及桨盘诱导阻力（与转速成正比，仅水平方向）
    ignition::math::Vector3d drag_body(
      -(drag_lin_ + drag_quad_ * std::abs(v_body.X())) * v_body.X() - rotor_drag_ * w_sum * v_body.X(),
      -(drag_lin_ + drag_quad_ * std::abs(v_body.Y())) * v_body.Y() - rotor_drag_ * w_sum * v_body.Y(),
      -(drag_lin_ + drag_quad_ * std::abs(v_body.Z())) * v_body.Z());
    link_->AddRelativeForce(drag_body);

    if (++pub_count_ % 10 == 0) {
      std_msgs::msg::Float32MultiArray m;
      m.data.assign(w_.begin(), w_.end());
      pub_->publish(m);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::event::ConnectionPtr conn_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<rm_drone_msgs::msg::MotorCommand>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;

  std::mutex mtx_;
  std::array<double, 4> cmd_{{0, 0, 0, 0}};
  std::array<double, 4> w_{{0, 0, 0, 0}};
  std::array<std::array<double, 2>, 4> pos_;
  std::array<double, 4> dir_;
  double last_cmd_{-1e9};
  double arm_, kf_, km_, tau_up_, tau_down_, max_w_, drag_lin_, drag_quad_, rotor_drag_, timeout_;
  uint64_t pub_count_{0};
};

GZ_REGISTER_MODEL_PLUGIN(QuadrotorDynamicsPlugin)

}  // namespace rm_drone
