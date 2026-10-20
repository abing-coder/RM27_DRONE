// 共轴能量机关/基地飞镖模块。ROS 输入只排队，物理更新线程统一处理，暂停时不推进规则时间。
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "rm_drone_gazebo_plugins/field_motion.hpp"

namespace rm_drone
{
class FieldMotionPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    kind_ = sdf->Get<std::string>("kind");
    motion_enabled_ = !sdf->HasElement("motion_enabled") || sdf->Get<bool>("motion_enabled");
    if (sdf->HasElement("initial_state")) {
      const auto state = sdf->Get<std::string>("initial_state");
      if (state == "inactive" || state == "activating" || state == "activated") {initial_state_ = state;}
    }
    joint_ = model->GetJoint(sdf->Get<std::string>("joint"));
    if (!joint_) {gzerr << "field_motion_plugin: joint missing\n"; return;}
    moving_link_ = model->GetLink(kind_ == "energy" ? "rotor" : "target");
    auto mount = model->GetLink(kind_ == "energy" ? "mount" : "rail");
    if (!moving_link_ || !mount) {gzerr << "field_motion_plugin: links missing\n"; return;}
    origin_ = moving_link_->WorldPose();
    // 场地电机/导轨视为刚性运动学约束。仅SetPosition会与ODE约束求解相互作用，使轴心漂移。
    // 保留碰撞几何，飞机仍会与道具接触；此插件不模拟电机力矩和机构弹性。
    // Gazebo 11 ODELink::SetKinematic(true) 在已为true时反而切回dynamic，必须先读状态。
    // https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODELink.cc
    if (!mount->GetKinematic()) {mount->SetKinematic(true);}
    if (!moving_link_->GetKinematic()) {moving_link_->SetKinematic(true);}
    node_ = gazebo_ros::Node::Get(sdf);
    const char * seed = std::getenv("RM_SIM_SEED");
    unsigned value = 2027;
    if (seed) {for (const unsigned char c : std::string(seed)) {value = value * 33u + c;}}
    energy_ = EnergyMotion(value);
    dart_ = DartMotion(value + (model->GetName() == "dart_target_blue" ? 1u : 0u));
    prefix_ = "/sim/field/" + (kind_ == "energy" ? std::string("energy") : model->GetName());
    status_ = node_->create_publisher<std_msgs::msg::String>(prefix_ + "/status", rclcpp::QoS(1).transient_local());
    subscribe("mode", "mode");
    if (kind_ == "energy") {
      transport_.reset(new gazebo::transport::Node());
      transport_->Init(model->GetWorld()->Name());
      visuals_ = transport_->Advertise<gazebo::msgs::Visual>("~/visual");
      for (const std::string team : {"red", "blue"}) {
        subscribe(team + "/state", team);
        mask_subs_.push_back(node_->create_subscription<std_msgs::msg::UInt8>(prefix_ + "/" + team + "/active_mask", 10,
          [this, team](std_msgs::msg::UInt8::SharedPtr m) {enqueue(team + "_mask", std::to_string(m->data & 31));}));
      }
    } else {
      for (const std::string event : {"open", "close", "fired", "hit"}) {
        events_.push_back(node_->create_subscription<std_msgs::msg::Empty>(prefix_ + "/" + event, 10,
          [this, event](std_msgs::msg::Empty::SharedPtr) {enqueue(event, "");}));
      }
    }
    connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&FieldMotionPlugin::update, this));
  }

  void Reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.clear();
    reset_ = true;
  }

private:
  void subscribe(const std::string & topic, const std::string & key)
  {
    strings_.push_back(node_->create_subscription<std_msgs::msg::String>(prefix_ + "/" + topic, 10,
      [this, key](std_msgs::msg::String::SharedPtr m) {enqueue(key, m->data);}));
  }
  void enqueue(const std::string & key, const std::string & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (commands_.size() < 64) {commands_.emplace_back(key, value);}
  }
  void color(const std::string & name, const std::string & team, double intensity)
  {
    gazebo::msgs::Visual msg;
    msg.set_name(model_->GetScopedName() + "::rotor::" + name);
    msg.set_parent_name(model_->GetScopedName() + "::rotor");
    auto * m = msg.mutable_material();
    gazebo::msgs::Set(m->mutable_diffuse(), ignition::math::Color(0, 0, 0, 1));
    gazebo::msgs::Set(m->mutable_ambient(), ignition::math::Color(0, 0, 0, 1));
    gazebo::msgs::Set(m->mutable_emissive(), team == "red" ? ignition::math::Color(intensity, 0.003, 0.003, 1) :
      ignition::math::Color(0.003, 0.003, intensity, 1));
    visuals_->Publish(msg);
  }
  void update()
  {
    const double t = model_->GetWorld()->SimTime().Double();
    std::deque<std::pair<std::string, std::string>> commands;
    bool reset;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      commands.swap(commands_);
      reset = reset_;
      reset_ = false;
    }
    if (reset || last_ < 0.0 || t < last_) {
      dart_.reset();
      energy_.reset();
      red_state_ = blue_state_ = initial_state_;
      mode_ = "small";
      masks_[0] = masks_[1] = 1;
      active_t_ = t;
      last_ = t;
      publish_t_ = -1.0;
      light_t_ = -1.0;
      lights_dirty_ = true;
    }
    const bool was_active = red_state_ == "activating" || blue_state_ == "activating";
    if (!commands.empty()) {lights_dirty_ = true;}
    for (const auto & [key, value] : commands) {
      if (kind_ == "energy") {
        if (key == "mode" && (value == "small" || value == "large")) {
          mode_ = value;
          active_t_ = t;
          if (value == "large") {energy_.restartProfile();}
          masks_[0] = masks_[1] = value == "large" ? 3 : 1;
        } else if ((key == "red" || key == "blue") &&
          (value == "inactive" || value == "activating" || value == "activated")) {
          (key == "red" ? red_state_ : blue_state_) = value;
        } else if (key == "red_mask" || key == "blue_mask") {
          masks_[key == "red_mask" ? 0 : 1] = static_cast<unsigned>(std::stoul(value));
        } else {RCLCPP_WARN(node_->get_logger(), "忽略能量机关指令 %s=%s", key.c_str(), value.c_str());}
      } else {
        if (key == "mode") {
          if (!dart_.setMode(value)) {RCLCPP_WARN(node_->get_logger(), "未知飞镖靶模式 %s", value.c_str());}
          else {dart_.close(t);}
        } else if (key == "open") {dart_.open(t);}
        else if (key == "close") {dart_.close(t);}
        else if (key == "fired") {dart_.fire(t);}
        else if (key == "hit") {dart_.hit(t);}
      }
    }
    const bool active = red_state_ == "activating" || blue_state_ == "activating";
    if (!was_active && active) {active_t_ = t; energy_.restartProfile();}
    if (kind_ == "energy") {
      if (motion_enabled_) {
        energy_.step(std::max(0.0, t - last_), std::max(0.0, last_ - active_t_), mode_ == "large" && active);
      }
      moving_link_->SetWorldPose(origin_ * ignition::math::Pose3d(0, 0, 0, energy_.phase, 0, 0));
    } else {
      dart_.update(t);
      auto p = origin_;
      p.Pos().Y() += dart_.y;  // 两方都按场地宽边移动，不受面板朝向180°翻转影响。
      moving_link_->SetWorldPose(p);
    }
    moving_link_->SetLinearVel(ignition::math::Vector3d::Zero);
    moving_link_->SetAngularVel(ignition::math::Vector3d::Zero);
    last_ = t;
    if (t - publish_t_ < 0.05) {return;}
    publish_t_ = t;
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << "{\"sim_time\":" << t;
    if (kind_ == "energy") {
      out << ",\"mode\":\"" << mode_ << "\",\"motion_enabled\":" << (motion_enabled_ ? "true" : "false")
          << ",\"phase\":" << energy_.phase << ",\"direction\":" << energy_.direction
          << ",\"a\":" << energy_.a << ",\"omega\":" << energy_.omega
          << ",\"red_state\":\"" << red_state_ << "\",\"blue_state\":\"" << blue_state_ << "\""
          << ",\"red_active_mask\":" << masks_[0] << ",\"blue_active_mask\":" << masks_[1];
      // 状态变化时更新，一秒重发一次兼容稍后加入的GUI，避免每个物理步重复材质更新。
      if (lights_dirty_ || t - light_t_ >= 1.0) {
        lights_dirty_ = false;
        light_t_ = t;
        for (int side = 0; side < 2; ++side) {
          const std::string team = side == 0 ? "red" : "blue";
          const auto & state = side == 0 ? red_state_ : blue_state_;
          for (int i = 0; i < 5; ++i) {
            const bool lit = state == "activated" || (state == "activating" && (masks_[side] & (1u << i)));
            const double intensity = lit ? 0.85 : 0.07;
            color(team + "_arm_" + std::to_string(i), team, intensity);
            color(team + "_rim_" + std::to_string(i), team, intensity);
          }
        }
      }
    } else {
      const auto p = model_->GetLink("target")->WorldPose();
      out << ",\"mode\":\"" << dart_.mode << "\",\"offset\":" << dart_.y
          << ",\"moving\":" << (dart_.moving ? "true" : "false")
          << ",\"can_hit\":" << (dart_.canHit(t) ? "true" : "false")
          << ",\"position\":[" << p.Pos().X() << ',' << p.Pos().Y() << ',' << p.Pos().Z() << ']';
    }
    out << '}';
    std_msgs::msg::String msg;
    msg.data = out.str();
    status_->publish(msg);
  }
  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::LinkPtr moving_link_;
  ignition::math::Pose3d origin_;
  gazebo::event::ConnectionPtr connection_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::transport::NodePtr transport_;
  gazebo::transport::PublisherPtr visuals_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> strings_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr> events_;
  std::vector<rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr> mask_subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::mutex mutex_;
  std::deque<std::pair<std::string, std::string>> commands_;
  DartMotion dart_;
  EnergyMotion energy_;
  std::string kind_, prefix_, mode_{"small"}, red_state_{"inactive"}, blue_state_{"inactive"};
  unsigned masks_[2]{1, 1};
  double last_{-1.0}, publish_t_{-1.0}, active_t_{0.0}, light_t_{-1.0};
  std::string initial_state_{"inactive"};
  bool reset_{false}, lights_dirty_{true}, motion_enabled_{true};
};
GZ_REGISTER_MODEL_PLUGIN(FieldMotionPlugin)
}  // namespace rm_drone
