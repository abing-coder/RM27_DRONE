// Simulated hangar clamp. It constrains the physical robot while docked; it
// does not generate or replace any navigation/IMU measurements.
#include <atomic>
#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/bool.hpp>

namespace rm_drone
{
class HangarDockPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    link_ = model_->GetLink("base_link");
    if (!link_) {gzerr << "hangar dock requires base_link\n"; return;}
    node_ = gazebo_ros::Node::Get(sdf);
    anchor_ = model_->WorldPose();
    const double floor = sdf->Get<double>("floor_height");
    anchor_.Pos().Z(floor + sdf->Get<double>("body_half_height"));
    arm_sub_ = node_->create_subscription<std_msgs::msg::Bool>("arm", 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {armed_.store(m->data);});
    dock_pub_ = node_->create_publisher<std_msgs::msg::Bool>("docked", 10);
    ground_pub_ = node_->create_publisher<std_msgs::msg::Bool>("grounded", 10);
    contacts_ = model_->GetWorld()->Physics()->GetContactManager();
    contacts_->SetNeverDropContacts(true);
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &) {Update();});
    if (!link_->GetKinematic()) {link_->SetKinematic(true);}
    model_->SetWorldPose(anchor_);
  }
private:
  void Update()
  {
    const double t = model_->GetWorld()->SimTime().Double();
    if (t < last_time_) {
      armed_.store(false); held_ = true; last_touch_ = -1;
      if (!link_->GetKinematic()) {link_->SetKinematic(true);}
      last_pub_ = -1;
    }
    last_time_ = t;
    if (held_ && armed_.load()) {
      held_ = false;
      last_touch_ = -1;
      link_->SetKinematic(false);
    } else if (!held_ && !armed_.load()) {
      const auto pose = model_->WorldPose();
      const auto delta = pose.Pos() - anchor_.Pos();
      const auto up = pose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);
      if (std::hypot(delta.X(), delta.Y()) < .06 && std::abs(delta.Z()) < .015 &&
        up.Z() > .98 && model_->WorldLinearVel().Length() < .15) {
        held_ = true;
        if (!link_->GetKinematic()) {link_->SetKinematic(true);}
      }
    }
    if (held_) {
      model_->SetWorldPose(anchor_);
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      model_->SetAngularVel(ignition::math::Vector3d::Zero);
    }
    // Physical support under the robot, rather than a proximity-to-map test.
    // Side-wall/armor collisions do not constitute a landing.
    if (!held_) {
      const auto & contacts = contacts_->GetContacts();
      for (unsigned int i = 0; i < contacts_->GetContactCount(); ++i) {
        const auto * contact = contacts[i];
        if (!contact || !contact->collision1 || !contact->collision2) {continue;}
        if (contact->collision1->GetLink() != link_ && contact->collision2->GetLink() != link_) {continue;}
        for (int j = 0; j < contact->count; ++j) {
          if (std::abs(contact->normals[j].Z()) > .8 &&
            contact->positions[j].Z() < model_->WorldPose().Pos().Z() - .01) {last_touch_ = t;}
        }
      }
    }
    if (t - last_pub_ > .1) {
      std_msgs::msg::Bool message;
      message.data = held_;
      dock_pub_->publish(message);
      message.data = held_ || (last_touch_ >= 0 && t - last_touch_ < .12);
      ground_pub_->publish(message);
      last_pub_ = t;
    }
  }
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::ContactManager * contacts_{nullptr};
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  ignition::math::Pose3d anchor_;
  std::atomic<bool> armed_{false};
  bool held_{true};
  double last_time_{-1}, last_pub_{-1}, last_touch_{-1};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dock_pub_, ground_pub_;
};
GZ_REGISTER_MODEL_PLUGIN(HangarDockPlugin)
}
