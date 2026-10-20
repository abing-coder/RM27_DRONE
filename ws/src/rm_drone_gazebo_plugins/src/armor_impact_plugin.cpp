// 装甲冲击检测：接触力超过仿真阈值 alpha 时记一次事件。
// 该版规则的官方接触时间/冲量阈值仍待公布，alpha 和裁判侧阈值都是仿真假设，不是官方标准。
// 接触传感器以高于 50 Hz 采样，减少短接触被漏掉。
#include <algorithm>
#include <cmath>
#include <mutex>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/impact_event.hpp>

namespace rm_drone
{

class ArmorImpactPlugin : public gazebo::SensorPlugin
{
public:
  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
  {
    sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(sensor);
    if (!sensor_) {
      gzerr << "armor_impact_plugin requires a contact sensor\n";
      return;
    }
    node_ = gazebo_ros::Node::Get(sdf);
    armor_name_ = sdf->HasElement("armor_name") ? sdf->Get<std::string>("armor_name") : sensor->Name();
    alpha_ = sdf->HasElement("force_threshold") ? sdf->Get<double>("force_threshold") : 2.0;
    world_ = gazebo::physics::get_world(sensor_->WorldName());
    link_ = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(sensor_->ParentName()));

    pub_ = node_->create_publisher<rm_drone_msgs::msg::ImpactEvent>("/referee/impact", 10);
    conn_ = sensor_->ConnectUpdated(std::bind(&ArmorImpactPlugin::OnUpdate, this));
    sensor_->SetActive(true);
  }

private:
  void OnUpdate()
  {
    const auto contacts = sensor_->Contacts();
    const double t = world_->SimTime().Double();
    double force = 0.0;
    std::string striker;
    for (int i = 0; i < contacts.contact_size(); ++i) {
      const auto & c = contacts.contact(i);
      const bool armor_is_1 = c.collision1().find(armor_name_) != std::string::npos;
      const std::string other = armor_is_1 ? c.collision2() : c.collision1();
      for (int j = 0; j < c.wrench_size(); ++j) {
        const auto & w = armor_is_1 ? c.wrench(j).body_1_wrench() : c.wrench(j).body_2_wrench();
        force += std::sqrt(w.force().x() * w.force().x() + w.force().y() * w.force().y() +
                           w.force().z() * w.force().z());
      }
      if (striker.empty()) {striker = other;}
    }

    const double dt = last_t_ > 0 ? t - last_t_ : 0.0;
    last_t_ = t;

    if (!active_ && force > alpha_) {
      active_ = true;
      t0_ = t;
      impulse_ = 0.0;
      peak_ = 0.0;
      striker_ = striker;
      approach_speed_ = ApproachSpeed(striker);
    }
    if (active_) {
      impulse_ += force * dt;
      peak_ = std::max(peak_, force);
      if (force <= alpha_) {
        rm_drone_msgs::msg::ImpactEvent ev;
        ev.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
        ev.armor_name = armor_name_;
        ev.striker_name = striker_.substr(0, striker_.find("::"));
        ev.duration = t - t0_;
        ev.impulse = impulse_;
        ev.peak_force = peak_;
        ev.approach_speed = approach_speed_;
        pub_->publish(ev);
        active_ = false;
      }
    }
  }

  // 撞击者沿装甲法向（装甲 link 局部 +x，朝外）指向装甲的速度分量
  double ApproachSpeed(const std::string & collision_name)
  {
    auto col = boost::dynamic_pointer_cast<gazebo::physics::Collision>(world_->EntityByName(collision_name));
    if (!col || !link_) {return 0.0;}
    const auto n = link_->WorldPose().Rot().RotateVector({1.0, 0.0, 0.0});
    const auto v_rel = col->GetLink()->WorldLinearVel() - link_->WorldLinearVel();
    return -v_rel.Dot(n);
  }

  gazebo::sensors::ContactSensorPtr sensor_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr link_;
  gazebo::event::ConnectionPtr conn_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<rm_drone_msgs::msg::ImpactEvent>::SharedPtr pub_;
  std::string armor_name_, striker_;
  double alpha_{2.0}, t0_{0}, impulse_{0}, peak_{0}, approach_speed_{0}, last_t_{-1};
  bool active_{false};
};

GZ_REGISTER_SENSOR_PLUGIN(ArmorImpactPlugin)

}  // namespace rm_drone
