// 模拟裁判系统：
//  - 接收装甲冲击事件，按（待官方公布的）接触时间/冲量阈值判定有效冲击，扣除基地/前哨站血量（表 5-2：200）
//  - 检测空中机器人是否飞出战场（R48）
//  - 发布场地障碍物与目标的 RViz Marker
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/impact_event.hpp>
#include <rm_drone_msgs/msg/referee_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_drone_control/field_map.hpp"

namespace rm_drone
{

class RefereeSimNode : public rclcpp::Node
{
public:
  RefereeSimNode()
  : Node("referee_sim")
  {
    min_duration_ = declare_parameter("min_contact_time", 0.0);
    max_duration_ = declare_parameter("max_contact_time", 0.2);
    min_impulse_ = declare_parameter("min_impulse", 0.15);
    max_impulse_ = declare_parameter("max_impulse", 100.0);
    min_speed_ = declare_parameter("min_approach_speed", 1.0);
    damage_ = declare_parameter("impact_damage", 200);
    drone_entity_ = declare_parameter<std::string>("drone_entity_name", "rm_drone");
    field_.load(declare_parameter<std::string>("field_map", ""));

    status_.red_base_hp = status_.blue_base_hp = 10000;
    status_.red_outpost_hp = status_.blue_outpost_hp = 1000;
    // 下面三项是仿真适配输入，不是官方裁判协议字段。默认准备阶段，不允许仅因估计就绪就起飞。
    status_.phase = declare_parameter<std::string>("phase", "setup");
    status_.flight_permitted = declare_parameter("flight_permitted", false);
    status_.hangar_released = declare_parameter("hangar_released", false);
    RCLCPP_WARN(get_logger(),
      "冲击有效性使用仿真假设：接触 %.0f–%.0f ms，冲量 %.2f–%.2f N·s，接近速度 ≥ %.2f m/s。不是该版规则的官方阈值。",
      min_duration_ * 1e3, max_duration_ * 1e3, min_impulse_, max_impulse_, min_speed_);

    impact_sub_ = create_subscription<rm_drone_msgs::msg::ImpactEvent>("/referee/impact", 10,
      std::bind(&RefereeSimNode::OnImpact, this, std::placeholders::_1));
    gt_sub_ = create_subscription<nav_msgs::msg::Odometry>("/drone/ground_truth/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        const auto & p = m->pose.pose.position;
        const bool out = std::abs(p.x) > field_.length / 2 || std::abs(p.y) > field_.width / 2 || p.z > field_.wall_height + 0.5;
        if (out && !status_.drone_out_of_field) {
          RCLCPP_ERROR(get_logger(), "空中机器人飞出战场 (%.2f %.2f %.2f)：按 R48 红牌", p.x, p.y, p.z);
          status_.last_event = "OUT_OF_FIELD";
        }
        status_.drone_out_of_field = status_.drone_out_of_field || out;
      });
    status_pub_ = create_publisher<rm_drone_msgs::msg::RefereeStatus>("/referee/status", 10);
    stop_red_pub_ = create_publisher<std_msgs::msg::Bool>("/referee/outpost_stop_red", 10);
    stop_blue_pub_ = create_publisher<std_msgs::msg::Bool>("/referee/outpost_stop_blue", 10);
    permit_sub_ = create_subscription<std_msgs::msg::Bool>("/referee/flight_permitted", 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {status_.flight_permitted = m->data;});
    hangar_sub_ = create_subscription<std_msgs::msg::Bool>("/referee/hangar_released", 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {status_.hangar_released = m->data;});
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/field/markers", rclcpp::QoS(1).transient_local());
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        status_.header.stamp = now();
        if (t0_ < 0) {t0_ = now().seconds();}
        status_.match_time = now().seconds() - t0_;
        status_pub_->publish(status_);
      });
    PublishField();
  }

private:
  void OnImpact(rm_drone_msgs::msg::ImpactEvent::SharedPtr e)
  {
    const bool valid = e->striker_name == drone_entity_ && e->duration >= min_duration_ && e->duration <= max_duration_ &&
      e->impulse >= min_impulse_ && e->impulse <= max_impulse_ && e->approach_speed >= min_speed_;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s <- %s：接触 %.1f ms，冲量 %.3f N·s，峰值 %.1f N，法向接近速度 %.2f m/s",
      e->armor_name.c_str(), e->striker_name.c_str(), e->duration * 1e3, e->impulse, e->peak_force, e->approach_speed);
    if (!valid) {
      ++status_.invalid_impacts;
      RCLCPP_WARN(get_logger(), "无效冲击 %s", buf);
      return;
    }
    ++status_.valid_impacts;
    int32_t * hp = nullptr;
    if (e->armor_name.rfind("red_base", 0) == 0) {hp = &status_.red_base_hp;}
    if (e->armor_name.rfind("blue_base", 0) == 0) {hp = &status_.blue_base_hp;}
    if (e->armor_name.rfind("red_outpost", 0) == 0) {hp = &status_.red_outpost_hp;}
    if (e->armor_name.rfind("blue_outpost", 0) == 0) {hp = &status_.blue_outpost_hp;}
    if (hp) {
      *hp = std::max(0, *hp - damage_);
      if (*hp == 0 && (hp == &status_.red_outpost_hp || hp == &status_.blue_outpost_hp)) {
        std_msgs::msg::Bool stop;
        stop.data = true;
        (hp == &status_.red_outpost_hp ? stop_red_pub_ : stop_blue_pub_)->publish(stop);
      }
    }
    status_.last_event = e->armor_name;
    RCLCPP_INFO(get_logger(), "\033[1;32m有效冲击 -%d\033[0m %s | 蓝基地 %d 红基地 %d 蓝前哨 %d 红前哨 %d", damage_, buf,
      status_.blue_base_hp, status_.red_base_hp, status_.blue_outpost_hp, status_.red_outpost_hp);
  }

  void PublishField()
  {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    auto base = [&](const std::string & ns, int type) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.ns = ns;
        m.id = id++;
        m.type = type;
        m.action = m.ADD;
        m.pose.orientation.w = 1.0;
        return m;
      };
    auto floor = base("field", visualization_msgs::msg::Marker::CUBE);
    floor.scale.x = field_.length;
    floor.scale.y = field_.width;
    floor.scale.z = 0.01;
    floor.pose.position.z = -0.005;
    floor.color.r = floor.color.g = floor.color.b = 0.25;
    floor.color.a = 1.0;
    arr.markers.push_back(floor);
    auto walls = base("field", visualization_msgs::msg::Marker::LINE_LIST);
    walls.scale.x = 0.05;
    walls.color.r = walls.color.g = walls.color.b = 0.9;
    walls.color.a = 1.0;
    const double L = field_.length / 2, W = field_.width / 2, H = field_.wall_height;
    const double cx[4] = {-L, L, L, -L}, cy[4] = {-W, -W, W, W};
    for (int i = 0; i < 4; ++i) {
      for (double z : {0.0, H}) {
        geometry_msgs::msg::Point a, b;
        a.x = cx[i];
        a.y = cy[i];
        a.z = z;
        b.x = cx[(i + 1) % 4];
        b.y = cy[(i + 1) % 4];
        b.z = z;
        walls.points.push_back(a);
        walls.points.push_back(b);
      }
    }
    arr.markers.push_back(walls);
    for (const auto & o : field_.obstacles) {
      auto m = base("obstacles", visualization_msgs::msg::Marker::CUBE);
      m.pose.position.x = o.center.x();
      m.pose.position.y = o.center.y();
      m.pose.position.z = o.center.z();
      m.scale.x = o.size.x();
      m.scale.y = o.size.y();
      m.scale.z = o.size.z();
      m.color.r = 0.65;
      m.color.g = 0.5;
      m.color.b = 0.3;
      m.color.a = 0.55;
      if (o.name.find("red") == 0) {m.color.r = 0.8;}
      if (o.name.find("blue") == 0) {m.color.b = 0.8;}
      arr.markers.push_back(m);
    }
    for (const auto & t : field_.targets) {
      auto m = base("targets", t.kind == "base" ? visualization_msgs::msg::Marker::ARROW : visualization_msgs::msg::Marker::CYLINDER);
      m.pose.position.x = t.position.x();
      m.pose.position.y = t.position.y();
      m.pose.position.z = t.position.z();
      if (t.kind == "base") {
        m.pose.orientation.z = std::sin(t.normal_yaw / 2);
        m.pose.orientation.w = std::cos(t.normal_yaw / 2);
        m.scale.x = 0.3;
        m.scale.y = m.scale.z = 0.04;
      } else {
        m.scale.x = m.scale.y = 2 * t.radius;
        m.scale.z = 0.12;
      }
      m.color.a = 1.0;
      (t.team == "red" ? m.color.r : m.color.b) = 1.0;
      arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
  }

  double min_duration_, max_duration_, min_impulse_, max_impulse_, min_speed_;
  int damage_;
  double t0_{-1};
  std::string drone_entity_;
  FieldMap field_;
  rm_drone_msgs::msg::RefereeStatus status_;
  rclcpp::Subscription<rm_drone_msgs::msg::ImpactEvent>::SharedPtr impact_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permit_sub_, hangar_sub_;
  rclcpp::Publisher<rm_drone_msgs::msg::RefereeStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_red_pub_, stop_blue_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rm_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_drone::RefereeSimNode>());
  rclcpp::shutdown();
  return 0;
}
