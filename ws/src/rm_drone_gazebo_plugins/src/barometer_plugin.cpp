// A simulated pressure sensor, not an odometry source: standard-atmosphere
// pressure with 0.8 Pa white noise and a slowly varying sensor bias.
#include <cmath>
#include <random>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
namespace rm_drone {
class BarometerPlugin : public gazebo::ModelPlugin {
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model; node_ = gazebo_ros::Node::Get(sdf);
    pub_ = node_->create_publisher<sensor_msgs::msg::FluidPressure>("barometer", rclcpp::SensorDataQoS());
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo &) {
      const double t = model_->GetWorld()->SimTime().Double();
      if (t < last_) {last_ = -1; bias_ = 0; random_.seed(2701);}
      if (last_ >= 0 && t-last_ < .05) {return;}
      const double dt = last_ >= 0 ? t-last_ : .05; last_ = t;
      bias_ += normal_(random_) * .01 * std::sqrt(dt);
      const double height = model_->WorldPose().Pos().Z();
      sensor_msgs::msg::FluidPressure sample;
      sample.header.stamp = rclcpp::Time(static_cast<int64_t>(t*1e9));
      sample.header.frame_id = "base_link";
      sample.fluid_pressure = 101325 * std::pow(1-2.25577e-5*height, 5.25588) + bias_ + .8*normal_(random_);
      sample.variance = .64; pub_->publish(sample);
    });
  }
private:
  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_;
  std::mt19937 random_{2701};
  std::normal_distribution<double> normal_{0,1};
  double last_{-1}, bias_{0};
};
GZ_REGISTER_MODEL_PLUGIN(BarometerPlugin)
}
