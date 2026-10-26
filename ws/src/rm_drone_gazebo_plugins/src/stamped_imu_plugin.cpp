// Copyright 2026 RM27_DRONE contributors
// SPDX-License-Identifier: MIT
// IMU samples are generated at the physics step boundary. Linear/angular
// velocity, sensor attitude and timestamp must describe the same step: Classic's
// asynchronous LinkData transport can otherwise pair old velocity with new pose.
// Publish only noisy inertial measurements, never simulator position/orientation.
#include <array>
#include <cmath>
#include <random>
#include <stdexcept>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace rm_drone {
class StampedImuPlugin : public gazebo::ModelPlugin {
public:
  void Load(gazebo::physics::ModelPtr model,sdf::ElementPtr sdf) override {
    model_=model;link_=model->GetLink("base_link");node_=gazebo_ros::Node::Get(sdf);
    if (!link_) {RCLCPP_ERROR(node_->get_logger(),"IMU requires base_link");return;}
    auto get=[&](const char * key,double value) {return sdf->HasElement(key)?sdf->Get<double>(key):value;};
    rate_=get("sample_rate",500.);gyro_nd_=get("gyro_noise_density",.001);acc_nd_=get("acc_noise_density",.01);
    gyro_rw_=get("gyro_random_walk",.00002);acc_rw_=get("acc_random_walk",.0004);
    for (double v:{rate_,gyro_nd_,acc_nd_,gyro_rw_,acc_rw_}) {
      if (!std::isfinite(v) || v<0) {throw std::invalid_argument("invalid simulated IMU noise/rate");}
    }
    if (rate_<=0) {throw std::invalid_argument("IMU sample rate must be positive");}
    const unsigned seed=sdf->HasElement("seed")?sdf->Get<unsigned>("seed"):2701;
    random_.seed(seed);
    for (int i=0;i<3;++i) {
      gyro_bias_[i]=(random_()%2?1.:-1.)*(.002+.001*Normal());
      acc_bias_[i]=(random_()%2?1.:-1.)*(.05+.02*Normal());
    }
    publisher_=node_->create_publisher<sensor_msgs::msg::Imu>("imu",rclcpp::SensorDataQoS().keep_last(200));
    update_=gazebo::event::Events::ConnectWorldUpdateEnd([this]{Publish();});
  }
private:
  double Normal() {return normal_(random_);}
  void Publish() {
    const auto time=model_->GetWorld()->SimTime();const double t=time.Double();
    const auto velocity=link_->WorldLinearVel();
    if (last_time_<0 || t<last_time_ || t-last_time_>.05) {last_time_=t;last_velocity_=velocity;return;}
    const double dt=t-last_time_;
    if (dt<1./rate_-1e-9) {return;}
    const auto rotation=link_->WorldPose().Rot();
    const auto gyro=rotation.RotateVectorReverse(link_->WorldAngularVel());
    const auto specific_force=rotation.RotateVectorReverse((velocity-last_velocity_)/dt-model_->GetWorld()->Gravity());
    last_velocity_=velocity;last_time_=t;
    if (!gyro.IsFinite() || !specific_force.IsFinite()) {return;}
    sensor_msgs::msg::Imu sample;
    sample.header.stamp=rclcpp::Time(time.sec*1000000000LL+time.nsec);
    sample.header.frame_id="imu_link";sample.orientation_covariance[0]=-1;
    std::array<double,3> w,a;
    for (int i=0;i<3;++i) {
      gyro_bias_[i]+=gyro_rw_*std::sqrt(dt)*Normal();acc_bias_[i]+=acc_rw_*std::sqrt(dt)*Normal();
      w[i]=gyro[i]+gyro_bias_[i]+gyro_nd_/std::sqrt(dt)*Normal();
      a[i]=specific_force[i]+acc_bias_[i]+acc_nd_/std::sqrt(dt)*Normal();
      sample.angular_velocity_covariance[4*i]=gyro_nd_*gyro_nd_/dt;
      sample.linear_acceleration_covariance[4*i]=acc_nd_*acc_nd_/dt;
    }
    sample.angular_velocity.x=w[0];sample.angular_velocity.y=w[1];sample.angular_velocity.z=w[2];
    sample.linear_acceleration.x=a[0];sample.linear_acceleration.y=a[1];sample.linear_acceleration.z=a[2];
    publisher_->publish(sample);
  }
  gazebo::physics::ModelPtr model_;gazebo::physics::LinkPtr link_;
  gazebo_ros::Node::SharedPtr node_;gazebo::event::ConnectionPtr update_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  ignition::math::Vector3d last_velocity_;
  std::array<double,3> gyro_bias_,acc_bias_;
  std::mt19937 random_;std::normal_distribution<double> normal_{0.,1.};
  double rate_,gyro_nd_,acc_nd_,gyro_rw_,acc_rw_,last_time_{-1};
};
GZ_REGISTER_MODEL_PLUGIN(StampedImuPlugin)
} // namespace rm_drone
