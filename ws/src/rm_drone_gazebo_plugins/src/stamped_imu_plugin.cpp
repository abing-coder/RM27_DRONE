// Copyright 2026 RM27_DRONE contributors
// SPDX-License-Identifier: MIT
// Read one atomic Gazebo IMU measurement, including its physical sample time.
// LastUpdateTime() is a scheduling time and remains zero with some Classic
// lockstep builds. Never substitute ROS receipt time for sensor sample time.
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace rm_drone {
class StampedImuPlugin : public gazebo::SensorPlugin {
public:
  void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf) override {
    sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(parent);
    node_ = gazebo_ros::Node::Get(sdf);
    if (!sensor_) {RCLCPP_ERROR(node_->get_logger(), "IMU plugin requires an ImuSensor"); return;}
    sensor_->SetWorldToReferenceOrientation(ignition::math::Quaterniond::Identity);
    frame_ = gazebo_ros::SensorFrameID(*parent, *sdf);
    publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("~/out", rclcpp::SensorDataQoS().keep_last(200));
    update_ = sensor_->ConnectUpdated([this] {Publish();});
  }
private:
  void Publish() {
    const auto measurement = sensor_->ImuMessage();
    const auto &stamp = measurement.stamp();
    const int64_t ns = static_cast<int64_t>(stamp.sec())*1000000000LL+stamp.nsec();
    if (ns <= 0 || ns == last_) {return;}
    last_ = ns;
    sensor_msgs::msg::Imu sample;
    sample.header.stamp = rclcpp::Time(ns);
    sample.header.frame_id = frame_;
    const auto &q = measurement.orientation();
    sample.orientation.x=q.x(); sample.orientation.y=q.y(); sample.orientation.z=q.z(); sample.orientation.w=q.w();
    const auto &w = measurement.angular_velocity();
    sample.angular_velocity.x=w.x(); sample.angular_velocity.y=w.y(); sample.angular_velocity.z=w.z();
    const auto &a = measurement.linear_acceleration();
    sample.linear_acceleration.x=a.x(); sample.linear_acceleration.y=a.y(); sample.linear_acceleration.z=a.z();
    // No orientation uncertainty model is supplied by Classic's IMU sensor.
    sample.orientation_covariance[0]=-1;
    using N = gazebo::sensors::SensorNoiseType;
    const N gyro[] = {N::IMU_ANGVEL_X_NOISE_RADIANS_PER_S,N::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S,N::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S};
    const N accel[] = {N::IMU_LINACC_X_NOISE_METERS_PER_S_SQR,N::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR,N::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR};
    for (int i=0;i<3;++i) {
      sample.angular_velocity_covariance[4*i]=gazebo_ros::NoiseVariance(sensor_->Noise(gyro[i]));
      sample.linear_acceleration_covariance[4*i]=gazebo_ros::NoiseVariance(sensor_->Noise(accel[i]));
    }
    publisher_->publish(sample);
  }
  gazebo::sensors::ImuSensorPtr sensor_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::string frame_;
  int64_t last_{-1};
};
GZ_REGISTER_SENSOR_PLUGIN(StampedImuPlugin)
} // namespace rm_drone
