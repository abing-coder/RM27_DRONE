// 误差状态卡尔曼滤波（ESKF）：
//  传播：IMU（仿真500 Hz）
//  更新：OpenVINS 位姿（对齐到场地坐标系 map）、下视 ToF 测高（减去场地结构高度）、停机零速约束（ZUPT）
//  初始化：仅在 IMU 静止且连续时，用机库已知位姿 + 静止均值。解锁后不再把位置拉回机库。
// mode=gt 显式转发仿真真值，只用于控制和规划调试。视觉失效不会改走真值。
// 同一路 IMU 同时送进 OpenVINS 和本滤波器，更新与传播噪声相关，协方差不是一致的，
// 不能把它当成全场绝对定位已经完成。
#include <deque>
#include <stdexcept>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/estimator_status.hpp>
#include <rm_drone_msgs/msg/armor_detection_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rm_drone_control/field_map.hpp"
#include "rm_drone_control/math_utils.hpp"
#include "rm_drone_estimator/health.hpp"

namespace rm_drone
{

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat15 = Eigen::Matrix<double, 15, 15>;

namespace
{
Mat3 skew(const Vec3 & v)
{
  Mat3 m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}
Eigen::Quaterniond expQ(const Vec3 & w)
{
  const double a = w.norm();
  if (a < 1e-9) {return Eigen::Quaterniond(1, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z()).normalized();}
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, w / a));
}
Vec3 logQ(Eigen::Quaterniond q)
{
  if (q.w() < 0) {q.coeffs() *= -1;}
  const double n = q.vec().norm();
  if (n < 1e-9) {return 2.0 * q.vec();}
  return 2.0 * std::atan2(n, q.w()) * q.vec() / n;
}
}  // namespace

class EskfNode : public rclcpp::Node
{
public:
  EskfNode()
  : Node("estimator")
  {
    mode_ = declare_parameter<std::string>("mode", "eskf");
    const auto team = declare_parameter<std::string>("team", "red");
    const auto map_path = declare_parameter<std::string>("field_map", "");
    init_z_offset_ = declare_parameter("init_z_offset", 0.023);
    acc_nd_ = declare_parameter("acc_noise_density", 0.015);
    gyro_nd_ = declare_parameter("gyro_noise_density", 0.0015);
    acc_rw_ = declare_parameter("acc_random_walk", 6e-4);
    gyro_rw_ = declare_parameter("gyro_random_walk", 3e-5);
    use_vio_ = declare_parameter("use_vio", true);
    use_tof_ = declare_parameter("use_tof", true);
    use_baro_ = declare_parameter("use_barometer", true);
    use_sim_truth_ = declare_parameter("use_sim_truth", false);
    runtime_ = declare_parameter<std::string>("runtime", "sim");
    tof_timeout_ = declare_parameter("tof_timeout", 0.25);
    tof_sigma_ = declare_parameter("tof_sigma", 0.02);
    tof_gate_ = declare_parameter("tof_gate", 0.15);
    tof_offset_ = declare_parameter("tof_offset_z", -0.02);
    vio_rate_ = declare_parameter("vio_update_rate", 50.0);
    vio_pos_sigma_min_ = declare_parameter("vio_pos_sigma_min", 0.02);
    vio_att_sigma_min_ = declare_parameter("vio_att_sigma_min", 0.01);
    vio_chi2_ = declare_parameter("vio_chi2_gate", 30.0);
    vio_timeout_ = declare_parameter("vio_timeout", 0.3);
    publish_tf_ = declare_parameter("publish_tf", true);
    use_armor_landmarks_ = declare_parameter("use_armor_landmarks", true);

    if (!map_path.empty()) {
      field_.load(map_path);
      have_map_ = true;
    }
    const int ti = team == "blue" ? 1 : 0;
    init_p_ = have_map_ ? field_.hangar_pos[ti] : Vec3::Zero();
    init_p_.z() += init_z_offset_;
    init_yaw_ = have_map_ ? field_.hangar_yaw[ti] : 0.0;

    auto qos = rclcpp::SensorDataQoS();
    // 可靠发布，RViz 的默认订阅才能连上；best-effort 订阅方仍然兼容
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS().keep_last(200),
      std::bind(&EskfNode::OnImu, this, std::placeholders::_1));
    baro_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>("barometer", qos,
      [this](sensor_msgs::msg::FluidPressure::SharedPtr m) {OnBarometer(*m);});
    landing_position_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("landing/position", qos,
      [this](geometry_msgs::msg::PointStamped::SharedPtr m) {
        const double age = (now()-rclcpp::Time(m->header.stamp)).seconds();
        Vec3 observed(m->point.x,m->point.y,m->point.z);
        if (mode_ == "gt" || !initialized_ || !vio_aligned_ || m->header.frame_id != "map" ||
          age < -.05 || age > .2 || !observed.allFinite() || (observed-p_).norm() > 1.0 ||
          (observed.head<2>()-init_p_.head<2>()).norm() > 1.2) {return;}
        Vec3 correction = .2*(observed-p_);
        // Height is observed independently from the same validated RGB-D pad,
        // including its tracked glyph after the full rim leaves the image.
        correction.z()=0;
        if (correction.norm() > .02) {correction *= .02/correction.norm();}
        ApplyMapTranslation(correction);
      });
    landing_height_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("landing/error", qos,
      [this](geometry_msgs::msg::PointStamped::SharedPtr m) {
        const double age=(now()-rclcpp::Time(m->header.stamp)).seconds();
        const double height=m->point.z;
        if (mode_=="gt" || !initialized_ || !vio_aligned_ || m->header.frame_id!="map" ||
          age<-.05 || age>.2 || !std::isfinite(height) || height<.02 || height>.9 ||
          (p_.head<2>()-init_p_.head<2>()).norm()>.9) {return;}
        const double observed_z=init_p_.z()-init_z_offset_+height;
        if (std::abs(observed_z-p_.z())>.35) {return;}
        ApplyMapTranslation(Vec3(0,0,std::clamp(.2*(observed_z-p_.z()),-.02,.02)));
      });
    tof_sub_ = create_subscription<sensor_msgs::msg::Range>("tof", qos, std::bind(&EskfNode::OnTof, this, std::placeholders::_1));
    visual_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("vio_visual_pose", qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        const rclcpp::Time stamp(m->header.stamp);
        const double age=(now()-stamp).seconds();
        if (age>=-.05 && age<vio_timeout_ && stamp>last_visual_pose_) {last_visual_pose_=stamp;}
      });
    vio_sub_ = create_subscription<nav_msgs::msg::Odometry>("vio_odom", qos, std::bind(&EskfNode::OnVio, this, std::placeholders::_1));
    landmark_sub_ = create_subscription<rm_drone_msgs::msg::ArmorDetectionArray>("armors", 1,
      [this](rm_drone_msgs::msg::ArmorDetectionArray::SharedPtr m) {OnLandmarks(*m);});
    map_reset_pub_ = create_publisher<std_msgs::msg::Empty>("ego/map_reset", 1);
    gt_sub_ = create_subscription<nav_msgs::msg::Odometry>("ground_truth/odom", qos, std::bind(&EskfNode::OnGt, this, std::placeholders::_1));
    arm_sub_ = create_subscription<std_msgs::msg::Bool>("arm", 10, [this](std_msgs::msg::Bool::SharedPtr m) {armed_ = m->data;});

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", odom_qos);
    status_pub_ = create_publisher<rm_drone_msgs::msg::EstimatorStatus>("estimator_status", 10);
    est_path_pub_ = create_publisher<nav_msgs::msg::Path>("est_path", 1);
    gt_path_pub_ = create_publisher<nav_msgs::msg::Path>("gt_path", 1);
    vio_path_pub_ = create_publisher<nav_msgs::msg::Path>("vio_path_map", 1);
    err_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("est_error", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&EskfNode::OnTimer, this));

    if (const char * why = rejectTruthConfig(runtime_, use_sim_truth_, mode_)) {
      RCLCPP_FATAL(get_logger(), "%s", why);
      throw std::runtime_error(why);
    }
    RCLCPP_INFO(get_logger(), "estimator mode=%s runtime=%s team=%s init=(%.2f %.2f %.2f) yaw=%.2f use_vio=%s",
      mode_.c_str(), runtime_.c_str(), team.c_str(), init_p_.x(), init_p_.y(), init_p_.z(), init_yaw_,
      use_vio_ ? "on" : "off");
  }

private:
  // ------------------------------------------------------------------ IMU
  void OnImu(sensor_msgs::msg::Imu::SharedPtr m)
  {
    if (mode_ == "gt") {return;}
    const Vec3 a(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
    const Vec3 w(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
    const rclcpp::Time t = m->header.stamp;

    if (!initialized_) {
      if (!init_buf_.empty()) {
        const double gap = (t - init_buf_.back().stamp).seconds();
        if (!std::isfinite(gap) || gap < 0.0 || gap > 0.02) {init_buf_.clear();}
      }
      init_buf_.push_back({a, w, t});
      if (init_buf_.size() >= 200) {Initialize(t);}
      return;
    }
    const double raw_dt = (t - t_).seconds();
    if (!std::isfinite(raw_dt) || raw_dt < 0.0 || raw_dt > 0.05) {
      imu_history_.clear(); pending_vio_.reset();
      imu_gap_ = true;
      t_ = t;
      P_.diagonal().array() += 0.05;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IMU 时间间隔异常 dt=%.4f，本拍不积分", raw_dt);
      return;
    }
    imu_gap_ = false;
    t_ = t;
    imu_history_.push_back({a,w,t});
    while (!imu_history_.empty() && ((t-imu_history_.front().stamp).seconds()>.5 || imu_history_.size()>512)) {
      imu_history_.pop_front();
    }
    Propagate(a, w, raw_dt);
    last_w_ = w - bg_;

    if (armed_) {left_pad_ = true;}
    // 未解锁只做静止零速，不再每拍把位置拉回机库。初始化时的机库位姿只是那一次的先验。
    if (!armed_ && w.norm() < 0.2) {
      const Mat3 R = q_.toRotationMatrix();
      const Vec3 expected = R.transpose() * Vec3(0, 0, 9.81);
      if ((a - expected).norm() < 1.0) {ba_ = 0.995 * ba_ + 0.005 * (a - expected);}
      ZeroVelocityUpdate();
    }
    if (pending_vio_ && rclcpp::Time(pending_vio_->header.stamp)<=t_) {
      auto pending=std::move(pending_vio_);OnVio(pending);
    }
    if (mode_ != "gt" && ++pub_count_ % 2 == 0) {PublishOdom(t);}
  }

  void Initialize(const rclcpp::Time & t)
  {
    Vec3 am = Vec3::Zero(), wm = Vec3::Zero();
    for (const auto & s : init_buf_) {
      am += s.a;
      wm += s.w;
    }
    am /= static_cast<double>(init_buf_.size());
    wm /= static_cast<double>(init_buf_.size());
    Vec3 acc_var = Vec3::Zero();
    for (const auto & s : init_buf_) {acc_var += (s.a - am).cwiseAbs2();}
    acc_var /= static_cast<double>(init_buf_.size());
    // 仿真 IMU 离散噪声约 0.2 m/s²，机库接触还会再抖一点。只拒绝明显在动或失重的窗口。
    const bool still = wm.norm() < 0.8 && acc_var.norm() < 4.0 && am.norm() > 8.5 && am.norm() < 11.0;
    if (!still) {
      RCLCPP_WARN(get_logger(), "IMU 初始化放弃：未静止 |w|=%.3f acc_var=%.4f |a|=%.2f", wm.norm(), acc_var.norm(), am.norm());
      init_buf_.clear();
      return;
    }
    const Eigen::Quaterniond q_rp = Eigen::Quaterniond::FromTwoVectors(am.normalized(), Vec3::UnitZ());
    // q_rp 把机体系下测得的“上”方向转到世界 z 轴（偏航任意），再替换为机库已知偏航
    q_ = (Eigen::Quaterniond(Eigen::AngleAxisd(wrapPi(init_yaw_ - yawOf(q_rp)), Vec3::UnitZ())) * q_rp).normalized();
    p_ = init_p_;
    v_.setZero();
    bg_ = wm;
    // 静止时比力扣除重力后的剩余量就是加速度零偏。不扣掉的话，0.05 m/s² 的仿真零偏
    // 在解锁前十几秒会把高度积出数米，飞控会追这个假高度。
    ba_ = am - q_.conjugate() * Vec3(0, 0, 9.81);
    P_.setZero();
    P_.block<3, 3>(0, 0) = Mat3::Identity() * 0.02 * 0.02;
    P_.block<3, 3>(3, 3) = Mat3::Identity() * 0.01 * 0.01;
    P_.block<3, 3>(6, 6) = Mat3::Identity() * std::pow(2.0 * M_PI / 180.0, 2);
    P_.block<3, 3>(9, 9) = Mat3::Identity() * 0.1 * 0.1;
    P_.block<3, 3>(12, 12) = Mat3::Identity() * 0.005 * 0.005;
    t_ = t;
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "ESKF 初始化完成：p=(%.2f %.2f %.2f) yaw=%.1f° bg=(%.4f %.4f %.4f) ba=(%.3f %.3f %.3f)",
      p_.x(), p_.y(), p_.z(), yawOf(q_) * 180 / M_PI, bg_.x(), bg_.y(), bg_.z(), ba_.x(), ba_.y(), ba_.z());
  }

  void Propagate(const Vec3 & a_m, const Vec3 & w_m, double dt)
  {
    if (dt <= 0.0) {return;}
    const Vec3 a = a_m - ba_, w = w_m - bg_;
    const Mat3 R = q_.toRotationMatrix();
    const Vec3 acc = R * a + Vec3(0, 0, -9.81);
    p_ += v_ * dt + 0.5 * acc * dt * dt;
    v_ += acc * dt;
    const Eigen::Quaterniond dq = expQ(w * dt);
    q_ = (q_ * dq).normalized();

    Mat15 F = Mat15::Identity();
    F.block<3, 3>(0, 3) = Mat3::Identity() * dt;
    F.block<3, 3>(3, 6) = -R * skew(a) * dt;
    F.block<3, 3>(3, 9) = -R * dt;
    F.block<3, 3>(6, 6) = dq.toRotationMatrix().transpose();
    F.block<3, 3>(6, 12) = -Mat3::Identity() * dt;
    Mat15 Q = Mat15::Zero();
    Q.block<3, 3>(3, 3) = Mat3::Identity() * acc_nd_ * acc_nd_ * dt;
    Q.block<3, 3>(6, 6) = Mat3::Identity() * gyro_nd_ * gyro_nd_ * dt;
    Q.block<3, 3>(9, 9) = Mat3::Identity() * acc_rw_ * acc_rw_ * dt;
    Q.block<3, 3>(12, 12) = Mat3::Identity() * gyro_rw_ * gyro_rw_ * dt;
    P_ = F * P_ * F.transpose() + Q;
    // A collision is not a nominal smooth IMU excitation. Reflect the velocity
    // uncertainty from an unresolved short impulse instead of trusting it as
    // highly precise motion until the next visual update.
    if (a_m.norm()>40 || w_m.norm()>8) {P_.block<3,3>(3,3) += Mat3::Identity()*.02;}
  }

  template<int M>
  bool Update(const Eigen::Matrix<double, M, 15> & H, const Eigen::Matrix<double, M, 1> & r,
    const Eigen::Matrix<double, M, M> & Rm, double chi2_gate)
  {
    const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + Rm;
    const Eigen::Matrix<double, M, M> S_inv = S.inverse();
    if (chi2_gate > 0.0 && (r.transpose() * S_inv * r)(0) > chi2_gate) {return false;}
    const Eigen::Matrix<double, 15, M> K = P_ * H.transpose() * S_inv;
    const Eigen::Matrix<double, 15, 1> dx = K * r;
    const Mat15 IKH = Mat15::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * Rm * K.transpose();
    p_ += dx.segment<3>(0);
    v_ += dx.segment<3>(3);
    q_ = (q_ * expQ(dx.segment<3>(6))).normalized();
    ba_ += dx.segment<3>(9);
    bg_ += dx.segment<3>(12);
    return true;
  }

  void ZeroVelocityUpdate()
  {
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Mat3::Identity();
    Update<3>(H, -v_, Mat3::Identity() * 0.01 * 0.01, 0.0);
  }

  void ApplyMapTranslation(const Vec3 & step)
  {
    p_ += step; t_mv_ += step; pending_map_shift_ += step;
    if (pending_map_shift_.norm() > .1) {
      map_reset_pub_->publish(std_msgs::msg::Empty()); pending_map_shift_.setZero();
    }
  }

  void OnBarometer(const sensor_msgs::msg::FluidPressure & sample)
  {
    if (mode_ == "gt" || !use_baro_ || !initialized_) {return;}
    const double pressure = sample.fluid_pressure;
    const double age = (now()-rclcpp::Time(sample.header.stamp)).seconds();
    if (!std::isfinite(pressure) || pressure < 90000 || pressure > 105000 || age < -.05 || age > .2) {return;}
    const double altitude = (1-std::pow(pressure/101325, 1/5.25588))/2.25577e-5;
    if (baro_count_ < 20) {
      if (armed_) {return;}
      baro_zero_ += altitude;
      if (++baro_count_ == 20) {baro_zero_ /= 20;}
      return;
    }
    const double measured = init_p_.z()+altitude-baro_zero_;
    const double residual = measured-p_.z();
    if (std::abs(residual) > .6) {return;}
    Eigen::Matrix<double,1,15> H = Eigen::Matrix<double,1,15>::Zero(); H(0,2)=1;
    Eigen::Matrix<double,1,1> innovation, noise; innovation << residual; noise << .08*.08;
    const Vec3 old=p_;
    if (Update<1>(H,innovation,noise,0.0) && vio_aligned_) {
      t_mv_ += p_-old; pending_map_shift_ += p_-old;
      if (pending_map_shift_.norm()>.1) {map_reset_pub_->publish(std_msgs::msg::Empty());pending_map_shift_.setZero();}
    }
  }

  // ------------------------------------------------------------------ ToF
  void OnTof(sensor_msgs::msg::Range::SharedPtr m)
  {
    if (mode_ == "gt" || !initialized_ || !use_tof_) {return;}
    // Clipped near-range readings are censored, not unbiased measurements.
    // In the hangar the sensor is only 2.5 mm above the deck, below its 20 mm
    // minimum. Fusing occasional noisy returns would lift the estimated robot.
    const double clip_margin = 0.024;  // three simulated ToF noise standard deviations
    const bool in_range = std::isfinite(m->range) &&
      m->range > m->min_range + clip_margin && m->range < m->max_range - clip_margin;
    if (!in_range) {
      ++tof_reject_;
      return;
    }
    // 射线沿机体 -Z（传感器 rpy 把 +X 转到下方），落点不一定在机体正下方。
    const Mat3 R = q_.toRotationMatrix();
    const Vec3 dir = R * Vec3(0, 0, -1);
    const Vec3 origin = p_ + R * Vec3(0, 0, tof_offset_);
    if (dir.z() > -0.3) {
      ++tof_reject_;
      return;
    }
    const double terrain0 = have_map_ ? field_.terrainHeight(origin.x(), origin.y(), origin.z()) : 0.0;
    const double range_pred0 = (terrain0 - origin.z()) / dir.z();
    const Vec3 hit = origin + dir * range_pred0;
    const double terrain = have_map_ ? field_.terrainHeight(hit.x(), hit.y(), origin.z()) : 0.0;
    const double range_pred = (terrain - origin.z()) / dir.z();
    const double innov = m->range - range_pred;
    if (range_pred <= m->min_range + clip_margin || range_pred >= m->max_range - clip_margin ||
      std::abs(innov) > tof_gate_) {
      ++tof_reject_;
      return;
    }
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0, 2) = -1.0 / dir.z();
    Eigen::Matrix<double, 1, 1> r, Rm;
    r << innov;
    Rm << tof_sigma_ * tof_sigma_;
    if (!Update<1>(H, r, Rm, 0.0)) {
      ++tof_reject_;
      return;
    }
    tof_reject_ = 0;
    last_tof_ok_ = now();
  }

  // ------------------------------------------------------------------ VIO
  void OnVio(nav_msgs::msg::Odometry::SharedPtr m)
  {
    if (mode_ == "gt" || !initialized_ || !use_vio_) {return;}
    const rclcpp::Time t = m->header.stamp;
    const double sample_age = (now() - t).seconds();
    if (t.nanoseconds() <= 0 || sample_age < -.05 || sample_age > vio_timeout_) {return;}
    if (t>t_) {
      // Independent callbacks can deliver fast VIO before its IMU sample.
      if (!pending_vio_ || t>rclcpp::Time(pending_vio_->header.stamp)) {pending_vio_=m;}
      return;
    }
    if (last_vio_used_.nanoseconds() > 0 && (t - last_vio_used_).seconds() < 1.0 / vio_rate_) {return;}
    last_vio_used_ = t;

    const Vec3 p_v(m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z);
    const Eigen::Quaterniond q_v = toEigen(m->pose.pose.orientation);
    if (!finiteVec(p_v) || !finiteQuat(q_v) ||
      !std::all_of(m->pose.covariance.begin(), m->pose.covariance.end(),
        [](double value) {return std::isfinite(value);})) {return;}

    if (!vio_aligned_) {
      // VIO 世界系与 map 都是重力对齐的，只差偏航和平移
      vio_yaw_ = wrapPi(yawOf(q_) - yawOf(q_v));
      R_mv_ = Eigen::AngleAxisd(vio_yaw_, Vec3::UnitZ()).toRotationMatrix();
      t_mv_ = p_ - R_mv_ * p_v;
      vio_aligned_ = true;
      vio_reject_ = 0;
      RCLCPP_INFO(get_logger(), "VIO 已对齐到场地坐标系：yaw_offset=%.2f° t=(%.2f %.2f %.2f)",
        vio_yaw_ * 180 / M_PI, t_mv_.x(), t_mv_.y(), t_mv_.z());
    }
    Vec3 p_m = R_mv_ * p_v + t_mv_;
    Eigen::Quaterniond q_m = Eigen::Quaterniond(R_mv_) * q_v;
    const Vec3 body_velocity=toEigen(m->twist.twist.linear);
    if (!body_velocity.allFinite() || body_velocity.norm()>5 ||
      !std::all_of(m->twist.covariance.begin(),m->twist.covariance.end(),[](double v){return std::isfinite(v);})) {return;}
    Vec3 velocity_m=q_m*body_velocity;
    const double delay=(t_-t).seconds();
    if (delay>vio_timeout_ || (delay>0 && (imu_history_.empty() || imu_history_.front().stamp>t))) {return;}
    // Compare measurements at the same physical time. A delayed pose during
    // a turn can otherwise fail the gate despite correct visual odometry.
    rclcpp::Time propagated=t;
    for (const auto & sample:imu_history_) {
      if (sample.stamp<=propagated) {continue;}
      const double dt=(sample.stamp-propagated).seconds();
      if (dt>.05) {return;}
      const Vec3 omega=sample.w-bg_;
      const Eigen::Quaterniond midpoint=q_m*expQ(.5*omega*dt);
      const Vec3 acceleration=midpoint*(sample.a-ba_)+Vec3(0,0,-9.81);
      p_m+=velocity_m*dt+.5*acceleration*dt*dt;
      velocity_m+=acceleration*dt;
      q_m=(q_m*expQ(omega*dt)).normalized();
      propagated=sample.stamp;
    }

    nav_msgs::msg::Odometry vm = *m;
    vm.header.frame_id = "map"; vm.header.stamp=t_;
    vm.pose.pose.position.x = p_m.x();
    vm.pose.pose.position.y = p_m.y();
    vm.pose.pose.position.z = p_m.z();
    vm.pose.pose.orientation = toMsg(q_m);
    AppendPath(vio_path_, vm.header, vm.pose.pose);

    double ps = vio_pos_sigma_min_, as = vio_att_sigma_min_;
    ps = std::max(ps, std::sqrt(std::max(0.0, m->pose.covariance[0] + m->pose.covariance[7] + m->pose.covariance[14]) / 3.0));
    as = std::max(as, std::sqrt(std::max(0.0, m->pose.covariance[21] + m->pose.covariance[28] + m->pose.covariance[35]) / 3.0));

    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = Mat3::Identity();
    H.block<3, 3>(3, 6) = Mat3::Identity();
    Eigen::Matrix<double, 6, 1> r;
    r.head<3>() = p_m - p_;
    r.tail<3>() = logQ(q_.conjugate() * q_m);
    Eigen::Matrix<double, 6, 6> Rm = Eigen::Matrix<double, 6, 6>::Zero();
    Rm.block<3, 3>(0, 0) = Mat3::Identity() * ps * ps;
    Rm.block<3, 3>(3, 3) = Mat3::Identity() * as * as;
    const double velocity_variance=std::max(.1*.1,
      (m->twist.covariance[0]+m->twist.covariance[7]+m->twist.covariance[14])/3);
    Rm.block<3,3>(0,0).diagonal().array()+=velocity_variance*delay*delay+acc_nd_*acc_nd_*delay*delay*delay/3;
    Rm.block<3,3>(3,3).diagonal().array()+=gyro_nd_*gyro_nd_*delay;
    if (Update<6>(H, r, Rm, vio_chi2_)) {
      Eigen::Matrix<double,3,15> Hv=Eigen::Matrix<double,3,15>::Zero(); Hv.block<3,3>(0,3)=Mat3::Identity();
      const double variance=std::max(.1*.1,(m->twist.covariance[0]+m->twist.covariance[7]+m->twist.covariance[14])/3);
      Update<3>(Hv,velocity_m-v_,Mat3::Identity()*variance,16.0);
      vio_reject_ = 0;
      last_vio_wall_ = t;  // health follows an accepted measurement's sampling time
    } else if (++vio_reject_ > 10) {
      // Reanchoring to an inconsistent VIO while flying hides estimator failure.
      // Only a stationary preflight estimator may establish a new alignment.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "VIO 连续未通过一致性检验，保持失效状态，不在飞行中重设坐标系");
      if (!armed_ && !left_pad_) {vio_aligned_ = false;}
    }
  }

  // Absolute translation anchors from observed fixed armor and the surveyed
  // field map. pose_map was formed with the image-time estimate, so its residual
  // is precisely the frame translation correction; no simulator pose is used.
  void OnLandmarks(const rm_drone_msgs::msg::ArmorDetectionArray & message)
  {
    if (mode_ == "gt" || !initialized_ || !use_vio_ || !use_armor_landmarks_ || !have_map_) {return;}
    const rclcpp::Time stamp(message.header.stamp);
    const double age = (now() - stamp).seconds();
    if (stamp <= last_landmark_ || age < -.05 || age > .2) {return;}
    last_landmark_ = stamp;
    Vec3 best = Vec3::Zero();
    double score = 1e9;
    bool found = false;
    for (const auto & armor : message.armors) {
      if (!armor.has_map_pose || armor.kind.empty() || armor.confidence < .8 ||
        armor.reprojection_error > 1.5 || armor.distance < .4 || armor.distance > 3.0) {continue;}
      const auto orientation = toEigen(armor.pose_map.orientation);
      const auto & ap = armor.pose_map.position;
      const Vec3 measured(ap.x, ap.y, ap.z);
      if (!finiteQuat(orientation) || !measured.allFinite()) {continue;}
      const Vec3 normal = orientation.normalized() * Vec3::UnitX();
      for (const auto & target : field_.targets) {
        if (target.team != armor.color || target.kind != armor.kind || std::abs(target.spin_speed) > 1e-3 ||
          armor.armor_type != (target.armor_type == "large" ? armor.LARGE : armor.SMALL)) {continue;}
        const double agreement = normal.dot(target.normal);
        const Vec3 residual = target.position - measured;
        if (agreement < .9 || residual.norm() > .8) {continue;}
        const double candidate = residual.norm() + 2 * (1-agreement);
        if (candidate < score) {score = candidate; best = residual; found = true;}
      }
    }
    if (!found) {landmark_count_ = 0; return;}
    if ((stamp - landmark_candidate_stamp_).seconds() > .3 || (best-landmark_candidate_).norm() > .06) {
      landmark_count_ = 0;
    }
    landmark_candidate_ = best;
    landmark_candidate_stamp_ = stamp;
    if (++landmark_count_ < 3) {return;}
    // Move the navigation-frame translation gradually, preserving velocity and
    // IMU biases. Apply exactly the same correction to the aligned VIO frame.
    Vec3 step = .2 * best;
    if (step.norm() > .02) {step *= .02/step.norm();}
    ApplyMapTranslation(step);
    if (best.norm() > .1) {
      for (int i = 0; i < 3; ++i) {P_(i,i) = std::max(P_(i,i), .05*.05);}
    }
    if (pending_map_shift_.norm() > .1) {
      map_reset_pub_->publish(std_msgs::msg::Empty()); pending_map_shift_.setZero();
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "固定装甲视觉地标校正：残差 %.3f m，本帧平移 %.3f m", best.norm(), step.norm());
  }

  // ------------------------------------------------------------------ 真值 / 输出
  void OnGt(nav_msgs::msg::Odometry::SharedPtr m)
  {
    gt_ = *m;
    have_gt_ = true;
    if (++gt_count_ % 20 == 0) {
      std_msgs::msg::Header h = m->header;
      h.frame_id = "map";
      AppendPath(gt_path_, h, m->pose.pose);
    }
    if (mode_ == "gt") {
      nav_msgs::msg::Odometry o = *m;
      o.header.frame_id = "map";
      o.child_frame_id = "base_link";
      // p3d 的 twist 为世界系，统一转换到机体系
      const Eigen::Quaterniond q = toEigen(m->pose.pose.orientation);
      o.twist.twist.linear = toMsg(Vec3(q.conjugate() * toEigen(m->twist.twist.linear)));
      o.twist.twist.angular = toMsg(Vec3(q.conjugate() * toEigen(m->twist.twist.angular)));
      odom_pub_->publish(o);
      PublishTf(o);
      if (gt_count_ % 20 == 0) {AppendPath(est_path_, o.header, o.pose.pose);}
    }
  }

  void PublishOdom(const rclcpp::Time & t)
  {
    nav_msgs::msg::Odometry o;
    o.header.stamp = t;
    o.header.frame_id = "map";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = p_.x();
    o.pose.pose.position.y = p_.y();
    o.pose.pose.position.z = p_.z();
    o.pose.pose.orientation = toMsg(q_);
    o.twist.twist.linear = toMsg(Vec3(q_.conjugate() * v_));
    o.twist.twist.angular = toMsg(last_w_);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        o.pose.covariance[i * 6 + j] = P_(i, j);
        o.pose.covariance[(i + 3) * 6 + j + 3] = P_(6 + i, 6 + j);
        o.twist.covariance[i * 6 + j] = P_(3 + i, 3 + j);
      }
    }
    odom_pub_->publish(o);
    PublishTf(o);
    if (pub_count_ % 20 == 0) {AppendPath(est_path_, o.header, o.pose.pose);}

    if (have_gt_ && pub_count_ % 20 == 0) {
      geometry_msgs::msg::Vector3Stamped e;
      e.header = o.header;
      e.vector.x = p_.x() - gt_.pose.pose.position.x;
      e.vector.y = p_.y() - gt_.pose.pose.position.y;
      e.vector.z = p_.z() - gt_.pose.pose.position.z;
      err_pub_->publish(e);
    }
  }

  void PublishTf(const nav_msgs::msg::Odometry & o)
  {
    if (!publish_tf_) {return;}
    geometry_msgs::msg::TransformStamped tf;
    tf.header = o.header;
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = o.pose.pose.position.x;
    tf.transform.translation.y = o.pose.pose.position.y;
    tf.transform.translation.z = o.pose.pose.position.z;
    tf.transform.rotation = o.pose.pose.orientation;
    tf_->sendTransform(tf);
  }

  void AppendPath(nav_msgs::msg::Path & path, const std_msgs::msg::Header & h, const geometry_msgs::msg::Pose & p)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = h;
    ps.header.frame_id = "map";
    ps.pose = p;
    path.header.frame_id = "map";
    path.header.stamp = h.stamp;
    path.poses.push_back(ps);
    if (path.poses.size() > 4000) {path.poses.erase(path.poses.begin());}
  }

  void OnTimer()
  {
    est_path_.header.frame_id = gt_path_.header.frame_id = vio_path_.header.frame_id = "map";
    est_path_pub_->publish(est_path_);
    gt_path_pub_->publish(gt_path_);
    vio_path_pub_->publish(vio_path_);
    rm_drone_msgs::msg::EstimatorStatus s;
    s.header.stamp = now();
    const double raw_age=last_vio_wall_.nanoseconds()>0 ? (now()-last_vio_wall_).seconds() : 1e3;
    const double visual_age=last_visual_pose_.nanoseconds()>0 ? (now()-last_visual_pose_).seconds() : 1e3;
    const double age=(raw_age<0 || visual_age<0) ? 1e3 : std::max(raw_age,visual_age);
    const EstimateHealth h = classifyEstimate(mode_ == "gt", use_vio_, initialized_, vio_aligned_, age, vio_timeout_, vio_reject_);
    s.source = imu_gap_ && mode_ != "gt" ? "imu_gap" : h.source;
    s.initialized = mode_ == "gt" ? have_gt_ : initialized_;
    s.vio_ok = h.vio_ok && !imu_gap_;
    const double tof_age = last_tof_ok_.nanoseconds() > 0 ? (now() - last_tof_ok_).seconds() : 1e3;
    s.tof_ok = mode_ != "gt" && tof_age < tof_timeout_ && tof_reject_ < 5;
    if (mode_ == "gt") {s.tof_ok = false;}
    s.vio_age = static_cast<float>(std::min(age, 1e3));
    s.position_std = static_cast<float>(std::sqrt(std::max(0.0, P_.block<3, 3>(0, 0).trace() / 3.0)));
    status_pub_->publish(s);
  }

  std::string mode_;
  FieldMap field_;
  bool have_map_{false};
  Vec3 init_p_;
  double init_yaw_, init_z_offset_;
  double acc_nd_, gyro_nd_, acc_rw_, gyro_rw_, tof_sigma_, tof_gate_, tof_offset_;
  double vio_rate_, vio_pos_sigma_min_, vio_att_sigma_min_, vio_chi2_, vio_timeout_, tof_timeout_;
  rclcpp::Time last_visual_pose_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr visual_sub_;
  bool use_vio_, use_tof_, use_sim_truth_, publish_tf_, use_armor_landmarks_, use_baro_;
  double baro_zero_{0}; int baro_count_{0};
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr landing_position_sub_, landing_height_sub_;
  rclcpp::Subscription<rm_drone_msgs::msg::ArmorDetectionArray>::SharedPtr landmark_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr map_reset_pub_;
  rclcpp::Time last_landmark_{0,0,RCL_ROS_TIME}, landmark_candidate_stamp_{0,0,RCL_ROS_TIME};
  int landmark_count_{0};
  Vec3 landmark_candidate_{Vec3::Zero()}, pending_map_shift_{Vec3::Zero()};
  std::string runtime_;

  struct ImuSample
  {
    Vec3 a, w;
    rclcpp::Time stamp;
  };
  bool initialized_{false}, armed_{false}, left_pad_{false}, vio_aligned_{false}, have_gt_{false}, imu_gap_{false};
  std::vector<ImuSample> init_buf_;
  std::deque<ImuSample> imu_history_;
  nav_msgs::msg::Odometry::SharedPtr pending_vio_;
  int tof_reject_{0};
  Vec3 p_{Vec3::Zero()}, v_{Vec3::Zero()}, ba_{Vec3::Zero()}, bg_{Vec3::Zero()}, last_w_{Vec3::Zero()};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Mat15 P_{Mat15::Identity()};
  rclcpp::Time t_{0, 0, RCL_ROS_TIME}, last_vio_used_{0, 0, RCL_ROS_TIME}, last_vio_wall_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tof_ok_{0, 0, RCL_ROS_TIME};
  double vio_yaw_{0.0};
  Mat3 R_mv_{Mat3::Identity()};
  Vec3 t_mv_{Vec3::Zero()};
  int vio_reject_{0};
  uint64_t pub_count_{0}, gt_count_{0};
  nav_msgs::msg::Odometry gt_;
  nav_msgs::msg::Path est_path_, gt_path_, vio_path_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr tof_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_, gt_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<rm_drone_msgs::msg::EstimatorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr est_path_pub_, gt_path_pub_, vio_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr err_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rm_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_drone::EskfNode>());
  rclcpp::shutdown();
  return 0;
}
