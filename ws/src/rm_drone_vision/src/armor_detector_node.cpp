// 装甲识别：YOLO11 关键点 / 传统灯条配对 -> 共用 PnP（IPPE）与 map 坐标转换
// 输出：/drone/armors（ArmorDetectionArray）、/drone/vision/debug_image（叠加识别结果的画面）、RViz Marker
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_drone_msgs/msg/armor_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_drone_control/math_utils.hpp"
#include "rm_drone_vision/yolo_detector.hpp"

namespace rm_drone
{

struct LightBar
{
  cv::Point2f top, bottom, center;
  float length, width, tilt_deg;
};

struct Armor
{
  LightBar left, right;
  bool large;
  std::vector<cv::Point2f> corners;  // 左上、左下、右下、右上
  std::string kind;
  float confidence{0.f};
  int class_id{-1};
};

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode()
  : Node("armor_detector")
  {
    enemy_color_ = declare_parameter<std::string>("enemy_color", "blue");
    color_thresh_ = declare_parameter("color_threshold", 70);
    bright_thresh_ = declare_parameter("brightness_threshold", 140);
    min_bar_ratio_ = declare_parameter("min_bar_ratio", 1.5);
    max_bar_ratio_ = declare_parameter("max_bar_ratio", 15.0);
    max_bar_tilt_ = declare_parameter("max_bar_tilt_deg", 40.0);
    min_bar_area_ = declare_parameter("min_bar_area", 3.0);
    small_large_split_ = declare_parameter("small_large_ratio_split", 3.4);
    bar_h_ = declare_parameter("light_bar_length", 0.056);
    small_w_ = declare_parameter("small_armor_bar_distance", 0.147);
    large_w_ = declare_parameter("large_armor_bar_distance", 0.242);
    const auto cam_xyz = declare_parameter<std::vector<double>>("cam_xyz", {0.045, 0.025, 0.008});
    const double pitch = declare_parameter("cam_pitch", -0.17);
    imshow_ = declare_parameter("imshow", true);
    max_reproj_ = declare_parameter("max_reprojection_error", 4.0);
    min_dist_ = declare_parameter("min_distance", 0.2);
    max_dist_ = declare_parameter("max_distance", 8.0);
    pose_slop_ = declare_parameter("pose_sync_slop", 0.05);
    backend_ = declare_parameter<std::string>("detector_backend", "traditional");
    const auto model = declare_parameter<std::string>("yolo_model", "");
    const auto device = declare_parameter<std::string>("yolo_device", "CPU");
    const auto score = declare_parameter("yolo_score_threshold", 0.7);
    const auto nms = declare_parameter("yolo_nms_threshold", 0.3);
    const auto threads = declare_parameter("yolo_threads", 2);
    if (backend_ == "yolo") {
      yolo_ = std::make_unique<YoloDetector>(model, device, enemy_color_, score, nms, threads);
    } else if (backend_ != "traditional") {
      throw std::invalid_argument("detector_backend must be yolo or traditional");
    }

    const Eigen::Matrix3d Ry = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d R_link_opt;
    R_link_opt << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    R_base_cam_ = Ry * R_link_opt;
    t_base_cam_ = Eigen::Vector3d(cam_xyz[0], cam_xyz[1], cam_xyz[2]);

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "color/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::SharedPtr m) {
        K_ = (cv::Mat_<double>(3, 3) << m->k[0], 0, m->k[2], 0, m->k[4], m->k[5], 0, 0, 1);
        D_ = cv::Mat(m->d.size() ? cv::Mat(m->d).clone() : cv::Mat::zeros(5, 1, CV_64F));
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::SharedPtr m) {
        const Eigen::Quaterniond q = toEigen(m->pose.pose.orientation);
        const Eigen::Vector3d p(m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z);
        if (!finiteQuat(q) || !p.allFinite()) {return;}
        const rclcpp::Time stamp(m->header.stamp);
        if (!poses_.empty() && stamp <= poses_.back().t) {
          if (now() < poses_.back().t) {
            // 仿真重置，不把旧场景的位姿和新场景图像插在一起。
            poses_.clear();
            pending_images_.clear();
            ++epoch_;
          } else {
            return;  // 重复/乱序 odom 不能产生零长度插值区间。
          }
        }
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = p;
        T.linear() = q.normalized().toRotationMatrix();
        poses_.push_back({stamp, T});
        while (poses_.size() > 200) {poses_.pop_front();}
        DrainImages();
      });
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "color/image_raw", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ArmorDetectorNode::OnImage, this, std::placeholders::_1));
    armor_pub_ = create_publisher<rm_drone_msgs::msg::ArmorDetectionArray>("armors", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("vision/debug_image", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vision/markers", 1);
    image_timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&ArmorDetectorNode::DrainImages, this));
    RCLCPP_INFO(get_logger(), "装甲识别后端=%s enemy=%s", backend_.c_str(), enemy_color_.c_str());
    worker_ = std::thread([this] {WorkerLoop();});
    if (imshow_) {
      gui_thread_ = std::thread([this] {GuiLoop();});
    }
  }

  ~ArmorDetectorNode() override
  {
    {
      std::lock_guard<std::mutex> lock(work_mutex_);
      stopping_ = true;
      next_frame_.reset();
    }
    work_ready_.notify_one();
    if (worker_.joinable()) {worker_.join();}
    imshow_ = false;
    if (gui_thread_.joinable()) {gui_thread_.join();}
  }

private:
  struct Frame
  {
    sensor_msgs::msg::Image::SharedPtr image;
    cv::Mat K, D;
    Eigen::Isometry3d T_map_base{Eigen::Isometry3d::Identity()};
    bool pose_ok{false};
    uint64_t epoch;
  };

  void WorkerLoop()
  {
    for (;;) {
      std::unique_ptr<Frame> frame;
      {
        std::unique_lock<std::mutex> lock(work_mutex_);
        work_ready_.wait(lock, [this] {return stopping_ || next_frame_ != nullptr;});
        if (stopping_) {return;}
        frame = std::move(next_frame_);
      }
      try {
        ProcessImage(*frame);
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "视觉帧处理失败: %s", e.what());
      }
    }
  }

  cv::Mat Binarize(const cv::Mat & bgr)
  {
    std::vector<cv::Mat> ch;
    cv::split(bgr, ch);
    cv::Mat diff, bright, bin;
    if (enemy_color_ == "red") {
      cv::subtract(ch[2], ch[0], diff);
    } else {
      cv::subtract(ch[0], ch[2], diff);
    }
    cv::threshold(diff, bin, color_thresh_, 255, cv::THRESH_BINARY);
    cv::Mat maxc;
    cv::max(ch[0], ch[2], maxc);
    cv::threshold(maxc, bright, bright_thresh_, 255, cv::THRESH_BINARY);
    cv::bitwise_and(bin, bright, bin);
    cv::dilate(bin, bin, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    return bin;
  }

  std::vector<LightBar> FindBars(const cv::Mat & bin)
  {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<LightBar> bars;
    for (const auto & c : contours) {
      if (cv::contourArea(c) < min_bar_area_ || c.size() < 5) {continue;}
      const cv::RotatedRect r = cv::minAreaRect(c);
      cv::Point2f p[4];
      r.points(p);
      std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) {return a.y < b.y;});
      LightBar b;
      b.top = (p[0] + p[1]) / 2;
      b.bottom = (p[2] + p[3]) / 2;
      b.center = r.center;
      b.length = cv::norm(b.top - b.bottom);
      b.width = cv::norm(p[0] - p[1]);
      if (b.width < 1e-3) {continue;}
      const float ratio = b.length / b.width;
      b.tilt_deg = std::abs(std::atan2(b.bottom.x - b.top.x, b.bottom.y - b.top.y)) * 180.0 / M_PI;
      if (ratio < min_bar_ratio_ || ratio > max_bar_ratio_ || b.tilt_deg > max_bar_tilt_) {continue;}
      bars.push_back(b);
    }
    std::sort(bars.begin(), bars.end(), [](const LightBar & a, const LightBar & b) {return a.center.x < b.center.x;});
    return bars;
  }

  std::vector<Armor> MatchBars(const std::vector<LightBar> & bars)
  {
    std::vector<Armor> armors;
    for (size_t i = 0; i < bars.size(); ++i) {
      for (size_t j = i + 1; j < bars.size(); ++j) {
        const auto & a = bars[i];
        const auto & b = bars[j];
        const float len_ratio = std::min(a.length, b.length) / std::max(a.length, b.length);
        const float avg_len = 0.5f * (a.length + b.length);
        const float dist = cv::norm(a.center - b.center);
        const float ratio = dist / avg_len;
        const float dy = std::abs(a.center.y - b.center.y) / avg_len;
        if (len_ratio < 0.6f || ratio < 0.8f || ratio > 5.5f || dy > 0.6f ||
          std::abs(a.tilt_deg - b.tilt_deg) > 15.0f) {continue;}
        // 两灯条之间不能夹着别的灯条
        bool blocked = false;
        for (size_t k = i + 1; k < j; ++k) {
          if (bars[k].center.y > std::min(a.top.y, b.top.y) && bars[k].center.y < std::max(a.bottom.y, b.bottom.y)) {
            blocked = true;
          }
        }
        if (blocked) {continue;}
        Armor ar{a, b, ratio > small_large_split_, {a.top, a.bottom, b.bottom, b.top}};
        armors.push_back(ar);
      }
    }
    return armors;
  }

  void DrawFeatures(const cv::Mat & bgr, cv::Mat & dbg)
  {
    if (++feat_frame_ % 2 != 0) {return;}
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::KeyPoint> kps;
    cv::FAST(gray, kps, 20, true);
    if (kps.size() > 250) {kps.resize(250);}
    for (const auto & k : kps) {cv::circle(dbg, k.pt, 3, cv::Scalar(0, 255, 0), -1);}
    char txt[64];
    std::snprintf(txt, sizeof(txt), "features %zu", kps.size());
    cv::putText(dbg, txt, {8, 44}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 0}, 1);
  }

  void Show(const cv::Mat & img)
  {
    if (!imshow_ || img.empty()) {return;}
    std::lock_guard<std::mutex> lk(gui_mtx_);
    latest_ = img.clone();
  }

  void GuiLoop()
  {
    const std::string win = "drone_camera";
    try {
      cv::namedWindow(win, cv::WINDOW_NORMAL);
      cv::resizeWindow(win, 640, 480);
      cv::moveWindow(win, 80, 80);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR(get_logger(), "无法创建相机窗口：%s", e.what());
      imshow_ = false;
      return;
    }
    RCLCPP_INFO(get_logger(), "相机窗口 drone_camera 已打开");
    cv::Mat splash(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
    cv::putText(splash, "waiting for camera...", {140, 240}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {220, 220, 220}, 2);
    while (rclcpp::ok() && imshow_) {
      cv::Mat frame;
      {
        std::lock_guard<std::mutex> lk(gui_mtx_);
        if (!latest_.empty()) {frame = latest_;}
      }
      try {
        cv::imshow(win, frame.empty() ? splash : frame);
        cv::waitKey(30);
      } catch (const cv::Exception & e) {
        RCLCPP_ERROR(get_logger(), "imshow 失败：%s", e.what());
        break;
      }
    }
    try {cv::destroyWindow(win);} catch (const cv::Exception &) {}
  }

  void OnImage(sensor_msgs::msg::Image::SharedPtr msg)
  {
    pending_images_.push_back({msg, std::chrono::steady_clock::now()});
    while (pending_images_.size() > 3) {pending_images_.pop_front();}
    DrainImages();
  }

  void DrainImages()
  {
    while (!pending_images_.empty()) {
      const auto & pending = pending_images_.front();
      const rclcpp::Time stamp(pending.image->header.stamp);
      const double age = (now() - stamp).seconds();
      if (stamp.nanoseconds() <= 0 || age > 0.2 || age < -0.05) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "丢弃过期/超前图像 age=%.3fs；请检查相机与 /clock，不能用旧帧定位", age);
        pending_images_.pop_front();
        continue;
      }
      const double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - pending.received).count();
      const bool bracketed = poses_.size() >= 2 && stamp <= poses_.back().t;
      // 相机和 odom 并行发布，图像先到时等待后一个位姿；不使用未来/最新位姿替代采样位姿。
      if ((!bracketed || age < 0.0) && waited < 0.1) {return;}
      auto frame = std::make_unique<Frame>();
      frame->image = pending.image;
      frame->K = K_.clone();
      frame->D = D_.clone();
      frame->pose_ok = age >= 0 && age < 0.2 && LookupPose(stamp, frame->T_map_base);
      frame->epoch = epoch_.load();
      pending_images_.pop_front();
      {
        // 单槽覆盖旧待处理帧；推理期间 ROS 回调仍可接收 odom、内参与新图像。
        std::lock_guard<std::mutex> lock(work_mutex_);
        next_frame_ = std::move(frame);
      }
      work_ready_.notify_one();
    }
  }

  void ProcessImage(const Frame & frame)
  {
    const auto & msg = frame.image;
    const rclcpp::Time stamp(msg->header.stamp);
    const auto fresh = [&] {
        const double age = (now() - stamp).seconds();
        return frame.epoch == epoch_.load() && age >= 0 && age < 0.2;
      };
    if (!fresh()) {return;}
    cv_bridge::CvImagePtr cv;
    try {
      cv = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
      return;
    }
    const cv::Mat & img = cv->image;
    RCLCPP_INFO_ONCE(get_logger(), "收到相机图像 %dx%d", img.cols, img.rows);
    cv::Mat dbg = img.clone();
    DrawFeatures(img, dbg);
    if (frame.K.empty()) {
      Show(dbg);
      return;
    }
    const auto start = std::chrono::steady_clock::now();
    std::vector<LightBar> bars;
    std::vector<Armor> armors;
    if (yolo_) {
      for (const auto & detection : yolo_->Detect(img)) {
        Armor ar{};
        ar.large = detection.large;
        ar.corners.assign(detection.corners.begin(), detection.corners.end());
        ar.kind = detection.kind;
        ar.confidence = detection.confidence;
        ar.class_id = detection.class_id;
        armors.push_back(ar);
        cv::rectangle(dbg, detection.box, {255, 180, 0}, 1);
        const auto text = detection.kind + " " + cv::format("%.2f", detection.confidence);
        cv::putText(dbg, text, detection.box.tl() + cv::Point(0, -5),
          cv::FONT_HERSHEY_SIMPLEX, 0.45, {255, 180, 0}, 1);
      }
      const auto stats = yolo_->Stats();
      const auto text = cv::format("model best=c%d %.2f raw=%d", stats.top_class, stats.top_score, stats.above_threshold);
      cv::putText(dbg, text, {8, img.rows - 12}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 180, 0}, 1);
    } else {
      bars = FindBars(Binarize(img));
      armors = MatchBars(bars);
    }
    const double infer_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (!fresh()) {return;}

    rm_drone_msgs::msg::ArmorDetectionArray out;
    out.header = msg->header;
    visualization_msgs::msg::MarkerArray markers;
    for (const auto & b : bars) {cv::line(dbg, b.top, b.bottom, {0, 255, 255}, 1);}

    const auto & T_map_base = frame.T_map_base;

    int id = 0;
    for (const auto & ar : armors) {
      const double hw = (ar.large ? large_w_ : small_w_) / 2.0, hh = bar_h_ / 2.0;
      const float fhw = static_cast<float>(hw), fhh = static_cast<float>(hh);
      const std::vector<cv::Point3f> obj{{-fhw, -fhh, 0}, {-fhw, fhh, 0}, {fhw, fhh, 0}, {fhw, -fhh, 0}};
      std::vector<cv::Mat> rvecs, tvecs;
      const int nsol = cv::solvePnPGeneric(obj, ar.corners, frame.K, frame.D, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
      int best = -1;
      double err = 1e9;
      for (int s = 0; s < nsol; ++s) {
        if (tvecs[s].empty() || !std::isfinite(tvecs[s].at<double>(2))) {continue;}
        const double z = tvecs[s].at<double>(2);
        const double dist = cv::norm(tvecs[s]);
        if (z < min_dist_ || dist > max_dist_) {continue;}
        std::vector<cv::Point2f> proj;
        cv::projectPoints(obj, rvecs[s], tvecs[s], frame.K, frame.D, proj);
        double e = 0;
        for (int k = 0; k < 4; ++k) {e += cv::norm(proj[k] - ar.corners[k]);}
        e /= 4.0;
        if (e < err) {err = e; best = s;}
      }
      if (best < 0 || err > max_reproj_) {continue;}
      cv::Mat rvec = rvecs[best], tvec = tvecs[best];

      cv::Mat Rcv;
      cv::Rodrigues(rvec, Rcv);
      Eigen::Matrix3d R_cam_armor;
      for (int r = 0; r < 3; ++r) {for (int c = 0; c < 3; ++c) {R_cam_armor(r, c) = Rcv.at<double>(r, c);}}
      const Eigen::Vector3d t_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

      rm_drone_msgs::msg::ArmorDetection d;
      d.armor_type = ar.large ? d.LARGE : d.SMALL;
      d.color = enemy_color_;
      d.kind = ar.kind;
      d.confidence = ar.confidence;
      d.class_id = ar.class_id;
      d.distance = static_cast<float>(t_cam.norm());
      d.reprojection_error = static_cast<float>(err);
      for (int k = 0; k < 4; ++k) {
        d.corners[2 * k] = ar.corners[k].x;
        d.corners[2 * k + 1] = ar.corners[k].y;
      }
      d.pose_camera.position.x = t_cam.x();
      d.pose_camera.position.y = t_cam.y();
      d.pose_camera.position.z = t_cam.z();
      d.pose_camera.orientation = toMsg(Eigen::Quaterniond(R_cam_armor));

      if (frame.pose_ok) {
        // 装甲坐标系 z 轴指向板内，取其反向为朝外法向；map 中的装甲姿态用“x 轴=朝外法向”表示
        const Eigen::Matrix3d R_map_cam = T_map_base.linear() * R_base_cam_;
        const Eigen::Vector3d p_map = T_map_base * (R_base_cam_ * t_cam + t_base_cam_);
        Eigen::Vector3d n_map = -(R_map_cam * R_cam_armor.col(2));
        const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
        n_map = n_map - n_map.dot(up) * up;
        if (n_map.norm() >= 1e-3) {n_map.normalize();}
        const double yaw = n_map.norm() > 0 ? std::atan2(n_map.y(), n_map.x()) : 0.0;
        d.pose_map.position.x = p_map.x();
        d.pose_map.position.y = p_map.y();
        d.pose_map.position.z = p_map.z();
        d.pose_map.orientation = toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, up)));
        // 此节点没有跨帧实例 ID。不同装甲不能共用一个“上一位置”跳变门限；
        // 目标关联由任务节点按目标板的位置门限处理。
        d.has_map_pose = n_map.norm() > 0 && p_map.allFinite();

        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = "map";
        mk.header.stamp = msg->header.stamp;
        mk.ns = "armor";
        mk.id = id++;
        mk.type = mk.ARROW;
        mk.action = mk.ADD;
        mk.pose = d.pose_map;
        mk.scale.x = 0.3;
        mk.scale.y = 0.03;
        mk.scale.z = 0.03;
        mk.color.a = 1.0;
        mk.color.g = 1.0;
        mk.lifetime = rclcpp::Duration::from_seconds(0.2);
        if (d.has_map_pose) {markers.markers.push_back(mk);}
      }
      out.armors.push_back(d);

      for (int k = 0; k < 4; ++k) {cv::line(dbg, ar.corners[k], ar.corners[(k + 1) % 4], {0, 255, 0}, 2);}
      char txt[64];
      std::snprintf(txt, sizeof(txt), "%s %.2fm e%.1f", ar.large ? "L" : "S", d.distance, err);
      cv::putText(dbg, txt, ar.corners[0] + cv::Point2f(0, -6), cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 0}, 1);
    }
    char hdr[160];
    std::snprintf(hdr, sizeof(hdr), "%s enemy=%s candidates=%zu poses=%zu %.1fms", backend_.c_str(),
      enemy_color_.c_str(), armors.size(), out.armors.size(), infer_ms);
    cv::putText(dbg, hdr, {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1);

    if (!fresh()) {return;}
    armor_pub_->publish(out);
    Show(dbg);
    if (debug_pub_->get_subscription_count() > 0) {
      debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg());
    }
    if (!markers.markers.empty()) {marker_pub_->publish(markers);}
  }

  std::string enemy_color_;
  std::string backend_;
  std::unique_ptr<YoloDetector> yolo_;
  std::thread worker_;
  std::mutex work_mutex_;
  std::condition_variable work_ready_;
  std::unique_ptr<Frame> next_frame_;
  bool stopping_{false};
  std::atomic<uint64_t> epoch_{0};
  int color_thresh_, bright_thresh_;
  double min_bar_ratio_, max_bar_ratio_, max_bar_tilt_, min_bar_area_, small_large_split_;
  double bar_h_, small_w_, large_w_;
  Eigen::Matrix3d R_base_cam_;
  Eigen::Vector3d t_base_cam_;
  cv::Mat K_, D_;
  bool LookupPose(const rclcpp::Time & stamp, Eigen::Isometry3d & out) const
  {
    if (poses_.size() < 2 || stamp.nanoseconds() <= 0) {return false;}
    if (stamp < poses_.front().t || stamp > poses_.back().t) {return false;}
    for (size_t i = 1; i < poses_.size(); ++i) {
      if (poses_[i].t < stamp) {continue;}
      const double dt = (poses_[i].t - poses_[i - 1].t).seconds();
      if (dt > pose_slop_ || dt <= 0.0) {return false;}
      const double a = (stamp - poses_[i - 1].t).seconds() / dt;
      out.translation() = (1.0 - a) * poses_[i - 1].T.translation() + a * poses_[i].T.translation();
      const Eigen::Quaterniond q0(poses_[i - 1].T.linear());
      const Eigen::Quaterniond q1(poses_[i].T.linear());
      out.linear() = q0.slerp(a, q1).toRotationMatrix();
      return true;
    }
    return false;
  }

  struct PoseSample
  {
    rclcpp::Time t;
    Eigen::Isometry3d T;
  };
  struct PendingImage
  {
    sensor_msgs::msg::Image::SharedPtr image;
    std::chrono::steady_clock::time_point received;
  };
  double max_reproj_, min_dist_, max_dist_, pose_slop_;
  std::deque<PoseSample> poses_;
  std::deque<PendingImage> pending_images_;
  std::atomic<bool> imshow_{true};
  cv::Mat latest_;
  std::mutex gui_mtx_;
  std::thread gui_thread_;
  int feat_frame_{0};
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<rm_drone_msgs::msg::ArmorDetectionArray>::SharedPtr armor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr image_timer_;
};

}  // namespace rm_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<rm_drone::ArmorDetectorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("armor_detector"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
