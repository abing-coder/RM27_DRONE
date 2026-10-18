// 装甲识别：颜色通道相减 -> 灯条轮廓筛选 -> 灯条配对 -> PnP（IPPE）-> 换算到 map 坐标系
// 输出：/drone/armors（ArmorDetectionArray）、/drone/vision/debug_image（叠加识别结果的画面）、RViz Marker
#include <algorithm>
#include <atomic>
#include <cmath>
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
    if (imshow_) {
      // GTK 窗口必须在同一条线程里创建和刷新，不能放在图像回调里。
      gui_thread_ = std::thread([this] {GuiLoop();});
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
        have_info_ = true;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::SharedPtr m) {odom_ = *m; have_odom_ = true;});
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "color/image_raw", rclcpp::SensorDataQoS(), std::bind(&ArmorDetectorNode::OnImage, this, std::placeholders::_1));
    armor_pub_ = create_publisher<rm_drone_msgs::msg::ArmorDetectionArray>("armors", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("vision/debug_image", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vision/markers", 1);
  }

  ~ArmorDetectorNode() override
  {
    imshow_ = false;
    if (gui_thread_.joinable()) {gui_thread_.join();}
  }

private:
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
    if (!have_info_) {
      Show(dbg);
      return;
    }
    const cv::Mat bin = Binarize(img);
    const auto bars = FindBars(bin);
    const auto armors = MatchBars(bars);

    rm_drone_msgs::msg::ArmorDetectionArray out;
    out.header = msg->header;
    visualization_msgs::msg::MarkerArray markers;
    for (const auto & b : bars) {cv::line(dbg, b.top, b.bottom, {0, 255, 255}, 1);}

    Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
    if (have_odom_) {
      T_map_base.translation() = Eigen::Vector3d(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
      T_map_base.linear() = toEigen(odom_.pose.pose.orientation).normalized().toRotationMatrix();
    }

    int id = 0;
    for (const auto & ar : armors) {
      const double hw = (ar.large ? large_w_ : small_w_) / 2.0, hh = bar_h_ / 2.0;
      const float fhw = static_cast<float>(hw), fhh = static_cast<float>(hh);
      const std::vector<cv::Point3f> obj{{-fhw, -fhh, 0}, {-fhw, fhh, 0}, {fhw, fhh, 0}, {fhw, -fhh, 0}};
      cv::Mat rvec, tvec;
      if (!cv::solvePnP(obj, ar.corners, K_, D_, rvec, tvec, false, cv::SOLVEPNP_IPPE)) {continue;}
      std::vector<cv::Point2f> proj;
      cv::projectPoints(obj, rvec, tvec, K_, D_, proj);
      double err = 0;
      for (int k = 0; k < 4; ++k) {err += cv::norm(proj[k] - ar.corners[k]);}
      err /= 4.0;

      cv::Mat Rcv;
      cv::Rodrigues(rvec, Rcv);
      Eigen::Matrix3d R_cam_armor;
      for (int r = 0; r < 3; ++r) {for (int c = 0; c < 3; ++c) {R_cam_armor(r, c) = Rcv.at<double>(r, c);}}
      const Eigen::Vector3d t_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

      rm_drone_msgs::msg::ArmorDetection d;
      d.armor_type = ar.large ? d.LARGE : d.SMALL;
      d.color = enemy_color_;
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

      if (have_odom_) {
        // 装甲坐标系 z 轴指向板内，取其反向为朝外法向；map 中的装甲姿态用“x 轴=朝外法向”表示
        const Eigen::Matrix3d R_map_cam = T_map_base.linear() * R_base_cam_;
        const Eigen::Vector3d p_map = T_map_base * (R_base_cam_ * t_cam + t_base_cam_);
        Eigen::Vector3d n_map = -(R_map_cam * R_cam_armor.col(2));
        const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
        n_map = (n_map - n_map.dot(up) * up).normalized();
        const double yaw = std::atan2(n_map.y(), n_map.x());
        d.pose_map.position.x = p_map.x();
        d.pose_map.position.y = p_map.y();
        d.pose_map.position.z = p_map.z();
        d.pose_map.orientation = toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, up)));
        d.has_map_pose = true;

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
        markers.markers.push_back(mk);
      }
      out.armors.push_back(d);

      for (int k = 0; k < 4; ++k) {cv::line(dbg, ar.corners[k], ar.corners[(k + 1) % 4], {0, 255, 0}, 2);}
      char txt[64];
      std::snprintf(txt, sizeof(txt), "%s %.2fm e%.1f", ar.large ? "L" : "S", d.distance, err);
      cv::putText(dbg, txt, ar.corners[0] + cv::Point2f(0, -6), cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 0}, 1);
    }
    char hdr[96];
    std::snprintf(hdr, sizeof(hdr), "enemy=%s bars=%zu armors=%zu", enemy_color_.c_str(), bars.size(), armors.size());
    cv::putText(dbg, hdr, {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1);

    armor_pub_->publish(out);
    Show(dbg);
    if (debug_pub_->get_subscription_count() > 0) {
      debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg());
    }
    if (!markers.markers.empty()) {marker_pub_->publish(markers);}
  }

  std::string enemy_color_;
  int color_thresh_, bright_thresh_;
  double min_bar_ratio_, max_bar_ratio_, max_bar_tilt_, min_bar_area_, small_large_split_;
  double bar_h_, small_w_, large_w_;
  Eigen::Matrix3d R_base_cam_;
  Eigen::Vector3d t_base_cam_;
  cv::Mat K_, D_;
  bool have_info_{false}, have_odom_{false};
  std::atomic<bool> imshow_{true};
  cv::Mat latest_;
  std::mutex gui_mtx_;
  std::thread gui_thread_;
  int feat_frame_{0};
  nav_msgs::msg::Odometry odom_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<rm_drone_msgs::msg::ArmorDetectionArray>::SharedPtr armor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rm_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_drone::ArmorDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
