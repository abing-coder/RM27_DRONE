// Landing feedback from the CAD's native colored octagonal H platform.
// No simulator pose/ground truth is subscribed. Missing/occluded targets publish
// no error; the mission holds and cannot interpret a missing image as zero error.
#include <algorithm>
#include <cmath>
#include <memory>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "rm_drone_control/math_utils.hpp"

namespace rm_drone {
class LandingDetector : public rclcpp::Node {
public:
  LandingDetector() : Node("landing_detector"), buffer_(get_clock()), listener_(buffer_)
  {
    team_ = declare_parameter<std::string>("team", "red");
    const auto pad = declare_parameter<std::vector<double>>("pad_xyz", {-1.03238311,-1.03237595,1.6});
    if (pad.size() != 3) {throw std::invalid_argument("pad_xyz must contain x,y,z");}
    pad_ = Eigen::Vector3d(pad[0],pad[1],pad[2]);
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("landing/camera_info", qos,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr m) {info_ = *m; have_info_ = true;});
    image_sub_ = create_subscription<sensor_msgs::msg::Image>("landing/image_raw", qos,
      [this](sensor_msgs::msg::Image::SharedPtr m) {rgb_ = m; TryImage();});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>("landing/depth/image_raw", qos,
      [this](sensor_msgs::msg::Image::SharedPtr m) {depth_ = m; TryImage();});
    position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("landing/position", qos);
    error_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("landing/error", qos);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("landing/debug_image", qos);
    retry_ = create_wall_timer(std::chrono::milliseconds(10),[this]{TryImage();});
  }
private:
  void TryImage()
  {
    if (!rgb_ || !depth_ || std::abs((rclcpp::Time(rgb_->header.stamp)-rclcpp::Time(depth_->header.stamp)).seconds()) > .02 ||
      rclcpp::Time(rgb_->header.stamp) == last_processed_) {return;}
    if (rclcpp::Time(rgb_->header.stamp)<last_processed_) {last_rim_=last_seen_=-1;have_glyph_offset_=false;}
    const rclcpp::Time stamp(rgb_->header.stamp);
    if (!buffer_.canTransform("map",rgb_->header.frame_id,stamp) ||
      !buffer_.canTransform("map","base_link",stamp)) {return;}
    // Image and odometry arrive independently. Do not consume a synchronized
    // RGB-D pair until both transforms at its sample time have arrived.
    last_processed_ = rgb_->header.stamp;
    Image(*rgb_);
  }
  static bool IsH(const std::vector<cv::Point> & contour)
  {
    const double area=cv::contourArea(contour);
    std::vector<cv::Point> hull;cv::convexHull(contour,hull);
    const double convex=cv::contourArea(hull);
    if (area<40 || convex<=0 || area/convex<.20 || area/convex>.85 || contour.size()<8) {return false;}
    // H has two deep, opposed notches with parallel mouths. This topology is
    // independent of stroke thickness; Hu distance alone confuses a rectangle
    // with a thick H and rejects thin H glyphs.
    const auto rect=cv::minAreaRect(contour);
    const double size=std::min(rect.size.width,rect.size.height);
    std::vector<int> indices;cv::convexHull(contour,indices,false,false);
    if (indices.size()<3) {return false;}
    std::vector<cv::Vec4i> defects;cv::convexityDefects(contour,indices,defects);
    std::vector<Eigen::Vector2d> mouths,directions;
    for (const auto & defect:defects) {
      if (defect[3]/256. < .12*size) {continue;}
      const auto a=contour[defect[0]],b=contour[defect[1]];
      Eigen::Vector2d axis(b.x-a.x,b.y-a.y);
      Eigen::Vector2d outward((a.x+b.x)*.5-rect.center.x,(a.y+b.y)*.5-rect.center.y);
      if (axis.norm()<1 || outward.norm()<1) {return false;}
      mouths.push_back(axis.normalized());directions.push_back(outward.normalized());
    }
    return mouths.size()==2 && std::abs(mouths[0].dot(mouths[1]))>.85 && directions[0].dot(directions[1])<-.8;
  }
  void Image(const sensor_msgs::msg::Image & message)
  {
    const rclcpp::Time stamp(message.header.stamp);
    const double age = (now() - stamp).seconds();
    if (!have_info_ || age < -.05 || age > .3 || !depth_ ||
      depth_->width != message.width || depth_->height != message.height ||
      info_.k[0] <= 0 || info_.k[4] <= 0 || message.width != info_.width || message.height != info_.height) {return;}
    try {
      const auto camera_tf = buffer_.lookupTransform("map", message.header.frame_id, stamp);
      const auto base_tf = buffer_.lookupTransform("map", "base_link", stamp);
      const Eigen::Quaterniond qc = toEigen(camera_tf.transform.rotation);
      const Eigen::Vector3d origin(camera_tf.transform.translation.x,camera_tf.transform.translation.y,camera_tf.transform.translation.z);
      const Eigen::Vector3d base(base_tf.transform.translation.x,base_tf.transform.translation.y,base_tf.transform.translation.z);
      if ((qc*Eigen::Vector3d::UnitZ()).z() > -.95) {return;}
      const cv::Mat depth = cv_bridge::toCvCopy(*depth_, "32FC1")->image;
      auto delta_at = [&](cv::Point2d point) -> Eigen::Vector3d {
        std::vector<float> values;
        for (int y=std::max(0,int(point.y)-1);y<=std::min(depth.rows-1,int(point.y)+1);++y) {
          for (int x=std::max(0,int(point.x)-1);x<=std::min(depth.cols-1,int(point.x)+1);++x) {
            const float d=depth.at<float>(y,x);
            if (std::isfinite(d) && d>.05 && d<1.2) {values.push_back(d);}
          }
        }
        if (values.size()<3) {return Eigen::Vector3d::Constant(NAN);}
        std::sort(values.begin(),values.end());
        return qc * Eigen::Vector3d((point.x-info_.k[2])/info_.k[0],(point.y-info_.k[5])/info_.k[4],1) * values[values.size()/2];
      };
      cv::Mat bgr = cv_bridge::toCvCopy(message, "bgr8")->image, hsv, mask;
      cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
      if (team_ == "red") {
        cv::Mat a, b;
        cv::inRange(hsv, cv::Scalar(0, 130, 80), cv::Scalar(12, 255, 255), a);
        cv::inRange(hsv, cv::Scalar(168, 130, 80), cv::Scalar(179, 255, 255), b);
        mask = a | b;
      } else {cv::inRange(hsv, cv::Scalar(95, 130, 80), cv::Scalar(135, 255, 255), mask);}
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      double best = 0;
      cv::Point2d center;
      cv::Point2d glyph_center;
      std::vector<cv::Point> selected;
      for (const auto & contour : contours) {
        const double area = cv::contourArea(contour);
        const auto rect = cv::boundingRect(contour);
        if (area < 300 || area > bgr.total() * .85 || rect.x < 3 || rect.y < 3 ||
          rect.br().x > bgr.cols-3 || rect.br().y > bgr.rows-3) {continue;}
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(contour, polygon, cv::arcLength(contour, true)*.018, true);
        if (polygon.size() < 6 || polygon.size() > 10 || !cv::isContourConvex(polygon)) {continue;}
        const auto box = cv::minAreaRect(contour);
        const double aspect = box.size.width / std::max(1.f, box.size.height);
        if (aspect < .7 || aspect > 1.43) {continue;}
        cv::Mat inside = cv::Mat::zeros(mask.size(), CV_8U), white;
        cv::drawContours(inside, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(255), -1);
        cv::inRange(bgr, cv::Scalar(180,180,180), cv::Scalar(255,255,255), white);
        if (cv::countNonZero(inside & white) < .035 * area) {continue;}
        std::vector<std::vector<cv::Point>> letters;
        cv::Mat masked=inside&white;
        cv::findContours(masked,letters,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        double h_area=0;cv::Point2d h_center;
        for (const auto & letter:letters) {
          if (IsH(letter) && cv::contourArea(letter)>h_area) {
            h_area=cv::contourArea(letter);h_center=cv::minAreaRect(letter).center;
          }
        }
        if (h_area==0) {continue;}
        if (area > best) {best = area; center = box.center;glyph_center=h_center; selected = contour;}
      }
      bool glyph = false;
      if (best == 0 && have_glyph_offset_ && last_seen_>0 && stamp.seconds()-last_seen_<1.0) {
        // At low height the outer rim leaves the field of view. Continue with
        // the native white H with a full or partially visible team-colored rim.
        cv::Mat white;
        cv::inRange(bgr, cv::Scalar(180,180,180), cv::Scalar(255,255,255), white);
        std::vector<std::vector<cv::Point>> letters;
        cv::findContours(white, letters, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto & contour : letters) {
          const double area = cv::contourArea(contour);
          const auto rect = cv::boundingRect(contour);
          const auto box = cv::minAreaRect(contour);
          const double aspect = box.size.width / std::max(1.f, box.size.height);
          if (area < 200 || rect.x < 2 || rect.y < 2 || rect.br().x > bgr.cols-2 || rect.br().y > bgr.rows-2 ||
            aspect < .65 || aspect > 1.55 || !IsH(contour)) {continue;}
          const double physical = std::sqrt(area)*(-delta_at(box.center).z())/info_.k[0];
          if (!std::isfinite(physical)) {continue;}
          if (physical < .10 || physical > .20) {continue;}
          if (area > best) {best = area; center = box.center; selected = contour; glyph = true;}
        }
      }
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
        "降落标记：候选=%d 字样续跟=%d 偏移已标定=%d 边框龄=%.2fs",
        best>0,glyph,have_glyph_offset_,last_rim_>0?stamp.seconds()-last_rim_:-1.);
      if (best > 0) {
        auto reject=[&](const char * why) {
          RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,"降落候选拒绝：%s",why);
          debug_pub_->publish(*cv_bridge::CvImage(message.header,"bgr8",bgr).toImageMsg());
        };
        Eigen::Vector3d delta = delta_at(center);
        if (glyph) {delta+=glyph_offset_;}
        if (!delta.allFinite() || -delta.z() < .05 || -delta.z() > .85) {reject("深度/高度无效");return;}
        const Eigen::Vector3d observed_pad = origin+delta;
        if ((observed_pad.head<2>()-pad_.head<2>()).norm() > .8 || std::abs(observed_pad.z()-pad_.z()) > .4) {reject("超出已知平台区域");return;}
        const double size = std::sqrt(best)*(-delta.z())/info_.k[0];
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
          "降落观测：相机高度=%.3fm 特征尺度=%.3fm 横向偏移=%.3fm",
          -delta.z(),size,delta.head<2>().norm());
        if ((!glyph && (size < .32 || size > .65)) || delta.head<2>().norm() > .8) {reject("尺寸/偏移不符");return;}
        geometry_msgs::msg::PointStamped position;
        position.header=message.header; position.header.frame_id="map";
        const Eigen::Vector3d measured_base=pad_-delta-(origin-base);
        position.point.x=measured_base.x(); position.point.y=measured_base.y(); position.point.z=measured_base.z();
        // Only the fully observed colored rim defines an absolute landmark.
        // The local glyph is a continuation of the last rim, not a new anchor.
        if (!glyph) {position_pub_->publish(position);}
        geometry_msgs::msg::PointStamped error;
        error.header = message.header; error.header.frame_id = "map";
        // Relative to body origin, including the camera lever arm. Translation
        // of the map estimate cancels from this feedback.
        error.point.x = observed_pad.x()-base.x();
        error.point.y = observed_pad.y()-base.y();
        error.point.z = base.z()-observed_pad.z();
        error_pub_->publish(error);
        last_seen_=stamp.seconds();
        if (!glyph) {
          const Eigen::Vector3d h_delta=delta_at(glyph_center);
          if (h_delta.allFinite() && (delta-h_delta).norm()<.12) {
            glyph_offset_=delta-h_delta;have_glyph_offset_=true;last_rim_=stamp.seconds();
          }
        }
        cv::drawContours(bgr, std::vector<std::vector<cv::Point>>{selected}, 0, cv::Scalar(0,255,0), 2);
        cv::circle(bgr, center, 3, cv::Scalar(0,255,255), -1);
      }
      debug_pub_->publish(*cv_bridge::CvImage(message.header, "bgr8", bgr).toImageMsg());
    } catch (const std::exception & e) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Landing frame rejected: %s", e.what());
    }
  }
  std::string team_;
  Eigen::Vector3d pad_{Eigen::Vector3d::Zero()};
  double last_rim_{-1},last_seen_{-1};
  bool have_glyph_offset_{false};
  Eigen::Vector3d glyph_offset_{Eigen::Vector3d::Zero()};
  bool have_info_{false};
  rclcpp::Time last_processed_{0,0,RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr retry_;
  sensor_msgs::msg::Image::SharedPtr rgb_, depth_;
  sensor_msgs::msg::CameraInfo info_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr error_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_drone::LandingDetector>());
  rclcpp::shutdown();
}
