// Model layout/class mapping follows TongjiSuperPower RM27; see models/SOURCE.md.
#include "rm_drone_vision/yolo_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace rm_drone
{
std::vector<YoloDetection> DecodeYolo11(
  const cv::Mat & channels, cv::Size image_size, double scale,
  const std::string & enemy, float score_threshold, float nms_threshold, YoloStats * stats)
{
  if (channels.type() != CV_32F || (channels.rows != 50 && channels.rows != 16) || channels.cols != 8400 ||
    !std::isfinite(scale) || scale <= 0 || image_size.area() <= 0 ||
    (enemy != "blue" && enemy != "red"))
  {
    throw std::invalid_argument("Expected armor pose [50,8400] or simulation [16,8400] float output and a red/blue enemy");
  }
  const bool simulation = channels.rows == 16;
  const int classes = simulation ? 4 : 38;
  const int keypoints = 4 + classes;
  std::vector<YoloDetection> candidates;
  if (stats) {*stats = YoloStats{};}
  for (int i = 0; i < channels.cols; ++i) {
    float score = -1;
    int class_id = -1;
    for (int c = 0; c < classes; ++c) {
      const float value = channels.at<float>(4 + c, i);
      if (std::isfinite(value) && value > score) {score = value; class_id = c;}
    }
    if (stats && score >= 0 && score <= 1) {
      if (score > stats->top_score) {stats->top_score = score; stats->top_class = class_id;}
      if (score >= score_threshold) {++stats->above_threshold;}
    }
    if (score < std::min(0.3f, score_threshold) || score > 1) {continue;}
    // Keep only illuminated enemy ordinary armor. No dart/buff/robot armor targets.
    const bool blue = enemy == "blue";
    const bool outpost = class_id == (simulation ? (blue ? 3 : 2) : (blue ? 18 : 19));
    const bool large_base = !simulation && class_id == (blue ? 21 : 22);
    const bool small_base = class_id == (simulation ? (blue ? 1 : 0) : (blue ? 25 : 26));
    const bool base = large_base || small_base;
    const float x = channels.at<float>(0, i), y = channels.at<float>(1, i);
    const float w = channels.at<float>(2, i), h = channels.at<float>(3, i);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w) || !std::isfinite(h) ||
      w <= 0 || h <= 0 || w / scale > 2 * image_size.width || h / scale > 2 * image_size.height)
    {continue;}
    const cv::Rect2f raw((x - w / 2) / scale, (y - h / 2) / scale, w / scale, h / scale);
    const cv::Rect2f clipped = raw & cv::Rect2f(0, 0, image_size.width, image_size.height);
    if (clipped.width < 2 || clipped.height < 2) {continue;}
    const auto rejected = [&](const std::string & reason) {
        if (stats && score >= 0.3f && score > stats->rejected_score) {
          stats->rejected_box = cv::Rect(clipped);
          stats->rejected_class = class_id;
          stats->rejected_score = score;
          stats->rejection = reason;
        }
      };
    if ((!outpost && !base) || score < score_threshold) {
      rejected(!outpost && !base ? "class rejected" : "score rejected");
      continue;
    }
    std::array<cv::Point2f, 4> points;
    bool valid = true;
    for (int k = 0; k < 4; ++k) {
      points[k] = {static_cast<float>(channels.at<float>(keypoints + 2*k, i) / scale),
        static_cast<float>(channels.at<float>(keypoints + 1 + 2*k, i) / scale)};
      const auto & p = points[k];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
        p.x >= image_size.width || p.y >= image_size.height ||
        p.x < raw.x - 0.1f * raw.width || p.x > raw.x + 1.1f * raw.width ||
        p.y < raw.y - 0.1f * raw.height || p.y > raw.y + 1.1f * raw.height)
      {valid = false;}
    }
    if (!valid) {rejected("keypoints rejected"); continue;}
    // Image roll is bounded by the aircraft; split into left/right bars, then top/bottom.
    std::sort(points.begin(), points.end(), [](const auto & a, const auto & b) {return a.x < b.x;});
    if (points[0].y > points[1].y) {std::swap(points[0], points[1]);}
    if (points[2].y < points[3].y) {std::swap(points[2], points[3]);}
    const std::vector<cv::Point2f> polygon(points.begin(), points.end());
    if (!cv::isContourConvex(polygon) || std::abs(cv::contourArea(polygon)) < 4 ||
      cv::norm(points[0] - points[1]) < 2 || cv::norm(points[2] - points[3]) < 2)
    {rejected("keypoints rejected"); continue;}
    candidates.push_back({cv::Rect(clipped), points, class_id, score, large_base, base ? "base" : "outpost"});
  }
  // NMS by semantic class, including the official map's small-base modules.
  std::vector<YoloDetection> result;
  for (int semantic : (simulation ? std::vector<int>{0, 1, 2, 3} : std::vector<int>{18, 19, 21, 22, 25, 26})) {
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<size_t> lookup;
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (candidates[i].class_id != semantic) {continue;}
      boxes.push_back(candidates[i].box);
      scores.push_back(candidates[i].confidence);
      lookup.push_back(i);
    }
    std::vector<int> kept;
    cv::dnn::NMSBoxes(boxes, scores, score_threshold, nms_threshold, kept);
    for (int i : kept) {result.push_back(candidates[lookup[i]]);}
  }
  return result;
}
}  // namespace rm_drone
