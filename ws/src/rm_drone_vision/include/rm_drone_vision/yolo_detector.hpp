#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace rm_drone
{
struct YoloStats
{
  int top_class{-1};
  float top_score{0.f};
  int above_threshold{0};
  // 最高分的被拒绝框仅供诊断，不进入 Detect() 返回的导航候选。
  cv::Rect rejected_box;
  int rejected_class{-1};
  float rejected_score{0.f};
  std::string rejection;
};

struct YoloDetection
{
  cv::Rect box;
  std::array<cv::Point2f, 4> corners;  // TL, BL, BR, TR: light-bar endpoints, not box corners
  int class_id;
  float confidence;
  bool large;
  std::string kind;
};

// The supplied YOLO11 has 38 class scores and 4 XY keypoints, output [1,50,8400].
// Padding is black on the bottom/right, as in the source inference implementation.
std::vector<YoloDetection> DecodeYolo11(
  const cv::Mat & channels, cv::Size image_size, double scale,
  const std::string & enemy, float score_threshold, float nms_threshold,
  YoloStats * stats = nullptr);

class YoloDetector
{
public:
  YoloDetector(const std::string & model, const std::string & device,
    const std::string & enemy, float score_threshold, float nms_threshold, int threads);
  ~YoloDetector();
  std::vector<YoloDetection> Detect(const cv::Mat & bgr);
  YoloStats Stats() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace rm_drone
