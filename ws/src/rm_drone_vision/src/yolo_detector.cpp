#include "rm_drone_vision/yolo_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

#ifdef RM_HAVE_OPENVINO
#include <openvino/openvino.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>
#endif

namespace rm_drone
{
struct YoloDetector::Impl
{
#ifdef RM_HAVE_OPENVINO
  ov::Core core;
  ov::CompiledModel model;
  ov::InferRequest request;
#endif
  int channels{50};
  std::string enemy;
  float score, nms;
  YoloStats stats;
};

YoloDetector::YoloDetector(const std::string & model, const std::string & device,
  const std::string & enemy, float score_threshold, float nms_threshold, int threads)
: impl_(std::make_unique<Impl>())
{
  if ((enemy != "blue" && enemy != "red") || !std::isfinite(score_threshold) ||
    !std::isfinite(nms_threshold) || score_threshold <= 0 || score_threshold > 1 ||
    nms_threshold <= 0 || nms_threshold > 1 || threads < 1)
  {throw std::invalid_argument("Invalid YOLO color/threshold/thread parameters");}
#ifdef RM_HAVE_OPENVINO
  auto net = impl_->core.read_model(model);
  if (net->inputs().size() != 1 || net->outputs().size() != 1 ||
    net->input().get_partial_shape() != ov::PartialShape{1, 3, 640, 640} ||
    (net->output().get_partial_shape() != ov::PartialShape{1, 50, 8400} &&
      net->output().get_partial_shape() != ov::PartialShape{1, 16, 8400}))
  {throw std::runtime_error("Armor model output must be legacy [1,50,8400] or simulation four-class [1,16,8400]");}
  impl_->channels = static_cast<int>(net->output().get_shape()[1]);
  ov::preprocess::PrePostProcessor ppp(net);
  ppp.input().tensor().set_element_type(ov::element::u8).set_shape({1,640,640,3})
    .set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
  ppp.input().model().set_layout("NCHW");
  ppp.input().preprocess().convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB).scale(255.f);
  ppp.output().tensor().set_element_type(ov::element::f32);
  ov::AnyMap properties{{ov::hint::performance_mode.name(), ov::hint::PerformanceMode::LATENCY}};
  if (device == "CPU") {properties[ov::inference_num_threads.name()] = threads;}
  impl_->model = impl_->core.compile_model(ppp.build(), device, properties);
  impl_->request = impl_->model.create_infer_request();  // reused, never recreated per image
  impl_->enemy = enemy;
  impl_->score = score_threshold;
  impl_->nms = nms_threshold;
#else
  (void)model;
  (void)device;
  throw std::runtime_error("OpenVINO was not found when building rm_drone_vision; install its C++ runtime and rebuild, or select detector:=traditional");
#endif
}

YoloDetector::~YoloDetector() = default;

YoloStats YoloDetector::Stats() const {return impl_->stats;}

std::vector<YoloDetection> YoloDetector::Detect(const cv::Mat & bgr)
{
#ifdef RM_HAVE_OPENVINO
  if (bgr.empty() || bgr.type() != CV_8UC3) {throw std::invalid_argument("YOLO requires a nonempty BGR8 image");}
  const double scale = std::min(640.0 / bgr.cols, 640.0 / bgr.rows);
  const int width = std::max(1, static_cast<int>(bgr.cols * scale));
  const int height = std::max(1, static_cast<int>(bgr.rows * scale));
  const bool simulation = impl_->channels == 16;
  const int pad_x = simulation ? (640-width)/2 : 0, pad_y = simulation ? (640-height)/2 : 0;
  cv::Mat input(640, 640, CV_8UC3, simulation ? cv::Scalar(114,114,114) : cv::Scalar(0,0,0));
  cv::resize(bgr, input(cv::Rect(pad_x, pad_y, width, height)), {width, height});
  impl_->request.set_input_tensor(ov::Tensor(ov::element::u8, {1,640,640,3}, input.data));
  impl_->request.infer();
  auto output = impl_->request.get_output_tensor();
  cv::Mat channels(impl_->channels, 8400, CV_32F, output.data<float>());
  if (simulation) {
    // Ultralytics standard centered letterbox; legacy RM27 keeps its original
    // top-left black padding. Convert coordinates before common validation.
    channels = channels.clone();
    channels.row(0) -= pad_x; channels.row(1) -= pad_y;
    for (int k = 0; k < 4; ++k) {channels.row(8+2*k) -= pad_x; channels.row(9+2*k) -= pad_y;}
  }
  return DecodeYolo11(channels, bgr.size(), scale, impl_->enemy, impl_->score, impl_->nms, &impl_->stats);
#else
  (void)bgr;
  throw std::runtime_error("YOLO backend unavailable");
#endif
}
}  // namespace rm_drone
