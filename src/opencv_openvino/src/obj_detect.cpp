#include <filesystem>
#include <iomanip>
#include <memory>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "interfaces/msg/detect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"

#define N_CLASSES 2           // hanya 2 kelas: Baskom & Flare
#define INPUT_WIDTH 640
#define INPUT_HEIGHT 640
#define CONF_THRESH 0.4f
#define SCORE_THRESH 0.4f
#define NMS_THRESH 0.4f

using namespace cv;
using namespace std;

const char* coconame[] = {"Baskom", "Flare"}; // indeks 0 = Baskom, 1 = Flare

const float color_list[80][3] = {
    {0.000f, 0.447f, 0.741f}, {0.850f, 0.325f, 0.098f}, {0.929f, 0.694f, 0.125f},
    {0.494f, 0.184f, 0.556f}, {0.466f, 0.674f, 0.188f}, {0.301f, 0.745f, 0.933f},
    {0.635f, 0.078f, 0.184f}, {0.300f, 0.300f, 0.300f}, {0.600f, 0.600f, 0.600f},
    {1.000f, 0.000f, 0.000f}, {1.000f, 0.500f, 0.000f}, {0.749f, 0.749f, 0.000f},
    {0.000f, 1.000f, 0.000f}, {0.000f, 0.000f, 1.000f}, {0.667f, 0.000f, 1.000f},
    {0.333f, 0.333f, 0.000f}, {0.333f, 0.667f, 0.000f}, {0.333f, 1.000f, 0.000f},
    {0.667f, 0.333f, 0.000f}, {0.667f, 0.667f, 0.000f}, {0.667f, 1.000f, 0.000f},
    {1.000f, 0.333f, 0.000f}, {1.000f, 0.667f, 0.000f}, {1.000f, 1.000f, 0.000f},
    {0.000f, 0.333f, 0.500f}, {0.000f, 0.667f, 0.500f}, {0.000f, 1.000f, 0.500f},
    {0.333f, 0.000f, 0.500f}, {0.333f, 0.333f, 0.500f}, {0.333f, 0.667f, 0.500f},
    {0.333f, 1.000f, 0.500f}, {0.667f, 0.000f, 0.500f}, {0.667f, 0.333f, 0.500f},
    {0.667f, 0.667f, 0.500f}, {0.667f, 1.000f, 0.500f}, {1.000f, 0.000f, 0.500f},
    {1.000f, 0.333f, 0.500f}, {1.000f, 0.667f, 0.500f}, {1.000f, 1.000f, 0.500f},
    {0.000f, 0.333f, 1.000f}, {0.000f, 0.667f, 1.000f}, {0.000f, 1.000f, 1.000f},
    {0.333f, 0.000f, 1.000f}, {0.333f, 0.333f, 1.000f}, {0.333f, 0.667f, 1.000f},
    {0.333f, 1.000f, 1.000f}, {0.667f, 0.000f, 1.000f}, {0.667f, 0.333f, 1.000f},
    {0.667f, 0.667f, 1.000f}, {0.667f, 1.000f, 1.000f}, {1.000f, 0.000f, 1.000f},
    {1.000f, 0.333f, 1.000f}, {1.000f, 0.667f, 1.000f}, {0.333f, 0.000f, 0.000f},
    {0.500f, 0.000f, 0.000f}, {0.667f, 0.000f, 0.000f}, {0.833f, 0.000f, 0.000f},
    {1.000f, 0.000f, 0.000f}, {0.000f, 0.167f, 0.000f}, {0.000f, 0.333f, 0.000f},
    {0.000f, 0.500f, 0.000f}, {0.000f, 0.667f, 0.000f}, {0.000f, 0.833f, 0.000f},
    {0.000f, 1.000f, 0.000f}, {0.000f, 0.000f, 0.167f}, {0.000f, 0.000f, 0.333f},
    {0.000f, 0.000f, 0.500f}, {0.000f, 0.000f, 0.667f}, {0.000f, 0.000f, 0.833f},
    {0.000f, 0.000f, 1.000f}, {0.000f, 0.000f, 0.000f}, {0.143f, 0.143f, 0.143f},
    {0.286f, 0.286f, 0.286f}, {0.429f, 0.429f, 0.429f}, {0.571f, 0.571f, 0.571f},
    {0.714f, 0.714f, 0.714f}, {0.857f, 0.857f, 0.857f}, {0.000f, 0.447f, 0.741f},
    {0.314f, 0.717f, 0.741f}, {0.50f, 0.5f, 0.0f}
};

struct Config {
   float confThreshold;
   float nmsThreshold;
   float scoreThreshold;
   int inpWidth;
   int inpHeight;
   std::string onnx_path;
};

struct Resize {
   cv::Mat resized_image; 
   int dw;
   int dh;
   int new_w;
   int new_h;
   int pad_x;
   int pad_y;
   float ratio;
};

struct Detection {
   int class_id;
   float confidence;
   cv::Rect box;
};

class YOLOV5 {
  public:
   YOLOV5(const Config &config);
   ~YOLOV5();
   std::vector<Detection> detect(cv::Mat& frame);

  private:
   float confThreshold;
   float nmsThreshold;
   float scoreThreshold;
   int inpWidth;
   int inpHeight;
   std::string onnx_path;
   Resize resize;
   ov::Tensor input_tensor;
   ov::InferRequest infer_request;
   ov::CompiledModel compiled_model;
   std::vector<uint8_t> input_blob_; // NHWC u8
   void initialmodel();
   void preprocess_img(const cv::Mat& frame);
   std::vector<Detection> postprocess_img(const float* detections, const ov::Shape& output_shape, const cv::Size &orig_size);
};

YOLOV5::YOLOV5(const Config &config) {
   this->confThreshold = config.confThreshold;
   this->nmsThreshold = config.nmsThreshold;
   this->scoreThreshold = config.scoreThreshold;
   this->inpWidth = config.inpWidth;
   this->inpHeight = config.inpHeight;
   this->onnx_path = config.onnx_path;
   this->initialmodel();
}
YOLOV5::~YOLOV5() {}

void YOLOV5::initialmodel() {
   ov::Core core;
   std::shared_ptr<ov::Model> model = core.read_model(this->onnx_path);

   ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
   ppp.input()
       .tensor()
       .set_element_type(ov::element::u8)
       .set_layout("NHWC")
       .set_color_format(ov::preprocess::ColorFormat::RGB);
   ppp.input()
       .preprocess()
       .convert_element_type(ov::element::f32)
       .convert_color(ov::preprocess::ColorFormat::RGB)
       .scale({255, 255, 255}); // bagi dengan 255 (normalisasi)
   ppp.input().model().set_layout("NCHW");
   ppp.output().tensor().set_element_type(ov::element::f32);
   model = ppp.build();
   this->compiled_model = core.compile_model(model, "CPU");
   this->infer_request = compiled_model.create_infer_request();
}

void YOLOV5::preprocess_img(const cv::Mat& frame) {
   float orig_w = static_cast<float>(frame.cols);
   float orig_h = static_cast<float>(frame.rows);

   // hitung
   float r = std::min((float)inpWidth / orig_w, (float)inpHeight / orig_h);
   int new_w = static_cast<int>(round(orig_w * r));
   int new_h = static_cast<int>(round(orig_h * r));
   int pad_x = (inpWidth - new_w) / 2;
   int pad_y = (inpHeight - new_h) / 2;

   resize.new_w = new_w;
   resize.new_h = new_h;
   resize.pad_x = pad_x;
   resize.pad_y = pad_y;
   resize.ratio = r;

   cv::Mat resized;
   cv::resize(frame, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

   cv::Mat out(cv::Size(inpWidth, inpHeight), frame.type(), cv::Scalar(114,114,114));
   resized.copyTo(out(cv::Rect(pad_x, pad_y, new_w, new_h)));

   // ubah BGR ke RGB
   cv::Mat rgb;
   cv::cvtColor(out, rgb, cv::COLOR_BGR2RGB);

   int H = rgb.rows;
   int W = rgb.cols;
   int C = rgb.channels();
   input_blob_.clear();
   input_blob_.reserve(H * W * C);
   for (int y = 0; y < H; ++y) {
       const cv::Vec3b* row_ptr = rgb.ptr<cv::Vec3b>(y);
       for (int x = 0; x < W; ++x) {
           // NHWC: push R,G,B (ppp set_color_format RGB)
           input_blob_.push_back(row_ptr[x][0]);
           input_blob_.push_back(row_ptr[x][1]);
           input_blob_.push_back(row_ptr[x][2]);
       }
   }

   // create input tensor using persistent input_blob_ memory
   ov::element::Type et = compiled_model.input().get_element_type();
   ov::Shape shape = compiled_model.input().get_shape(); // likely {1,H,W,C} if NHWC declared
   // Create tensor with input_blob_.data()
   input_tensor = ov::Tensor(et, shape, input_blob_.data());
   infer_request.set_input_tensor(input_tensor);
}

std::vector<Detection> YOLOV5::postprocess_img(const float* detections, const ov::Shape& output_shape, const cv::Size &orig_size) {
   std::vector<cv::Rect> boxes;
   std::vector<int> class_ids;
   std::vector<float> confidences;

   if (output_shape.size() < 3) return {};

   size_t num_rows = static_cast<size_t>(output_shape[1]);
   size_t row_len = static_cast<size_t>(output_shape[2]);
   if (row_len < 6) return {};

   for (size_t i = 0; i < num_rows; ++i) {
      const float* det = detections + i * row_len;
      float obj = det[4];
      if (obj <= 0.0f) continue;

      int best_cls = -1;
      float best_score = 0.0f;
      for (size_t c = 5; c < row_len; ++c) {
         float s = det[c];
         if (s > best_score) { best_score = s; best_cls = static_cast<int>(c - 5); }
      }
      float final_conf = obj * best_score;
      if (final_conf < this->scoreThreshold) continue;

      float cx = det[0];
      float cy = det[1];
      float w = det[2];
      float h = det[3];

      float x1 = (cx - w * 0.5f - resize.pad_x) / resize.ratio;
      float y1 = (cy - h * 0.5f - resize.pad_y) / resize.ratio;
      float bw = w / resize.ratio;
      float bh = h / resize.ratio;

      int ix = static_cast<int>(std::round(x1));
      int iy = static_cast<int>(std::round(y1));
      int iw = static_cast<int>(std::round(bw));
      int ih = static_cast<int>(std::round(bh));

      // clamp to image
      ix = std::max(0, std::min(ix, orig_size.width - 1));
      iy = std::max(0, std::min(iy, orig_size.height - 1));
      if (iw <= 0 || ih <= 0) continue;
      if (ix + iw > orig_size.width) iw = orig_size.width - ix;
      if (iy + ih > orig_size.height) ih = orig_size.height - iy;

      boxes.emplace_back(ix, iy, iw, ih);
      class_ids.push_back(best_cls);
      confidences.push_back(final_conf);
   }

   std::vector<int> nms_result;
   if (!boxes.empty()) {
      cv::dnn::NMSBoxes(boxes, confidences, this->scoreThreshold, this->nmsThreshold, nms_result);
   }

   std::vector<Detection> output;
   for (size_t i = 0; i < nms_result.size(); ++i) {
      int idx = nms_result[i];
      if (idx < 0 || idx >= (int)boxes.size()) continue;
      Detection d;
      d.class_id = class_ids[idx];
      d.confidence = confidences[idx];
      d.box = boxes[idx];
      output.push_back(d);
   }
   return output;
}

std::vector<Detection> YOLOV5::detect(cv::Mat& frame) {
   cv::Size orig_size(frame.cols, frame.rows);
   preprocess_img(frame);
   try {
      infer_request.infer();
   } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("YOLOV5"), "Infer failed: %s", e.what());
      return {};
   }

   ov::Tensor output_tensor = infer_request.get_output_tensor(0);
   ov::Shape out_shape = output_tensor.get_shape();
   const float* data = output_tensor.data<const float>();
   return postprocess_img(data, out_shape, orig_size);
}

// Node ROS2
class ObjectDetection : public rclcpp::Node {
public:
   ObjectDetection()
   : Node("object_detection")
   {
      object_pub_ = this->create_publisher<interfaces::msg::Detect>("objects", 10);
      image_with_box_pub_ = this->create_publisher<sensor_msgs::msg::Image>("objects_box", 10);

      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          "camera", rclcpp::SensorDataQoS(),
          std::bind(&ObjectDetection::image_callback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(33),
          std::bind(&ObjectDetection::process_frame_from_video, this));

      config_ = {CONF_THRESH, NMS_THRESH, SCORE_THRESH, INPUT_WIDTH, INPUT_HEIGHT, std::string("src/opencv_openvino/model/waras.onnx")};

      try {
         yolov5_ = std::make_unique<YOLOV5>(config_);
      } catch (const std::exception &e) {
         RCLCPP_ERROR(this->get_logger(), "Failed to init YOLO model: %s", e.what());
      }

      video_path_ = "src/opencv_openvino/include/fourth.mp4";
      cap_.open(video_path_);
      if (!cap_.isOpened()) {
         RCLCPP_WARN(this->get_logger(), "Cannot open video file: %s", video_path_.c_str());
      }
   }

private:
   void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
      cv::Mat frame;
      try {
         frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
      } catch (const std::exception &e) {
         RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
         return;
      }
      process_and_publish(frame, msg->header.stamp);
   }

   void process_frame_from_video() {
      if (!cap_.isOpened() || !yolov5_) return;
      cv::Mat frame;
      if (!cap_.read(frame)) {
         cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
         return;
      }
      process_and_publish(frame, this->now());
   }

   void process_and_publish(cv::Mat &frame, const rclcpp::Time &stamp) {
      if (!yolov5_) return;
      cv::Mat draw_frame = frame.clone();

      std::vector<Detection> dets = yolov5_->detect(draw_frame);

      for (const auto &d : dets) {
         interfaces::msg::Detect msg;

         // ambil data dan sesuaikan dengan msg dari interface
         msg.label = std::string(coconame[ std::max(0, std::min(d.class_id, N_CLASSES-1)) ]);
         msg.x = static_cast<int32_t>(d.box.x);
         msg.y = static_cast<int32_t>(d.box.y);
         msg.w = static_cast<int32_t>(d.box.width);
         msg.h = static_cast<int32_t>(d.box.height);
         msg.confidence = d.confidence;
         object_pub_->publish(msg);

         cv::Scalar color = cv::Scalar(
             color_list[std::max(0, std::min(d.class_id, N_CLASSES-1))][0] * 255.0f,
             color_list[std::max(0, std::min(d.class_id, N_CLASSES-1))][1] * 255.0f,
             color_list[std::max(0, std::min(d.class_id, N_CLASSES-1))][2] * 255.0f);

         cv::Rect r(msg.x, msg.y, msg.w, msg.h);
         r &= cv::Rect(0, 0, frame.cols, frame.rows);
         cv::rectangle(draw_frame, r, color, 2);

         char textbuf[128];
         std::snprintf(textbuf, sizeof(textbuf), "%s %.1f%%", msg.label.c_str(), msg.confidence * 100.0f);
         int baseLine = 0;
         cv::Size label_size = cv::getTextSize(textbuf, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
         cv::rectangle(draw_frame,
                       cv::Point(r.x, r.y - label_size.height - baseLine),
                       cv::Point(r.x + label_size.width, r.y),
                       color * 0.7, cv::FILLED);
         cv::Scalar meanc = cv::mean(color);
         cv::Scalar txt_color = (meanc[0] + meanc[1] + meanc[2]) / 3.0 > 128 ? cv::Scalar(0,0,0) : cv::Scalar(255,255,255);
         cv::putText(draw_frame, textbuf, cv::Point(r.x, r.y - baseLine), cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1);
      }

      // publish gambar dengan box
      std_msgs::msg::Header header;
      builtin_interfaces::msg::Time stamp_msg;
      stamp_msg.sec = static_cast<int32_t>(stamp.nanoseconds() / 1000000000);
      stamp_msg.nanosec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000);
      header.stamp = stamp_msg;
      header.frame_id = "camera";

      auto img_msg = cv_bridge::CvImage(header, "bgr8", draw_frame).toImageMsg();
      image_with_box_pub_->publish(*img_msg);

      RCLCPP_INFO(this->get_logger(), "Published %zu detections and image_with_box", dets.size());
   }

   rclcpp::Publisher<interfaces::msg::Detect>::SharedPtr object_pub_;
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_with_box_pub_;
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
   rclcpp::TimerBase::SharedPtr timer_;

   std::unique_ptr<YOLOV5> yolov5_;
   Config config_;
   cv::VideoCapture cap_;
   std::string video_path_;
};

int main(int argc, char* argv[]) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<ObjectDetection>());
   rclcpp::shutdown();
   return 0;
};
