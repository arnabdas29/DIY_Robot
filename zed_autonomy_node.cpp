#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

namespace {

constexpr float kMinSafeDistanceM = 0.60f;
constexpr float kCautionDistanceM = 1.20f;
constexpr float kMaxThrottle = 0.75f;
constexpr float kMaxTurnRate = 1.0f;

float clamp(float value, float min_value, float max_value) {
  return std::max(min_value, std::min(max_value, value));
}

}  // namespace

class ZedAutonomyNode : public rclcpp::Node {
public:
  ZedAutonomyNode()
  : Node("zed_autonomy_node") {
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed2/zed_node/depth/depth_registered",
        rclcpp::SensorDataQoS(),
        std::bind(&ZedAutonomyNode::depthCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ZedAutonomyNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
                "ZED autonomy node started. Subscribing to /zed2/zed_node/depth/depth_registered");
  }

private:
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
      std::lock_guard<std::mutex> lock(depth_mutex_);
      latest_depth_ = frame.clone();
    } catch (const cv_bridge::Exception& ex) {
      RCLCPP_WARN(this->get_logger(), "Depth conversion failed: %s", ex.what());
    }
  }

  void controlLoop() {
    cv::Mat depth;
    {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      if (latest_depth_.empty()) {
        return;
      }
      depth = latest_depth_.clone();
    }

    geometry_msgs::msg::Twist twist_msg;
    auto decision = computeCommand(depth);
    twist_msg.linear.x = decision.linear_x;
    twist_msg.angular.z = decision.angular_z;
    cmd_pub_->publish(twist_msg);
  }

  struct CommandDecision {
    double linear_x;
    double angular_z;
  };

  CommandDecision computeCommand(const cv::Mat& depth) {
    CommandDecision cmd{0.0, 0.0};
    if (depth.empty()) {
      return cmd;
    }

    cv::Mat roi;
    const int center_x = depth.cols / 2;
    const int center_y = depth.rows / 2;
    const int roi_w = depth.cols / 3;
    const int roi_h = depth.rows / 2;

    const int x0 = std::max(0, center_x - roi_w / 2);
    const int x1 = std::min(depth.cols, center_x + roi_w / 2);
    const int y0 = std::max(0, center_y - roi_h / 2);
    const int y1 = std::min(depth.rows, center_y + roi_h / 2);

    roi = depth(cv::Rect(x0, y0, x1 - x0, y1 - y0));

    float min_center = std::numeric_limits<float>::infinity();
    float min_left = std::numeric_limits<float>::infinity();
    float min_right = std::numeric_limits<float>::infinity();

    const int roi_cx = roi.cols / 2;
    for (int y = 0; y < roi.rows; ++y) {
      const float* row = roi.ptr<float>(y);
      for (int x = 0; x < roi.cols; ++x) {
        const float d = row[x];
        if (!std::isfinite(d) || d <= 0.0f || d > 10.0f) {
          continue;
        }

        if (x < roi_cx - roi.cols / 6) {
          min_left = std::min(min_left, d);
        } else if (x > roi_cx + roi.cols / 6) {
          min_right = std::min(min_right, d);
        } else {
          min_center = std::min(min_center, d);
        }
      }
    }

    if (min_center == std::numeric_limits<float>::infinity()) {
      min_center = 10.0f;
    }
    if (min_left == std::numeric_limits<float>::infinity()) {
      min_left = 10.0f;
    }
    if (min_right == std::numeric_limits<float>::infinity()) {
      min_right = 10.0f;
    }

    const float clear_margin = 0.05f;
    const float turn_gain = 0.8f;

    if (min_center < kMinSafeDistanceM) {
      cmd.linear_x = 0.0;
      cmd.angular_z = (min_left > min_right) ? kMaxTurnRate : -kMaxTurnRate;
      return cmd;
    }

    if (min_left < kCautionDistanceM || min_right < kCautionDistanceM) {
      const float left_gap = std::max(0.0f, kCautionDistanceM - min_left);
      const float right_gap = std::max(0.0f, kCautionDistanceM - min_right);
      const float diff = right_gap - left_gap;
      cmd.linear_x = 0.25 + (1.0 - std::min(1.0f, min_center / kCautionDistanceM)) * 0.45f;
      cmd.angular_z = clamp(diff * turn_gain, -kMaxTurnRate, kMaxTurnRate);
      return cmd;
    }

    cmd.linear_x = kMaxThrottle;
    cmd.angular_z = 0.0;

    const float left_bias = (kCautionDistanceM - min_left) * 0.25f;
    const float right_bias = (kCautionDistanceM - min_right) * 0.25f;
    cmd.angular_z = clamp((right_bias - left_bias) * 0.8f, -kMaxTurnRate, kMaxTurnRate);

    if (std::fabs(cmd.angular_z) < clear_margin) {
      cmd.angular_z = 0.0;
    }

    return cmd;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex depth_mutex_;
  cv::Mat latest_depth_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ZedAutonomyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
