#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Slamtec SDK headers
#include "rplidar.h"

using namespace rp::standalone::rplidar;

namespace {

}  // namespace

class LidarControllerNode : public rclcpp::Node {
public:
  LidarControllerNode()
  : Node("lidar_controller_node"),
    lidarPort_("/dev/ttyUSB0"),
    lidarBaudrate_(460800),
    driver_(RPlidarDriver::CreateDriver(CHANNEL_TYPE_SERIALPORT)) {
    if (!driver_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct RPLidar driver.");
      return;
    }

    if (IS_FAIL(driver_->connect(lidarPort_.c_str(), lidarBaudrate_))) {
      RCLCPP_ERROR(this->get_logger(), "Cannot connect to RPLidar on %s", lidarPort_.c_str());
      return;
    }

    driver_->startMotor();
    driver_->startScan(0, 1);

    RCLCPP_INFO(this->get_logger(), "Autonomous navigator running. Publishing cmd_vel.");

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&LidarControllerNode::scanLoop, this));
  }

  ~LidarControllerNode() override {
    if (driver_) {
      driver_->stop();
      driver_->stopMotor();
      RPlidarDriver::DisposeDriver(driver_);
      driver_ = nullptr;
    }
  }

private:
  void scanLoop() {
    if (!driver_) {
      return;
    }

    const float SAFE_DISTANCE_MM = 1200.0f;
    const float CAUTION_DISTANCE_MM = 3000.0f;
    const float SIDE_CLEARANCE_MM = 600.0f;
    const float MAX_THROTTLE = 40.0f;
    const float MIN_THROTTLE = 10.0f;

    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    u_result res = driver_->grabScanDataHq(nodes, count);
    if (!IS_OK(res)) {
      return;
    }

    driver_->ascendScanData(nodes, count);

    float minFrontDist = 10000.0f;
    float minLeftDist = 10000.0f;
    float minRightDist = 10000.0f;

    for (size_t i = 0; i < count; ++i) {
      if (nodes[i].quality == 0) {
        continue;
      }

      float distance = nodes[i].dist_mm_q2 / 4.0f;
      if (distance < 50.0f) {
        continue;
      }

      float angle = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;

      if (angle >= 345.0f || angle <= 15.0f) {
        minFrontDist = std::min(minFrontDist, distance);
      } else if (angle >= 300.0f && angle < 345.0f) {
        minLeftDist = std::min(minLeftDist, distance);
      } else if (angle > 15.0f && angle <= 60.0f) {
        minRightDist = std::min(minRightDist, distance);
      }
    }

    int steer = 90;
    int throttle = 40;

    float frontRisk = 0.0f;
    if (minFrontDist < CAUTION_DISTANCE_MM) {
      float normalized = (CAUTION_DISTANCE_MM - minFrontDist) / (CAUTION_DISTANCE_MM - SAFE_DISTANCE_MM);
      frontRisk = std::clamp(normalized, 0.0f, 1.0f);
    }

    float throttleScale = 1.0f - frontRisk;
    float desiredThrottle = MAX_THROTTLE * throttleScale;
    if (desiredThrottle < MIN_THROTTLE) {
      desiredThrottle = MIN_THROTTLE;
    }
    throttle = static_cast<int>(desiredThrottle);

    float sideBias = minLeftDist - minRightDist;
    int steerOffset = static_cast<int>(sideBias * 0.03f * frontRisk);
    steer = 90 + steerOffset;

    if (steer < 45) {
      steer = 45;
    }
    if (steer > 135) {
      steer = 135;
    }

    if (minFrontDist < SAFE_DISTANCE_MM) {
      throttle = 15;
      if (minLeftDist > minRightDist) {
        steer = 60;
      } else {
        steer = 120;
      }
    }

    if (minFrontDist < 300.0f && minLeftDist < 350.0f && minRightDist < 350.0f) {
      steer = 90;
      throttle = -30;
    }

    if (minLeftDist < SIDE_CLEARANCE_MM && minLeftDist < minRightDist) {
      steer = 100;
    } else if (minRightDist < SIDE_CLEARANCE_MM && minRightDist < minLeftDist) {
      steer = 80;
    }

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = static_cast<double>(throttle) / 40.0;
    twist_msg.angular.z = static_cast<double>(steer - 90) / 45.0;
    cmd_pub_->publish(twist_msg);
  }

  std::string lidarPort_;
  uint32_t lidarBaudrate_;
  RPlidarDriver* driver_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
