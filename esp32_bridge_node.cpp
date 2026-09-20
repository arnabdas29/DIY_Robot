#include <fcntl.h>
#include <functional>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

int openSerialPort(const std::string& portName, int baudRate) {
  int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    std::cerr << "Failed to open serial port: " << portName << std::endl;
    return -1;
  }

  struct termios options;
  tcgetattr(fd, &options);
  cfsetispeed(&options, baudRate);
  cfsetospeed(&options, baudRate);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  tcsetattr(fd, TCSANOW, &options);
  return fd;
}

void sendCommand(int serialFd, int steerAngle, int throttle) {
  if (serialFd < 0) {
    return;
  }

  std::string packet = "CMD:" + std::to_string(steerAngle) + "," + std::to_string(throttle) + "\n";
  write(serialFd, packet.c_str(), packet.length());
}

}  // namespace

class Esp32BridgeNode : public rclcpp::Node {
public:
  Esp32BridgeNode()
  : Node("esp32_bridge_node"),
    esp32Port_("/dev/ttyUSB1"),
    serialFd_(openSerialPort(esp32Port_, B115200)) {
    if (serialFd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "ESP32 serial link unavailable; waiting for port %s", esp32Port_.c_str());
    }

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&Esp32BridgeNode::cmdVelCallback, this, std::placeholders::_1));
  }

  ~Esp32BridgeNode() override {
    if (serialFd_ >= 0) {
      close(serialFd_);
      serialFd_ = -1;
    }
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    int throttle = static_cast<int>(msg->linear.x * 40.0);
    int steer = static_cast<int>((msg->angular.z * 45.0) + 90.0);

    if (steer < 45) {
      steer = 45;
    }
    if (steer > 135) {
      steer = 135;
    }

    if (throttle < -40) {
      throttle = -40;
    }
    if (throttle > 40) {
      throttle = 40;
    }

    sendCommand(serialFd_, steer, throttle);
  }

  std::string esp32Port_;
  int serialFd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Esp32BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
