#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

// Slamtec SDK headers
#include "rplidar.h"

using namespace rp::standalone::rplidar;

// Serial configuration helper
int openSerialPort(const std::string& portName, int baudRate) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Failed to open serial port: " << portName << std::endl;
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;         // No parity
    options.c_cflag &= ~CSTOPB;         // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;              // 8 data bits
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_oflag &= ~OPOST;          // Raw output

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

void sendCommand(int serialFd, int steerAngle, int throttle) {
    if (serialFd < 0) return;
    // Format: "CMD:<steer>,<throttle>\n"
    std::string packet = "CMD:" + std::to_string(steerAngle) + "," + std::to_string(throttle) + "\n";
    write(serialFd, packet.c_str(), packet.length());
}

int main(int argc, char* argv[]) {
    const std::string lidarPort = "/dev/ttyUSB0";     // Verify your RPLidar port
    const std::string esp32Port = "/dev/ttyUSB1";     // Verify your ESP32 port
    const uint32_t lidarBaudrate = 460800;            // C1 standard baudrate is 460800

    // 1. Open Serial Port to ESP32
    int esp32Fd = openSerialPort(esp32Port, B115200);
    if (esp32Fd < 0) {
        std::cerr << "Warning: Proceeding without active ESP32 serial link." << std::endl;
    }

    // 2. Initialize RPLidar Driver
    RPlidarDriver* driver = RPlidarDriver::CreateDriver(
        CHANNEL_TYPE_SERIALPORT
    );
    if (!driver) {
        std::cerr << "Failed to construct RPLidar driver." << std::endl;
        return -1;
    }

    if (IS_FAIL(driver->connect(lidarPort.c_str(), lidarBaudrate))) {
        std::cerr << "Cannot connect to RPLidar on " << lidarPort << std::endl;
        RPlidarDriver::DisposeDriver(driver);
        return -1;
    }

    // Start motor and scanning
    driver->startMotor();
    driver->startScan(0, 1);

    std::cout << "Autonomous navigator running. Streaming to ESP32..." << std::endl;

    const float SAFE_DISTANCE_MM = 1200.0f;    // Very close obstacle threshold (~1.2 m)
    const float CAUTION_DISTANCE_MM = 3000.0f;  // Start gentle slowdown at ~3m
    const float SIDE_CLEARANCE_MM = 600.0f;     // Side clearance threshold for minor course corrections
    const float MAX_THROTTLE = 40.0f;
    const float MIN_THROTTLE = 10.0f;

    while (true) {
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        u_result res = driver->grabScanDataHq(nodes, count);
        if (IS_OK(res)) {
            driver->ascendScanData(nodes, count);

            float minFrontDist = 10000.0f;
            float minLeftDist = 10000.0f;
            float minRightDist = 10000.0f;

            for (size_t i = 0; i < count; ++i) {
                // Quality check and non-zero distance
                if (nodes[i].quality == 0) continue;

                float distance = nodes[i].dist_mm_q2 / 4.0f;
                if (distance < 50.0f) continue; // Ignore sensor self-blindspot

                float angle = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;

                // Center sector: [345° to 360°] and [0° to 15°]
                if (angle >= 345.0f || angle <= 15.0f) {
                    minFrontDist = std::min(minFrontDist, distance);
                }
                // Left sector: [300° to 345°]
                else if (angle >= 300.0f && angle < 345.0f) {
                    minLeftDist = std::min(minLeftDist, distance);
                }
                // Right sector: [15° to 60°]
                else if (angle > 15.0f && angle <= 60.0f) {
                    minRightDist = std::min(minRightDist, distance);
                }
            }

            // Smooth reactive obstacle avoidance using risk scaling
            int steer = 90;
            int throttle = 40; // Default cruise speed

            float frontRisk = 0.0f;
            if (minFrontDist < CAUTION_DISTANCE_MM) {
                frontRisk = std::clamp((CAUTION_DISTANCE_MM - minFrontDist) / (CAUTION_DISTANCE_MM - SAFE_DISTANCE_MM), 0.0f, 1.0f);
            }

            // Smooth slowdown based on front obstacle distance
            float throttleScale = 1.0f - frontRisk;
            float desiredThrottle = MAX_THROTTLE * throttleScale;
            if (desiredThrottle < MIN_THROTTLE) desiredThrottle = MIN_THROTTLE;
            throttle = static_cast<int>(desiredThrottle);

            // Smooth turn away from the side with less clearance
            float sideBias = minLeftDist - minRightDist;
            int steerOffset = static_cast<int>(sideBias * 0.03f * frontRisk);
            steer = 90 + steerOffset;

            // Keep steering in a reasonable range
            if (steer < 45) steer = 45;
            if (steer > 135) steer = 135;

            // Very close obstacles: stronger braking and sharper avoidance
            if (minFrontDist < SAFE_DISTANCE_MM) {
                throttle = 15;
                if (minLeftDist > minRightDist) {
                    steer = 60; // gentle left turn to clear the right side
                } else {
                    steer = 120; // gentle right turn to clear the left side
                }
            }

            // Dead-end / trap: reverse if boxed in
            if (minFrontDist < 300.0f && minLeftDist < 350.0f && minRightDist < 350.0f) {
                steer = 90;
                throttle = -30; // Reverse command
            }

            // Additional side clearance corrections, but still smooth
            if (minLeftDist < SIDE_CLEARANCE_MM && minLeftDist < minRightDist) {
                steer = 100;
            } else if (minRightDist < SIDE_CLEARANCE_MM && minRightDist < minLeftDist) {
                steer = 80;
            }

            sendCommand(esp32Fd, steer, throttle);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // ~50 Hz cycle
    }

    // Cleanup
    driver->stop();
    driver->stopMotor();
    RPlidarDriver::DisposeDriver(driver);
    if (esp32Fd >= 0) close(esp32Fd);

    return 0;
}
