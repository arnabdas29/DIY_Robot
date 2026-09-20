#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <thread>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

static volatile std::sig_atomic_t keep_running = 1;

static void signal_handler(int)
{
    keep_running = 0;
}

static float get_angle_deg(const sl_lidar_response_measurement_node_hq_t& node)
{
    return static_cast<float>(node.angle_z_q14) * 90.0f / 16384.0f;
}

static float get_distance_mm(const sl_lidar_response_measurement_node_hq_t& node)
{
    return static_cast<float>(node.dist_mm_q2) / 4.0f;
}

int main(int argc, char** argv)
{
    const char* serial_port =
        argc > 1 ? argv[1] : "/dev/ttyUSB0";

    const std::uint32_t baud_rate =
        argc > 2
            ? static_cast<std::uint32_t>(std::stoul(argv[2]))
            : 460800;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "Opening RPLIDAR C1 on "
              << serial_port
              << " at "
              << baud_rate
              << " baud\n";

    auto channel_result =
        createSerialPortChannel(serial_port, baud_rate);

    if (!channel_result)
    {
        std::cerr << "Failed to create serial channel\n";
        return 1;
    }

    IChannel* channel = *channel_result;

    auto driver_result = createLidarDriver();

    if (!driver_result)
    {
        std::cerr << "Failed to create RPLIDAR driver\n";
        delete channel;
        return 1;
    }

    ILidarDriver* lidar = *driver_result;

    sl_result result = lidar->connect(channel);

    if (SL_IS_FAIL(result))
    {
        std::cerr << "Cannot connect to RPLIDAR. Error: 0x"
                  << std::hex << result << std::dec << "\n";
        delete lidar;
        delete channel;
        return 1;
    }

    sl_lidar_response_device_info_t device_info {};
    result = lidar->getDeviceInfo(device_info);

    if (SL_IS_OK(result))
    {
        std::cout << "Model: "
                  << static_cast<int>(device_info.model)
                  << "\nFirmware: "
                  << (device_info.firmware_version >> 8)
                  << "."
                  << (device_info.firmware_version & 0xFF)
                  << "\nHardware: "
                  << static_cast<int>(device_info.hardware_version)
                  << "\nSerial: ";

        for (unsigned char byte : device_info.serialnum)
        {
            std::cout << std::hex
                      << std::setw(2)
                      << std::setfill('0')
                      << static_cast<int>(byte);
        }

        std::cout << std::dec << "\n";
    }

    sl_lidar_response_device_health_t health {};
    result = lidar->getHealth(health);

    if (SL_IS_FAIL(result))
    {
        std::cerr << "Unable to read LiDAR health\n";
        delete lidar;
        delete channel;
        return 1;
    }

    std::cout << "Health status: "
              << static_cast<int>(health.status)
              << "\n";

    if (health.status == SL_LIDAR_STATUS_ERROR)
    {
        std::cerr << "LiDAR reports an internal error. Code: "
                  << health.error_code
                  << "\n";
        delete lidar;
        delete channel;
        return 1;
    }
    
    //Reset Before start
    
    lidar->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    lidar->reset();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    result = lidar->startScan(false, true);

    if (SL_IS_FAIL(result))
    {
        std::cerr << "startScan failed: 0x"
                  << std::hex << result << std::dec << "\n";
        return 1;
    }

    result = lidar->startScan(false, true);

    if (SL_IS_FAIL(result))
    {
        std::cerr << "Unable to start scanning. Error: 0x"
                  << std::hex << result << std::dec << "\n";
        delete lidar;
        delete channel;
        return 1;
    }

    constexpr std::size_t MAX_NODES = 8192;
    auto* nodes =
        new sl_lidar_response_measurement_node_hq_t[MAX_NODES];

    std::cout
        << "Scanning...\n"
        << "angle_deg,distance_m,quality,x_m,y_m\n";

    while (keep_running)
    {
        std::size_t count = MAX_NODES;

        result = lidar->grabScanDataHq(nodes, count, 5000);
        
        if (result == SL_RESULT_OPERATION_TIMEOUT)
        {
            std::cerr << "Scan timeout: no complete scan received\n";
            continue;
        }

        if (SL_IS_FAIL(result))
        {
            std::cerr << "Scan failed: 0x"
                      << std::hex << result << std::dec << "\n";
            continue;
        }

        lidar->ascendScanData(nodes, count);

        for (std::size_t i = 0; i < count; ++i)
        {
            const float distance_mm = get_distance_mm(nodes[i]);

            if (distance_mm <= 0.0f)
            {
                continue;
            }

            const float angle_deg = get_angle_deg(nodes[i]);
            const float distance_m = distance_mm / 1000.0f;
            const float angle_rad =
                angle_deg * static_cast<float>(M_PI) / 180.0f;

            const float x_m =
                distance_m * std::cos(angle_rad);

            const float y_m =
                distance_m * std::sin(angle_rad);

            const int quality =
                static_cast<int>(nodes[i].quality >> 2);

            std::cout
                << std::fixed << std::setprecision(3)
                << angle_deg << ","
                << distance_m << ","
                << quality << ","
                << x_m << ","
                << y_m << "\n";
        }

        std::cout.flush();
    }

    std::cout << "\nStopping RPLIDAR...\n";

    lidar->stop();

    delete[] nodes;
    delete lidar;
    delete channel;

    return 0;
}
