from rplidar import RPLidar
import time

PORT = "/dev/ttyUSB0"

lidar = RPLidar(PORT)

try:
    print("Starting RPLIDAR...")

    info = lidar.get_info()
    print("Info:", info)

    health = lidar.get_health()
    print("Health:", health)

    print("Scanning...")

    for scan in lidar.iter_scans():

        print("Number of measurements:", len(scan))

        for quality, angle, distance_mm in scan[:10]:

            distance_m = distance_mm / 1000.0

            print(
                f"angle={angle:.1f} "
                f"distance={distance_m:.2f} m "
                f"quality={quality}"
            )

        print("------------------------")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    lidar.stop()
    lidar.disconnect()

