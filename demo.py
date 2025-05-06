import time
from build.RPLidar import RPLidar

# 定义圆周率常量
M_PI = 3.141593

print("RPLidar module loaded successfully!")
lidar = RPLidar("/dev/ttyUSB0")

# liadr的频率是10HZ
while True:
    # 获取数据
    range_data, intensity_data = lidar.get_data()
    scan_time = lidar.get_scan_duration()
    angle_max, angle_min = lidar.get_angle()
    max_distance = lidar.get_max_distance()
    node_count = lidar.get_nodes_count()

    # 格式化打印数据
    if range_data and intensity_data:
        print(f"Scan Time: {scan_time:.2f} seconds")
        print(f"Angle Range: {angle_min:.2f} to {angle_max:.2f} degrees")
        print(f"Max Distance: {max_distance:.2f} meters")
        print(f"Node Count: {node_count}")

        # 打印前 5 个范围数据和强度数据
        print("Range Data (first 5):", ", ".join(f"{r:.2f} m" for r in range_data[:5]))
        print("Intensity Data (first 5):", ", ".join(f"{i:.2f}" for i in intensity_data[:5]))

    time.sleep(0.1)