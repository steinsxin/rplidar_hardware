from build.RPLidar import RPLidar
M_PI= 3.141593
print("RPLidar module loaded successfully!")
lidar = RPLidar("/dev/ttyUSB0")

while True:
    range_data, intensity_data = lidar.get_data()  # 正确接收返回值

    if range_data:  
        print("Range Data (first 5):", range_data[:5])
        print("Intensity Data (first 5):", intensity_data[:5])
