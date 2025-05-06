# RPLidar Hardware

## 文件结构

```
.
├── build
├── CMakeLists.txt
├── demo.py
├── ros_demo.py
├── sdk
└── src
```

- `demo.py`：单纯的激光雷达数据获取示例程序。
- `ros_demo.py`：获取并处理数据的 ROS 示例程序。

------



## 编译与安装

### 编译步骤

1. 创建并进入 `build` 目录：

   ```
   mkdir build && cd build
   ```

2. 运行 CMake 并编译：

   ```
   cmake ..
   make
   ```

------



## 测试与运行

### 测试环境

确保已经激活了 ROS 2 环境：

```
conda activate ros2
```

### 运行示例程序

运行 ROS 示例程序：

```
python ros_demo.py
```

### 检查数据发布

在终端中输入以下命令，查看 `/scan_data` 话题的数据

```
ros2 topic echo /scan_data
```

------



## RViz 可视化

### 启动 RViz

在终端中输入：

```
rviz2
```

### 设置 Fixed Frame

1. 在左上角找到 **Global Options**（全局选项）
2. 点击 `Fixed Frame`
3. 将其值改为 `laser`（必须和你发布的 `scan_msg.header.frame_id` 一致）
4. 回车确认

### 添加 LaserScan 显示项

1. 在左下角点击 **“Add”** 按钮
2. 在弹出的对话框中选择：
   - `By display type` → `LaserScan`
   - 点击 **“OK”**
3. 在左侧新出现的 `LaserScan` 显示项中：
   - 设置 **Topic** 为 `/scan_data`（如果你发布的是这个话题）

如果你正确设置了 `Fixed Frame`，且程序正在持续发布数据，RViz 就会显示激光点图

------

## 注意事项

- 确保激光雷达设备已正确连接到 `/dev/ttyUSB0`
- 如果使用其他端口，请修改 `RPLidar` 的初始化参数
- 如果在 RViz 中没有看到数据，请检查 `/scan_data` 话题是否正在发布数据

