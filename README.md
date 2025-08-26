# 软件名：lidar_matching

--------

## 版本号：2.0.0 (ROS2版本)
- 日期：2025年8月
- 负责人：赵俊
- 说明：ROS2版本的二维点云栅格地图匹配

--------

## 版本号：1.1.3 (ROS1版本)
- 日期：2024年5月15日
- 负责人：赵俊
- 说明：加入了点云匹配和建图的配置

--------

## 版本号：1.1.2
- 日期：2024年5月10日
- 负责人：赵俊
- 说明：laser2d.cpp点云匹配，2d_mapping.cpp建图dome

--------

## 版本号：1.1.1
- 日期：2024年5月08日
- 负责人：赵俊
- 说明：二维点云栅格地图匹配

--------

## 版本号：1.1.0
- 日期：2025年5月3日
- 负责人：赵俊
- 说明：1.1.0

--------

## ROS2 使用说明

### 构建
```bash
# 在ROS2工作空间的src目录下
colcon build --packages-select lidar_matching
```

### 运行
```bash
# 方法1：使用launch文件
ros2 launch lidar_matching lidar_matching.launch.py

# 方法2：直接运行节点
ros2 run lidar_matching 2d_mapping
```

### 参数配置
参数配置文件位于 `params/lidar_2d.yaml`，主要参数包括：
- `map_resolution`: 地图分辨率 (默认: 0.03)
- `map_width`: 地图宽度 (默认: 800)
- `map_height`: 地图高度 (默认: 800)
- `min_add_scan_shift`: 最小扫描位移 (默认: 1.0)
- `min_add_scan_angle`: 最小扫描角度 (默认: 0.5)

### 话题
- 订阅: `/scan` (sensor_msgs/LaserScan)
- 发布: 
  - `/current_pose` (nav_msgs/Odometry)
  - `/grid_map` (nav_msgs/OccupancyGrid)

### TF变换
- 发布从 `map` 到 `laser_link` 的变换
