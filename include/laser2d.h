#ifndef LASER2D
#define LASER2D

#include <iostream>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Eigenvalues>

// ROS2 includes
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

struct point{double x,y,z;double roll,pitch,yaw;};

struct SearchParams {
    float angle_resolution;
    float map_resolution;
    float map_resolution_x;
    int step_size;
    int map_width, map_height;
};

class RadarToGridMapConverter
{
public:
   RadarToGridMapConverter(float r,int w,int h,int s,float a);
  ~RadarToGridMapConverter();  

    void setInputTarget(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void radarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg_data);
    void setInputSource(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void align(point pose_guess);
    point getpose();
    void setGridValue(int x, int y, int value ,std::vector<int8_t>& grid_value)
    {
        if (x >= 0 && x < search_params.map_width && y >= 0 && y < search_params.map_height)
        {
            grid_value[x + y * search_params.map_width] = value;
        }
    }
private:

    sensor_msgs::msg::LaserScan::SharedPtr copied_scan;

    SearchParams search_params;

    // Cached parameters for faster coordinate conversion
    float inv_map_resolution{1.0f};
    int center_x{0};
    int center_y{0};

    int fraction1,fraction2;

    std::vector<int8_t> grid_map;
    point _pose_;
    point _pose;

    void matching_angle(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // --- Helpers and precomputed kernel ---
    inline bool inBounds(int x, int y) const
    {
        return x >= 0 && x < search_params.map_width && y >= 0 && y < search_params.map_height;
    }

    inline int idx(int x, int y) const
    {
        return x + y * search_params.map_width;
    }

    // Convert polar measurement (range, angle) and pose to grid coordinates
    inline void polarToGrid(float range, float angle, const point& pose, int& x, int& y) const
    {
        float wx = range * std::cos(angle) + static_cast<float>(pose.x);
        float wy = range * std::sin(angle) + static_cast<float>(pose.y);
        x = static_cast<int>(wx * inv_map_resolution) + center_x;
        y = static_cast<int>(wy * inv_map_resolution) + center_y;
    }

    void buildKernel();

    inline int kernelAt(int dx, int dy) const
    {
        return kernel_values[(dx + kernel_radius) * kernel_diameter + (dy + kernel_radius)];
    }

    int kernel_radius{8};
    int kernel_diameter{17}; // 2*radius + 1
    std::vector<int> kernel_values; // precomputed [0..100]

    
    void gatGridValue(int x, int y,int& fraction)
    {
        if (x >= 0 && x < search_params.map_width && y >= 0 && y < search_params.map_height)
        {
            fraction = fraction+grid_map[x + y * search_params.map_width];
        }
    }

    tf2::Transform createTransform(float x, float y, float z, float roll, float pitch, float yaw)
    {
        tf2::Transform transform_;
        tf2::Quaternion q;
        transform_.setOrigin(tf2::Vector3(x,y,z));
        q.setRPY(roll,pitch,yaw); //q from rpy
        transform_.setRotation(q);//trans from q

        return transform_;
    }
};

#endif // LASER2D
