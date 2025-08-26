#include <laser2d.h>

RadarToGridMapConverter::RadarToGridMapConverter(float r,int w,int h,int s,float a)
{
    search_params.angle_resolution=a;
    search_params.map_resolution=r;
    search_params.step_size=s;
    search_params.map_width=w; 
    search_params.map_height=h;
    // 当前pose和之前的pose
    _pose_.x = _pose_.y = _pose_.z = 0.0;
    _pose_.roll = _pose_.pitch = _pose_.yaw = 0.0;
    grid_map.assign(search_params.map_width * search_params.map_height, 0);
    // cache for fast conversions
    inv_map_resolution = 1.0f / search_params.map_resolution;
    center_x = search_params.map_width / 2;
    center_y = search_params.map_height / 2;
    buildKernel();
}

RadarToGridMapConverter::~RadarToGridMapConverter(){} 

void RadarToGridMapConverter::radarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg_data)
{   
    // 初始化变换对象
    //search_params.map_resolution_x = search_params.map_resolution*search_params.step_size;
    // 调整步长以进行更精细的匹配
    //while(search_params.map_resolution_x > search_params.map_resolution/2)
    //{
        matching_angle(msg_data);

        while(fraction2 > fraction1)
        {   
            _pose_.x = _pose.x;
            _pose_.y = _pose.y;
            _pose_.yaw = _pose.yaw;
        
            fraction1 = fraction2;
            matching_angle(msg_data);
        }
    //    search_params.map_resolution_x = search_params.map_resolution_x/2;
    //}
}

void RadarToGridMapConverter::setInputSource(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    fraction1=0;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        float range = msg->ranges[i];
        float angle = msg->angle_min + i * msg->angle_increment+_pose_.yaw;

        if (std::isfinite(range))
        {
            int x, y;
            polarToGrid(range, angle, _pose_, x, y);
            if (x >= 0 && x < search_params.map_width && y >= 0 && y < search_params.map_height)
            {
                gatGridValue(x, y, fraction1);
            }
        }
    }
    radarCallback(msg);
}

void RadarToGridMapConverter::matching_angle(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int fraction_angle;
    double yaw_angle;
    int maxn=0;//用来记最大值 
    for(int i = -6 ;i<7;i++)
    {  
        yaw_angle = _pose_.yaw +(i*search_params.angle_resolution);

        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                fraction_angle = 0;
                if(dx !=0 || dy !=0)
                {
                    for (size_t j = 0; j < msg->ranges.size(); ++j)
                    {
                        float range = msg->ranges[j];
                        float angle = msg->angle_min + j * msg->angle_increment+yaw_angle;

                        if (std::isfinite(range))
                        {
                            int x, y;
                            polarToGrid(range, angle, _pose_, x, y);
                            // apply sub-cell offset in world scaled to cells
                            int off_x = static_cast<int>((dx * search_params.map_resolution) * inv_map_resolution);
                            int off_y = static_cast<int>((dy * search_params.map_resolution) * inv_map_resolution);
                            x += off_x;
                            y += off_y;
                            if (x >= 0 && x < search_params.map_width && y >= 0 && y < search_params.map_height)
                            {
                                gatGridValue(x, y, fraction_angle);
                            }
                        }
                    }
                    if(fraction_angle > maxn)
                    {
                        _pose.x = _pose_.x+(dx*search_params.map_resolution);
                        _pose.y = _pose_.y+(dy*search_params.map_resolution);
                        _pose.yaw = yaw_angle;
                        maxn = fraction_angle;
                    }
                }
            }
        }
    }
    fraction2 = maxn;
}

void RadarToGridMapConverter::setInputTarget(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        float range = msg->ranges[i];
        if (!std::isfinite(range)) continue;

        float angle = msg->angle_min + i * msg->angle_increment + _pose_.yaw;
        int x, y;
        polarToGrid(range, angle, _pose_, x, y);
        if (!inBounds(x, y)) continue;

        int center_index = idx(x, y);
        if (grid_map[center_index] != 100)
        {
            setGridValue(x, y, 100,grid_map); // Set the original laser point to 100

            // 预裁剪邻域边界，避免循环内频繁判断
            int min_dx = std::max(-kernel_radius, -x);
            int max_dx = std::min( kernel_radius, search_params.map_width - 1 - x);
            int min_dy = std::max(-kernel_radius, -y);
            int max_dy = std::min( kernel_radius, search_params.map_height - 1 - y);

            for (int dy = min_dy; dy <= max_dy; ++dy)
            {
                int ny = y + dy;
                int row_base = ny * search_params.map_width;
                for (int dx = min_dx; dx <= max_dx; ++dx)
                {
                    if (dx == 0 && dy == 0) continue; // center already set
                    int nx = x + dx;
                    int index = nx + row_base;
                    if (grid_map[index] == 100) continue;

                    int value = kernelAt(dx, dy);
                    if (value > grid_map[index])
                    {
                        grid_map[index] = value;
                    }
                }
            }
        }
    }
}


point RadarToGridMapConverter::getpose()
{
    return _pose_;
}

void RadarToGridMapConverter::align(point pose_guess)
{
    _pose_ = pose_guess;
}


void RadarToGridMapConverter::buildKernel()
{
    kernel_diameter = kernel_radius * 2 + 1;
    kernel_values.assign(kernel_diameter * kernel_diameter, 0);

    const double R = static_cast<double>(kernel_radius);
    for (int dy = -kernel_radius; dy <= kernel_radius; ++dy)
    {
        for (int dx = -kernel_radius; dx <= kernel_radius; ++dx)
        {
            double d = std::sqrt(static_cast<double>(dx * dx + dy * dy));
            double p = std::max(0.0, 1.0 - d / R);
            int value = static_cast<int>(p * 100.0);
            kernel_values[(dx + kernel_radius) * kernel_diameter + (dy + kernel_radius)] = value;
        }
    }
    // ensure center is 100 in kernel, though we set it explicitly in code
    kernel_values[kernel_radius * kernel_diameter + kernel_radius] = 100;
}