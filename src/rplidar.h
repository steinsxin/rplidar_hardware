#include <iostream> 
#include <chrono>
#include <math.h>
#include <cstring>
#include "sl_lidar.h"

using namespace sl;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

enum {
    LIDAR_A_SERIES_MINUM_MAJOR_ID   = 0,
    LIDAR_S_SERIES_MINUM_MAJOR_ID   = 5,
    LIDAR_T_SERIES_MINUM_MAJOR_ID   = 8,
};

using seconds_duration = std::chrono::duration<double>;                     //秒
using milliseconds_duration = std::chrono::duration<double,std::milli>;     //毫秒
using microseconds_duration = std::chrono::duration<double,std::micro>;     //微秒
using chrono_time = decltype(std::chrono::high_resolution_clock::now());    //当前时间 decltype()获取返回变量类型

class rplidar{
    public:
        rplidar(std::string serial_port);
        ~rplidar();
        bool set_scan_mode();
        bool checkRPLIDARHealth();
        bool getRPLIDARDeviceInfo();
        void stop();
        bool start();
        std::tuple<std::vector<float>, std::vector<float>> get_data();
        std::tuple<float, float> get_angle();
        float get_max_distance();
        float get_nodes_count();
        float get_scan_duration();
    private:
        ILidarDriver * drv = nullptr;
        size_t angle_compensate_multiple = 1;                               // it stand of angle compensate at per 1 dreege
        bool scan_frequency_tunning_after_scan = false;
        bool is_scanning = false;
        bool auto_standby = false;
        std::string scan_mode = "Standard";
        std::string frame_id = "laser_frame";
        float scan_frequency = 10;  // 采集数据的频率HZ
        int serial_baudrate = 460800;
        bool angle_compensate = true;
        float scan_duration;
        sl_result     op_result;

        size_t count;
        int angle_compensate_nodes_count;
        float max_distance = 8.0;
        bool inverted = false;
        bool flip_x_axis = false;

        float angle_min;
        float angle_max;
};