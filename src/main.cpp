#include <iostream> 
#include "sl_lidar.h"
using namespace sl;

bool is_scanning = false;
bool auto_standby = false;
std::string scan_mode = "Standard";
float scan_frequency = 10;  // 采集数据的频率HZ
float max_distance = 8.0;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

size_t angle_compensate_multiple = 1; // it stand of angle compensate at per 1 degree
enum {
    LIDAR_A_SERIES_MINUM_MAJOR_ID   = 0,
    LIDAR_S_SERIES_MINUM_MAJOR_ID   = 5,
    LIDAR_T_SERIES_MINUM_MAJOR_ID   = 8,
};

bool set_scan_mode(ILidarDriver * drv) {
    sl_result     op_result;
    LidarScanMode current_scan_mode;
    if (scan_mode.empty()) {
        op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    }
    else {
        std::vector<LidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

        if (SL_IS_OK(op_result)) {
            sl_u16 selectedScanMode = sl_u16(-1);
            for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == scan_mode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == sl_u16(-1)) {
                printf("scan mode `%s' is not supported by lidar, supported modes:\n", scan_mode.c_str());
                for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    printf("\t%s: max_distance: %.1f m, Point number: %.1fK\n", iter->scan_mode,
                        iter->max_distance, (1000 / iter->us_per_sample));
                }
                op_result = SL_RESULT_OPERATION_FAIL;
            }
            else {
                op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
            }
        }
    }

    if (SL_IS_OK(op_result))
    {
        //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
        int points_per_circle = (int)(1000 * 1000 / current_scan_mode.us_per_sample / scan_frequency);
        angle_compensate_multiple = points_per_circle / 360.0 + 1;
        if (angle_compensate_multiple < 1)
            angle_compensate_multiple = 1.0;
        max_distance = (float)current_scan_mode.max_distance;
        printf("current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz\n",
            current_scan_mode.scan_mode, (int)(1000 / current_scan_mode.us_per_sample + 0.5), max_distance, scan_frequency);
        return true;
    }
    else
    {
        printf("Can not start scan: %08x!", op_result);
        return false;
    }
}

void stop(ILidarDriver * drv)
{
    if (nullptr == drv) {
        return;
    }

    printf("RPLidar Stop");
    drv->stop();
    drv->setMotorSpeed(0);
    is_scanning = false;
}

bool start(ILidarDriver * drv)
{
    if (nullptr == drv) {
        return false;
    }

    printf("RPLidar Start\n");
    drv->setMotorSpeed();
    if (!set_scan_mode(drv)) {
        stop(drv);
        printf("Failed to set scan mode\n");
        return false;
    }
    is_scanning = true;
    return true;
}

bool checkRPLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;
    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { 
        printf("RPLidar health status : %d\n", healthinfo.status);
        switch (healthinfo.status) {
            case SL_LIDAR_STATUS_OK:
                printf("RPLidar health status : OK.\n");
                return true;
            case SL_LIDAR_STATUS_WARNING:
                printf("RPLidar health status : Warning.\n");
                return true;
            case SL_LIDAR_STATUS_ERROR:
                printf("Error, RPLidar internal error detected. Please reboot the device to retry.\n");
                return false;
            default:
                printf("Error, Unknown internal error detected. Please reboot the device to retry.\n");
                return false;
        }
    } else {
        printf("Error, cannot retrieve RPLidar health code: %x\n", op_result);
        return false;
    }
}

bool getRPLIDARDeviceInfo(ILidarDriver * drv){
    sl_result     op_result;
    sl_lidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);

    if (SL_IS_FAIL(op_result)) {
        if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
            printf("Error, operation time out. SL_RESULT_OPERATION_TIMEOUT!\n");
        } else {
            printf("Error, unexpected error, code: %x \n",op_result);
        }
        return false;
    }

    char sn_str[37] = {'\0'}; 
    for (int pos = 0; pos < 16 ;++pos) {
        sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
    }
    printf("RPLidar S/N: %s\n",sn_str);
    printf("Firmware Ver: %d.%02d\n",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    printf("Hardware Rev: %d\n",(int)devinfo.hardware_version);
    return true;
}

int main() 
{
    ILidarDriver * drv = nullptr;
    drv = *createLidarDriver();
    if (nullptr == drv) {
        /* don't start spinning without a driver object */
        std::cout << "Failed to construct driver" << std::endl; 
        return -1;
    }

    std::string serial_port = "/dev/ttyUSB0";
    int serial_baudrate = 460800;
    IChannel* _channel;
    _channel = *createSerialPortChannel(serial_port, serial_baudrate);

    if (SL_IS_FAIL((drv)->connect(_channel))){
        printf("Error, cannot bind to the specified serial port %s.\n",serial_port.c_str());
        delete drv; drv = nullptr;
        return -1;
    }

    if (!getRPLIDARDeviceInfo(drv)) {
        delete drv; drv = nullptr;
        return -1;
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        delete drv; drv = nullptr;
        return -1;
    }

    sl_lidar_response_device_info_t devinfo;
    sl_result     op_result;
    op_result = drv->getDeviceInfo(devinfo);
    bool scan_frequency_tunning_after_scan = false;

    if( (devinfo.model>>4) > LIDAR_S_SERIES_MINUM_MAJOR_ID){
        scan_frequency_tunning_after_scan = true;
    }
    
    if(!scan_frequency_tunning_after_scan){ //for RPLIDAR A serials
        //start RPLIDAR A serials  rotate by pwm
        drv->setMotorSpeed(600);
    }

    /* start motor and scanning */
    if (!auto_standby && !start(drv)) {
        delete drv; drv = nullptr;
        return -1;
    }
    
    while (true){
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);
        
    }


    std::cout << "Hello, World!" << std::endl; 
    return 0; 
}