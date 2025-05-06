#include "rplidar.h"

// 声明
sl_lidar_response_measurement_node_hq_t nodes[8192];
float getAngle(const sl_lidar_response_measurement_node_hq_t& node);

float getAngle(const sl_lidar_response_measurement_node_hq_t& node){
    return node.angle_z_q14 * 90.f / 16384.f;
}

rplidar::rplidar(std::string serial_port){
    printf("Hello rplidar\n");
    drv = *createLidarDriver();
    if (nullptr == drv) {
        /* don't start spinning without a driver object */
        std::cout << "Failed to construct driver" << std::endl; 
    }

    IChannel* _channel;
    _channel = *createSerialPortChannel(serial_port, serial_baudrate);

    if (SL_IS_FAIL((drv)->connect(_channel))){
        printf("Error, cannot bind to the specified serial port %s.\n",serial_port.c_str());
        delete drv; drv = nullptr;
    }

    if (!getRPLIDARDeviceInfo()) {
        delete drv; drv = nullptr;
        printf("Error, cannot getRPLIDARDeviceInfo.\n");
    }

    // check health...
    if (!checkRPLIDARHealth()) {
        printf("Error, cannot checkRPLIDARHealth.\n");
        delete drv; drv = nullptr;
    }

    sl_lidar_response_device_info_t devinfo;
    op_result = drv->getDeviceInfo(devinfo);

    if( (devinfo.model>>4) > LIDAR_S_SERIES_MINUM_MAJOR_ID){
        scan_frequency_tunning_after_scan = true;
    }
    
    if(!scan_frequency_tunning_after_scan){ //for RPLIDAR A serials
        //start RPLIDAR A serials  rotate by pwm
        drv->setMotorSpeed(600);
    }

    /* start motor and scanning */
    if (!start()) {
        printf("Error, cannot start.\n");
        delete drv; drv = nullptr;
    }

    this->count = _countof(nodes);
}

std::tuple<std::vector<float>, std::vector<float>> rplidar::get_data(){
    std::vector<float> range_data;
    std::vector<float> intensity_data;

    auto start = std::chrono::high_resolution_clock::now();
    op_result = drv->grabScanDataHq(nodes, count);
    auto end = std::chrono::high_resolution_clock::now();
    this->scan_duration = seconds_duration(end-start).count();
    // printf("scan_duration: %.2f\n",this->scan_duration);

    if (op_result == SL_RESULT_OK) {
        // Init...
        if(scan_frequency_tunning_after_scan) { // Set scan frequency(For Slamtec Tof lidar)
            printf("set lidar scan frequency to %.1f Hz(%.1f Rpm) ", scan_frequency, scan_frequency*60);
            drv->setMotorSpeed(scan_frequency*60); //rpm 
            scan_frequency_tunning_after_scan = false;
            return std::make_tuple(std::vector<float>{}, std::vector<float>{});
        }

        op_result = drv->ascendScanData(nodes, count);
        angle_min = DEG2RAD(0.0f);
        angle_max = DEG2RAD(359.0f);
        if (op_result == SL_RESULT_OK) {
            if(angle_compensate){
                angle_compensate_nodes_count = 360*angle_compensate_multiple;
                int angle_compensate_offset = 0;
                auto angle_compensate_nodes = new sl_lidar_response_measurement_node_hq_t[angle_compensate_nodes_count];
                memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(sl_lidar_response_measurement_node_hq_t));

                size_t i = 0, j = 0;
                for( ; i < count; i++ ) {
                    if (nodes[i].dist_mm_q2 != 0) {
                        float angle = getAngle(nodes[i]);
                        int angle_value = (int)(angle * angle_compensate_multiple);
                        if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                        for (j = 0; j < angle_compensate_multiple; j++) {
                            int angle_compensate_nodes_index = angle_value-angle_compensate_offset + j;
                            if(angle_compensate_nodes_index >= angle_compensate_nodes_count)
                                angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
                            angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                        }
                    }
                }
                
                // Initialize range_data with the correct size
                range_data.resize(angle_compensate_nodes_count);
                intensity_data.resize(angle_compensate_nodes_count);

                // send data
                bool reversed = (angle_max > angle_min);
                bool reverse_data = (!inverted && reversed) || (inverted && !reversed);

                size_t scan_midpoint = angle_compensate_nodes_count / 2;
                for (size_t i = 0; i < angle_compensate_nodes_count; i++) {
                    float read_value = (float)angle_compensate_nodes[i].dist_mm_q2 / 4.0f / 1000;
                    size_t apply_index = i;
                    if (reverse_data) {
                        apply_index = angle_compensate_nodes_count - 1 - i;
                    }
                    if (flip_x_axis) {
                        if (apply_index >= scan_midpoint)
                            apply_index = apply_index - scan_midpoint;
                        else
                            apply_index = apply_index + scan_midpoint;
                    }
        
                    if (read_value == 0.0)
                        range_data[apply_index] = std::numeric_limits<float>::infinity();
                    else
                        range_data[apply_index] = read_value;
                    intensity_data[apply_index] = (float)(angle_compensate_nodes[apply_index].quality >> 2);
                }

                if (angle_compensate_nodes) {
                    delete[] angle_compensate_nodes;
                    angle_compensate_nodes = nullptr;
                }

                return std::make_tuple(range_data, intensity_data);
            }
        }
    }
    return std::make_tuple(std::vector<float>{}, std::vector<float>{});
}

std::tuple<float, float> rplidar::get_angle(){
    return std::make_tuple(this->angle_max, this->angle_min);
}

float rplidar::get_max_distance(){
    return this->max_distance;
}

float rplidar::get_nodes_count(){
    return this->angle_compensate_nodes_count;
}

float rplidar::get_scan_duration(){
    return this->scan_duration;
}


bool rplidar::set_scan_mode(){
    
    sl_result     op_result;
    LidarScanMode current_scan_mode;
    if (this->scan_mode.empty()) {
        op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    }
    else {
        std::vector<LidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

        if (SL_IS_OK(op_result)) {
            sl_u16 selectedScanMode = sl_u16(-1);
            for (std::vector<LidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == this->scan_mode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == sl_u16(-1)) {
                printf("scan mode `%s' is not supported by lidar, supported modes:\n", this->scan_mode.c_str());
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

bool rplidar::checkRPLIDARHealth(){
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

bool rplidar::getRPLIDARDeviceInfo(){
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

void rplidar::stop(){
    if (nullptr == drv) {
        return;
    }

    printf("RPLidar Stop\n");
    drv->stop();
    drv->setMotorSpeed(0);
    is_scanning = false;
}

bool rplidar::start(){
    if (nullptr == drv) {
        return false;
    }

    printf("RPLidar Start\n");
    drv->setMotorSpeed();
    if (!set_scan_mode()) {
        stop();
        printf("Failed to set scan mode\n");
        return false;
    }
    is_scanning = true;
    return true;
}

rplidar::~rplidar(){
    printf("Exit\n");
}