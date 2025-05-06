#include <iostream> 
#include <vector>
#include "rplidar.h"

int main() 
{
    rplidar test("/dev/ttyUSB0");
    while (true) {
        // 拿到返回的两个 vector
        auto data = test.get_data();
        std::vector<float> range_data = std::get<0>(data);
        std::vector<float> intensity_data = std::get<1>(data);
        
        // 打印前 5 个元素作为示例
        if (!range_data.empty()) {
            std::cout << "Range data (first 5 values): ";
            for (size_t i = 0; i < std::min(range_data.size(), size_t(5)); ++i) {
                std::cout << range_data[i] << " ";
            }
            std::cout << std::endl;

            std::cout << "Intensity data (first 5 values): ";
            for (size_t i = 0; i < std::min(intensity_data.size(), size_t(5)); ++i) {
                std::cout << intensity_data[i] << " ";
            }
            std::cout << std::endl;
        }
    }

    return 0; 
}