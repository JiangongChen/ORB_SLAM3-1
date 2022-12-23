//
// Created by jjc7191 on 12/22/2022.
//

#ifndef IMUDATA_H
#define IMUDATA_H
#include <vector>

class IMUData {
public:
    long ts_;
    std::vector<float> gyro_;
    std::vector<float> acce_;

    IMUData(long ts, std::vector<float>& gyro, std::vector<float>& acce) {
        ts_ = ts;
        gyro_ = gyro;
        acce_ = acce;
    }
};

#endif //IMUDATA_H
