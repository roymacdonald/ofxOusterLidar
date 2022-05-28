//
//  ofxOusterIMU.cpp
//  example
//
//  Created by admin on 2022-05-26.
//

#include "ofxOusterIMU.h"
std::ostream& operator<<(std::ostream& os, const ofxOusterIMUData & d) {
    
    os << "sys_timestamp: " << d.sys_timestamp << std::endl;
    os << "accel_timestamp: " << d.accel_timestamp << std::endl;
    os << "gyro_timestamp: " << d.gyro_timestamp << std::endl;
    os << "accel: " << d.accel << std::endl;
    os << "gyro: " << d.gyro << std::endl;
    
    return os;
}
