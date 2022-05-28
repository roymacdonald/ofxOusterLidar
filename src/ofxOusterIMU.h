//
//  ofxOusterIMU.h
//  example
//
//  Created by admin on 2022-05-26.
//

#pragma once
#include "ofVectorMath.h"
#include "ouster/types.h"

class ofxOusterIMUData{
public:
    glm::vec3 accel;
    glm::vec3 gyro;
    
    uint64_t sys_timestamp;
    uint64_t accel_timestamp;
    uint64_t gyro_timestamp;
    
    ofxOusterIMUData(){}
    
    ofxOusterIMUData(ouster::sensor::packet_format& format, std::vector<uint8_t>& buffer){
        auto * buff = buffer.data();
        sys_timestamp = format.imu_sys_ts(buff);
        accel_timestamp = format.imu_accel_ts(buff);
        gyro_timestamp = format.imu_gyro_ts(buff);
        accel.x = format.imu_la_x(buff);
        accel.y = format.imu_la_y(buff);
        accel.z = format.imu_la_z(buff);
        gyro.x = format.imu_av_x(buff);
        gyro.y = format.imu_av_y(buff);
        gyro.z = format.imu_av_z(buff);

    }
    

    // IMU packet accessors

    friend std::ostream& operator<<(std::ostream& os, const ofxOusterIMUData & d);
    
};

class ofxOusterIMUFusion{
public:
    glm::quat quat;
    glm::vec3 euler;
    glm::vec3 earthAccel;
    glm::vec3 position = {0,0,0};
    glm::vec3 vel = {0,0,0};
    float deltaTime;
    
    void updatePosition(const glm::vec3 & acc){
//        speed += earthAccel*deltaTime;
//        position += speed * deltaTime;
        auto & dt = deltaTime;
        position += vel*dt + earthAccel*(dt*dt*0.5);
        vel += (acc+earthAccel)*(dt*0.5);
        earthAccel = acc;
        
    }
    
};


