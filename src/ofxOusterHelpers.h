//
//  Header.h
//  LidarSlam
//
//  Created by Roy Macdonald on 21-06-22.
//

#pragma once
#include "ouster/client.h"
#include "ouster/netcompat.h"
#include "ouster/types.h"
#include "ouster/lidar_scan.h"

#include "ouster/os_pcap.h"
#include "ofxOusterIMU.h"

namespace ofxOuster {


static void exportImuData(const string& filepath, std::shared_ptr<ouster::sensor_utils::playback_handle> playbackHandle, ouster::sensor::sensor_info &sensorInfo){
    auto packetFormat = ouster::sensor::get_format(sensorInfo);
    std::vector<uint8_t> imu_buf(packetFormat.imu_packet_size + 1);
    ouster::sensor_utils::packet_info packet_info;
    ofBuffer out_buffer;
    while (ouster::sensor_utils::next_packet_info(*playbackHandle, packet_info)) {
        if(packet_info.dst_port == sensorInfo.udp_port_imu) {
            auto packet_size = ouster::sensor_utils::read_packet(*playbackHandle, imu_buf.data(), packetFormat.imu_packet_size);
            if (packet_size == packetFormat.imu_packet_size){
                ofxOusterIMUData imu_data(packetFormat, imu_buf);
                stringstream ss;
                ss << setprecision(17);
                ss<< std::fixed;
                ss << imu_data.sys_timestamp << ",";
                ss << imu_data.accel_timestamp << ",";
                ss << imu_data.gyro_timestamp << ",";
                ss << imu_data.accel << ",";
                ss << imu_data.gyro << "\n";
                out_buffer.append(ss.str());
            }
        }
    }
    ofBufferToFile(filepath, out_buffer);
    
}

static void exportScanToCSV(const string& filepath, std::shared_ptr<ouster::sensor_utils::playback_handle> playbackHandle, ouster::sensor::sensor_info &sensorInfo){
    auto packetFormat = ouster::sensor::get_format(sensorInfo);
    ouster::sensor_utils::packet_info packet_info;
    std::vector<uint8_t> lidar_buf(packetFormat.lidar_packet_size + 1);
    
    ouster::ScanBatcher batch_to_scan(sensorInfo.format.columns_per_frame, packetFormat);
    
    size_t w = sensorInfo.format.columns_per_frame;
    size_t h = sensorInfo.format.pixels_per_column;
    
    auto scan = ouster::LidarScan(w, h, sensorInfo.format.udp_profile_lidar);
    ouster::XYZLut lut = ouster::make_xyz_lut(sensorInfo);
    
    while (ouster::sensor_utils::next_packet_info(*playbackHandle, packet_info)) {
        
        
        auto packet_size = ouster::sensor_utils::read_packet(*playbackHandle, lidar_buf.data(), packetFormat.lidar_packet_size);
        
        if (packet_size == packetFormat.lidar_packet_size && packet_info.dst_port == sensorInfo.udp_port_lidar) {
            if (batch_to_scan(lidar_buf.data(), scan)) {
                auto range = scan.field(ouster::sensor::ChanField::RANGE);
                auto cloud = ouster::cartesian(range, lut);
                
                ofBuffer out_buffer;
                
                stringstream ss;
                for(size_t i = 0; i < cloud.rows(); i++){
                    ss << cloud(i, 0) << ", " << cloud(i, 1) << ", " << cloud(i, 2) << std::endl;
                }
                out_buffer.append(ss.str());
                ofBufferToFile(filepath + ofToString(scan.frame_id)+".csv", out_buffer);
            }
        }
    }
}

}//namespace ofxOuster
