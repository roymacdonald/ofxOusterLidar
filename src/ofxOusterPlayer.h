//
//  ofxOusterPlayer.hpp
//  example
//
//  Created by Roy Macdonald on 6/13/22.
//

#pragma once

#pragma once
#include "ofMain.h"

#include "ouster/client.h"
#include "ouster/netcompat.h"
#include "ouster/types.h"
#include "ouster/lidar_scan.h"

#include "ouster/os_pcap.h"
#include "ofxOusterIMU.h"

enum PlaybackDataType{
    PLAYBACK_NONE = 0,
    PLAYBACK_LIDAR = 1,
    PLAYBACK_IMU = 2
};
class ofxOusterPlayer{
public:
    ofxOusterPlayer(){}
    ~ofxOusterPlayer();
    /// Loads a .pcap file. that you can either save using this addon or Ouster Studio
    //
    bool load(const std::string& pcapDataFile, const std::string& jsonConfigFile, uint16_t lidar_port, uint16_t imu_port);
    
    
    /// closes a file being read. You only need to call this after reading a file but it will be called automatically in the destructor
    void closeFile();
    
    PlaybackDataType getNextScan();
    
    ofEvent<ouster::LidarScan> lidarDataEvent;
    ofEvent<ofxOusterIMUData> imuDataEvent;
    
    
    ouster::LidarScan ls_write;
    ofxOusterIMUData imuData;
    
    
    // when true it will respect the packages timestamp, when false it will not and run as fast as possible
    bool bRunRealtime = true;
    
    /// Playback Control
    void play();
    void pause();
    void stop();
    void nextFrame();
    void firstFrame();
    
    bool isPlaying(){return bIsPlaying;}
    
    const ouster::sensor::sensor_info& getSensorInfo() {return sensorInfo;}
    
    
protected:
    
    uint64_t timestampDiff = 0;
    uint64_t updatesDiff = 0;
    uint64_t lastLidarData = 0;
    uint64_t lidarDataDiff = 0;
    uint64_t frameCount = 0;
    
    
    std::string _dataFile = "";
    std::string _configFile = "";
    
    void _update(ofEventArgs&);
    
    std::unique_ptr<ouster::ScanBatcher> _batchScan = nullptr;
    
    std::vector<uint8_t> lidar_buf;
    std::vector<uint8_t> imu_buf;
    
    std::shared_ptr<ouster::sensor_utils::playback_handle> playbackHandle = nullptr;
    uint64_t lasttimestamp = 0;
    uint64_t lastUpdateMicros = 0;
    ofEventListener _updateListener;
    
    bool bIsPlaying = false;
    ouster::sensor::sensor_info sensorInfo;
    
    ouster::sensor_utils::packet_info packet_info;
};
