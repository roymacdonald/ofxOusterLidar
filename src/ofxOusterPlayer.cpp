//
//  ofxOusterPlayer.cpp
//  example
//
//  Created by Roy Macdonald on 6/13/22.
//

#include "ofxOusterPlayer.h"


ofxOusterPlayer::~ofxOusterPlayer(){
    closeFile();
}


bool ofxOusterPlayer::load(const std::string& dataFile, const std::string& configFile){
    if(dataFile.empty() || configFile.empty()){
        ofLogError("ofxOusterPlayer") << "Can't load. One of the filenames is empty";
        return false;
    }
    if(!ofFile::doesFileExist(dataFile)){
        ofLogError("ofxOusterPlayer") << "Can't load. Data file at " << dataFile << "  does not exist!";
        return false;
    }
    
    if(!ofFile::doesFileExist(configFile)){
        ofLogError("ofxOusterPlayer") << "Can't load. Configuration file at " << configFile << "  does not exist!";
        return false;
    }
    
    _dataFile = dataFile;
    _configFile = configFile;
    
    playbackHandle = ouster::sensor_utils::replay_initialize(dataFile);
    if(playbackHandle){
    sensorInfo = ouster::sensor::parse_metadata(ofBufferFromFile(configFile).getText());
    
    
    ofLogVerbose("ofxOusterPlayer") << "Using lidar_mode: " << ouster::sensor::to_string(sensorInfo.mode);
    ofLogVerbose("ofxOusterPlayer") << sensorInfo.prod_line << " sn: " << sensorInfo.sn << " firmware rev: " << sensorInfo.fw_rev ;
    
    lasttimestamp = 0;
    

    if(sensorInfo.udp_port_lidar  == 0){
        sensorInfo.udp_port_lidar = 7502;
        ofLogVerbose("ofxOusterPlayer::load") << "using default lidar port ";
    }
    if(sensorInfo.udp_port_imu  == 0){
        sensorInfo.udp_port_imu = 7503;
        ofLogVerbose("ofxOusterPlayer::load") << "using default imu port ";
    }
    
    auto packetFormat = ouster::sensor::get_format(sensorInfo);
    
    
    lidar_buf.resize(packetFormat.lidar_packet_size + 1);
    imu_buf.resize(packetFormat.imu_packet_size + 1);
    
    
    ls_write = std::move(ouster::LidarScan (sensorInfo.format.columns_per_frame, sensorInfo.format.pixels_per_column, sensorInfo.format.udp_profile_lidar));
    
    _batchScan = make_unique<ouster::ScanBatcher>(sensorInfo);
        return true;
    }
    return false;
}

void ofxOusterPlayer::play(){
    if(!bIsPlaying){
        bIsPlaying = true;
        _updateListener = ofEvents().update.newListener(this, &ofxOusterPlayer::_update);
    }
}
void ofxOusterPlayer::pause(){
    if(bIsPlaying){
        bIsPlaying = false;
        _updateListener.unsubscribe();
    }
}
void ofxOusterPlayer::stop(){
    pause();
    closeFile();
    load(_dataFile, _configFile);
}
void ofxOusterPlayer::nextFrame(){
    if(!bIsPlaying){
        getNextScan();
    }
}
void ofxOusterPlayer::closeFile(){
    if(playbackHandle != nullptr){
        ouster::sensor_utils::replay_uninitialize(*playbackHandle);
        lasttimestamp = 0;
        pause();
        playbackHandle = nullptr;
    }
}


void ofxOusterPlayer::_update(ofEventArgs&){
    auto d = getNextScan();
    if(d == PlaybackDataType::NONE){
        pause();
    }
}

            
PlaybackDataType ofxOusterPlayer::getNextScan(){
    if(playbackHandle != nullptr){
        
        ouster::sensor_utils::packet_info packet_info;
        auto packetFormat = ouster::sensor::get_format(sensorInfo);
        
        while   (ouster::sensor_utils::next_packet_info(*playbackHandle,  packet_info)) {
            auto micros = ofGetElapsedTimeMicros();
            if(bRunRealtime){
//                startTimeMicros = micros;
                if(lasttimestamp != 0 ){
                    auto timestampDiff = packet_info.timestamp.count() - lasttimestamp;
                    auto updatesDiff = micros - lastUpdateMicros;
                    bool bNeedsSleep = true;
                    if(updatesDiff >= timestampDiff){
                        bNeedsSleep = false;
                    }
                    if(bNeedsSleep){
                        ofSleepMillis(floor((timestampDiff - updatesDiff)/1000.0f));
                    }
                    
                }else{
                    
                    lastUpdateMicros = ofGetElapsedTimeMicros();
                }
            }
            if(packet_info.dst_port == sensorInfo.udp_port_lidar) {
                auto packet_size = ouster::sensor_utils::read_packet(*playbackHandle, lidar_buf.data(), packetFormat.lidar_packet_size);
                if (packet_size == packetFormat.lidar_packet_size){
                    if ((*_batchScan)(lidar_buf.data(), ls_write)) {
                        ofNotifyEvent(lidarDataEvent, ls_write,this);
                        return PlaybackDataType::LIDAR;
                    }
                }else{
                    ofLogWarning("ofxOusterPlayer::getNextScan") << " reading PCAP file. Wrong lidar packet size. expecting: " << packetFormat.lidar_packet_size << "  received: " << packet_size;
                }
            }else if(packet_info.dst_port == sensorInfo.udp_port_imu) {
                auto packet_size = ouster::sensor_utils::read_packet(*playbackHandle, imu_buf.data(), packetFormat.imu_packet_size);
                if (packet_size == packetFormat.imu_packet_size){
                    imuData.set(packetFormat, imu_buf);
                    ofNotifyEvent(imuDataEvent, imuData, this);
                    return PlaybackDataType::IMU;
                }else{
                    ofLogWarning("ofxOusterPlayer::getNextScan") << " reading PCAP file. wrong imu packet size. expecting: " << packetFormat.lidar_packet_size << "  received: " << packet_size ;
                }
            }else{
                ofLogWarning("ofxOusterPlayer::getNextScan") <<  " reading PCAP file. Unknown dest port " << packet_info.dst_port << " lidar port: " << sensorInfo.udp_port_lidar << " imu port: " << sensorInfo.udp_port_imu ;
            }
            
            lasttimestamp = packet_info.timestamp.count();
        }
        
        ofLogNotice("ofxOusterPlayer::getNextScan") << "No more packets\n";
        
        return PlaybackDataType::NONE;
        
    }else{
        
    }
}


void ofxOusterPlayer::exportImuData(const string& filepath, std::shared_ptr<ouster::sensor_utils::playback_handle> playbackHandle, ouster::sensor::sensor_info &sensorInfo){
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

void ofxOusterPlayer::exportScanToCSV(const string& filepath, std::shared_ptr<ouster::sensor_utils::playback_handle> playbackHandle, ouster::sensor::sensor_info &sensorInfo){
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
