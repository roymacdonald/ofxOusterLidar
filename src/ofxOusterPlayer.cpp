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


bool ofxOusterPlayer::load(const std::string& dataFile, const std::string& configFile, uint16_t lidar_port, uint16_t imu_port){
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
    frameCount = 0;

    if(sensorInfo.udp_port_lidar  == 0 || lidar_port != 7502){
        sensorInfo.udp_port_lidar = lidar_port;
        ofLogVerbose("ofxOusterPlayer::load") << "using default lidar port ";
    }
    if(sensorInfo.udp_port_imu  == 0 || imu_port != 7503){
        sensorInfo.udp_port_imu = imu_port;
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
    load(_dataFile, _configFile, sensorInfo.udp_port_lidar, sensorInfo.udp_port_imu);
}
void ofxOusterPlayer::nextFrame(){
    if(!bIsPlaying){
        getNextScan();
    }
}
void ofxOusterPlayer::firstFrame(){
    if(playbackHandle != nullptr){
        ouster::sensor_utils::replay_reset(*playbackHandle);
        lasttimestamp = 0;
        frameCount = 0;
        lastUpdateMicros = 0;
    }
}

void ofxOusterPlayer::closeFile(){
    if(playbackHandle != nullptr){
        ouster::sensor_utils::replay_uninitialize(*playbackHandle);
        lasttimestamp = 0;
        frameCount = 0;
        lastUpdateMicros = 0;
        pause();
        playbackHandle = nullptr;
    }
}


void ofxOusterPlayer::_update(ofEventArgs&){
    auto d = getNextScan();
    if(d == PlaybackDataType::PLAYBACK_NONE){
        pause();
    }
}

            
PlaybackDataType ofxOusterPlayer::getNextScan(){
    if(playbackHandle != nullptr){
        
        
        auto packetFormat = ouster::sensor::get_format(sensorInfo);
        
        while   (ouster::sensor_utils::next_packet_info(*playbackHandle,  packet_info)) {
            auto micros = ofGetElapsedTimeMicros();
            if(bRunRealtime){
//                startTimeMicros = micros;
                if(lasttimestamp != 0 ){
                    timestampDiff = packet_info.timestamp.count() - lasttimestamp;
                    updatesDiff = micros - lastUpdateMicros;
                    bool bNeedsSleep = true;
                    if(updatesDiff >= timestampDiff){
                        bNeedsSleep = false;
                    }
                    if(bNeedsSleep){
                        ofSleepMillis(floor((timestampDiff - updatesDiff)/1000.0f));
                    }
                    
//                }else{
//                    playbackFps = updatesDiff
                   
                }
                lastUpdateMicros = micros;//ofGetElapsedTimeMicros();
                lasttimestamp = packet_info.timestamp.count();
            }
            if(packet_info.dst_port == sensorInfo.udp_port_lidar) {
                auto packet_size = ouster::sensor_utils::read_packet(*playbackHandle, lidar_buf.data(), packetFormat.lidar_packet_size);
                if (packet_size == packetFormat.lidar_packet_size){
                    if ((*_batchScan)(lidar_buf.data(), ls_write)) {
                        auto micros = ofGetElapsedTimeMicros();
                        lidarDataDiff = micros -lastLidarData;
                        lastLidarData = micros;
                        frameCount ++;
                        ofNotifyEvent(lidarDataEvent, ls_write,this);
                        return PlaybackDataType::PLAYBACK_LIDAR;
                    }
                }else{
                    ofLogWarning("ofxOusterPlayer::getNextScan") << " reading PCAP file. Wrong lidar packet size. expecting: " << packetFormat.lidar_packet_size << "  received: " << packet_size;
                }
            }else if(packet_info.dst_port == sensorInfo.udp_port_imu) {
                auto packet_size = ouster::sensor_utils::read_packet(*playbackHandle, imu_buf.data(), packetFormat.imu_packet_size);
                if (packet_size == packetFormat.imu_packet_size){
                    imuData.set(packetFormat, imu_buf);
                    ofNotifyEvent(imuDataEvent, imuData, this);
                    return PlaybackDataType::PLAYBACK_IMU;
                }else{
                    ofLogWarning("ofxOusterPlayer::getNextScan") << " reading PCAP file. wrong imu packet size. expecting: " << packetFormat.lidar_packet_size << "  received: " << packet_size ;
                }
            }else{
                ofLogWarning("ofxOusterPlayer::getNextScan") <<  " reading PCAP file. Unknown dest port " << packet_info.dst_port << " lidar port: " << sensorInfo.udp_port_lidar << " imu port: " << sensorInfo.udp_port_imu ;
            }
            
            
        }
        
        ofLogNotice("ofxOusterPlayer::getNextScan") << "No more packets\n";
        
        return PlaybackDataType::PLAYBACK_NONE;
        
//    }else{
        
    }
    return PlaybackDataType::PLAYBACK_NONE;
}


