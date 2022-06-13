//
//  ofxOusterClient.cpp
//  example
//
//  Created by Roy Macdonald on 6/13/22.
//

#include "ofxOusterClient.h"


using namespace ouster;

ofxOusterClient::ofxOusterClient(){
    _initValues();
}

ofxOusterClient::~ofxOusterClient(){
    ofThread::waitForThread(true, 30000);
}

ofxOusterClient::ofxOusterClient(const std::string& hostname_, int lidar_port_ , int imu_port_)
{
    _initValues();
    setup(hostname_, lidar_port_, imu_port_);
}


void ofxOusterClient::setup(const std::string& hostname_, int lidar_port_, int imu_port_)
{
    if(!_bisSetup){
        _bUseSimpleSetup = true;
        hostname = hostname_;
        lidar_port = lidar_port_;
        imu_port = imu_port_;
        _bisSetup = true;
//    _updateListener = ofEvents().update.newListener(this, &ofxOuster::_update);
    }
}

ofxOusterClient::ofxOusterClient(const std::string& hostname_,
                     const std::string& udp_dest_host_,
                     ouster::sensor::lidar_mode mode_,
                     ouster::sensor::timestamp_mode ts_mode_,
                     int lidar_port_, int imu_port_,
                     int timeout_sec_)
{
    
    _initValues();
    setup(hostname_, udp_dest_host_, mode_, ts_mode_, lidar_port_, imu_port_, timeout_sec_);
    
}

void ofxOusterClient::setup(const std::string& hostname_,
                      const std::string& udp_dest_host_,
                      ouster::sensor::lidar_mode mode_,
                      ouster::sensor::timestamp_mode ts_mode_,
                      int lidar_port_, int imu_port_,
                      int timeout_sec_)
{
    if(!_bisSetup){
        _bUseSimpleSetup = false;
        hostname = hostname_;
        udp_dest_host = udp_dest_host_;
        mode = mode_;
        ts_mode = ts_mode_;
        lidar_port = lidar_port_;
        imu_port = imu_port_;
        timeout_sec = timeout_sec_;
        _bisSetup = true;
//        _updateListener = ofEvents().update.newListener(this, &ofxOuster::_update);
//        closeFile();
    }
}



bool ofxOusterClient::_initClient()
{
    if(_bisSetup && !cli && !_clientInited)
    {
        
        string _hostname;
        string _udp_dest_host;
        {
            std::lock_guard<ofMutex> lock(hostMutex);
            _hostname = hostname;
            _udp_dest_host = udp_dest_host;
        }
        if(_bUseSimpleSetup)
        {
            cli = sensor::init_client(_hostname, lidar_port, imu_port);
        }
        else
        {
            cli = sensor::init_client(_hostname, _udp_dest_host, mode, ts_mode, lidar_port, imu_port, timeout_sec);
        }
        if (!cli) {
            ofLogError("ofxOuster") << "Failed to connect to client at: " << _hostname << std::endl;
            return false;
        }
        
        
        setMetadata(sensor::get_metadata(*cli));
        
        
        _clientInited = true;
        return true;
    }
    return false;
}


void ofxOusterClient::threadedFunction(){
    if(ofThread::isThreadRunning())
    {
        if(_bisSetup )
        {
            if(!_clientInited){
                if(_initClient() == false)
                {
                    ofLogError("ofxOuster::threadedFunction")<< "Client init failed. Stopping thread";
                    stopThread();
                }
            }
            if(_clientInited){
                ouster::sensor::sensor_info _sensorInfo;
                {
                    std::lock_guard<ofMutex> lock(metadataMutex);
                    _sensorInfo  = sensorInfo;
                }
                
                uint32_t H = _sensorInfo.format.pixels_per_column;
                uint32_t W = _sensorInfo.format.columns_per_frame;
                auto packetFormat = sensor::get_format(_sensorInfo);
                
                
                std::vector<uint8_t> lidar_buf(packetFormat.lidar_packet_size + 1);
                std::vector<uint8_t> imu_buf(packetFormat.imu_packet_size + 1);
                
                
                ouster::LidarScan ls_write (W, H, sensorInfo.format.udp_profile_lidar);
                
                auto _batchScan = ouster::ScanBatcher(W, packetFormat);
                uint64_t lasttimestamp = 0;
                uint64_t startTimeMicros = 0;
                while (isThreadRunning()) {
                        sensor::client_state st = sensor::poll_client(*cli);
                        if (st & sensor::client_state::CLIENT_ERROR) {
                            ofLogError("ofxOuster::threadedFunction")<< "Client error. Closing thread";
                            stopThread();
                            break;
                        }
                        
                        if (st & sensor::client_state::LIDAR_DATA) {
                            if (sensor::read_lidar_packet(*cli, lidar_buf.data(),packetFormat)) {
                                if (_batchScan(lidar_buf.data(), ls_write)) {
                                      lidarScanChannel.send(ls_write);
                                  }
                            }else
                            {
                                ofLogWarning("ofxOuster::threadedFunction") << "wrong packet size";
                            }
                        }
                        if (st & sensor::client_state::IMU_DATA) {
                            if(sensor::read_imu_packet(*cli, imu_buf.data(), packetFormat)){
                                ofxOusterIMUData imu_data(packetFormat, imu_buf);
                                imuChannel.send(imu_data);
                            }
                        }
                        if (st & sensor::EXIT) {
                            stopThread();
                            break;
                            
                        }
                }
            }
        }else{
            cout <<"not setup\n";
        }
    }
}

void ofxOusterClient::setMetadata(const string& _metadata){
    
    sensor::sensor_info _sensorInfo = sensor::parse_metadata(_metadata);
    
    
    ofLogVerbose("ofxOuster") << "Using lidar_mode: " << sensor::to_string(_sensorInfo.mode);
    ofLogVerbose("ofxOuster") << _sensorInfo.prod_line << " sn: " << _sensorInfo.sn << " firmware rev: " << _sensorInfo.fw_rev ;
    
    
    {
        /// unnamed scope for locking mutex and automatically unlocking upon scope end
        std::lock_guard<ofMutex> lock(metadataMutex);
        metadata = _metadata;
        sensorInfo = _sensorInfo;
    }
    
}

void ofxOusterClient::_initValues()
{
    mode  = ouster::sensor::MODE_UNSPEC;
    ts_mode  = ouster::sensor::TIME_FROM_UNSPEC;
    lidar_port = 0;
    imu_port = 0;
    timeout_sec = 30;
    _bisSetup = false;
    /// initialize IMU fusion
//    FusionAhrsInitialise(&ahrs);
    _clientInited = false;    
}

bool ofxOusterClient::isSetup(){
    return _bisSetup;
}
bool ofxOusterClient::isInited(){
    return _clientInited;
}
const ouster::sensor::sensor_info& ofxOusterClient::getSensorInfo() {
    return sensorInfo;
}