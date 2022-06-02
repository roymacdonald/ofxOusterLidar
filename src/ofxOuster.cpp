#include "ofxOuster.hpp"





namespace sensor = ouster::sensor;
ofxOuster::ofxOuster()
{
	_initValues();
    
}

ofxOuster::~ofxOuster()
{
	ofThread::waitForThread(true, 30000);
    closeFile();
}



ofxOuster::ofxOuster(const std::string& hostname_, int lidar_port_ , int imu_port_)
{
	_initValues();
	setup(hostname_, lidar_port_, imu_port_);
}


void ofxOuster::setup(const std::string& hostname_, int lidar_port_, int imu_port_)
{
    if(!_bisSetup){
	_bUseSimpleSetup = true;
	hostname = hostname_;
	lidar_port = lidar_port_;
	imu_port = imu_port_;
	_bisSetup = true;
	_updateListener = ofEvents().update.newListener(this, &ofxOuster::_update);
    }
}

ofxOuster::ofxOuster(const std::string& hostname_,
					 const std::string& udp_dest_host_,
					 ouster::sensor::lidar_mode mode_,
					 ouster::sensor::timestamp_mode ts_mode_,
					 int lidar_port_, int imu_port_,
					 int timeout_sec_)
{
    
	_initValues();
	setup(hostname_, udp_dest_host_, mode_, ts_mode_, lidar_port_, imu_port_, timeout_sec_);
    
}

void ofxOuster::setup(const std::string& hostname_,
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
        _updateListener = ofEvents().update.newListener(this, &ofxOuster::_update);
        closeFile();
    }
}

bool ofxOuster::_initClient()
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


void ofxOuster::_ingestLidarData(ouster::ScanBatcher& scanBatcher, std::vector<uint8_t>& buffer ,ouster::LidarScan& scan){
    if (scanBatcher(buffer.data(), scan)) {
        lidarScanChannel.send(scan);
    }
}

void ofxOuster::_ingestImuData(ouster::sensor::packet_format & pf,  std::vector<uint8_t>& buffer){
    ofxOusterIMUData imu_data(pf, buffer);
    updateImuFusion(imu_data);
    imuChannel.send(imuFusion);
}


void ofxOuster::threadedFunction(){
	if(ofThread::isThreadRunning())
	{
		if(_bisSetup )
		{
			if(!_clientInited && playbackHandle == nullptr){
				if(_initClient() == false)
				{
					ofLogError("ofxOuster::threadedFunction")<< "Client init failed. Stopping thread";
					stopThread();
				}
			}
			if(_clientInited || (playbackHandle != nullptr)){
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
                    if(playbackHandle != nullptr){
//                        cout <<",xmjxsjm\n";
                        ouster::sensor_utils::packet_info packet_info;
                        
                        if (ouster::sensor_utils::next_packet_info(*playbackHandle, packet_info)) {
                            auto micros = ofGetElapsedTimeMicros();
//                            cout << (packet_info.timestamp.count() - lasttimestamp) << "  " << (micros - startTimeMicros)<< endl;
                            startTimeMicros = micros;
                            if(lasttimestamp != 0){
                                
                                ofSleepMillis(floor((packet_info.timestamp.count() - lasttimestamp)/1000.0f));
                                
                            }else{
                                startTimeMicros = ofGetElapsedTimeMicros();
                            }
                            if(packet_info.dst_port == sensorInfo.udp_port_lidar) {
                                auto packet_size = ouster::sensor_utils::read_packet(*playbackHandle, lidar_buf.data(), packetFormat.lidar_packet_size);
                                if (packet_size == packetFormat.lidar_packet_size){
                                    _ingestLidarData(_batchScan, lidar_buf,ls_write);
                                }else{
                                    cout << "wrong lidar packet size. expecting: " << packetFormat.lidar_packet_size << "  received: " << packet_size << endl;
                                }
                            }else if(packet_info.dst_port == sensorInfo.udp_port_imu) {
                                auto packet_size = ouster::sensor_utils::read_packet(*playbackHandle, imu_buf.data(), packetFormat.imu_packet_size);
                                if (packet_size == packetFormat.imu_packet_size){
                                    _ingestImuData(packetFormat, imu_buf);
                                }else{
                                    cout << "wrong imu packet size. expecting: " << packetFormat.lidar_packet_size << "  received: " << packet_size << endl;
                                }
                            }else{
                                cout << "unknown dest port " << packet_info.dst_port << " lidar port: " << sensorInfo.udp_port_lidar << " imu port: " << sensorInfo.udp_port_imu << endl;
                            }
                            
                            lasttimestamp = packet_info.timestamp.count();
                        }else{
                            cout << "No more packets\n";
                        }
                        
                    }else{
                        sensor::client_state st = sensor::poll_client(*cli);
                        if (st & sensor::client_state::CLIENT_ERROR) {
                            ofLogError("ofxOuster::threadedFunction")<< "Client error. Closing thread";
                            stopThread();
                            break;
                        }
                        
                        if (st & sensor::client_state::LIDAR_DATA) {
                            if (sensor::read_lidar_packet(*cli, lidar_buf.data(),packetFormat)) {
                                _ingestLidarData(_batchScan, lidar_buf , ls_write);
                            }else
                            {
                                ofLogWarning("ofxOuster::threadedFunction") << "wrong packet size";
                            }
                        }
                        if (st & sensor::client_state::IMU_DATA) {
                            if(sensor::read_imu_packet(*cli, imu_buf.data(), packetFormat)){
                                _ingestImuData(packetFormat, imu_buf);
                            }
                        }
                        if (st & sensor::EXIT) {
                            stopThread();
                            break;
                            
                        }
                    }
				}
			}
        }else{
            cout <<"not setup\n";
        }
	}
}
void ofxOuster::_initRenderer()
{
	if(!_renderer)
	{
		auto H = sensorInfo.format.pixels_per_column;
		auto W = sensorInfo.format.columns_per_frame;
		
		_renderer = make_unique<ofxOusterRenderer>(sensorInfo, "Renderer");
		
		auto xyz_lut = make_unique<ouster::XYZLut>(ouster::make_xyz_lut(sensorInfo));
		_renderer->setCloud(
							xyz_lut->direction.data(),
							xyz_lut->offset.data(),
							H * W,
							W,
							{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
        
        _renderer->setGuiPosition(_guiPos);
	}
}

void ofxOuster::_update(ofEventArgs&)
{
    if(_clientInited || (playbackHandle != nullptr)){
        if(_bisSetup && !_renderer){
            _initRenderer();
        }
        bool bNewData = false;
        while(lidarScanChannel.tryReceive(_readScan))
        {
            bNewData = true;
            // The lidarScanChannel is an ofThreadChannel which behaves as a safe queue between threads.
            // We call tryReceive inside a while loop in order to empty the queue and get the newest data, in case that one thread is running significantly slower that the other.
        }
        if(bNewData && _renderer)
        {
            _renderer->render(_readScan);
        }
        
        bNewData = false;
        ofxOusterIMUFusion imuData;
        while(imuChannel.tryReceive(imuData))
        {
            bNewData = true;
        }
        if(bNewData)
        {
            
            
//            imuFrames.push_back(imuData);
//            cout << imuData;
            currentPos = imuData.position;
            
            
        }
    }
}


void ofxOuster::draw(ofEasyCam & cam)
{
	if(_renderer)
	{
		_renderer->draw(cam, node.getGlobalTransformMatrix());
	}
}


void ofxOuster::drawGui()
{
	
	if(_renderer)
	{
		_renderer->drawGui();
	}
	
}



const string& ofxOuster::getMetadata() const
{
	return metadata;
}

const ouster::sensor::sensor_info& ofxOuster::getSensorInfo() const
{
	return sensorInfo;
}


ouster::sensor::lidar_mode ofxOuster::getLidarMode()
{
	return mode;
}


ouster::sensor::timestamp_mode ofxOuster::getTimestampMode()
{
	return ts_mode;
}


int ofxOuster::getLidarPort()
{
	return lidar_port;
}


int ofxOuster::getImuPort()
{
	return imu_port;
}


int ofxOuster::getTimeout()
{
	return timeout_sec;
}

void ofxOuster::setMetadata(const string& _metadata){
    
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

void ofxOuster::_initValues()
{
	mode  = ouster::sensor::MODE_UNSPEC;
	ts_mode  = ouster::sensor::TIME_FROM_UNSPEC;
	lidar_port = 0;
	imu_port = 0;
	timeout_sec = 30;
	_bisSetup = false;

    /// initialize IMU fusion
    FusionAhrsInitialise(&ahrs);
	_clientInited = false;
	
}

void ofxOuster::setGuiPosition(const glm::vec2& pos){
    if(_renderer)
    {
        _renderer->setGuiPosition(pos);
    }else{
        _guiPos = pos;
    }
}

ofxOusterRenderer* ofxOuster::getRenderer(){
    if(_renderer){
        return _renderer.get();
    }else{
        return nullptr;
    }
}


bool ofxOuster::updateImuFusion(ofxOusterIMUData & data){
    if(lastImuTimestamp > 0){
        const FusionVector gyroscope = {data.gyro.x,data.gyro.y,data.gyro.z};
        const FusionVector accelerometer = {data.accel.x,data.accel.y,data.accel.z};
        
        imuFusion.deltaTime = (data.sys_timestamp - lastImuTimestamp)/1000000000.0f;
        
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, imuFusion.deltaTime);//timestamp from IMU is in nanoseconds and Fusion requires it to be in seconds
        
            auto fquat = FusionAhrsGetQuaternion(&ahrs);
//        auto feuler = FusionQuaternionToEuler(fquat);
        auto fearth = FusionAhrsGetEarthAcceleration(&ahrs);
//        auto linear = FusionAhrsGetLinearAcceleration(&ahrs);
        imuFusion.quat = *reinterpret_cast<glm::quat*>(&fquat);
//        imuFusion.euler = *reinterpret_cast<glm::vec3*>(&feuler);
        imuFusion.updatePosition(*reinterpret_cast<glm::vec3*>(&fearth));
//        imuFusion.updatePosition(*reinterpret_cast<glm::vec3*>(&linear));
        
//        imuFusion.euler += imuFusion.deltaTime * data.gyro;
        
//        gyroscope
        
//        cout<< "earth: " << fearth.axis.x << ", "<< fearth.axis.y << ", "<< fearth.axis.z << endl;
        
//        cout<< "linear: " << linear.axis.x << ", "<< linear.axis.y << ", "<< linear.axis.z << endl;
        
//        node.setPosition(imuFusion.position);//.x, imuFusion.position.y, 0.0f);
        // node.setOrientation(imuFusion.quat);
//        node.setOrientation(imuFusion.euler);
        
        return true;
    }
    lastImuTimestamp = data.sys_timestamp ;
    
    
    return false;
}




void ofxOuster::load(const std::string& dataFile, const std::string& configFile){
    
    playbackHandle = ouster::sensor_utils::replay_initialize(dataFile);
    
    setMetadata(ofBufferFromFile(configFile).getText());

    if(sensorInfo.udp_port_lidar  == 0){
        sensorInfo.udp_port_lidar = 7502;
        ofLogVerbose("ofxOuster::load") << "using default lidar port ";
    }
    if(sensorInfo.udp_port_imu  == 0){
        sensorInfo.udp_port_imu = 7503;
        ofLogVerbose("ofxOuster::load") << "using default imu port ";
    }
    
    _bisSetup = true;
    _updateListener = ofEvents().update.newListener(this, &ofxOuster::_update);
    startThread();
    
    

    
}
void ofxOuster::closeFile(){
    if(playbackHandle != nullptr){
        ouster::sensor_utils::replay_uninitialize(*playbackHandle);
    }
}

void exportImuData(const string& filepath, std::shared_ptr<ouster::sensor_utils::playback_handle> playbackHandle, ouster::sensor::sensor_info &sensorInfo){
    auto packetFormat = sensor::get_format(sensorInfo);
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

void exportScanToCSV(const string& filepath, std::shared_ptr<ouster::sensor_utils::playback_handle> playbackHandle, ouster::sensor::sensor_info &sensorInfo){
    auto packetFormat = sensor::get_format(sensorInfo);
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
                auto range = scan.field(sensor::ChanField::RANGE);
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

