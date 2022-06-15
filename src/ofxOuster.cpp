#include "ofxOuster.hpp"





namespace sensor = ouster::sensor;
ofxOuster::ofxOuster()
{
}

ofxOuster::~ofxOuster()
{

}




void ofxOuster::connect(const std::string& hostname_, int lidar_port_, int imu_port_)
{
    closeFile();
    _client = make_unique<ofxOusterClient>(hostname_, lidar_port_,imu_port_);
    _listeners.unsubscribeAll();
    _listeners.push(ofEvents().update.newListener(this, &ofxOuster::_update));
    
    
}


void ofxOuster::connect(const std::string& hostname_,
					  const std::string& udp_dest_host_,
					  ouster::sensor::lidar_mode mode_,
					  ouster::sensor::timestamp_mode ts_mode_,
					  int lidar_port_, int imu_port_,
					  int timeout_sec_)
{
    closeFile();
    _client = make_unique<ofxOusterClient>(hostname_, udp_dest_host_, mode_, ts_mode_, lidar_port_,imu_port_,timeout_sec_);
    _listeners.unsubscribeAll();
    _listeners.push(ofEvents().update.newListener(this, &ofxOuster::_update));
    
}

void ofxOuster::_initRenderer()
{
	if(!_renderer)
	{
        ouster::sensor::sensor_info sensorInfo;
        if(_client){
            sensorInfo = _client->getSensorInfo();
        }else if(_player){
            sensorInfo = _player->getSensorInfo();
        }else{
            ofLogError("ofxOuster::_initRenderer") << "can not init renderer whith no client or player";
        }
		_renderer = make_unique<ofxOusterRenderer>(sensorInfo);

        _renderer->setGuiPosition(_guiPos);
	}
}

void ofxOuster::_update(ofEventArgs&)
{
    
    if(_client && _client->isInited()){
        if(_client->isSetup() && !_renderer){
            _initRenderer();
        }
        bool bNewData = false;
        while(_client->lidarScanChannel.tryReceive(_readScan))
        {
            bNewData = true;
            // The lidarScanChannel is an ofThreadChannel which behaves as a safe queue between threads.
            // We call tryReceive inside a while loop in order to empty the queue and get the newest data, in case that one thread is running significantly slower that the other.
        }
        if(bNewData && _renderer)
        {
            _renderer->render(_readScan);
            ofNotifyEvent(lidarDataEvent, _readScan, this);
        }
        ofxOusterIMUData imuData;
        while(_client->imuChannel.tryReceive(imuData))
        {
            ofNotifyEvent(imuDataEvent, imuData, this);
        }
    }
}

void ofxOuster::drawPointCloud()
{
    if(_renderer)
    {
        _renderer->drawPointCloud();
    }
}



void ofxOuster::draw(ofEasyCam & cam)
{
	if(_renderer)
	{
        _renderer->draw(cam);
	}
//    if(_player){
//        stringstream ss;
//        ss << "timestampDiff: " << _player->timestampDiff << "\n";
//        ss << "updatesDiff: " << _player->updatesDiff << "\n";
//        ss << "FPS: " << (1000000.0/(float)_player->updatesDiff)<< "\n";
//
//
//        ss << "lidarDataDiff: " << _player->lidarDataDiff << "\n";
//        ss <<"fps: " << (1000000.0/(float)_player->lidarDataDiff) << "\n";
//        ss << "FrameCount: " << _player->frameCount << "\n";
//
//        ofDrawBitmapStringHighlight(ss.str(), ofGetWidth()/2, 30);
//
//    }
}


void ofxOuster::drawGui()
{
	
	if(_renderer)
	{
		_renderer->drawGui();
	}
	
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

//
//bool ofxOuster::updateImuFusion(ofxOusterIMUData & data){
//    if(lastImuTimestamp > 0){
//        const FusionVector gyroscope = {data.gyro.x,data.gyro.y,data.gyro.z};
//        const FusionVector accelerometer = {data.accel.x,data.accel.y,data.accel.z};
//
//        imuFusion.deltaTime = (data.sys_timestamp - lastImuTimestamp)/1000000000.0f;
//
//        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, imuFusion.deltaTime);//timestamp from IMU is in nanoseconds and Fusion requires it to be in seconds
//
//        auto fquat = FusionAhrsGetQuaternion(&ahrs);
////        auto feuler = FusionQuaternionToEuler(fquat);
//        auto fearth = FusionAhrsGetEarthAcceleration(&ahrs);
////        auto linear = FusionAhrsGetLinearAcceleration(&ahrs);
//        imuFusion.quat = *reinterpret_cast<glm::quat*>(&fquat);
////        imuFusion.euler = *reinterpret_cast<glm::vec3*>(&feuler);
//        imuFusion.updatePosition(*reinterpret_cast<glm::vec3*>(&fearth));
////        imuFusion.updatePosition(*reinterpret_cast<glm::vec3*>(&linear));
//
////        imuFusion.euler += imuFusion.deltaTime * data.gyro;
//
////        gyroscope
//
////        cout<< "earth: " << fearth.axis.x << ", "<< fearth.axis.y << ", "<< fearth.axis.z << endl;
//
////        cout<< "linear: " << linear.axis.x << ", "<< linear.axis.y << ", "<< linear.axis.z << endl;
//
////        node.setPosition(imuFusion.position);//.x, imuFusion.position.y, 0.0f);
//        // node.setOrientation(imuFusion.quat);
////        node.setOrientation(imuFusion.euler);
//
//        lastImuTimestamp = data.sys_timestamp ;
//        return true;
//    }
//    lastImuTimestamp = data.sys_timestamp ;
//
//
//    return false;
//}




void ofxOuster::load(const std::string& dataFile, const std::string& configFile){
    
    closeFile();
    if(!_player){
        _player = make_unique<ofxOusterPlayer>();
    }
    
    if(_player->load(dataFile, configFile)){
        _listeners.push(_player->lidarDataEvent.newListener(this, &ofxOuster::onLidarData));
        _listeners.push(_player->imuDataEvent.newListener(this, &ofxOuster::onImuData));
        _player->play();
        if(!_renderer){
            _initRenderer();
        }
    }
}
void ofxOuster::closeFile(){
    if(_player){
        _player->closeFile();
    }
    _listeners.unsubscribeAll();
}
void ofxOuster::onLidarData(ouster::LidarScan& scan){
    _readScan = scan;
    if(_renderer)_renderer->render(_readScan);
    ofNotifyEvent(lidarDataEvent, _readScan, this);
}
void ofxOuster::onImuData(ofxOusterIMUData& data){
    ofNotifyEvent(imuDataEvent, data, this);
}
const ouster::sensor::sensor_info& ofxOuster::getSensorInfo() const{
    if(_client){
        return _client->getSensorInfo();
    }else if(_player){
        return _player->getSensorInfo();
    }
    
    ofLogError("ofxOuster::getSensorInfo()") << "cant get sensor info it there is no client or player";
    return dummyInfo;
}
