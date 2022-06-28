#include "ofxOuster.hpp"





namespace sensor = ouster::sensor;
ofxOuster::ofxOuster():_bRendererEnabled(true)
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
    _client->startThread();
    
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
    _client->startThread();
    
}

void ofxOuster::_initRenderer()
{
    if(_bRendererEnabled){
        if(!_renderer)
        {
            ouster::sensor::sensor_info sensorInfo;
            if(_client){
                sensorInfo = _client->getSensorInfo();
            }else if(_player){
                sensorInfo = _player->getSensorInfo();
            }else{
                ofLogError("ofxOuster::_initRenderer") << "can not init renderer whith no client or player";
                return;
            }
            _renderer = make_unique<ofxOusterRenderer>(sensorInfo);

            _renderer->setGuiPosition(_guiPos);
        }
    }
}

void ofxOuster::_update(ofEventArgs&)
{
    
    if((_client && _client->isInited() )|| _player){
        if(_bRendererEnabled && !_renderer && (_client->isSetup() || _player )  ){
            _initRenderer();
        }
        
        bool bNewData = false;
        while(getLidarScanChannel()->tryReceive(_readScan))
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
        while(getImuChannel()->tryReceive(imuData))
        {
            ofNotifyEvent(imuDataEvent, imuData, this);
        }
    }
}

void ofxOuster::drawPointCloud()
{
    if(_bRendererEnabled && _renderer)
    {
        _renderer->drawPointCloud();
    }
}



void ofxOuster::draw(ofEasyCam & cam)
{
	if(_bRendererEnabled && _renderer)
	{
        _renderer->draw(cam);
	}
}


void ofxOuster::drawGui()
{
	
	if(_bRendererEnabled && _renderer)
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
    if(_bRendererEnabled && _renderer){
        return _renderer.get();
    }else{
        return nullptr;
    }
}

bool ofxOuster::load(const std::string& dataFile, const std::string& configFile, uint16_t lidar_port, uint16_t imu_port){
    
    closeFile();
    if(!_player){
        _player = make_unique<ofxOusterPlayer>();
    }
    
    if(_player->load(dataFile, configFile, lidar_port, imu_port)){
        _listeners.unsubscribeAll();
        _listeners.push(ofEvents().update.newListener(this, &ofxOuster::_update));
        _player->play();
        return true;
    }
    return false;
}

void ofxOuster::closeFile(){
    if(_player){
        _player->closeFile();
    }
    _listeners.unsubscribeAll();
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

#define LOG_PLAYBACK_ERROR(cmd) if(!_player) {ofLogError(string("ofxOuster::")+ cmd) << "You can only call this function when playing back a .pcap file."; return;}

void ofxOuster::play(){
    LOG_PLAYBACK_ERROR("play")
    _player->play();
}
void ofxOuster::pause(){
    LOG_PLAYBACK_ERROR("Pause")
    _player->pause();
}
void ofxOuster::stop(){
    LOG_PLAYBACK_ERROR("Stop")
    _player->stop();
}
void ofxOuster::nextFrame(){
    LOG_PLAYBACK_ERROR("Next Frame")
    _player->nextFrame();
}

void ofxOuster::firstFrame(){
    LOG_PLAYBACK_ERROR("First Frame")
    _player->firstFrame();
}


bool ofxOuster::isPlaying(){
//    LOG_PLAYBACK_ERROR("Is PLaying")
    if(_player){
        return _player->isPlaying();
    }
    return false;
    
}


bool ofxOuster::isFileLoaded(){
    if(_player){
        return true;
    }
    return false;
}


/// Returns true if ithere is an active connection to an Ouster device
bool ofxOuster::isConnected(){
    if(_client){
        return true;
    }
    return false;
}

const ouster::XYZLut& ofxOuster::getLut(){
    auto r = getRenderer();
    if(r){
        return r->getLut();
    }
    
    if(bMakeLut){
        bMakeLut = false;
        auto & info = getSensorInfo();
        
        lut = ofxOusterRenderer::makeLut(info);
    }
    
    return lut;
}
ofThreadChannel<ouster::LidarScan> * ofxOuster::getLidarScanChannel(){
    if(_client){
        return &(_client->lidarScanChannel);
    }
    else if(_player){
        return &(_player->lidarScanChannel);
    }
    return nullptr;
}
ofThreadChannel<ofxOusterIMUData> * ofxOuster::getImuChannel(){
    if(_client){
        return &(_client->imuChannel);
    }
    else if(_player){
        return &(_player->imuChannel);
    }
    return nullptr;
}

void ofxOuster::enableRenderer(){
    _bRendererEnabled = true;
}
void ofxOuster::disableRenderer(){
    _bRendererEnabled = false;
    _renderer = nullptr;
}
bool ofxOuster::isRendererEnabled(){
    return _bRendererEnabled;
}
