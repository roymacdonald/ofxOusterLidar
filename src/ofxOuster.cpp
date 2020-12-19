//
//  ofxOuster.cpp
//  ofxOuster_example2
//
//  Created by Roy Macdonald on 9/19/20.
//

#include "ofxOuster.hpp"
namespace sensor = ouster::sensor;
ofxOuster::ofxOuster()
{
	_initValues();
}

ofxOuster::~ofxOuster()
{
	ofThread::waitForThread(true, 30000);
}



ofxOuster::ofxOuster(const std::string& hostname_, int lidar_port_ , int imu_port_)
{
	_initValues();
	setup(hostname_, lidar_port_, imu_port_);
}


void ofxOuster::setup(const std::string& hostname_, int lidar_port_, int imu_port_)
{
	_bUseSimpleSetup = true;
	hostname = hostname_;
	lidar_port = lidar_port_;
	imu_port = imu_port_;
	_bisSetup = true;
	_updateListener = ofEvents().update.newListener(this, &ofxOuster::_update);
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
}

void ofxOuster::threadedFunction(){
	if(ofThread::isThreadRunning())
	{
		if(_bisSetup && !cli)
		{
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
				return ;
			}
		}
		
		auto _metadata = sensor::get_metadata(*cli);
		auto _sensorInfo = sensor::parse_metadata(_metadata);
		auto packetFormat = sensor::get_format(_sensorInfo);
		
			ofLogVerbose("ofxOuster") << "Using lidar_mode: " << sensor::to_string(_sensorInfo.mode);
			ofLogVerbose("ofxOuster") << _sensorInfo.prod_line << " sn: " << _sensorInfo.sn << " firmware rev: " << _sensorInfo.fw_rev ;

			uint32_t H ;
			uint32_t W ;
		{
			/// unnamed scope for locking mutex and automatically unlocking upon scope end
			std::lock_guard<ofMutex> lock(metadataMutex);
			metadata = _metadata;
			sensorInfo = _sensorInfo;
			H = sensorInfo.format.pixels_per_column;
			W = sensorInfo.format.columns_per_frame;
			

			
			
			
		}
			_initRenderer();
//		{
			/// unnamed scope for locking mutex and automatically unlocking upon scope end
//			std::lock_guard<ofMutex> lock(buf_mutex);
			// These are only used inside the threaded function so there is no need to lock
			std::vector<uint8_t> lidar_buf(packetFormat.lidar_packet_size + 1);
			std::vector<uint8_t> imu_buf(packetFormat.imu_packet_size + 1);
			


		
		
			ouster::LidarScan ls_write (W, H);
			auto _batchScan = ouster::ScanBatcher(W, packetFormat);
			
			
			
			while (isThreadRunning()) {
				
				sensor::client_state st = sensor::poll_client(*cli);
				if (st & sensor::client_state::CLIENT_ERROR) {
					ofLogError("ofxOuster::threadedFunction")<< "Client error. Closing thread";
					stopThread();
					break;
				}
				
				if (st & sensor::client_state::LIDAR_DATA) {
					if (sensor::read_lidar_packet(*cli, lidar_buf.data(),
												  packetFormat)) {
						if (_batchScan(lidar_buf.data(), ls_write)) {
							lidarScanChannel.send(ls_write);
						}
					}
				}
				if (st & sensor::client_state::IMU_DATA) {
					sensor::read_imu_packet(*cli, imu_buf.data(), packetFormat);
				}
				if (st & sensor::EXIT) {
					stopThread();
					break;
				}
			}
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
	}
}

void ofxOuster::_update(ofEventArgs&)
{
	if(!_renderer)
	{
		_initRenderer();
	}
	bool bNewData = false;
		while(lidarScanChannel.tryReceive(_readScan))
		{
			bNewData = true;
//			if(_readScan){
////				_newData = true;
//			}else
//			{
//				ofLogError("ofxOuster::_update:") <<"_readScan is invalid;";
//			}
		}
		
		//		void draw(const LidarScan& ls, const size_t which_cloud = 0,
		//				  const bool cloud_swap = true, const bool show_image = true) {
		if(bNewData && _renderer)
		{
			_renderer->render(_readScan);
		}
	
	
}


void ofxOuster::draw()
{
	
	if(_renderer)
	{
		_renderer->draw();
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


void ofxOuster::_initValues()
{
	mode  = ouster::sensor::MODE_UNSPEC;
	ts_mode  = ouster::sensor::TIME_FROM_UNSPEC;
	lidar_port = 0;
	imu_port = 0;
	timeout_sec = 30;
	_bisSetup = false;
	
}

