//
//  ofxOuster.cpp
//  ofxOuster_example2
//
//  Created by Roy Macdonald on 9/19/20.
//

#include "ofxOuster.hpp"

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
	if(_bisSetup && ofThread::isThreadRunning())
	{
		socket_init();
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
		auto packetFormat = sensor::get_format(_sensorInfo.format);
		
		{
			/// unnamed scope for locking mutex and automatically unlocking upon scope end
			std::lock_guard<ofMutex> lock(metadataMutex);
			metadata = _metadata;
			sensorInfo = _sensorInfo;
			auto H = sensorInfo.format.pixels_per_column;
			auto W = sensorInfo.format.columns_per_frame;
			
			xyz_lut = make_unique<ouster::XYZLut>(ouster::make_xyz_lut(sensorInfo));
			
			_batchData = make_unique<ofxOusterBatchData>(W, H, packetFormat);
			
		}
		{
			/// unnamed scope for locking mutex and automatically unlocking upon scope end
			std::lock_guard<ofMutex> lock(buf_mutex);
			lidar_buf.resize(packetFormat.lidar_packet_size + 1);
			imu_buf.resize(packetFormat.imu_packet_size + 1);
			
		}
		
		_bClientInited = true;
		
		
		while (isThreadRunning()) {
			sensor::client_state st = sensor::poll_client(*cli);
			if (st & sensor::CLIENT_ERROR) {
				ofLogError("ofxOuster::threadedFunction")<< "Client error. Closing thread";
				stopThread();
				break;
			} else if (st & sensor::LIDAR_DATA) {
				if (sensor::read_lidar_packet(*cli, lidar_buf.data(), packetFormat))
				{
					_handle_lidar(lidar_buf.data(), packetFormat);
					
					if(_bRendererInited)
						_batchData->add(lidar_buf.data());
					
				}
			} else if (st & sensor::IMU_DATA) {
				if (sensor::read_imu_packet(*cli, imu_buf.data(), packetFormat))
				{
					_handle_imu(imu_buf.data(), packetFormat);
					
				}
			}
		}
		socket_quit();
	}
}
void ofxOuster::_initRenderer()
{
	if(!_bRendererInited && _bClientInited)
	{
	auto H = sensorInfo.format.pixels_per_column;
	auto W = sensorInfo.format.columns_per_frame;
	
	_renderer = make_unique<ofxOusterRenderer>(sensorInfo, "Renderer");
	
	_renderer->setCloud(
						xyz_lut->direction.data(),
						xyz_lut->offset.data(),
						H * W,
						W,
						{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
	_bRendererInited = true;
	}
}

void ofxOuster::_update(ofEventArgs&)
{
	
	if(_batchData && _renderer)
	{
		_newData = false;
		while(_batchData->sendChannel.tryReceive(_readScan))
		{
			if(_readScan){
				_newData = true;
			}else
			{
				ofLogError("ofxOuster::_update:") <<"_readScan is invalid;";
			}
		}
		
		//		void draw(const LidarScan& ls, const size_t which_cloud = 0,
		//				  const bool cloud_swap = true, const bool show_image = true) {
		if(_newData && _renderer)
		{
			_renderer->render(_readScan.get());
		}
	}
	else if(!_renderer && _bClientInited)
	{
		_initRenderer();
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



//const std::vector<uint8_t>& ofxOuster::getLidarBuffer() const
//{
//	return lidar_buf_front;
//}
//const std::vector<uint8_t>& ofxOuster::getImuBuffer() const
//{
//	return imu_buf_front;
//}

const string& ofxOuster::getMetadata() const
{
	return metadata;
}

const ouster::sensor::sensor_info& ofxOuster::getSensorInfo() const
{
	return sensorInfo;
}



void ofxOuster::_handle_lidar(uint8_t* buf, const sensor::packet_format& pf) {
	n_lidar_packets++;
	lidar_col_0_ts = pf.col_timestamp(pf.nth_col(0, buf));
	lidar_col_0_h_angle = pf.col_h_angle(pf.nth_col(0, buf));
}

void ofxOuster::_handle_imu(uint8_t* buf, const sensor::packet_format& pf) {
	n_imu_packets++;
	imu_ts = pf.imu_gyro_ts(buf);
	imu_av_z = pf.imu_av_z(buf);
	imu_la_y = pf.imu_la_y(buf);
}

void ofxOuster::_print_headers() {
	std::cout << std::setw(15) << "n_lidar_packets" << std::setw(15)
	<< "col_0_azimuth" << std::setw(20) << "col_0_ts" << std::setw(15)
	<< "n_imu_packets" << std::setw(15) << "im_av_z" << std::setw(15)
	<< "im_la_y" << std::setw(20) << "imu_ts" << std::endl;
}

void ofxOuster::_print_stats() {
	std::cout << "\r" << std::setw(15) << n_lidar_packets << std::setw(15)
	<< lidar_col_0_h_angle << std::setw(20) << lidar_col_0_ts
	<< std::setw(15) << n_imu_packets << std::setw(15) << imu_av_z
	<< std::setw(15) << imu_la_y << std::setw(20) << imu_ts;
	std::flush(std::cout);
}



//void ofxOuster::_swapBuffers(std::vector<uint8_t> & front, std::vector<uint8_t> & back)
//{
//	std::lock_guard<ofMutex> lock(buf_mutex);
//	std::swap(front, back);
//}


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
	n_lidar_packets = 0;
	n_imu_packets = 0;
	lidar_col_0_ts = 0;
	imu_ts = 0;
	lidar_col_0_h_angle = 0.0f;
	imu_av_z = 0.0f;
	imu_la_y = 0.0f;
	
	mode  = ouster::sensor::MODE_UNSPEC;
	ts_mode  = ouster::sensor::TIME_FROM_UNSPEC;
	lidar_port = 0;
	imu_port = 0;
	timeout_sec = 30;
	_bisSetup = false;
	_bClientInited = false;
	_bRendererInited = false;
}

