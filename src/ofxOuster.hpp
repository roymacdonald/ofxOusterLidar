//
//  ofxOuster.hpp
//  ofxOuster_example2
//
//  Created by Roy Macdonald on 9/19/20.
//
#include "ofMain.h"

#include "ouster/client.h"
#include "ouster/impl/client_impl.h"
#include "ouster/impl/compat.h"
#include "ouster/types.h"
#include "ouster/packet.h"

#include "ouster/lidar_scan.h"
#include "ofxOusterBatchData.hpp"
#include "ofxOusterRenderer.hpp"

using namespace ouster;

namespace sensor = ouster::sensor;

class ofxOuster: public ofThread {
public:
	
	ofxOuster();
	~ofxOuster();
	
	
	/**
	 * configure the sensor
	 * @param hostname hostname or ip of the sensor
	 * @param lidar_port port on which the sensor will send lidar data
	 * @param imu_port port on which the sensor will send imu data
	 */
	ofxOuster(const std::string& hostname_,
			  int lidar_port_ = 7502, int imu_port_ = 7503);
	
	
	/**
	 * configure the sensor
	 * @param hostname hostname or ip of the sensor
	 * @param udp_dest_host hostname or ip where the sensor should send data
	 * @param lidar_port port on which the sensor will send lidar data
	 * @param imu_port port on which the sensor will send imu data
	 * @param timeout_sec how long to wait for the sensor to initialize
	 */
	ofxOuster(const std::string& hostname,
			  const std::string& udp_dest_host,
			  ouster::sensor::lidar_mode mode = ouster::sensor::MODE_UNSPEC,
			  ouster::sensor::timestamp_mode ts_mode = ouster::sensor::TIME_FROM_UNSPEC,
			  int lidar_port = 0, int imu_port = 0,
			  int timeout_sec = 30);
	
	
	/**
	 * configure the sensor
	 * @param hostname hostname or ip of the sensor
	 * @param lidar_port port on which the sensor will send lidar data
	 * @param imu_port port on which the sensor will send imu data
	 */
	void setup(const std::string& hostname_, int lidar_port_ = 7502, int imu_port_ = 7503);
	
	/**
	 * configure the sensor
	 * @param hostname hostname or ip of the sensor
	 * @param udp_dest_host hostname or ip where the sensor should send data
	 * @param lidar_port port on which the sensor will send lidar data
	 * @param imu_port port on which the sensor will send imu data
	 * @param timeout_sec how long to wait for the sensor to initialize
	 */
	void setup(const std::string& hostname_,
			   const std::string& udp_dest_host_,
			   ouster::sensor::lidar_mode mode_ = ouster::sensor::MODE_UNSPEC,
			   ouster::sensor::timestamp_mode ts_mode_ = ouster::sensor::TIME_FROM_UNSPEC,
			   int lidar_port_ = 0,
			   int imu_port_ = 0,
			   int timeout_sec_ = 30);
		
	
//	const std::vector<uint8_t>& getLidarBuffer() const;
//
//	const std::vector<uint8_t>& getImuBuffer() const;
	
	const string& getMetadata() const;
	
	const ouster::sensor::sensor_info& getSensorInfo() const;
	
	
	
	ouster::sensor::lidar_mode getLidarMode();
	
	ouster::sensor::timestamp_mode getTimestampMode();
	
	
	
	int getLidarPort();
	
	int getImuPort();
	
	int getTimeout();


	void draw();
	void drawGui();
	
protected:
	
	virtual void threadedFunction() override;
private:
	std::atomic<bool> _bUseSimpleSetup;
	std::atomic<bool> _bisSetup;
	std::atomic<bool> _bClientInited;
	std::atomic<bool> _bRendererInited;
	
	
	std::atomic<ouster::sensor::lidar_mode> mode;
	std::atomic<ouster::sensor::timestamp_mode> ts_mode;
	std::atomic<int> lidar_port;
	std::atomic<int> imu_port;
	std::atomic<int> timeout_sec;
	
	
	ofMutex hostMutex;
	std::string hostname;
	
	std::string udp_dest_host;
	
	std::shared_ptr<ouster::sensor::client> cli;
	
	
	ofMutex metadataMutex;
	string metadata;
	ouster::sensor::sensor_info sensorInfo;
	
	
	
	
	//back buffers are accessed exclusively by the threaded function.
	//once a back buffer is aquired it is swapped with the front one while locking a mutex.\\
	//This makes safe to access the front buffer from any other thread
	
	std::vector<uint8_t> lidar_buf;
//	std::vector<uint8_t> lidar_buf_back;
//	std::vector<uint8_t> imu_buf_front;
	std::vector<uint8_t> imu_buf;
	
    

	ofMutex buf_mutex;
	
	
	
	void _initValues();
	
	std::atomic<uint64_t> n_lidar_packets;
	std::atomic<uint64_t> n_imu_packets;
	
	std::atomic<uint64_t> lidar_col_0_ts;
	std::atomic<uint64_t> imu_ts;
	
	std::atomic<float> lidar_col_0_h_angle;
	std::atomic<float> imu_av_z;
	std::atomic<float> imu_la_y;
	
	void _handle_lidar(uint8_t* buf, const sensor::packet_format& pf);
	
	void _handle_imu(uint8_t* buf, const sensor::packet_format& pf);
	
	void _print_headers();
	
	void _print_stats();


    unique_ptr<ouster::XYZLut> xyz_lut = nullptr;
	unique_ptr<ofxOusterBatchData> _batchData = nullptr;
	std::unique_ptr<ouster::LidarScan> _readScan = nullptr;

	
	
	
	ofEventListener _updateListener;
	void _update(ofEventArgs&);
	
	bool _newData = false;

	
	unique_ptr<ofxOusterRenderer> _renderer = nullptr;
	
	void _initRenderer();
	


	
};
