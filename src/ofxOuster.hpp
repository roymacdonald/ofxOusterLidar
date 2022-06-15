#pragma once
#include "ofMain.h"

#include "ouster/types.h"

#include "ouster/lidar_scan.h"


#include "ofxOusterRenderer.hpp"
#include "ofxOusterIMU.h"
#include "ofxOusterClient.h"
#include "ofxOusterPlayer.h"
//#include "Fusion.h"

class ofxOuster {
public:
	
	ofxOuster();
	~ofxOuster();
	
	
    /**
     * configure the sensor
     * @param hostname hostname or ip of the sensor
     * @param lidar_port port on which the sensor will send lidar data
     * @param imu_port port on which the sensor will send imu data
     */
    void connect(const std::string& hostname_, int lidar_port_ = 7502, int imu_port_ = 7503);
    
    /**
     * configure the sensor
     * @param hostname hostname or ip of the sensor
     * @param udp_dest_host hostname or ip where the sensor should send data
     * @param lidar_port port on which the sensor will send lidar data
     * @param imu_port port on which the sensor will send imu data
     * @param timeout_sec how long to wait for the sensor to initialize
     */
    void connect(const std::string& hostname_,
               const std::string& udp_dest_host_,
               ouster::sensor::lidar_mode mode_ = ouster::sensor::MODE_UNSPEC,
               ouster::sensor::timestamp_mode ts_mode_ = ouster::sensor::TIME_FROM_UNSPEC,
               int lidar_port_ = 7502,
               int imu_port_ = 7503,
               int timeout_sec_ = 30);
        
    /**
     * Loads a .pcap file. that you can either save using this addon or Ouster Studio
     * @param pcapDataFile file path to the PCAP data file
     * @param jsonConfigFile file path to the associated JSON configuration file
     */
    void load(const std::string& pcapDataFile, const std::string& jsonConfigFile);
    
    
    /// closes a file being read. You only need to call this after reading a file but it will be called automatically in the destructor
    void closeFile();
    
    
    
    
	const ouster::sensor::sensor_info& getSensorInfo() const;
	

    
    /// Sets the position of the Gui that gets drawn by calling drawGui()
    /// @param pos the position in the current viewport
    void setGuiPosition(const glm::vec2& pos);
    
    

    /// Draw the pointcloud and raw image data (if enabled).
    ///  You need to pass a camera to draw . Passing it externally allows you to draw more stuff with that camera
    void draw(ofEasyCam & cam);
    
    /// Draw only the point cloud mesh.
    void drawPointCloud();
    
    
    /// Draws the Gui to set the drawing parameters
	void drawGui();
    

    /// get the pointer to the renderer in case access to it is needed.
    ofxOusterRenderer* getRenderer();
    

    
    ofEvent<ouster::LidarScan> lidarDataEvent;
    ofEvent<ofxOusterIMUData> imuDataEvent;
    
protected:
    void onLidarData(ouster::LidarScan&);
    void onImuData(ofxOusterIMUData&);
    
private:
    std::atomic<bool> _bRendererInited;
    
    /// The pointer to the renderer in case access to it is needed.
    unique_ptr<ofxOusterRenderer> _renderer = nullptr;
    
	
	ofEventListeners _listeners;
    
	void _update(ofEventArgs&);
	
	ouster::LidarScan _readScan;
    
	void _initRenderer();
		
    ofxOusterIMUData imuData;

    glm::vec2 _guiPos = {10,10};
    unique_ptr<ofxOusterClient> _client = nullptr;
    unique_ptr<ofxOusterPlayer> _player = nullptr;
    
    ouster::sensor::sensor_info dummyInfo;
};
