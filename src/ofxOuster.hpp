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
     * @param lidar_port (optional) Network port through which the lidar data was being recorded
     * @param imu_port (optional) Network port through which the IMU data was being recorded
     * Even when this functions load an already recorded file, such file is actually a raw network stream saved to disk.
     * So, in order to decode the data properly the network ports through which data came through is needed.
     * Specifying the network ports is optional, and most probably useful when reading a file that was recorded on ports other than the default or the jsonConfig file does not include such information
     * @returns true if  loading was successful, false otherwise
     */
    bool load(const std::string& pcapDataFile, const std::string& jsonConfigFile, uint16_t lidar_port = 7502, uint16_t imu_port = 7503);
    
    
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
    

    ///Event triggered when a new frame has arrived.
    /// The event itself will contain a reference to the new frame, which is accessible through the callback function registered to this event.
    /// The scan data come in the ouster::LidarScan format, which is "raw". You still need to process it to make it useful. Ouster provides tools for such through its SDK
    ofEvent<ouster::LidarScan> lidarDataEvent;
    ///Event triggered when new IMU data arrives.
    ///The event itself will contain a reference to the new data, which is accessible through the callback function registered to this event.
    /// Notice that this will be triggered several times for each frame that arrives, as the default sampling for the IMU is 100 times per second.
    /// The data is the raw unprocessed sensor data coming from the device. It is up to you to use it somehow. Check the ofxOusterIMU.h file for what kind of data is available
    ofEvent<ofxOusterIMUData> imuDataEvent;
    
    
    /// Playback control.
    /// Only useful when loading a pcap file.
    void play();
    void pause();
    void stop();
    void nextFrame();
    void firstFrame();
    
    bool isPlaying();
    
    /// Returns true if there is a pcap file loaded, false otherwise
    bool isFileLoaded();
    
    
    /// Returns true if ithere is an active connection to an Ouster device
    bool isConnected();
    
    const ouster::XYZLut& getLut();
    
    
    /// Thread channel getters. YOu should not need to use these unless you know what you are doing.
    
    ofThreadChannel<ouster::LidarScan> * getLidarScanChannel();
    ofThreadChannel<ofxOusterIMUData> * getImuChannel();
    
    
    /// Enable/Disable the renderer.
    /// This is mostly useful when you want to ingest the thread channels on a different thread and/or class.
    /// Be careful that if you disable the renderer and dont ingest the data it will just pile up and might make your app crash after a while because of lack of memory.
    /// If you want to have a headless app or not rendering the Lidars data you should disable it and implement your own ofThreadChannel "receiver".
    /// Use the following functions to get the thread channels.
    /// Check openFrameworksFolder/examples/threads/threadChannelExample/ for how to use these.
    /// ofThreadChannel<ouster::LidarScan> * getLidarScanChannel();
    /// ofThreadChannel<ofxOusterIMUData> * getImuChannel();
    
    void enableRenderer();
    void disableRenderer();
    bool isRendererEnabled();

    
    uint64_t getFrameCount();
    
    
    /// Record the current stream into a PCAP file.
    /// The PCAP file is the raw, unprocessed data stream coming from the lidar.
    /// When played back it would replicate the exact behaviour as if the data was streaming live
    /// \param filepath the file path where to save the PCAP file
    /// \return boolean. If false the recording was not able to start. True if recording started successfully.
    
    bool recordToPCap(const string& filepath);
    
    
    /// \return boolean. true if recording to PCAP file. false otherwise.
    bool isRecording();
    
protected:
    void onLidarData(ouster::LidarScan&);
    void onImuData(ofxOusterIMUData&);
    
private:
    
    std::atomic<bool> _bRendererEnabled;
    /// The pointer to the renderer in case access to it is needed.
    unique_ptr<ofxOusterRenderer> _renderer = nullptr;
    
	
	ofEventListeners _listeners;
    
	void _update(ofEventArgs&);
	
public:
	ouster::LidarScan _readScan;

private:
	void _initRenderer();
		
    ofxOusterIMUData imuData;

    glm::vec2 _guiPos = {10,10};

    unique_ptr<ofxOusterClient> _client = nullptr;
    unique_ptr<ofxOusterPlayer> _player = nullptr;

    ouster::sensor::sensor_info dummyInfo;
    
//    std::shared_ptr<ouster::sensor_utils::record_handle> _recorder = nullptr;
    
    ouster::XYZLut lut;
    bool bMakeLut = true;
    
};
