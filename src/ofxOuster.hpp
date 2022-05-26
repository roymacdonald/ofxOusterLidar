#include "ofMain.h"

#include "ouster/client.h"
#include "ouster/netcompat.h"
#include "ouster/types.h"

#include "ouster/lidar_scan.h"

#include "ofxOusterRenderer.hpp"



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
			   int lidar_port_ = 7502,
			   int imu_port_ = 7503,
			   int timeout_sec_ = 30);
		
	

	const string& getMetadata() const;
	
    
	const ouster::sensor::sensor_info& getSensorInfo() const;
	
	
	
	ouster::sensor::lidar_mode getLidarMode();
	
    
    
	ouster::sensor::timestamp_mode getTimestampMode();
	

    
    /// Sets the position of the Gui that gets drawn by calling drawGui()
    /// @param pos the position in the current viewport
    void setGuiPosition(const glm::vec2& pos);
    
    
    /// Returns the network port at which the lidar data is being received
	int getLidarPort();
	
    /// Returns the network port at which the IMU data is being received
	int getImuPort();
	
    /// Returns the amout of time that it will wait for the lidar to connect.
	int getTimeout();


    /// Draw the pointcloud and raw image data (if enabled).
    ///  You need to pass a camera to draw . Passing it externally allows you to draw more stuff with that camera
	void draw(ofEasyCam & cam);
    
    /// Draws the Gui to set the drawing parameters
	void drawGui();
    

    /// get the pointer to the renderer in case access to it is needed.
    ofxOusterRenderer* getRenderer();
    

    
protected:
	
	virtual void threadedFunction() override;
private:
    
    
    /// The pointer to the renderer in case access to it is needed.
    unique_ptr<ofxOusterRenderer> _renderer = nullptr;
    

    
	bool _initClient();
	
	std::atomic<bool> _clientInited;
	
	std::atomic<bool> _bUseSimpleSetup;
	std::atomic<bool> _bisSetup;
	
	std::atomic<bool> _bRendererInited;
	
	
	std::atomic<ouster::sensor::lidar_mode> mode;
	std::atomic<ouster::sensor::timestamp_mode> ts_mode;
	
	std::atomic<int> lidar_port;
	std::atomic<int> imu_port;
	std::atomic<int> timeout_sec;
	
	
	ofMutex hostMutex;
	std::string hostname;
	
	std::string udp_dest_host;
	
	std::shared_ptr<ouster::sensor::client> cli = nullptr;
	
	ofMutex metadataMutex;
	string metadata;
	ouster::sensor::sensor_info sensorInfo;
	
	
	void _initValues();
	

  

	
	
	
	ofEventListener _updateListener;
	void _update(ofEventArgs&);
	
	

	
	void _initRenderer();
		
	ofThreadChannel<ouster::LidarScan> lidarScanChannel;
	
	
	ouster::LidarScan _readScan;

    glm::vec2 _guiPos = {10,10};
	
};
