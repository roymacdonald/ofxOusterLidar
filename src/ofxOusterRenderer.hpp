//
//  ofxOusterRenderer.hpp
//  lidarSketch
//
//  Created by Roy Macdonald on 10/12/20.
//

#pragma once


#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ouster/colormaps.h"
#include "ouster/image_processing.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"


#include "ofMain.h"

#include "ofxGui.h"

#include "ofxOusterColorMap.h"

#define USE_OFX_DROPDOWN

#ifdef USE_OFX_DROPDOWN
#include "ofxDropdown.h"
#endif
//#include "ofxAutoReloadedShader.h"
#include <limits>
// we don't align because the visualizer may be compiled with different
// compilation options as internal C++ code, leading to problems. besides, the
// performance here is not super critical
using mat4d = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;

class ofxOusterRenderer {
public:

	ofxOusterRenderer(const ouster::sensor::sensor_info & info);

	ofxPanel gui;

	ofParameter<float> point_size = {"Point Size", 3, 1, 10};

	ofParameter<float> range_max = {"range_max", 0.5, 0, 1};
	
	ofParameter<int> color_map_mode = {"Color Map Mode", 0, 0, 7};
	
	void render(const ouster::LidarScan& _readScan);

    void drawPointCloud();
    
    void draw(ofEasyCam &cam);

	void drawGui();
	
	
    void setGuiPosition(const glm::vec2& pos);
    
	ofxOusterColorMap colorMap;
	
    
    /// returns the collections of points that make the point cloud.
    /// This points are already transformed into realworld coordinates and are in the lidar's coordinate space,
    /// which means that the (0,0,0) is at the center of the lidar.
    const vector<glm::vec3>& getPointCloud();
    
    ofVboMesh points;
    
private:

        void loadShader();
        
        ofShader pointShader;
    //    ofxAutoReloadedShader pointShader;




#ifdef USE_OFX_DROPDOWN
	unique_ptr<ofxIntDropdown> displayModeDropdown;
	unique_ptr<ofxIntDropdown> colorMapDropdown;
#endif
	std::string name;

	ofEventListeners listeners;

	void _setupParameters();

    ouster::XYZLut lut;
    
    void makeLut(const ouster::sensor::sensor_info & info);
    
    
};

//
