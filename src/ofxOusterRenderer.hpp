//
//  ofxOusterRenderer.hpp
//  lidarSketch
//
//  Created by Roy Macdonald on 10/12/20.
//

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include "ouster/lidar_scan.h"
#include "ouster/types.h"

#include "ofxPointShader.h"


// we don't align because the visualizer may be compiled with different
// compilation options as internal C++ code, leading to problems. besides, the
// performance here is not super critical
using mat4d = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;



class ofxOusterRenderer : public ofxPointShader{
public:

	ofxOusterRenderer(const ouster::sensor::sensor_info & info);

	
	void render(const ouster::LidarScan& _readScan);

    void drawPointCloud();
    
    void draw(ofEasyCam &cam);

    /// returns the collections of points that make the point cloud.
    /// This points are already transformed into realworld coordinates and are in the lidar's coordinate space,
    /// which means that the (0,0,0) is at the center of the lidar.
    const vector<glm::vec3>& getPointCloud();
    
    ofVboMesh points;
    
    const ouster::XYZLut& getLut(){return lut;}
    
    
private:


	std::string name;

    ouster::XYZLut lut;
    
    void makeLut(const ouster::sensor::sensor_info & info);
    
    
};

//
