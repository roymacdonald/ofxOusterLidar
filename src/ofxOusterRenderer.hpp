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
    
    const ouster::XYZLut& getLut(){return lut;}
    
    static ouster::XYZLut makeLut(const ouster::sensor::sensor_info & info);
    
private:
    
    void _setupMesh();
    
    ofVboMesh mesh;


    ouster::XYZLut lut;
        
    
};

//
