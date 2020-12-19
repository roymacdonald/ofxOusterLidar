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
#include "ouster/autoexposure.h"
#include "ouster/beam_uniformity.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"


#include "ofMain.h"

#include "ofxGui.h"
#include "ofxDropdown.h"

// we don't align because the visualizer may be compiled with different
// compilation options as internal C++ code, leading to problems. besides, the
// performance here is not super critical
using mat4d = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;



/**
 * Class to deal with point clouds.
 * Each point cloud consists of n points with w poses.
 * The ith point will be transformed by the (i % w)th pose.
 *
 * For example for 2048 x 64 Ouster lidar point cloud,
 * we may have w = 2048 poses and n = 2048 * 64 = 131072.
 *
 * We also keep track of the map pose and the extrinsic calibration as mentioned
 * in the comment in the point_vertex_shader.
 *
 * The map_pose is used to efficiently transform the whole point cloud without
 * having to update all ~2048 poses.
 */
class Cloud {
    const size_t n;
    const size_t w;
	
	ofVboMesh mesh;
	
	std::vector<GLfloat> range_data;
    std::vector<GLfloat> key_data;
    

	
	
	ofTexture transformationTex;
	ofBufferObject transformationBuffer;
    std::vector<GLfloat> transformation;  // set automatically by setColumnPoses
    mat4d map_pose;
    std::array<GLfloat, 16> extrinsic_data;  // row major


	ofShader pointShader;
	
   public:
    /**
     * Set up the Cloud. Most of these arguments should correspond to CloudSetup
     *
     * @param xyz        Cartesian point clouds in the same format as
     *                   impl::Cloud::xyz, compatible with output of
     *                   make_xyz_lut.
     * @param off        Cartesian point clouds in the same format as
     *                   impl::Cloud::xyz, compatible with output of
     *                   make_xyz_lut.
     * @param n          Number of points, e.g. 64 * 2048 = 131072
     * @param w          Number of poses, e.g. 2048
     * @param extrinsics Extrinsic calibration of sensor, row major
     */
    template <class T>
    Cloud(T* xyz, T* off, const size_t n, const size_t w,
          const std::array<double, 16>& extrinsic)
        : n(n),
          w(w),
	range_data(n),
	key_data(n),
          transformation(12 * w, 0)
	{

		pointShader.load("point_program");
		
        map_pose.setIdentity();
        std::vector<GLfloat> trans_index_buffer_data(n);
			  
        for (size_t i = 0; i < n; i++) {
            trans_index_buffer_data[i] = ((i % w) + 0.5) / (GLfloat)w;
        }

        for (size_t v = 0; v < w; v++) {
            transformation[3 * v] = 1;
            transformation[3 * (v + w) + 1] = 1;
            transformation[3 * (v + 2 * w) + 2] = 1;
        }

		
		
		transformationBuffer.allocate();
		transformationBuffer.bind(GL_TEXTURE_BUFFER);
		transformationBuffer.setData(transformation,GL_STREAM_DRAW);

		// using GL_RGBA32F allows to read each row of each matrix
		// as a float vec4 from the shader.
		// Note that we're allocating the texture as a Buffer Texture:
		// https://www.opengl.org/wiki/Buffer_Texture
		transformationTex.allocateAsBufferTexture(transformationBuffer,GL_RGBA32F);

		// now we bind the texture to the shader as a uniform
		// so we can read the texture buffer from it
		
		pointShader.begin();
		pointShader.setUniformTexture("transformation",transformationTex,0);
//		pointShader.end();
//
//
//		pointShader.begin();
		mesh.getVbo().setAttributeData(pointShader.getAttributeLocation("trans_index"),trans_index_buffer_data.data(), 1, n, GL_STATIC_DRAW, sizeof(GLfloat));
		pointShader.end();
		
        setXYZ(xyz);
        setOffset(off);
        std::copy(extrinsic.begin(), extrinsic.end(), extrinsic_data.begin());
			  
			  
		
			  
    }
    /**
     * set the range values
     *
     * @param x pointer to array of at least as many elements as there are
     *          points, representing the range of the points
     */
    template <class T>
    void setRange(T* x) {
        std::copy(x, x + n, range_data.begin());
		
		pointShader.begin();
		mesh.getVbo().setAttributeData(pointShader.getAttributeLocation("range"), range_data.data(), 1, n , GL_STATIC_DRAW, sizeof(GLfloat));
//		mesh.getVbo().setAttributeDivisor(pointShader.getAttributeLocation("range"), 1);
		pointShader.end();
		
		
		
    }

    /**
     * set the key values, used for colouring.
     *
     * @param x        pointer to array of at least as many elements as there
     *                 are points, preferably normalized between 0 and 1
     */
    template <class T>
    void setKey(T* x) {
        std::copy(x, x + n, key_data.begin());
		pointShader.begin();
		mesh.getVbo().setAttributeData(pointShader.getAttributeLocation("key"), key_data.data(), 1, n , GL_STATIC_DRAW, sizeof(GLfloat));
		pointShader.end();
    }

    /**
     * convenience function equivalent to setRange and setKey
     *
     * @param cloud_id index of which cloud to update
     * @param r        range
     * @param k        key
     */
    template <class T, class U>
    void setRangeAndKey(T* r, U* k) {
        setRange(r);
        setKey(k);
    }

	
    /**
     * set the XYZ values
     *
     * @param x        pointer to array of exactly 3n where n is number of
     *                 points, so that the xyz position of the ith point is
     *                 i, i + n, i + 2n
     */
    template <class T>
    void setXYZ(T* xyz) {
		
		
		std::vector<GLfloat> xyz_data (3 * n);
		// I am not sure if all this is necesary
        for (size_t i = 0; i < n; i++) {
            for (size_t k = 0; k < 3; k++) {
                xyz_data[3 * i + k] = static_cast<GLfloat>(xyz[i + n * k]);
            }
        }
		
		mesh.getVbo().setVertexData(xyz_data.data(), 3, n, GL_STATIC_DRAW);
//		xyz_changed = true;
    }

    /**
     * set the offset values
     *
     * @param x        pointer to array of exactly 3n where n is number of
     *                 points, so that the xyz position of the ith point is
     *                 i, i + n, i + 2n
     */
    template <class T>
    void setOffset(T* off) {
		std::vector<GLfloat> off_data(3 * n);
		
        for (size_t i = 0; i < n; i++) {
            for (size_t k = 0; k < 3; k++) {
                off_data[3 * i + k] = static_cast<GLfloat>(off[i + n * k]);
            }
        }
		
		pointShader.begin();
		mesh.getVbo().setAttributeData(pointShader.getAttributeLocation("offset"), off_data.data(), 3, n , GL_STATIC_DRAW, sizeof(GLfloat)*3);
		pointShader.end();
		
    }

    /**
     * set the ith point cloud map pose
     *
     * @param map_pose homogeneous transformation matrix of the pose
     */
    void setMapPose(const mat4d& mat){ map_pose = mat; }

    /**
     * Set the per-column poses
     *
     * @param rotation rotation matrix 9 by w column major
     * @param translation translation vector array column major
     */
    template <class T>
    void setColumnPoses(T* rotation, T* translation) {
        for (size_t v = 0; v < w; v++) {
            for (size_t u = 0; u < 3; u++) {
                // u are columns of rotation matrix and rows of the texture
                for (size_t rgb = 0; rgb < 3; rgb++) {
                    transformation[(u * w + v) * 3 + rgb] =
                        static_cast<GLfloat>(rotation[v + u * 3 * w + rgb * w]);
                }
            }
            for (size_t rgb = 0; rgb < 3; rgb++) {
                transformation[9 * w + 3 * v + rgb] =
                    static_cast<GLfloat>(translation[v + rgb * w]);
            }
        }
		
    }


	/**
	 * render the point cloud with the point of view of the Camera
	 */
	void draw() {
	
		pointShader.begin();
		
		mesh.draw();
		
		pointShader.end();
	}
	
    
};


class ofxOusterRenderer {
public:
	
	ofxOusterRenderer(const ouster::sensor::sensor_info & info, const std::string& name_);
    

	ofTexture palette_texture;

	

	ofFloatImage image;
	ofFloatImage noiseImage;
	
	
	
	enum CloudDisplayMode {
        MODE_RANGE = 0,
        MODE_INTENSITY = 1,
        MODE_AMBIENT = 2,
        MODE_REFLECTIVITY = 3,
        NUM_MODES = 4
    };
	
	ofxPanel gui;
	
	
	ofParameter<float> point_size = {"Point Size", 3, 1, 10};
	ofParameter<bool> show_noise = {"Show Noise", true};
	ofParameter<int> display_mode = {"Display Mode", (int)MODE_INTENSITY, 0, (int) NUM_MODES -1};
	ofParameter<bool> cycle_range = {"Cycle Range", false};
	ofParameter<bool> show_image = {"Show Image", true};
	ofParameter<bool> show_ambient = {"Show Ambient", false};
	ofParameter<bool> cloud_swap = {"Cloud Swap", true};
//	ofParameter<size_t> which_cloud = {"Which cloud", 0, 0, }
	
	
    template <class T>
	void setCloud(T* xyz, T* off, const size_t n, const size_t w,
	const std::array<double, 16>& extrinsic)
	{
		cloud = make_unique<Cloud>(xyz, off, n, w, extrinsic);
	}
	
	void render(const ouster::LidarScan& _readScan);

	void draw();
	
	void drawGui();
	
	
	std::string getName() const ;
	size_t getHeight() const ;
	size_t getWidth() const ;
	
	
private:
	

	 ouster::viz::AutoExposure range_ae;
	 ouster::viz::AutoExposure intensity_ae;
	 ouster::viz::AutoExposure ambient_ae;
	 ouster::viz::AutoExposure reflectivity_ae;
	 ouster::viz::BeamUniformityCorrector ambient_buc;
	
	
	const std::vector<int> px_offset;

	const double aspect_ratio;
    const size_t h, w;
    
    
	unique_ptr<ofxIntDropdown> displayModeDropdown;
	
	std::string name;
		
	ofEventListeners listeners;
		
	void _cycleRangeChanged(bool&);
	void _displayModeChanged(int&);

	std::unique_ptr<Cloud> cloud;

	void _setupParameters();
	
	
};

//
