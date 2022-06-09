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


	std::vector<GLfloat> range_data;
    std::vector<GLfloat> key_data;


	

    std::array<GLfloat, 16> extrinsic_data;  // row major


    void loadShader();
    
	ofShader pointShader;
//	ofxAutoReloadedShader pointShader;
   public:
       ofVboMesh mesh;
       ofTexture transformationTex;
       ofBufferObject transformationBuffer;
       std::vector<GLfloat> transformation;  // set automatically by setColumnPoses
       mat4d map_pose;


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
        loadShader();

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


		mesh.setMode(OF_PRIMITIVE_POINTS);

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


		glm::mat4 mat = glm::make_mat4(&extrinsic_data.data()[0]);
		pointShader.setUniformMatrix4f("extrinsic", mat);
//		glUniformMatrix4fv(ids.model_id, 1, GL_FALSE, extrinsic_data.data());

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

//		std::cout << "Cloud::setRange " << range_data.size() << std::endl;

		float mx = - std::numeric_limits<float>::max();
		float mn = -mx;
		for(auto& r: range_data)
		{
			if(r > mx) mx = r;
			if(r < mn) mn = r;
		}

//		std::cout << "Range min: " << mn << " max: " << mx << std::endl;

		pointShader.begin();
		mesh.getVbo().setAttributeData(pointShader.getAttributeLocation("range"), range_data.data(), 1, n , GL_STATIC_DRAW, sizeof(GLfloat));

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
//		std::cout << "Cloud::setKey " << key_data.size() << std::endl;
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


		std::vector<glm::vec3> xyz_data (n);
		// I am not sure if all this is necesary
//		std::cout << "setXYZ: " << n << std::endl;
        for (size_t i = 0; i < n; i++) {

				xyz_data[i].x = static_cast<float>(xyz[i + n * 0]);
				xyz_data[i].y = static_cast<float>(xyz[i + n * 1]);
				xyz_data[i].z = static_cast<float>(xyz[i + n * 2]);
        }

		mesh.addVertices(xyz_data);
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
    void setMapPose(const mat4d& mat){
        map_pose = mat;
    }

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
		transformationBuffer.updateData(0,transformation);
    }


	/**
	 * render the point cloud with the point of view of the Camera
	 */
	void draw(float range_scale, float range_max, ofxOusterColorMap& colorMap) {

		pointShader.begin();
		pointShader.setUniform1f("range_scale",range_scale);
		pointShader.setUniform1f("range_max",range_max);
		pointShader.setUniform1f("colorMapSize", colorMap.img.getWidth());
		pointShader.setUniformTexture("palette", colorMap.img.getTexture(), 0);
		
		ofSetColor(255);
		mesh.draw();
		pointShader.end();
		
	}
    
    /**
     * Get the range data that was set using setRange
     */
    const std::vector<GLfloat>& getRangeData(){
        return range_data;
    }

    
    /**
     * Returns the mesh that is used for rendering the point cloud but it does not have the range data applied to it.
     * Although this mesh is build so it represents the lidars configuration, so the only thing that needs to be done is to multiply each vertex by the range data to get the real point cloud
     */
    const ofVboMesh& getMesh(){
        return mesh;
    }

};


class ofxOusterRenderer {
public:

	ofxOusterRenderer(const ouster::sensor::sensor_info & info, const std::string& name_);




//	ofFloatImage image;
//	ofFloatImage noiseImage;
//	

	ofxPanel gui;


	ofParameter<float> point_size = {"Point Size", 3, 1, 10};

	ofParameter<float> range_max = {"range_max", 0.5, 0, 1};

//	ofParameter<int> display_mode = {"Display Mode", (int)MODE_RANGE, 0, (int) NUM_MODES -1};
	
	ofParameter<bool> show_image = {"Show Image", false};
	
//	ofParameter<int> color_map_mode = {"Color Map Mode", 0, 0, 7};
	

//    ofxButton saveCloud;

//
//    template <class T>
//	void setCloud(T* xyz, T* off, const size_t n, const size_t w, const std::array<double, 16>& extrinsic)
//	{
//		cloud = make_unique<Cloud>(xyz, off, n, w, extrinsic);
//	}

	void render(const ouster::LidarScan& _readScan);

    void drawPointCloud();
    void draw(ofEasyCam &cam);//, const glm::mat4& transform);

	void drawGui();


	std::string getName() const ;
	size_t getHeight() const ;
	size_t getWidth() const ;
	
	
    void setGuiPosition(const glm::vec2& pos);
    
//	ofxOusterColorMap colorMap;
	
    
    /// returns the collections of points that make the point cloud.
    /// This points are already transformed into realworld coordinates and are in the lidar's coordinate space,
    /// which means that the (0,0,0) is at the center of the lidar.
    const vector<glm::vec3>& getPointCloud();
    
    ofVboMesh points;
    
private:


//	 ouster::viz::AutoExposure range_ae;
//	 ouster::viz::AutoExposure intensity_ae;
//	 ouster::viz::AutoExposure ambient_ae;
//	 ouster::viz::AutoExposure reflectivity_ae;
//	 ouster::viz::BeamUniformityCorrector ambient_buc;


//	const std::vector<int> px_offset;

//	const double aspect_ratio;
    const size_t h, w;


#ifdef USE_OFX_DROPDOWN
	unique_ptr<ofxIntDropdown> displayModeDropdown;
	unique_ptr<ofxIntDropdown> colorMapDropdown;
#endif
	std::string name;

	ofEventListeners listeners;

//	void _cycleRangeChanged(bool&);
//	void _displayModeChanged(int&);

//	std::unique_ptr<Cloud> cloud = nullptr;

	void _setupParameters();

//    ofEasyCam cam;

//    vector<glm::vec3> pointCloud;

    ouster::XYZLut lut;
    
    
    
    
    void makeLut(const ouster::sensor::sensor_info & info);
    
    
};

//
