//
//  ofxOusterBatchData.hpp
//  lidarSketch
//
//  Created by Roy Macdonald on 10/11/20.
//

#pragma once
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iterator>

#include "impl/packet_impl.h"
#include "types.h"
#include "ouster/lidar_scan.h"

#include "ofMain.h"
class ofxOusterBatchData
{
public:
	
	///\brief constructor
	///\param width the scan width
	///\param height the scan height
	///\param packet_format packet format object which is retrieve from the sensor on startup.
	ofxOusterBatchData(int width, int height, const ouster::sensor::packet_format& packet_format);
	
	~ofxOusterBatchData();
	
	
	///\brief thread channel used to make thread safe the add function.
	ofThreadChannel<std::unique_ptr<ouster::LidarScan>> sendChannel;
	
	
	///\brief add data to the accumulator.
	///\this function is thread safe. Once it has acumulated a complete scan it will be sent througb the sendChannel threadChannel.
	void add(const uint8_t* packet_buf);
	
	
	
private:
	int h;
	int w;
	int scanH;// i am not sure if this is equal to h. Might not be. better to be redundant than wrong?
	int next_m_id;
	int32_t cur_f_id = -1;
	ouster::sensor::packet_format pf;
	
	const std::chrono::nanoseconds invalid_ts;
	std::chrono::nanoseconds scan_ts;
	
	std::unique_ptr<ouster::LidarScan> _currentScan = nullptr;
	
	
	void _initScan();
	

	ouster::LidarScan::Pixel empty;
	

};
