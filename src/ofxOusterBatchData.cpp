//
//  ofxOusterBatchData.cpp
//  lidarSketch
//
//  Created by Roy Macdonald on 10/11/20.
//

#include "ofxOusterBatchData.hpp"
#include "ofConstants.h"

void ofxOusterBatchData::_initScan()
{
	_currentScan = std::make_unique<ouster::LidarScan>(w, scanH);
}



ofxOusterBatchData::ofxOusterBatchData(int w_, int h_,
									   const ouster::sensor::packet_format& pf_):
	w(w_),
	scanH(h_),
	h(pf.pixels_per_column),
	next_m_id(w_),
	pf(pf_),
	empty(ouster::LidarScan::Pixel::empty_val()),
	invalid_ts(-1LL),
	scan_ts(invalid_ts)
{
	_initScan();
}

ofxOusterBatchData::~ofxOusterBatchData()
{
	sendChannel.close();
}




void ofxOusterBatchData::add(const uint8_t* packet_buf){
	
	
	auto it = _currentScan->begin();
	
	for (int icol = 0; icol < pf.columns_per_packet; icol++) {
		const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
		const uint16_t m_id = pf.col_measurement_id(col_buf);
		const uint16_t f_id = pf.col_frame_id(col_buf);
		const std::chrono::nanoseconds ts(pf.col_timestamp(col_buf));
		const bool valid = pf.col_valid(col_buf) == 0xffffffff;
		
		// drop invalid / out-of-bounds data in case of misconfiguration
		if (!valid || m_id >= w || f_id + 1 == cur_f_id) continue;
		
		if (f_id != cur_f_id) {
			// if not initializing with first packet
			if (scan_ts != invalid_ts) {
				// zero out remaining missing columns
				std::fill(it + (h * next_m_id), it + (w * h), empty);
				//                    f(scan_ts);
				
				sendChannel.send(std::move(_currentScan));
				//As we used the std::move, _currentScan became invalid, probably, so thats why we make it again calling _initScan
				_initScan();
				
				
			}
			
			// start new frame
			scan_ts = ts;
			next_m_id = 0;
			cur_f_id = f_id;
		}
		
		// zero out missing columns if we jumped forward
		if (m_id >= next_m_id) {
			std::fill(it + (h * next_m_id), it + (h * m_id), empty);
			next_m_id = m_id + 1;
		}
		
		// index of the first point in current packet
		const std::ptrdiff_t idx = h * m_id;
		
		for (uint8_t ipx = 0; ipx < h; ipx++) {
			const uint8_t* px_buf = pf.nth_px(ipx, col_buf);
			
			// i, ts, reflectivity, ring, noise, range (mm)
			it[idx + ipx] =
			ouster::LidarScan::pixel(ipx, m_id, ts, scan_ts, pf.px_range(px_buf),
									 pf.px_signal_photons(px_buf), pf.px_noise_photons(px_buf),
									 pf.px_reflectivity(px_buf));
		}
	}
}

