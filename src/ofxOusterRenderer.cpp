//
//  ofxOusterRenderer.cpp
//  lidarSketch
//
//  Created by Roy Macdonald on 10/12/20.
//

#include "ofxOusterRenderer.hpp"



ofxOusterRenderer::ofxOusterRenderer(const ouster::sensor::sensor_info & info, const std::string& name_)
:name(name_)

,px_offset(info.format.pixel_shift_by_row),
//	aspect_ratio((info.beam_altitude_angles.front() -
//				  info.beam_altitude_angles.back()) /
//				 360.0),  // beam angles are in degrees
h(info.format.pixels_per_column),
w(info.format.columns_per_frame)
{
	
	
	image.allocate(  w, h , OF_IMAGE_COLOR);
	
	_setupParameters();
}


void ofxOusterRenderer::render(ouster::LidarScan* _readScan)
{
	
	if(_readScan && cloud)
	{
		
		auto& imdata = image.getPixels();
		Eigen::ArrayXd range = _readScan->range().cast<double>();
		if (show_image || display_mode == MODE_RANGE) {
			if (!cycle_range) {
				range_ae(range);
			}
		}
		Eigen::ArrayXd intensity = _readScan->intensity().cast<double>();
		if (show_image || display_mode == MODE_INTENSITY) {
			intensity_ae(intensity);
		}
		Eigen::ArrayXd noise = _readScan->noise().cast<double>();
		Eigen::ArrayXXd noise_destaggered(h, w);
		for (size_t u = 0; u < h; u++) {
			for (size_t v = 0; v < w; v++) {
				const size_t vv = (v + px_offset[u]) % w;
				if (cycle_range) {
					imdata[u * w + vv] =
					std::fmod(range(u * w + v) * ouster::sensor::range_unit, 2.0) /
					2.0;
				} else {
					imdata[u * w + vv] = range(u * w + v);
				}
				imdata[(u + h) * w + vv] = intensity(u * w + v);
			}
		}
		
		// TODO: optimize and move destaggering logic to ouster::LidarScan
		if ((show_image && show_noise) || display_mode == MODE_NOISE) {
			// we need to destagger noise because the
			// BeamUniformityCorrector only works on destaggered stuff
			for (size_t u = 0; u < h; u++) {
				for (size_t v = 0; v < w; v++) {
					const size_t vv = (v + px_offset[u]) % w;
					noise_destaggered(u, vv) = noise(u * w + v);
				}
			}
			noise_buc.correct(noise_destaggered);
			noise_ae(Eigen::Map<Eigen::ArrayXd>(noise_destaggered.data(), w * h));
			for (size_t u = 0; u < h; u++) {
				for (size_t v = 0; v < w; v++) {
					imdata[2 * h * w + u * w + v] = noise_destaggered(u, v);
				}
			}
		}
		
		if (show_image) {
			if (show_noise) {
				//					point_viz.resizeImage(w, 3 * h);
				//					point_viz.setImageAspectRatio(3 * aspect_ratio);
			} else {
				//					point_viz.resizeImage(w, 2 * h);
				//					point_viz.setImageAspectRatio(2 * aspect_ratio);
			}
			image.update();

		}
		
		if (display_mode == MODE_INTENSITY)
		{
			
			cloud->setRange(_readScan->range().data());
			cloud->setKey(intensity.data());
		}
		else if (display_mode == MODE_RANGE)
		{
			cloud->setRange(_readScan->range().data());
			cloud->setKey(range.data());
		}
		else if (display_mode == MODE_NOISE)
		{
			// zzz...
			for (size_t u = 0; u < h; u++) {
				for (size_t v = 0; v < w; v++) {
					const size_t vv = (v + px_offset[u]) % w;
					noise(u * w + v) = noise_destaggered(u, vv);
				}
			}
			cloud->setRange(_readScan->range().data());
			cloud->setKey(noise.data());
		}
		else if (display_mode == MODE_REFLECTIVITY)
		{
			Eigen::ArrayXd reflectivity = _readScan->reflectivity().cast<double>();
			reflectivity_ae(reflectivity);
			cloud->setRange(_readScan->range().data());
			cloud->setKey(reflectivity.data());
		}
	}
}


void ofxOusterRenderer::_cycleRangeChanged(bool&)
{
	ofLogNotice("ofxOusterRenderer::_displayModeChanged") << "Cycling range " << (cycle_range.get()?"every 2.0 m":"disabled");
}


void ofxOusterRenderer::_displayModeChanged(int&)
{
	
	switch (display_mode.get()) {
		case MODE_INTENSITY:
			ofLogNotice("ofxOusterRenderer::_displayModeChanged") << "Coloring point cloud by intensity";
			break;
		case MODE_RANGE:
			ofLogNotice("ofxOusterRenderer::_displayModeChanged") << "Coloring point cloud by range";
			break;
		case MODE_NOISE:
			ofLogNotice("ofxOusterRenderer::_displayModeChanged") << "Coloring point cloud by noise";
			break;
		case MODE_REFLECTIVITY:
			ofLogNotice("ofxOusterRenderer::_displayModeChanged") << "Coloring point cloud by reflectivity";
			
	}
}



std::string ofxOusterRenderer::getName() const
{
	return name;
	
}


size_t ofxOusterRenderer::getHeight() const
{
	return h;
	
}


size_t ofxOusterRenderer::getWidth() const
{
	return w;
	
}


void ofxOusterRenderer::draw()
{
	if(cloud) cloud->draw();
}

void ofxOusterRenderer::drawGui()
{
	gui.draw();
}


void ofxOusterRenderer::_setupParameters()
{
	
	gui.setup(name + " params");
	
	gui.add(point_size);
	gui.add(show_noise);
	
	map<int, string> displayModesNames = 	{
		{MODE_RANGE, "RANGE"},
		{MODE_INTENSITY, "INTENSITY"},
		{MODE_NOISE, "NOISE"},
		{MODE_REFLECTIVITY, "REFLECTIVITY"}};
	
	displayModeDropdown =  make_unique<ofxIntDropdown>(display_mode, displayModesNames);
		
	displayModeDropdown->disableMultipleSelection();
	displayModeDropdown->enableCollapseOnSelection();
	
		
	
	
	
	gui.add(displayModeDropdown.get());
	gui.add(cycle_range);
	gui.add(show_image);
	
	listeners.push(display_mode.newListener(this, &ofxOusterRenderer::_displayModeChanged));
	listeners.push(cycle_range.newListener(this, &ofxOusterRenderer::_cycleRangeChanged));
	
	
	
}
