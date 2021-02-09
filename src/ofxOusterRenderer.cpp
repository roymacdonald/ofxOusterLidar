#include "ofxOusterRenderer.hpp"



ofxOusterRenderer::ofxOusterRenderer(const ouster::sensor::sensor_info & info, const std::string& name_)
:name(name_)
,px_offset(info.format.pixel_shift_by_row),
aspect_ratio((info.beam_altitude_angles.front() -
info.beam_altitude_angles.back()) /
360.0),  // beam angles are in degrees
h(info.format.pixels_per_column),
w(info.format.columns_per_frame)
{


	std::cout << "info.format.pixels_per_column: " << h << std::endl;
	std::cout << "info.format.columns_per_frame: " << w << std::endl;

	image.allocate(  w, h *3 , OF_IMAGE_GRAYSCALE);

	_setupParameters();
}


void printRowsCols(ouster::img_t<double> & i, const std::string & name){
	std::cout << name << " : rows: " << i.rows() << "  cols: " << i.cols() << std::endl;
}

void ofxOusterRenderer::render(const ouster::LidarScan& _readScan)
{

	if(cloud)
	{

//		std::cout << "render\n";

		using glmap_t = Eigen::Map<Eigen::Array<GLfloat, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

		ouster::img_t<double> range = _readScan.field(ouster::LidarScan::RANGE).cast<double>();
//        if (show_image || display_mode == MODE_RANGE) {
//            if (!cycle_range) {
                range_ae( Eigen::Map<Eigen::ArrayXd>(range.data(), range.size()));
//            }
//        }
		ouster::img_t<double> intensity = _readScan.field(ouster::LidarScan::INTENSITY).cast<double>();

//        if (show_image || display_mode == MODE_INTENSITY) {
            intensity_ae( Eigen::Map<Eigen::ArrayXd>(intensity.data(), intensity.size()));
//        }
        ouster::img_t<double> ambient = _readScan.field(ouster::LidarScan::AMBIENT).cast<double>();


//		printRowsCols(range, "range");
//		printRowsCols(intensity, "intensity");
//		printRowsCols(ambient, "ambient");

		auto intensity_destaggered = ouster::destagger<double>(intensity, px_offset);
        auto range_destaggered = ouster::destagger<double>(range, px_offset);


		auto& imdata = image.getPixels();

        if (cycle_range) {
            glmap_t(imdata.getData(), h, w) =
                range_destaggered.cast<GLfloat>().unaryExpr(
                    [](const GLfloat x) -> GLfloat {
                        return std::fmod(x * ouster::sensor::range_unit, 2.0) * 0.5;
                    });
        } else {
            glmap_t(imdata.getData(), h, w) = range_destaggered.cast<GLfloat>();
        }

        glmap_t(imdata.getData() + w * h, h, w) = intensity_destaggered.cast<GLfloat>();

        ouster::img_t<double> ambient_destaggered{h, w};
//        if ((show_image && show_ambient) || display_mode == MODE_AMBIENT) {
            // we need to destagger ambient because the
            // BeamUniformityCorrector only works on destaggered stuff
            ambient_destaggered = ouster::destagger<double>(ambient, px_offset);
            ambient_buc.correct(ambient_destaggered);
            ambient_ae(Eigen::Map<Eigen::ArrayXd>(ambient_destaggered.data(),
                                                  ambient_destaggered.size()));

//            if (show_image && show_ambient) {
                glmap_t(imdata.getData() + 2 * w * h, h, w) = ambient_destaggered.cast<GLfloat>();
//            }
//        }

//        if (show_image) {
//            if (show_ambient) {
//				image.resize(w, h);
////                point_viz.resizeImage(w, 3 * h);
//            } else {
////                point_viz.resizeImage(w, 2 * h);
//            }
			image.update();
//        }

        auto range_data = _readScan.field(ouster::LidarScan::RANGE).data();
//		std::cout << "range data size: " << _readScan.field(ouster::LidarScan::RANGE).size() << std::endl;


        switch (+display_mode) {
            case MODE_INTENSITY:
                cloud->setRangeAndKey(range_data, intensity.data());
                break;
            case MODE_RANGE:
                cloud->setRangeAndKey(range_data, range.data());
                break;
            case MODE_AMBIENT:
                ambient = ouster::stagger<double>(ambient_destaggered, px_offset);
                cloud->setRangeAndKey( range_data, ambient.data());
                break;
            case MODE_REFLECTIVITY:
                ouster::img_t<double> reflectivity = _readScan.field(ouster::LidarScan::REFLECTIVITY).cast<double>();
                reflectivity_ae(Eigen::Map<Eigen::ArrayXd>( reflectivity.data(), reflectivity.size()));
                cloud->setRangeAndKey(range_data, reflectivity.data());
                break;
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
		case MODE_AMBIENT:
			ofLogNotice("ofxOusterRenderer::_displayModeChanged") << "Coloring point cloud by noise";
			break;
		case MODE_REFLECTIVITY:
			ofLogNotice("ofxOusterRenderer::_displayModeChanged") << "Coloring point cloud by reflectivity";
			break;
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
	if (cloud) {
		cam.begin();
		cloud->draw(range_scale.get(), range_max.get(), colorMap);
		cam.end();
	}
	else
	{
		ofLogError("ofxOusterRenderer::draw") << "Cloud not inited";
	}
}

void ofxOusterRenderer::drawImage() {
	if (show_image)
	{
		ofRectangle r(0, 0, image.getWidth(), image.getHeight());
		ofRectangle s(0, 0, ofGetWidth(), ofGetHeight());
		r.scaleTo(s, OF_ASPECT_RATIO_KEEP, OF_ALIGN_HORZ_CENTER, OF_ALIGN_VERT_BOTTOM, OF_ALIGN_HORZ_CENTER, OF_ALIGN_VERT_BOTTOM);
		image.draw(r);
	}
}

void ofxOusterRenderer::drawGui(int n = 0)
{
	gui.setPosition(gui.getPosition().x + (n * gui.getWidth() / 2), 0);
	gui.draw();
}


void ofxOusterRenderer::_setupParameters()
{
	string settings = ofToDataPath(name + "-CloudSettings.json", true);

	gui.setup(name + " params", settings);
	gui.add(showPointCloud);

	gui.add(cloudX);
	gui.add(cloudY);
	gui.add(cloudZ);
	gui.add(cloudAngleX);
	gui.add(cloudAngleY);
	gui.add(cloudAngleZ);

	gui.add(point_size);
	gui.add(range_scale);
	gui.add(range_max);
	gui.add(show_noise);
	
#ifdef USE_OFX_DROPDOWN
	map<int, string> displayModesNames = 	{
		{MODE_RANGE, "RANGE"},
		{MODE_INTENSITY, "INTENSITY"},
		{MODE_AMBIENT, "AMBIENT"},
		{MODE_REFLECTIVITY, "REFLECTIVITY"}};


	displayModeDropdown =  make_unique<ofxIntDropdown>(display_mode, displayModesNames);
	
	displayModeDropdown->disableMultipleSelection();
	displayModeDropdown->enableCollapseOnSelection();

	gui.add(displayModeDropdown.get());
	
	
	
	map<int, string> ColorMapNames = 	{
		{COLORMAP_VIRIDIS, "VIRIDIS"},
		{COLORMAP_PARULA, "PARULA"},
		{COLORMAP_MAGMA, "MAGMA"},
		{COLORMAP_RAINBOW, "RAINBOW"},
		{COLORMAP_TOSQEX, "TOSQEX"},
		{COLORMAP_OUTRUN, "OUTRUN"},
		{COLORMAP_CUBEHELIX, "CUBEHELIX"},
		{COLORMAP_SPEZIA, "SPEZIA"}
	};

	colorMapDropdown =  make_unique<ofxIntDropdown>(color_map_mode, ColorMapNames);
	
	colorMapDropdown->disableMultipleSelection();
	colorMapDropdown->enableCollapseOnSelection();

	gui.add(colorMapDropdown.get());
	
	
	
#else
	gui.add(display_mode);
	gui.add(color_map_mode);
#endif
	gui.add(cycle_range);
	gui.add(show_image);
	gui.add(show_ambient);

	gui.loadFromFile(settings);


	listeners.push(display_mode.newListener(this, &ofxOusterRenderer::_displayModeChanged));
	listeners.push(cycle_range.newListener(this, &ofxOusterRenderer::_cycleRangeChanged));
	listeners.push(point_size.newListener([&](float&){
		glPointSize(point_size.get());
	}));
	
	listeners.push(color_map_mode.newListener([&](int& i){
		colorMap.selectMap((ColorMaps)i);
	}));
	
	
}
