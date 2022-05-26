#include "ofxOusterRenderer.hpp"

#define STRINGIFY(x) #x
void Cloud::loadShader(){
  static const string shader_header = "#version 150\n";
        
        static const string fragShader = shader_header + STRINGIFY(
        in float vcolor;
        uniform sampler2DRect palette;
        out vec4 color;
        void main() {
            color = texture(palette, vec2(vcolor, 1));
        }
        );
        
        string vertShader = shader_header + STRINGIFY(
        in vec4 position;
        in vec3 offset;
        in float range;
        in float key;
        in float trans_index;
        uniform sampler2D transformation;
        uniform mat4 modelMatrix;
        uniform mat4 modelViewProjectionMatrix;
        uniform mat4 projectionMatrix;
        uniform mat4 modelViewMatrix;
        uniform mat4 textureMatrix;
        uniform mat4 globalColor;
        uniform float range_scale;
        uniform float range_max;
        uniform mat4 extrinsic;
        uniform float colorMapSize;
        out float vcolor;
        void main(){
            vec4 local_point;
            local_point =  vec4(position.xyz * range * range_scale, 1.0);
            vcolor = (range * colorMapSize)/(226326.f*range_max);
            gl_Position = modelViewProjectionMatrix * local_point;
        }
        );
        
        bool _bVertLoaded = pointShader.setupShaderFromSource(GL_VERTEX_SHADER, vertShader);
        bool _bFragLoaded = pointShader.setupShaderFromSource(GL_FRAGMENT_SHADER, fragShader);
        
        if(ofIsGLProgrammableRenderer()){
            pointShader.bindDefaults();
        }
        pointShader.linkProgram();
        
        
//        if(!pointShader.load("point_program"))
        if(!_bVertLoaded || !_bFragLoaded)
        {
            std::cout << "point shader FAILED to load\n";
        }
}

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

		using glmap_t = Eigen::Map<Eigen::Array<GLfloat, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

		ouster::img_t<double> range = _readScan.field(ouster::LidarScan::RANGE).cast<double>();
        if (show_image || display_mode == MODE_RANGE) {
            if (!cycle_range) {
                range_ae( Eigen::Map<Eigen::ArrayXd>(range.data(), range.size()));
            }
        }
		ouster::img_t<double> intensity = _readScan.field(ouster::LidarScan::INTENSITY).cast<double>();

        if (show_image || display_mode == MODE_INTENSITY) {
            intensity_ae( Eigen::Map<Eigen::ArrayXd>(intensity.data(), intensity.size()));
        }
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
        if ((show_image && show_ambient) || display_mode == MODE_AMBIENT) {
            // we need to destagger ambient because the
            // BeamUniformityCorrector only works on destaggered stuff
            ambient_destaggered = ouster::destagger<double>(ambient, px_offset);
            ambient_buc.correct(ambient_destaggered);
            ambient_ae(Eigen::Map<Eigen::ArrayXd>(ambient_destaggered.data(),
                                                  ambient_destaggered.size()));

            if (show_image && show_ambient) {
                glmap_t(imdata.getData() + 2 * w * h, h, w) = ambient_destaggered.cast<GLfloat>();
            }
        }

        if (show_image) {
//            if (show_ambient) {
//				image.resize(w, h);
////                point_viz.resizeImage(w, 3 * h);
//            } else {
////                point_viz.resizeImage(w, 2 * h);
//            }
			image.update();
        }

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


void ofxOusterRenderer::draw(ofEasyCam &cam)
{
	if(cloud){
		cam.begin();
		cloud->draw(range_scale.get(), range_max.get(), colorMap);
		cam.end();
	}else
	{
		ofLogError("ofxOusterRenderer::draw" ) << "Cloud not inited";
	}

	if (show_image)
	{
		ofRectangle r (0, 0, image.getWidth(), image.getHeight());
		ofRectangle s (0, 0, ofGetWidth(), ofGetHeight());
		r.scaleTo(s, OF_ASPECT_RATIO_KEEP, OF_ALIGN_HORZ_CENTER, OF_ALIGN_VERT_BOTTOM, OF_ALIGN_HORZ_CENTER, OF_ALIGN_VERT_BOTTOM);
		image.draw(r);
	}
}

void ofxOusterRenderer::drawGui()
{
	gui.draw();
}


void ofxOusterRenderer::_setupParameters()
{

	gui.setup(name + " params");

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
	display_mode.set(MODE_RANGE);
	gui.add(color_map_mode);
#endif
	gui.add(cycle_range);
	gui.add(show_image);
	gui.add(show_ambient);



	listeners.push(display_mode.newListener(this, &ofxOusterRenderer::_displayModeChanged));
	listeners.push(cycle_range.newListener(this, &ofxOusterRenderer::_cycleRangeChanged));
	listeners.push(point_size.newListener([&](float&){
		glPointSize(point_size.get());
	}));
	
	listeners.push(color_map_mode.newListener([&](int& i){
		colorMap.selectMap((ColorMaps)i);
	}));
	
	
}

void ofxOusterRenderer::setGuiPosition(const glm::vec2& pos){
    gui.setPosition(pos.x, pos.y);
}


const vector<glm::vec3>& ofxOusterRenderer::getPointCloud(){
    
    if(cloud){
        pointCloud = cloud->getMesh().getVertices();
        const auto & range = cloud->getRangeData();
        if(range.size() != pointCloud.size()){
            ofLogNotice("ofxOusterRenderer::getPointCloud") << "pointCloud mesh and range data differ in size!";
        }
        size_t n = std::min(range.size(), pointCloud.size());
        for(size_t i = 0; i < n; i ++){
            pointCloud[i] *= range[i];
        }
    }else{
        ofLogNotice("ofxOusterRenderer::getPointCloud") << "the cloud has not been setup. cant retrieve its points.";
    }
    return pointCloud;
}


