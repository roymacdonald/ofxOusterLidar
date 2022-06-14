#include "ofxOusterRenderer.hpp"

#define STRINGIFY(x) #x
void ofxOusterRenderer::loadShader(){
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
        uniform mat4 modelViewProjectionMatrix;
        uniform float range_max;
        uniform float colorMapSize;
        out float vcolor;
        void main(){
            float range = length(position.xyz);
            vcolor = (range * colorMapSize)/(226326.f*range_max);
            gl_Position = modelViewProjectionMatrix * position;
        }
        );
        
        bool _bVertLoaded = pointShader.setupShaderFromSource(GL_VERTEX_SHADER, vertShader);
        bool _bFragLoaded = pointShader.setupShaderFromSource(GL_FRAGMENT_SHADER, fragShader);
        
        if(ofIsGLProgrammableRenderer()){
            pointShader.bindDefaults();
        }
        pointShader.linkProgram();
        
        

        if(!_bVertLoaded || !_bFragLoaded)
        {
            std::cout << "point shader FAILED to load\n";
        }
}

ofxOusterRenderer::ofxOusterRenderer(const ouster::sensor::sensor_info & info)
{



	_setupParameters();
    
    makeLut(info);
    
    loadShader();
}

void glmToEigenMat(const glm::mat4& mat, mat4d & eMatrix){
    for(size_t i = 0; i < 4; i++){
        for(size_t j = 0; j < 4; j++){
            eMatrix(i, j) = mat[j][i];
        }
    }
}

void ofxOusterRenderer::makeLut(const ouster::sensor::sensor_info & info){

    glm::quat q(glm::radians(glm::vec3(-90.0, 90.0, 0)));
    

    
    /// the default coordinates space is rotated, in relationship to OF's.
    /// Here it is corrected so y axis = up/down, x axis = right/left, z = closest/furthest
    /// also scale so 1 pixel == 1 millimeter
    
    auto transformMatrix =  glm::toMat4(q);
    auto correctionMatrix = glm::scale(transformMatrix, glm::vec3(1000.0f));
    
    
    mat4d eMatrix;
    glmToEigenMat(correctionMatrix, eMatrix);
        
    lut = ouster::make_xyz_lut(info.format.columns_per_frame, info.format.pixels_per_column,
    ouster::sensor::range_unit, info.lidar_origin_to_beam_origin_mm,
    eMatrix * info.lidar_to_sensor_transform ,
    info.beam_azimuth_angles,
    info.beam_altitude_angles);
    
    
}



void ofxOusterRenderer::render(const ouster::LidarScan& _readScan)
{


    auto range = _readScan.field(ouster::sensor::ChanField::RANGE);
    auto cloud = ouster::cartesian(range, lut);
        
    points.clear();
    points.setMode(OF_PRIMITIVE_POINTS);
    for(size_t i = 0; i < cloud.rows(); i++){
        points.addVertex({cloud(i, 0),cloud(i, 1),cloud(i, 2)});
    }
    
    
}



void ofxOusterRenderer::drawPointCloud()
{
    
        pointShader.begin();
        pointShader.setUniform1f("range_max",range_max);
        pointShader.setUniform1f("colorMapSize", colorMap.img.getWidth());
        pointShader.setUniformTexture("palette", colorMap.img.getTexture(), 0);
        
        ofSetColor(255);
        points.draw();
        pointShader.end();
        
    
}

void ofxOusterRenderer::draw(ofEasyCam &cam)
{
    
    cam.begin();
    drawPointCloud();
    cam.end();
    
}

void ofxOusterRenderer::drawGui()
{
	gui.draw();
}


void ofxOusterRenderer::_setupParameters()
{

	gui.setup(name + " params");

	gui.add(point_size);
	gui.add(range_max);
	
#ifdef USE_OFX_DROPDOWN
//	map<int, string> displayModesNames = 	{
//		{MODE_RANGE, "RANGE"},
//		{MODE_INTENSITY, "INTENSITY"},
//		{MODE_AMBIENT, "AMBIENT"},
//		{MODE_REFLECTIVITY, "REFLECTIVITY"}};
//
//
//	displayModeDropdown =  make_unique<ofxIntDropdown>(display_mode, displayModesNames);
//
//	displayModeDropdown->disableMultipleSelection();
//	displayModeDropdown->enableCollapseOnSelection();
//
//	gui.add(displayModeDropdown.get());
//
//

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
//	display_mode.set(MODE_RANGE);
	gui.add(color_map_mode);
#endif


//	listeners.push(display_mode.newListener(this, &ofxOusterRenderer::_displayModeChanged));
	
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
    
    return points.getVertices();
}


