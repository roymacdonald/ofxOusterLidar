//
//  ofxPointRenderer.cpp
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 17-06-22.
//

#include "ofxPointShader.h"

ofxPointShader::ofxPointShader(){
    _setupParameters();
}

void ofxPointShader::_setupParameters()
{

    gui.setup("PointShader params");


    gui.add(range_max);
    
    colorMap.setup(gui);
        
}


void ofxPointShader::setGuiPosition(const glm::vec2& pos){
    gui.setPosition(pos.x, pos.y);
}


void ofxPointShader::begin(){
    pointShader.begin();
    pointShader.setUniform1f("range_max",range_max);
    pointShader.setUniform1f("colorMapSize", colorMap.img.getWidth());
    pointShader.setUniformTexture("palette", colorMap.img.getTexture(), 0);
}

void ofxPointShader::begin(const glm::mat4& transform ){
    begin();
    pointShader.setUniformMatrix4f("transform", transform);
    
}
void ofxPointShader::end(){
    pointShader.end();
}

bool ofxPointShader::loadShader(const string& vert_shader_code, const string& frag_shader_code){
  
        bool _bVertLoaded = pointShader.setupShaderFromSource(GL_VERTEX_SHADER, vert_shader_code);
        bool _bFragLoaded = pointShader.setupShaderFromSource(GL_FRAGMENT_SHADER, frag_shader_code);
        
        if(ofIsGLProgrammableRenderer()){
            pointShader.bindDefaults();
        }
        pointShader.linkProgram();
        
        

        if(!_bVertLoaded || !_bFragLoaded)
        {
            std::cout << "point shader FAILED to load\n";
            return false;
        }
    return true;
    
}
bool ofxPointShader::loadShaderFromFiles(const string& vert_shader_code, const string& frag_shader_code){
    return pointShader.load(vert_shader_code, frag_shader_code, "");
}
