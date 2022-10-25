//
//  ofxPointRenderer.cpp
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 17-06-22.
//

#include "ofxPointShader.h"

#define STRINGIFY(x) #x

ofxPointShader::ofxPointShader(string _name):name(_name){
    _setupParameters();
    
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
          uniform vec3 origin;
          out float vcolor;
          void main(){
              float range = length(position.xyz-origin);
              vcolor = (range * colorMapSize)/range_max;
              gl_Position = modelViewProjectionMatrix * position;
          }
          );
          
    
    loadShader(vertShader, fragShader);
    
}

void ofxPointShader::_setupParameters()
{

    string filename = ofToDataPath(name + "_params_settings.json");
    gui.setup(name + " params", filename);


    gui.add(range_max);
    gui.add(point_size);
    
    colorMap.setup(&gui, name);
    
    listeners.push(point_size.newListener([&](float&){
        glPointSize(point_size.get());
    }));
    
    if(ofFile::doesFileExist(filename)){
        gui.loadFromFile(filename);
    }
    
    
    
}
void ofxPointShader::setRangeUnit(double u){
    range_unit = u;
}
void ofxPointShader::setGuiPosition(const glm::vec2& pos){
    gui.setPosition(pos.x, pos.y);
}


void ofxPointShader::begin(){
    shader.begin();
    shader.setUniform1f("range_max",range_max/range_unit);
    shader.setUniform1f("colorMapSize", colorMap.img.getWidth());
    shader.setUniformTexture("palette", colorMap.img.getTexture(), 0);
}

void ofxPointShader::begin(const glm::mat4& transform ){
    begin();
    shader.setUniformMatrix4f("transform", transform);
    
}
void ofxPointShader::end(){
    shader.end();
}

bool ofxPointShader::loadShader(const string& vert_shader_code, const string& frag_shader_code){
  
        bool _bVertLoaded = shader.setupShaderFromSource(GL_VERTEX_SHADER, vert_shader_code);
        bool _bFragLoaded = shader.setupShaderFromSource(GL_FRAGMENT_SHADER, frag_shader_code);
        
        if(ofIsGLProgrammableRenderer()){
            shader.bindDefaults();
        }
        shader.linkProgram();
        
        

        if(!_bVertLoaded || !_bFragLoaded)
        {
            std::cout << "point shader FAILED to load\n";
            return false;
        }
    return true;
    
}
bool ofxPointShader::loadShaderFromFiles(const string& vert_shader_code, const string& frag_shader_code){
    return shader.load(vert_shader_code, frag_shader_code, "");
}
void ofxPointShader::drawGui()
{
    gui.draw();
}
