//
//  ofxPointRenderer.hpp
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 17-06-22.
//

#pragma once

#include "ofMain.h"

#include "ofxGui.h"

#include "ofxOusterColorMap.h"


#define USE_OFX_AUTO_RELOADED_SHADER

#ifdef USE_OFX_AUTO_RELOADED_SHADER
#include "ofxAutoReloadedShader.h"
#endif


class ofxPointShader{
public:
    ofxPointShader();
    
    ofxPanel gui;

    
    ofParameter<float> range_max = {"range_max", 0.5, 0, 1};
    

    void drawGui();
    
    void setGuiPosition(const glm::vec2& pos);
    
    ofxOusterColorMap colorMap;
    
    void begin();
    void begin(const glm::mat4& transform );
    void end();
    
    bool loadShader(const string& vert_shader_code, const string& frag_shader_code);
    bool loadShaderFromFiles(const string& vert_shader_code, const string& frag_shader_code);
protected:

        
#ifdef USE_OFX_AUTO_RELOADED_SHADER
    ofxAutoReloadedShader pointShader;
#else
    ofShader pointShader;
#endif

    

    void _setupParameters();
    
};

