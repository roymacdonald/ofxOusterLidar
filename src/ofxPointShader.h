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


//#define USE_OFX_AUTO_RELOADED_SHADER

#ifdef USE_OFX_AUTO_RELOADED_SHADER
#include "ofxAutoReloadedShader.h"
#endif


class ofxPointShader{
public:
    ofxPointShader(string name);
    
    virtual ~ofxPointShader(){}
    
    ofxPanel gui;

    /// maximum range in meters
    ofParameter<float> range_max = {"range_max", 200, 0, 500};
    
    void drawGui();
    
    void setGuiPosition(const glm::vec2& pos);
    
    ofxOusterColorMap colorMap;
    
    virtual void begin();
    virtual void begin(const glm::mat4& transform );
    
    void end();
    
    bool loadShader(const string& vert_shader_code, const string& frag_shader_code);
    bool loadShaderFromFiles(const string& vert_shader_code, const string& frag_shader_code);
    
    
    /// set the range unit in meters. if the range unit is millimeters the set to 0.001 (which is the default).
    /// if it is in meters set to 1. etc.
    void setRangeUnit(double u);

#ifdef USE_OFX_AUTO_RELOADED_SHADER
    ofxAutoReloadedShader shader;
#else
    ofShader shader;
#endif

    
protected:

    /** Unit of range from sensor packet, in meters. */
    double range_unit = 0.001;
        

    

    void _setupParameters();

    string name;
    
};

