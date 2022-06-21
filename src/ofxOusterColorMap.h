//
//  ofxOusterColorMap.h
//  renderererTest
//
//  Created by Roy Macdonald on 1/22/21.
//

#pragma once

#include "ouster/colormaps.h"
#include "ofTexture.h"
#include "ofxBaseGui.h"
#include "ofParameter.h"
#include "ofxPanel.h"

#define USE_OFX_DROPDOWN

#ifdef USE_OFX_DROPDOWN
#include "ofxDropdown.h"
#endif

class ofxOusterColorMap;

class ofxColorMapGui: public ofxBaseGui {
public:
    ofxColorMapGui(){}
    ofxColorMapGui(ofxGuiGroup* gui, string name, ofxOusterColorMap* colorMap);

    virtual ~ofxColorMapGui(){
    }

    void setup(ofxGuiGroup* gui,  string name, ofxOusterColorMap* colorMap);
    
    // Abstract methods we must implement, but have no need for!
    virtual bool mouseMoved(ofMouseEventArgs & args){return false;}
    virtual bool mousePressed(ofMouseEventArgs & args){return false;}
    virtual bool mouseDragged(ofMouseEventArgs & args){return false;}
    virtual bool mouseReleased(ofMouseEventArgs & args){return false;}
    virtual bool mouseScrolled(ofMouseEventArgs & args){return false;}


    ofAbstractParameter & getParameter(){return void_param;}

    unique_ptr<ofxIntDropdown> colorMapDropdown = nullptr;
    
protected:
    ofParameter<void> void_param = {"void_param"};
    ofParameter<int> color_map_mode  = {"Color Map Mode", 0, 0, 7};
    
    void generateDraw(){
    }

    void render();
    

    void valueChanged(std::string & value){   }
    bool setValue(float mx, float my, bool bCheckBounds){return false;}
    
    ofxOusterColorMap* _colorMap = nullptr;
    
private:
    ofEventListeners listeners;
    

  
};




enum ColorMaps{
    COLORMAP_VIRIDIS = 0,
    COLORMAP_PARULA,
    COLORMAP_MAGMA,
    COLORMAP_RAINBOW,
    COLORMAP_TOSQEX,
    COLORMAP_OUTRUN,
    COLORMAP_CUBEHELIX,
    COLORMAP_SPEZIA,
	COLORMAP_NUM
};






class ofxOusterColorMap{
public:
	
    ofxOusterColorMap();
	
    void setup(ofxGuiGroup* gui, string name);
    
	ofFloatImage img;
	
    void selectMap(size_t index);
    
    void setColorMapFromData(const float data[][3], int dataLength);
    
    const map<int, string>& getNamesMap(){return ColorNames;}
    
    /// think of colorMap as an color image of any width (defaults to 256) and height 1;
    void addColorMap(const string& name, ofFloatPixels colorMap );
    
    /// make a new color map based on the vector of colors passed. any number of colors can be passed and the map will be generated with values by interpolating between colors
    void makeColorMap(const string& name, const vector<ofFloatColor>& colors);
    
    
protected:
    unique_ptr<ofxColorMapGui> colorMapGui = nullptr;
    
    vector< ofFloatPixels > ownedColorMaps;
    
    map<int, string> ColorNames =     {
        {COLORMAP_VIRIDIS, "VIRIDIS"},
        {COLORMAP_PARULA, "PARULA"},
        {COLORMAP_MAGMA, "MAGMA"},
        {COLORMAP_RAINBOW, "RAINBOW"},
        {COLORMAP_TOSQEX, "TOSQEX"},
        {COLORMAP_OUTRUN, "OUTRUN"},
        {COLORMAP_CUBEHELIX, "CUBEHELIX"},
        {COLORMAP_SPEZIA, "SPEZIA"},
    };
    
    
};

