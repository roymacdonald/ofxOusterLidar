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
    ofxColorMapGui(ofParameter<int> _param, ofxOusterColorMap* colorMap, float width = defaultWidth, float height = defaultHeight);

    virtual ~ofxColorMapGui(){
    }

    ofxColorMapGui * setup(ofParameter<int> param,ofxOusterColorMap* colorMap, float width = defaultWidth, float height = defaultHeight);
    
    // Abstract methods we must implement, but have no need for!
    virtual bool mouseMoved(ofMouseEventArgs & args){return false;}
    virtual bool mousePressed(ofMouseEventArgs & args){return false;}
    virtual bool mouseDragged(ofMouseEventArgs & args){return false;}
    virtual bool mouseReleased(ofMouseEventArgs & args){return false;}
    virtual bool mouseScrolled(ofMouseEventArgs & args){return false;}

    
    ofAbstractParameter & getParameter(){return _param;}

protected:
    
    ofParameter<int> _param;
    void generateDraw(){
    }

    void render();
    

    void valueChanged(std::string & value){   }
    bool setValue(float mx, float my, bool bCheckBounds){return false;}
    
    ofxOusterColorMap* _colorMap = nullptr;
    
};




enum ColorMaps{
    COLORMAP_VIRIDIS,
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
	
	ofFloatImage img;
	
    ofParameter<int> color_map_mode = {"Color Map Mode", 0, 0, 7};
    
    
    void setup(ofxPanel& gui, bool bMakeDropdown = true);
    
    void selectMap(ColorMaps index);
    
    void setColorMapFromData(const float data[][3], int dataLength);
private:
    ofEventListeners listeners;
    
    unique_ptr<ofxIntDropdown> colorMapDropdown = nullptr;
    unique_ptr<ofxColorMapGui> colorMapGui = nullptr;
    
    
};

