//
//  ofxOusterColorMap.cpp
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 17-06-22.
//

#include "ofxOusterColorMap.h"

ofxColorMapGui::ofxColorMapGui(ofParameter<int> _param, ofxOusterColorMap* colorMap, float width , float height ){
        setup(_param,colorMap, width,height);
    }


    ofxColorMapGui * ofxColorMapGui::setup(ofParameter<int> param,ofxOusterColorMap* colorMap, float width, float height){
        _colorMap = colorMap;
            _param.makeReferenceTo(param);
            b.width  = width;
            b.height = height;
            setNeedsRedraw();
            return this;
    }
    

void ofxColorMapGui::render() {
    if(_colorMap){
        ofPushStyle();
        _colorMap->img.draw(b);
        ofPopStyle();
    }
}


    
ofxOusterColorMap::ofxOusterColorMap(){
    selectMap(COLORMAP_VIRIDIS);
}
    void ofxOusterColorMap::setup(ofxPanel& gui, bool bMakeDropdown){
        if(bMakeDropdown){
            map<int, string> ColorMapNames =     {
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
            
            colorMapGui = make_unique<ofxColorMapGui>(color_map_mode, this);
            
            gui.add(colorMapGui.get());
            
        }else{
            gui.add(color_map_mode);
        }
        listeners.push(color_map_mode.newListener([&](int& i){
            selectMap((ColorMaps)i);
        }));
        
    }
    
    void ofxOusterColorMap::selectMap(ColorMaps index){
        switch(index){
            case COLORMAP_VIRIDIS: setColorMapFromData(viridis, viridis_n);break;
            case COLORMAP_PARULA: setColorMapFromData(parula, parula_n);break;
            case COLORMAP_MAGMA: setColorMapFromData(magma, magma_n);break;
            case COLORMAP_RAINBOW: setColorMapFromData(rainbow, rainbow_n);break;
            case COLORMAP_TOSQEX: setColorMapFromData(tosqex, tosqex_n);break;
            case COLORMAP_OUTRUN: setColorMapFromData(outrun, outrun_n);break;
            case COLORMAP_CUBEHELIX: setColorMapFromData(cubehelix, cubehelix_n);break;
            case COLORMAP_SPEZIA: setColorMapFromData(spezia, spezia_n);break;
            default: break;
        }
    }
    
    void ofxOusterColorMap::setColorMapFromData(const float data[][3], int dataLength){
        img.setFromPixels(reinterpret_cast<const float*>(&data[0][0]), dataLength, 1 ,OF_IMAGE_COLOR);
    }

