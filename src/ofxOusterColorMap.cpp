//
//  ofxOusterColorMap.cpp
//  lidarSlamOusterTest
//
//  Created by Roy Macdonald on 17-06-22.
//

#include "ofxOusterColorMap.h"
#include <numeric>
ofxColorMapGui::ofxColorMapGui(ofxGuiGroup* gui, string name, ofxOusterColorMap* colorMap){
    setup(gui, name, colorMap);
}


void ofxColorMapGui::render() {
    if(_colorMap){
        ofPushStyle();
        _colorMap->img.draw(b);
        ofPopStyle();
    }
}


void ofxColorMapGui::setup(ofxGuiGroup* gui, string name, ofxOusterColorMap* colorMap){
    
    _colorMap = colorMap;
    if(!name.empty()) color_map_mode.setName(name);
    
    b.width  = defaultWidth;
    b.height = defaultHeight;
    setNeedsRedraw();
    
    color_map_mode.setMax(colorMap->getNamesMap().size());
    
    colorMapDropdown =  make_unique<ofxIntDropdown>(color_map_mode, colorMap->getNamesMap());
    
    
    colorMapDropdown->disableMultipleSelection();
    colorMapDropdown->enableCollapseOnSelection();
    if(gui){
        gui->add(colorMapDropdown.get());
        gui->add(this);
    }
    
    listeners.push(color_map_mode.newListener([&](int& i){
        if(_colorMap) _colorMap->selectMap(i);
    }));
}



//------------------------------------------------------------------------

ofxOusterColorMap::ofxOusterColorMap(){
    selectMap(COLORMAP_VIRIDIS);
}

void ofxOusterColorMap::setup(ofxGuiGroup* gui, string name){
    colorMapGui = make_unique<ofxColorMapGui>(gui, name, this);

}
void ofxOusterColorMap::selectMap(size_t index){
    if(index < COLORMAP_NUM){
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
    }else if( ownedColorMaps.size() ){
        size_t i = index - COLORMAP_NUM;
        if(i < ownedColorMaps.size()){
            img.setFromPixels(ownedColorMaps[i]);
        }
    }
}

void ofxOusterColorMap::setColorMapFromData(const float data[][3], int dataLength){
    img.setFromPixels(reinterpret_cast<const float*>(&data[0][0]), dataLength, 1 ,OF_IMAGE_COLOR);
}

void ofxOusterColorMap::addColorMap(const string& name, ofFloatPixels colorMap ){
    int index = COLORMAP_NUM + ownedColorMaps.size();
    ColorNames[index] = name;
    ownedColorMaps.push_back(colorMap);
    if(colorMapGui && colorMapGui->colorMapDropdown){
        colorMapGui->colorMapDropdown->add(index, name);
    }
}
vector<int> distributeBins(size_t num, size_t total, bool printVals = false){
    if(num < 1){
        ofLogError("distributeBins") << "Num must be more than 1";
        return vector<int>();
    }
    if(num == 1){
        return vector<int> (2, total);
    }
    
    vector<int> numValsPerBinsAccum (num,0);
    vector<int> numValsPerBins (num,0);
    
    float n = total / (float)num;
    
    for(size_t i = 0; i < num; i++){
        numValsPerBinsAccum[i] = round( n * (i+1));
        if(i > 0){
            numValsPerBins[i] = numValsPerBinsAccum[i] - numValsPerBinsAccum[i-1];
        }else{
            numValsPerBins[i] = numValsPerBinsAccum[i] ;
        }
    }
    
    int sum = std::accumulate(numValsPerBins.begin(),numValsPerBins.end(),0);
    if(sum != total){
        ofLogError("ofxOusterColorMap::makeColorMap") << "Sum of all values is wrong: " << sum << " should be " << total;
        return vector<int>();
    }
    
    return  numValsPerBins;
}

void ofxOusterColorMap::makeColorMap(const string& name, const vector<ofFloatColor>& colors){
        
    if(colors.size() < 2){
        ofLogError("ofxOusterColorMap::makeColorMap") << "Number of colors passed must be more than 1";
        return;
    }
    int w = 256;
    
    auto bins = distributeBins(colors.size()-1, w -1);
    
    ownedColorMaps.push_back(ofFloatPixels());
    auto& colorMap = ownedColorMaps.back();
    colorMap.allocate(w, 1, OF_IMAGE_COLOR);
    
    size_t x = 0;
    for(size_t b = 0; b < bins.size(); b++){
        for(size_t i = 0; i < bins[b]; i++){
            
            float amt = ofMap(i,0, bins[b], 0,1);
            colorMap.setColor(x, 0, colors[b].getLerped(colors[b+1], amt));
            x++;
        }
    }
    colorMap.setColor(w-1, 0, colors.back());
    
    int index = COLORMAP_NUM + ownedColorMaps.size()-1;
    ColorNames[index] = name;
    if(colorMapGui && colorMapGui->colorMapDropdown){
        colorMapGui->colorMapDropdown->add(index, name);
    }

}

