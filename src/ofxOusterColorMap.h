#pragma once
#include "ouster/colormaps.h"
#include "ofTexture.h"

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
	
	ofxOusterColorMap(){
		selectMap(COLORMAP_VIRIDIS);
	}
	
	ofFloatImage img;
	
	
	void selectMap(ColorMaps index){
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
	
	void setColorMapFromData(const float data[][3], int dataLength){
		img.setFromPixels(reinterpret_cast<const float*>(&data[0][0]), dataLength, 1 ,OF_IMAGE_COLOR);
	}

	
};
