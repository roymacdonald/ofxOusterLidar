#include "ofMain.h"
#include "ofApp.h"


int main( ){
    ofGLFWWindowSettings settings;
    settings.setGLVersion(3,2);
    settings.setSize(1050,1050);
    auto window = ofCreateWindow(settings);

    // this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
    ofRunApp(new ofApp());
}
