#pragma once


#include "ofMain.h"
#include "ofxOuster.hpp"
#include "ofxGui.h"


class ofApp : public ofBaseApp{
	public:

	void setup();
	void update();
	void draw();
	void exit();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);		

	ofxOuster lidar;

	ofxPanel gui;
	ofParameter<string> lidarIp = {"Lidar's IP", "192.168.0.155"};
	ofParameter<string> udpDestIp = {"UDP dest IP", "192.168.0.100"};
	ofParameter<void> connect = {"Connect"};
	
	ofEventListeners listeners;

    ofMesh storedPoints;
    
    bool bShowStoredPoints = false;
    bool bShowLivePoints = true;

    ofEasyCam cam;
    
    };
