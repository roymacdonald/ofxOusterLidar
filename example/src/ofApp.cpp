#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    
    ofSetBackgroundAuto(false);

	string settings = "network_settings.json";
	gui.setup("ofxOuster", settings);
	gui.add(lidarIp);
	gui.add(udpDestIp);
	gui.add(connect);

	gui.loadFromFile(settings);
	
	
	listeners.push(connect.newListener([&](){
		lidar.setup(lidarIp.get(), udpDestIp.get());
		lidar.startThread();
	}));

    
}

//--------------------------------------------------------------
void ofApp::update(){

    

}

void ofApp::exit(){
}


//--------------------------------------------------------------
void ofApp::draw(){
  ofBackground(0,0,0);
  cam.begin();
  ofEnableDepthTest();
  lidar.draw();
  ofDisableDepthTest();
  cam.end();

  lidar.drawGui(0);
  gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

