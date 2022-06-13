#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    
    ofSetBackgroundAuto(false);

	string settings = "network_settings.json";
	gui.setup("ofxOuster", settings);
	gui.add(lidarIp);
	gui.add(udpDestIp);
	gui.add(connect);
    gui.add(openPcap);
    
    
    if(ofFile::doesFileExist(settings)){
        gui.loadFromFile(settings);
    }
	
	listeners.push(connect.newListener([&](){
		lidar.connect(lidarIp.get(), udpDestIp.get());
	}));
    
    
    listeners.push(openPcap.newListener([&](){
        string dir = "/Users/roy/Desktop/OusterPCAP/";
        lidar.load(dir+ "2022-05-27-20-08-04_OS-1-64-992214000010-1024x10.pcap",
                   dir+ "2022-05-27-20-08-04_OS-1-64-992214000010-1024x10.json");
    }));
    
    
    lidar.setGuiPosition(gui.getShape().getBottomLeft() + glm::vec2(0, 20));
    
    cam.setFarClip(100000);
    cam.setNearClip(0);
    cam.setRelativeYAxis(!cam.getRelativeYAxis());
    
    
}

//--------------------------------------------------------------
void ofApp::update(){

    

}

void ofApp::exit(){
}


//--------------------------------------------------------------
void ofApp::draw(){
  ofBackground(0,0,0);
	
    if(bShowStoredPoints){
        cam.begin();
        ofPushStyle();
        ofSetColor(ofColor::red);
        storedPoints.draw();
        ofPopStyle();
        cam.end();
    }
    
    if(bShowLivePoints){
        lidar.draw(cam);
    
        lidar.drawGui();
    }
	gui.draw();
	
    stringstream ss;
    
    ss << "Press [ key ]:\n";
    ss << "      [space]: to store the current point cloud.\n";
    ss << "      [  1  ]: to " << (bShowStoredPoints?"HIDE":"SHOW") << " Stored Points.\n" ;
    ss << "      [  2  ]: to " << (bShowLivePoints?"HIDE":"SHOW") << " Live Points.";
    
    
    ofBitmapFont bf;
    auto r = bf.getBoundingBox(ss.str(), 0, 0);
    
    ofDrawBitmapStringHighlight(ss.str(), 20, ofGetHeight() - 20 - r.height);
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

    if(key == ' '){
        if(lidar.getRenderer()){
            storedPoints.setMode(OF_PRIMITIVE_POINTS);
            storedPoints.addVertices(lidar.getRenderer()->getPointCloud());
        }
    }else if(key == '1'){
        bShowStoredPoints ^= true;
    }else if(key == '2'){
        bShowLivePoints ^= true;
    }
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

