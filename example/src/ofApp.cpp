#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofSetLogLevel(OF_LOG_VERBOSE);
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
        //when connect is pressed try to connect to a lidar at the ip address specified in lidarIp
		lidar.connect(lidarIp.get(), udpDestIp.get());
	}));
    
    
    listeners.push(openPcap.newListener([&](){
        // when openPcap is pressed do the following
        
        string dir = "/Users/roy/Desktop/park_stephan/";
        lidar.load(dir+ "2022-06-02-13-50-11_OS-1-64-992214000010-1024x10.pcap",
                   dir+ "2022-06-02-13-50-11_OS-1-64-992214000010-1024x10.json");
//        lidar.load("/Volumes/MacHD/Users/roy/Desktop/park_stephan/2022-06-02-13-50-11_OS-1-64-992214000010-1024x10.pcap",        "/Volumes/MacHD/Users/roy/Desktop/park_stephan/2022-06-02-13-50-11_OS-1-64-992214000010-1024x10.json");
//                lidar.load("/Users/roy/Downloads/OS0_128_freeway_sample/OS0_128_freeway_sample.pcap",
//                            "/Users/roy/Downloads/OS0_128_freeway_sample/OS0_2048x10_128.json", 47691, 37873);
    }));
    
    // Move the lidars gui so it does not overlap with ofApp's gui.
    lidar.setGuiPosition(gui.getShape().getBottomLeft() + glm::vec2(0, 20));
    
    
    // this are just some ofEasyCam settings I find better for viewing pointclouds
    cam.setFarClip(1000000);
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
	
    
    lidar.draw(cam);
    
    lidar.drawGui();
    
	gui.draw();
	

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

//    if(key == ' '){
//        
//    }
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

