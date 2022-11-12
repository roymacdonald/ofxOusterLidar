#include "ofApp.h"

#define FILE_SETTINGS "file_Settings.json"

string validateFileInJson(ofJson& json, string key){
    if(json.contains(key)){
        if(ofFile::doesFileExist(json[key])){
            return json[key];
        }else{
            ofLogWarning("Opening Saved Datapaths")<<"File does not exist: " << json[key];
        }
    }else{
        ofLogWarning("Opening Saved Datapaths")<<"Invalid key name: " << key;
    }
    return "";
}

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    ofSetBackgroundAuto(false);
//    ofxGuiSetTextPadding(30);
    
    gui.setup();
    connectParamGroup.add(lidarIp);
    connectParamGroup.add(localIp);
    connectParamGroup.add(connect);
    connectParamGroup.add(bRecord);
    playbackParamGroup.add(openPcapDialogParam);
    playbackParamGroup.add(openPcapParam);
    
    gui.add(connectParamGroup);
    gui.add(playbackParamGroup);
    
    
    
    
    listeners.push(connect.newListener([&](){
        // set to the correct IP address of both your computer and the lidar
        lidar.connect(lidarIp.get(), localIp.get());
    }));
    
    
    listeners.push(openPcapParam.newListener(this, &ofApp::tryOpenPcap));
    
    listeners.push(openPcapDialogParam.newListener(this, &ofApp::openPcapDialog));
    

    lidar.setGuiPosition(gui.getShape().getBottomLeft() + glm::vec2(0, 20));

    

    listeners.push(bRecord.newListener([&](bool&){
            if(bRecord.get()){
                auto res = ofSystemSaveDialog("LidarRecording_"+ofGetTimestampString(), "Choose where to save your recording");
            
                if(res.bSuccess){
                    recFile = res.getPath();
                    if(!lidar.recordToPCap(res.getPath())){
                        bRecord  = false;
                    }
                }else{
                    bRecord = false;
                }
            }else{
                if(lidar.isRecording()) lidar.endRecording();
                recFile = "";
            }
        }
    ));
    
    
    cam.setFarClip(999999);
    cam.setNearClip(0.00001);
    
}

//--------------------------------------------------------------
bool ofApp::openPcap(string pcap, string config){
    return lidar.load(pcap,config);
}
//--------------------------------------------------------------
void ofApp::openPcapDialog(){
    string pcap;
    string config;
    auto res = ofSystemLoadDialog("Select .pcap file");
    if(res.bSuccess){
        pcap = res.getPath();
        if(ofFile::doesFileExist(pcap)){
            
            config = ofFilePath::join(ofFilePath::getEnclosingDirectory(pcap),ofFilePath::getBaseName(pcap)) + ".json";
            if(!ofFile::doesFileExist(config)){
                res = ofSystemLoadDialog("Select .json configuration file");
                if(res.bSuccess){
                    config = res.getPath();
                }else{
                    config = "";
                }
            }
            if(openPcap(pcap, config)){
                ofJson json;
                json["PCAP"] = pcap;
                json["Config"] = config;
                
                ofSaveJson(FILE_SETTINGS, json);
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::tryOpenPcap(){
    string pcap, config;
    if(ofFile::doesFileExist(FILE_SETTINGS)){
        ofJson json = ofLoadJson(FILE_SETTINGS);
        
        pcap = validateFileInJson(json, "PCAP");
        config = validateFileInJson(json, "Config");
        
        if(!pcap.empty() && !config.empty() ){
            openPcap(pcap, config);
        }else{
            ofLogWarning("ofApp::openPcap") << "one of the filepaths is empty. Opening dialog";
            openPcapDialog();
        }
    }
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

