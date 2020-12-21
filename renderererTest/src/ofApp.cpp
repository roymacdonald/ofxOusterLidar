#include "ofApp.h"
#include "udpdata2.h"
//--------------------------------------------------------------
void ofApp::setup(){
	
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	info = ouster::sensor::metadata_from_json("OS1_2048x10_64U.json");
	
	renderer = make_unique<ofxOusterRenderer>(info, "renderer");
	
	
	
	uint32_t H = info.format.pixels_per_column;
	uint32_t W = info.format.columns_per_frame;
	auto packetFormat = ouster::sensor::get_format(info);
	
	
	
	
	auto xyz_lut = make_unique<ouster::XYZLut>(ouster::make_xyz_lut(info));
	renderer->setCloud(
					   xyz_lut->direction.data(),
					   xyz_lut->offset.data(),
					   H * W,
					   W,
					   {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
	
	
	ouster::LidarScan ls_write (W, H);
	auto _batchScan = ouster::ScanBatcher(W, packetFormat);
	
	
	
	bool bSuccess = false;
	
	std::vector<const unsigned char *> packets;
	// all the following packets are actual data packages taken from sample data provided by Ouster.
	// The provided samples are in pcap format. Which I had to open in wireshark, select a few UDP packages and export as C arrays.
	// For some reason those exported C arrays had an extra header of 8 bytes which I had to remove manually (Sublime Text Command+D multiline selection made it a breeze.
	packets.push_back(&pkt9_1[0]);
	packets.push_back(&pkt18_1[0]);
	packets.push_back(&pkt27_1[0]);
	packets.push_back(&pkt36_1[0]);
	packets.push_back(&pkt45_1[0]);
	packets.push_back(&pkt54_1[0]);
	packets.push_back(&pkt63_1[0]);
	packets.push_back(&pkt72_1[0]);
	packets.push_back(&pkt81_1[0]);
	packets.push_back(&pkt90_1[0]);
	packets.push_back(&pkt101_1[0]);
	packets.push_back(&pkt111_1[0]);
	packets.push_back(&pkt120_1[0]);
	packets.push_back(&pkt129_1[0]);
	packets.push_back(&pkt138_1[0]);
	packets.push_back(&pkt147_1[0]);
	packets.push_back(&pkt156_1[0]);
	packets.push_back(&pkt165_1[0]);
	packets.push_back(&pkt174_1[0]);
	packets.push_back(&pkt183_1[0]);
	packets.push_back(&pkt192_1[0]);
	packets.push_back(&pkt201_1[0]);
	packets.push_back(&pkt210_1[0]);
	packets.push_back(&pkt219_1[0]);
	packets.push_back(&pkt229_1[0]);
	packets.push_back(&pkt238_1[0]);
	packets.push_back(&pkt247_1[0]);
	packets.push_back(&pkt256_1[0]);
	packets.push_back(&pkt265_1[0]);
	packets.push_back(&pkt274_1[0]);
	packets.push_back(&pkt283_1[0]);
	packets.push_back(&pkt292_1[0]);
	packets.push_back(&pkt301_1[0]);
	packets.push_back(&pkt310_1[0]);
	packets.push_back(&pkt319_1[0]);
	packets.push_back(&pkt328_1[0]);
	packets.push_back(&pkt338_1[0]);
	packets.push_back(&pkt347_1[0]);
	packets.push_back(&pkt356_1[0]);
	packets.push_back(&pkt365_1[0]);
	packets.push_back(&pkt374_1[0]);
	packets.push_back(&pkt383_1[0]);
	
	
	// The following for loop makes exactly the same as it would happen when UDP packages were arriving over the network,
	// All of which is handled internally on a different thread by ofxOuster
	for(auto data: packets){
		if (_batchScan(data, ls_write)) {
			//Scan Batch returns true when it detects that a new package has begun
			cout << "batchscan break\n";
			break;
		}
	}
	
	renderer->render(ls_write);
}

//--------------------------------------------------------------
void ofApp::update(){
	
	
	
}

void ofApp::exit(){
}


//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0,0,0);
	
	
	ofEnableDepthTest();
	
	renderer->draw();
	
	ofDisableDepthTest();
	
	renderer->drawGui();
	
	
	
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

