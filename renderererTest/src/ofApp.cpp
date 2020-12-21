#include "ofApp.h"
#include "udpdata2.h"
//--------------------------------------------------------------
void ofApp::setup(){
    
//	auto res = ofSystemLoadDialog("load metadata json");
//	if(res.bSuccess){
//		info = ouster::sensor::metadata_from_json(res.getPath());
//	}
	
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	info = ouster::sensor::metadata_from_json("OS1_2048x10_64U.json");
	
//	res = ofSystemLoadDialog("load data csv");
	
//	vector<glm::highp_f64vec3> data;
//	ouster::LidarScan::Points points;
	
//	if(res.bSuccess){
//		auto buffer = ofBufferFromFile(res.getPath());
	
//	auto buffer = ofBufferFromFile("cloud_2.csv");
//
//		//Read file line by line
////		for (ofBuffer::Line it = buffer.getLines().begin(), end = buffer.getLines().end(); it != end; ++it) {
//		for(auto &line: buffer.getLines()){
//
//                //Split line into strings
//			vector<string> words = ofSplitString(line, ",");
//			if(words.size() != 3)
//			{
//				cout << "if(words.size() == 3)\n";
//				continue;
//			}
//			data.push_back({ofToDouble(words[0]),ofToDouble(words[1]),ofToDouble(words[2])});
//		}
//
//		cout << "data.size(): " << data.size() << endl;
//
//		points.resize(data.size(), Eigen::NoChange );
//
//		for(size_t i = 0; i < data.size(); i ++)
//		{
//			points(i,0) = data[i].x;
//			points(i,1) = data[i].y;
//			points(i,2) = data[i].z;
//		}
		
//	}

	
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
		
	
		auto n = H * W;
	
		std::vector<glm::vec3> xyz_data (n);
		
	auto xyz =xyz_lut->direction.data();
	
        for (size_t i = 0; i < n; i++) {
				xyz_data[i].x = static_cast<float>(xyz[i + n * 0]);
				xyz_data[i].y = static_cast<float>(xyz[i + n * 1]);
				xyz_data[i].z = static_cast<float>(xyz[i + n * 2]);
        }
		
		
		mesh.addVertices(xyz_data);
		
	mesh.setMode(OF_PRIMITIVE_POINTS);
	pointShader.load("point_program");

//	ouster::LidarScan scan;
	
	
	
//	ouster::XYZLut lut = ouster::make_xyz_lut(info);
//	   std::vector<ouster::LidarScan::Points> clouds;

//	   for (const LidarScan& scan : scans) {
//		   // compute a point cloud using the lookup table
//		   clouds.push_back(ouster::cartesian(scan, lut));
//
//		   // channel fields can be queried as well
//		   auto n_returns = (scan.field(LidarScan::Field::RANGE) != 0).count();
//
//		   std::cerr << "  Frame no. " << scan.frame_id << " with " << n_returns
//					 << " returns" << std::endl;
//	   }

	   
				
				



			ouster::LidarScan ls_write (W, H);
			auto _batchScan = ouster::ScanBatcher(W, packetFormat);
			
			
//			std::vector<uint8_t> lidar_buf(packetFormat.lidar_packet_size + 1);
	
//	cout << "packetFormat.lidar_packet_size: " << packetFormat.lidar_packet_size << "\n";
	
	
	bool bSuccess = false;
	
	
    std::vector<const unsigned char *> packets;

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


	
    
    for(auto data: packets){
        if (_batchScan(data, ls_write)) {
			cout << "batchscan break\n";
			
            break;
        }
        
    }

	
	
	
//	scan = ls_write;
	
	
	// pre-compute a table for efficiently calculating point clouds from range
//	   ouster::XYZLut lut = ouster::make_xyz_lut(info);
//	   ouster::LidarScan::Points cloud = ouster::cartesian(ls_write, lut);

	   
		   // compute a point cloud using the lookup table
		   

		   // channel fields can be queried as well
//		   auto n_returns = (ls_write.field(ouster::LidarScan::Field::RANGE) != 0).count();
//
//		   std::cout << "  Frame no. " << ls_write.frame_id << " with " << n_returns
//					 << " returns" << std::endl;
//
//			   std::string filename = ofToDataPath("cloudtest_"+ ofGetTimestampString()+"csv", true);
//			   std::ofstream out;
//			   out.open(filename);
//			   out << std::fixed << std::setprecision(4);
//
//			   // write each point, filtering out points without returns
//			   for (int i = 0; i < cloud.rows(); i++) {
//				   auto xyz = cloud.row(i);
//				   if (!xyz.isApproxToConstant(0.0))
//					   out << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
//			   }
//
//			   out.close();
//			   std::cout << "  Wrote " << filename << std::endl;

	
		renderer->render(ls_write);

//	if(bSuccess)
//	{
//
//	}else
//	{
//		cout << "data load failed\n";
//	}
//
}

//--------------------------------------------------------------
void ofApp::update(){

    

}

void ofApp::exit(){
}


//--------------------------------------------------------------
void ofApp::draw(){
  ofBackground(0,0,0);

//	cam.begin();
//	pointShader.begin();
//	mesh.draw();
//	pointShader.end();
//	cam.end();
	ofEnableDepthTest();

	renderer->draw();

	ofDisableDepthTest();
//
	renderer->drawGui();
//

	
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if(key == ' '){
//		renderer->render(scan);
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

