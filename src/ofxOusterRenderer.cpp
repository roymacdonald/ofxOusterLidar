#include "ofxOusterRenderer.hpp"




ofxOusterRenderer::ofxOusterRenderer(const ouster::sensor::sensor_info & info):ofxPointShader("Ouster Renderer")
{
    
    
    this -> setRangeUnit( ouster::sensor::range_unit);
    
    makeLut(info);
    
    
}

void glmToEigenMat(const glm::mat4& mat, mat4d & eMatrix){
    for(size_t i = 0; i < 4; i++){
        for(size_t j = 0; j < 4; j++){
            eMatrix(i, j) = mat[j][i];
        }
    }
}

void ofxOusterRenderer::makeLut(const ouster::sensor::sensor_info & info){

    glm::quat q(glm::radians(glm::vec3(-90.0, 90.0, 0)));
    

    
    /// the default coordinates space is rotated, in relationship to OF's.
    /// Here it is corrected so y axis = up/down, x axis = right/left, z = closest/furthest
    /// also scale so 1 pixel == 1 millimeter
    
    auto transformMatrix =  glm::toMat4(q);
    auto correctionMatrix = glm::scale(transformMatrix, glm::vec3(1000.0f));
    
    
    mat4d eMatrix;
    glmToEigenMat(correctionMatrix, eMatrix);
        
    
    lut = ouster::make_xyz_lut(info.format.columns_per_frame, info.format.pixels_per_column,
    ouster::sensor::range_unit, info.lidar_origin_to_beam_origin_mm,
    eMatrix * info.lidar_to_sensor_transform ,
    info.beam_azimuth_angles,
    info.beam_altitude_angles);
    
    
}



void ofxOusterRenderer::render(const ouster::LidarScan& _readScan)
{

    auto range = _readScan.field(ouster::sensor::ChanField::RANGE);
    
    auto cloud = ouster::cartesian(range, lut);
        
    points.clear();
    points.setMode(OF_PRIMITIVE_POINTS);
    for(size_t i = 0; i < cloud.rows(); i++){
        points.addVertex({cloud(i, 0),cloud(i, 1),cloud(i, 2)});
    }

}


void ofxOusterRenderer::drawPointCloud()
{
    begin();
    ofSetColor(255);
    points.draw();
    end();
}

void ofxOusterRenderer::draw(ofEasyCam &cam)
{
    
    cam.begin();
    drawPointCloud();
    cam.end();
    
}


const vector<glm::vec3>& ofxOusterRenderer::getPointCloud(){
    
    return points.getVertices();
}


