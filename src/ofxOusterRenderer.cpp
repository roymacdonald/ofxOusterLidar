#include "ofxOusterRenderer.hpp"


#define STRINGIFY(x) #x


ofxOusterRenderer::ofxOusterRenderer(const ouster::sensor::sensor_info & info):ofxPointShader("Ouster Renderer")
{
    
    
    setRangeUnit( ouster::sensor::range_unit);
    
    lut = makeLut(info);
    
//    cout << "offset rows: " << lut.offset.rows() << " cols: " << lut.offset.cols()  <<endl;
//    cout << "direction rows: " << lut.direction.rows() << " cols: " << lut.direction.cols()  <<endl;
    
//    loadShaderFromFiles("shader.vert", "shader.frag");
//    listener = shader.shaderReloaded.newListener(this, &ofxOusterRenderer::_setupVertexBufferObjects);
    
    static const string shader_header = "#version 150\n";
          
          static const string fragShader = shader_header + STRINGIFY(
          in float vcolor;
          uniform sampler2DRect palette;
          out vec4 color;
          void main() {
              color = texture(palette, vec2(vcolor, 1));
          }
          );
          
          string vertShader = shader_header + STRINGIFY(
            in vec4 position;
            in vec3 direction;
            in float range;
            uniform mat4 modelViewProjectionMatrix;
            uniform float range_max;
            uniform float colorMapSize;
            out float vcolor;
            void main(){
                vcolor = (range * colorMapSize)/range_max;
                gl_Position = modelViewProjectionMatrix * (position  +(vec4(direction, 0.0f) * range));
            }
          );
          
    
    loadShader(vertShader, fragShader);
    
    
    
    _setupMesh();

}

void ofxOusterRenderer::_setupMesh(){
    vector<glm::vec3> direction (lut.direction.rows(),{0.0f,0.0f,0.0f});
    auto& v = mesh.getVertices();
    v.resize(direction.size());
    for(size_t i = 0; i < lut.direction.rows(); i++){
        direction[i] = {lut.direction(i, 0), lut.direction(i, 1), lut.direction(i, 2)};
        v[i] = {lut.offset(i, 0), lut.offset(i, 1), lut.offset(i, 2)};
    }
    
    mesh.getVbo().setAttributeData(shader.getAttributeLocation("direction"), &direction[0].x, 3, direction.size() , GL_STATIC_DRAW, sizeof(glm::vec3));

    
    mesh.setMode(OF_PRIMITIVE_POINTS);
}

void glmToEigenMat(const glm::mat4& mat, mat4d & eMatrix){
    for(size_t i = 0; i < 4; i++){
        for(size_t j = 0; j < 4; j++){
            eMatrix(i, j) = mat[j][i];
        }
    }
}

ouster::XYZLut ofxOusterRenderer::makeLut(const ouster::sensor::sensor_info & info){

    
    
    glm::quat q(glm::radians(glm::vec3(-90.0, 90.0, 0)));
    

    
    /// the default coordinates space is rotated, in relationship to OF's.
    /// Here it is corrected so y axis = up/down, x axis = right/left, z = closest/furthest
    /// also scale so 1 pixel == 1 millimeter
    
    auto transformMatrix =  glm::toMat4(q);
    auto correctionMatrix = glm::scale(transformMatrix, glm::vec3(1000.0f));
    
    
    mat4d eMatrix;
    glmToEigenMat(correctionMatrix, eMatrix);
        
    
    return ouster::make_xyz_lut(info.format.columns_per_frame, info.format.pixels_per_column,
    ouster::sensor::range_unit, info.lidar_origin_to_beam_origin_mm,
    eMatrix * info.lidar_to_sensor_transform ,
    info.beam_azimuth_angles,
    info.beam_altitude_angles);
    
    
}



void ofxOusterRenderer::render(const ouster::LidarScan& _readScan)
{

    ouster::img_t<GLfloat> range = _readScan.field(ouster::sensor::ChanField::RANGE).cast<GLfloat>();
    shader.begin();
    mesh.getVbo().setAttributeData(shader.getAttributeLocation("range"), range.data(), 1, range.rows() * range.cols() , GL_STATIC_DRAW, sizeof(GLfloat));

    shader.end();
}


void ofxOusterRenderer::drawPointCloud()
{
    
    ofSetColor(255);
    begin();
    mesh.draw();
    end();
    
}

void ofxOusterRenderer::draw(ofEasyCam &cam)
{
    cam.begin();
    drawPointCloud();
    cam.end();
    
}


