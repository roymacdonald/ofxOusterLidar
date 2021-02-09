#version 150

in vec4 position;
in vec3 offset;
in float range;
in float key;
in float trans_index;

uniform sampler2D transformation;
uniform mat4 modelMatrix;
uniform mat4 modelViewProjectionMatrix;
uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform mat4 textureMatrix;
uniform mat4 globalColor;
uniform float range_scale;
uniform float range_max;
uniform mat4 extrinsic;
uniform float colorMapSize;

 out float vcolor;
//out vec4  vcolor;
// out vec4 outPosition;

void main(){
    vec4 local_point;

    local_point =  vec4(position.xyz * range * range_scale, 1.0);
    vcolor = (range * colorMapSize)/(226326.f*range_max);
    gl_Position = modelViewProjectionMatrix * local_point;
}