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

uniform mat4 extrinsic;

// out float vcolor;
out vec4  vcolor;
// out vec4 outPosition;

void main(){
    vec4 local_point;

// float scale = 100000.0f;
//   local_point = position * range * scale;
//   local_point += vec4(offset, 0.0);


    // if(range > 0){
      local_point =  vec4(position.xyz * range * range_scale, 1.0);
      // local_point = extrinsic * local_point;

    // } else {
    //   local_point = vec4(0, 0, 0, 1.0);
    // }
	// Here, we get the four columns of the transformation.
	// Since this version of GLSL doesn't have texel fetch,
	// we use texture2D instead. Numbers are chosen to index
	// the middle of each pixel.
	// |     r0     |     r1     |     r2     |     t     |
	// 0   0.125  0.25  0.375   0.5  0.625  0.75  0.875   1
    // vec4 r0 = texture(transformation, vec2(trans_index, 0.125));
    // vec4 r1 = texture(transformation, vec2(trans_index, 0.375));
    // vec4 r2 = texture(transformation, vec2(trans_index, 0.625));
    // vec4 t = texture(transformation, vec2(trans_index, 0.875));
	// mat4 car_pose = mat4(
	// 		r0.x, r0.y, r0.z, 0,
	// 		r1.x, r1.y, r1.z, 0,
	// 		r2.x, r2.y, r2.z, 0,
	// 		t.x, t.y, t.z, 1
	// 		);

    // outPosition = modelViewProjectionMatrix * car_pose * local_point;
    // outPosition = modelViewProjectionMatrix * vec4(position, 1.0);
    // gl_Position = modelViewProjectionMatrix * vec4(position.xyz * 1000, 1.0);
  gl_Position = modelViewProjectionMatrix * local_point;
  // gl_Position = modelViewProjectionMatrix * local_point;
  // gl_Position = modelViewProjectionMatrix * car_pose * local_point;
	// vcolor = sqrt(key);
	// float c = range/226326.f;
	
	// // float c = sqrt(key);
	float c = 1.0;
	vcolor = vec4(c, c,c, 1.0);


	// vcolor = vec4(offset, 1.0);	
	 // vcolor = vec4(((offset + 1.0)/2.0), 1.0);
	// vcolor = (position + 1.0)/2.0;
	// vcolor = vec4(trans_index, trans_index, trans_index, 1.0);	
    // vcolor = vec4(r0.xyz, 1.0);
    // vcolor = vec4(r1.xyz, 1.0);
    // vcolor = vec4(r2.xyz, 1.0);
    // vcolor = vec4(t.xyz, 1.0);
    // 
}
