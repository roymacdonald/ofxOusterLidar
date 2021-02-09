#version 150

//<<<<<<< HEAD
// in float vcolor;
////in vec4 vcolor;
// uniform sampler2DRect palette;
//
//out vec4 color;
//
//void main() {
////	color = vcolor;
//	// color = vec4(vcolor,vcolor, vcolor, 1.0);
//	 // color = vec4( 1.0);
//    color = texture(palette, vec2(vcolor, 1)); 
//=======
in float vcolor;
uniform sampler2DRect palette;
out vec4 color;

void main() {
    color = texture2D(palette, vec2(vcolor, 1));
    //color = vec4(1.0, 0.0, 0.0, 1.0);
//>>>>>>> mgs2
}
