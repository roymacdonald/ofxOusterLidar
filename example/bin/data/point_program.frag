#version 150

// in float vcolor;
in vec4 vcolor;
// uniform sampler2D palette;

out vec4 color;

void main() {
	color = vcolor;
	// color = vec4(vcolor,vcolor, vcolor, 1.0);
	// color = vec4( 1.0);
    // color = texture2D(palette, vec2(vcolor, 1));
}
