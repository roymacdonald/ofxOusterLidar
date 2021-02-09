#version 150

in float vcolor;
uniform sampler2DRect palette;
out vec4 color;

void main() {
    color = texture2D(palette, vec2(vcolor, 1));
    //color = vec4(1.0, 0.0, 0.0, 1.0);
}
