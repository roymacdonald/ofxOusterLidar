#version 320

in float vcolor;
uniform sampler2D palette;

out vec4 color;

void main() {
    color = texture2D(palette, vec2(vcolor, 1));
}
