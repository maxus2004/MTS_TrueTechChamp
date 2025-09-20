#version 330
uniform sampler2D uGrid;
in vec2 fragTexCoord;
out vec4 fragColor;

void main() {
    float v = texture(uGrid, fragTexCoord).r; 
    if (v < 0.33) fragColor = vec4(0.51, 0.51, 0.51, 1.0);
    else if (v < 0.66) fragColor = vec4(0.0, 0.0, 0.0, 1.0);
    else fragColor = vec4(1.0, 1.0, 1.0, 1.0);
}
