#version 330
uniform sampler2D uGrid;
in vec2 fragTexCoord;
out vec4 fragColor;

void main() {
    int grid_pixel = int(texture(uGrid, fragTexCoord).r * 255); 
    int pathfind_pixel = int(texture(uGrid, fragTexCoord).g * 255); 

    if (grid_pixel == 0) fragColor = vec4(0.51, 0.51, 0.51, 1.0);
    else if (grid_pixel == 1 ) fragColor = vec4(0.0, 0.0, 0.0, 1.0);
    else fragColor = vec4(1.0, 1.0, 1.0, 1.0);

    if(pathfind_pixel != 0){
        fragColor *= vec4(1.0, 0.6, 0.6, 1.0);
    }
}
