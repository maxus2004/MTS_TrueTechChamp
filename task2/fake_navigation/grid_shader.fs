#version 330

uniform sampler2D grid;
uniform sampler2D cost_grid;
uniform sampler2D pathfind_grid;
uniform float time;

in vec2 fragTexCoord;
out vec4 fragColor;

void main() {
    int grid_pixel = int(texture(grid, fragTexCoord).r * 255); 
    float cost_pixel = texture(cost_grid, fragTexCoord).r;
    float pathfind_pixel = texture(pathfind_grid, fragTexCoord).r;

    vec4 baseColor;

    if (grid_pixel == 1) {
        baseColor = vec4(0.0, 0.0, 0.0, 1.0);
    }
    else if (grid_pixel == 0) {
        baseColor = vec4(0.5, 0.5, 0.5, 1.0);
    }
    else {
        baseColor = vec4(1.0, 1.0, 1.0, 1.0);
    }

    baseColor *= vec4(1.0, 1-cost_pixel, 1-cost_pixel, 1.0);

    if(pathfind_pixel<10000 && pathfind_pixel!=0){
        baseColor *= vec4(int(-pathfind_pixel-time*50)%40/80.0+0.5, int(-pathfind_pixel-time*50)%40/80.0+0.5, 1.0, 1.0);
    }

    fragColor = baseColor;
}
