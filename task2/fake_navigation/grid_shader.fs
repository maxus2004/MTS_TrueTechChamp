#version 330

uniform sampler2D uGrid;
uniform sampler2D uColorMap;
in vec2 fragTexCoord;
out vec4 fragColor;

void main() {
    int grid_pixel = int(texture(uGrid, fragTexCoord).r * 255); 
    int allowed_pixel = int(texture(uGrid, fragTexCoord).g * 255); 
    int pathfind_pixel = int(texture(uGrid, fragTexCoord).b * 255);

    vec4 baseColor;

    if (grid_pixel == 1) {
        baseColor = vec4(0.0, 0.0, 0.0, 1.0);
    }
    else if (grid_pixel == 0) {
        vec4 c = texture(uColorMap, fragTexCoord);
        float grayFactor = 0.3;
        baseColor = vec4(
            c.b * grayFactor,
            c.g * grayFactor,
            c.r * grayFactor,
            1.0
        );
        
        baseColor += vec4(0.5, 0.5, 0.5, 0.0);
    }
    else if (grid_pixel == 2) {
        
        vec4 c = texture(uColorMap, fragTexCoord);
        baseColor = vec4(
            1.0 - c.b,  
            1.0 - c.g,  
            1.0 - c.r,  
            1.0
        );
    }
    else {
        baseColor = vec4(1.0, 1.0, 1.0, 1.0);
    }

    if (allowed_pixel != 0) {
        baseColor *= vec4(1.0, 0.6, 0.6, 1.0);
    }

    if (allowed_pixel == 0 && pathfind_pixel != 0) {
        baseColor = vec4(pathfind_pixel%10/20.0+0.5, pathfind_pixel%10/20.0+0.5, 1.0, 1.0);
    }

    fragColor = baseColor;
}
