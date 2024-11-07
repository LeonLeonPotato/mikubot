#version 410
layout(location = 0) in vec3 frag_position; 
out vec4 fragColor;

void main() {
    float distance = length(frag_position);
    float alpha = 1.0 - (distance / 500.0);
    alpha = clamp(alpha, 0.2, 1.0);
    fragColor = vec4(alpha, alpha, alpha, 1.0);
}