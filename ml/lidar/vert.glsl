#version 410
in vec3 in_position;

out vec3 frag_position;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    vec4 world_position = model * vec4(in_position, 1.0);

    frag_position = vec3(world_position);

    float distance = length(world_position.xyz);
    float point_size = 10.0 - distance / (50.0);
    gl_PointSize = clamp(point_size, 5.0, 50.0);

    gl_Position = projection * view * world_position;
}
