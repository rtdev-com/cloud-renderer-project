#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;

uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;

out vec4 out_color;

void main() {

  out_color = u_color;
}
