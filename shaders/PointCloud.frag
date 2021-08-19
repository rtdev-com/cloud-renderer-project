#version 410

uniform vec4 u_color;

in vec4 v_position;
in vec4 v_density;

out vec4 out_color;

void main() {
  // float d = v_density.r;
  float d = 1 - v_density.r;
  out_color = vec4( d, d, d, 1 );
  // out_color = vec4( vec3( u_color ), v_density.r );
}
