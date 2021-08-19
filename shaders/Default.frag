#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;

uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;

out vec4 out_color;

void main() {

  float r = distance( vec3( v_position ), u_light_pos );
  vec4 l = normalize( vec4( u_light_pos - vec3( v_position ), 1 ) );
  vec3 k = vec3( 1, 1, 1 );
  vec3 L = k * ( ( u_light_intensity * vec3( u_color ) ) / (r * r) ) * max( 0, dot( v_normal, l ) );
  out_color = vec4( L, 1 );
}
