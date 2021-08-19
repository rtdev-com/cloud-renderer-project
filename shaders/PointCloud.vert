#version 410

// Uniform variables are constant throughout the entire shader
// execution. They are also read-only to enable parallelization.
uniform mat4 u_model;
uniform mat4 u_view_projection;
uniform vec3 u_cam_pos;
// uniform vec3 u_color;
uniform float pt_size;
uniform vec3 u_offset;

// In a vertex shader, the "in" variables are read-only per-vertex 
// properties. An example of this was shown in the rasterizer project, 
// where each vertex had an associated "color" or "uv" value which we 
// would later interpolate using barycentric coordinates.
in vec4 in_position;
in vec4 in_density;

// In a vertex shader, the "out" variables are per-vertex properties
// that are read/write. These properties allow us to communicate
// information from the vertex shader to the fragment shader.
// That is, in the linked fragment shader, these values become the 
// "in" variables.
out vec4 v_position;
out vec4 v_density;

// Every shader features a "main" function.
// This is typically where we write to the "out" variables that the
// fragment shaders get to use. It is also where "gl_Position" is set,
// which is the final screen-space location of this vertex which the
// GPU's triangle rasterizer takes in.
void main() {
  // Here, we just apply the model's transformation to the various
  // per-vertex properties. That way, when the fragment shader reads
  // them, we already have the position in world-space.

  vec4 pos = in_position + vec4( u_offset, 1 );

  v_position = u_model * pos;
  v_density = in_density;
  
  // The final screen-space location of this vertex which the
  // GPU's triangle rasterizer takes in.
  gl_Position = u_view_projection * v_position;

  float d = distance( pos, vec4( u_cam_pos, 1 ) );
  gl_PointSize = pt_size * pt_size / ( d * d );
}
