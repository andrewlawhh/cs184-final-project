#version 330

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 in_position;
in vec4 in_normal;
in vec4 in_tangent;
in vec2 in_uv;

out vec4 v_position;
out vec4 v_normal;
out vec2 v_uv;
out vec4 v_tangent;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_3, uv).x;
}

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  

  vec4 change = in_normal * h(in_uv) * u_height_scaling;
  vec4 np = in_position + change;
  
  v_position = u_model * np;


  //v_position = u_model * in_position;
  v_normal = normalize(u_model * in_normal);
  
 
  v_uv = in_uv;
  v_tangent = normalize(u_model * in_tangent);
  gl_Position = u_view_projection * v_position;
}
