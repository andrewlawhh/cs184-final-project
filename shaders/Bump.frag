#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_3, uv).x;
 }
void main() {
  // YOUR CODE HERE
  
  vec3 b = cross(vec3(v_normal), vec3(v_tangent));
  mat3 tbn;
  tbn[0] = vec3(v_tangent);
  tbn[1] = b;
  tbn[2] = vec3(v_normal);


  float du = (h(vec2(v_uv.x + 1 / u_texture_3_size.x ,v_uv.y)) - h(v_uv)) * u_normal_scaling * u_height_scaling;
  float dv = (h(vec2(v_uv.x  ,v_uv.x + 1 / u_texture_3_size.y)) - h(v_uv)) * u_normal_scaling * u_height_scaling;

  vec3 no = vec3(-du, -dv, 1);
  vec3 nd = tbn * no;

  vec3 pos = vec3(v_position);
  vec3 nor = nd;
  vec3 l = normalize(u_light_pos - pos);
  float d = length(u_light_pos -pos);
  float cos = max(0.0, dot(normalize(nor), l));
  vec3 diffuse = (u_light_intensity / (d*d)) * cos;



  vec3 v = normalize(u_cam_pos - pos);
  vec3 h = normalize(v + l);
  float cos2 = max(0.0, dot(normalize(nor), h));
  vec3 specular = (u_light_intensity / (d*d)) * pow(cos2,20) ;

  
  vec3 temp = diffuse + specular + vec3(0.0,0.0,0.0);
  temp = vec3(0.15,0.15,0.15) + specular + diffuse;
  out_color = vec4(temp,  1);
  //out_color = vec4(1,1,1,1);
}

