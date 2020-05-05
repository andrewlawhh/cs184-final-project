#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 pos = vec3(v_position);
  vec3 nor = vec3(v_normal);
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
 
}

