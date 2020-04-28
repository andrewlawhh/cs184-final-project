#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"
#include "particle.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(Particle &pm) {
  // TODO (Part 3): Handle collisions with planes.
  Vector3D displacement_vector = pm.pos_temp - pm.position;
  Vector3D direction = displacement_vector.unit();
  double t_position = displacement_vector.norm();
  double t_plane = dot((this->point - pm.position), this->normal) / dot(direction, this->normal);
  if (abs(t_position) >= abs(t_plane)) {
    Vector3D tangent_point =  pm.position + direction * t_plane;
    Vector3D correction_vector = (tangent_point - pm.position) * (1 - SURFACE_OFFSET);
    pm.pos_temp = pm.position + correction_vector * (1 - this->friction);
  }
}

void Plane::render(GLShader &shader) {
   //nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

   //Vector3f sPoint(point.x, point.y, point.z);
   //Vector3f sNormal(normal.x, normal.y, normal.z);
   //Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
   //                   normal.x - normal.y);
   //sParallel.normalize();
   //Vector3f sCross = sNormal.cross(sParallel);

   //MatrixXf positions(3, 4);
   //MatrixXf normals(3, 4);

   //positions.col(0) << sPoint + 2 * (sCross + sParallel);
   //positions.col(1) << sPoint + 2 * (sCross - sParallel);
   //positions.col(2) << sPoint + 2 * (-sCross + sParallel);
   //positions.col(3) << sPoint + 2 * (-sCross - sParallel);

   //normals.col(0) << sNormal;
   //normals.col(1) << sNormal;
   //normals.col(2) << sNormal;
   //normals.col(3) << sNormal;

   //if (shader.uniform("u_color", false) != -1) {
   //  shader.setUniform("u_color", nanogui::Color(10,10,10,0));
   //}
   //shader.uploadAttrib("in_position", positions);
   //if (shader.attrib("in_normal", false) != -1) {
   //  shader.uploadAttrib("in_normal", normals);
   //}

   //shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
