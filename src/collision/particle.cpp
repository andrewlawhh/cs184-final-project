#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "particle.h"

using namespace nanogui;
using namespace CGL;

void Particle::collide(Particle &p) {
  // TODO (Part 3): Handle collisions with spheres.
  Vector3D origin_to_pm = p.position - this->position;
  if (origin_to_pm.norm() <= this->radius) {
    Vector3D tangent_point = this->position + origin_to_pm.unit() * this->radius;
    Vector3D correction_vector = tangent_point - p.position;
    p.position = p.last_position + correction_vector * (1 - this->friction);
  }
}

bool Particle::set_incline_direction(Particle& p) {
    return false;
}

void Particle::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  shader.setUniform("u_color", nanogui::Color(28,163,236,200), false);
  m_sphere_mesh.draw_sphere(shader, position, radius * 0.92);
}
