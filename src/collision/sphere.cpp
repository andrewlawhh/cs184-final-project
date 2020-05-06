#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"
#include "particle.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(Particle &p) {
  // TODO (Part 3): Handle collisions with spheres.
  Vector3D origin_to_pm = p.position - this->origin;
  if (origin_to_pm.norm() <= this->radius) {
    Vector3D tangent_point = this->origin + origin_to_pm.unit() * this->radius;
    Vector3D correction_vector = tangent_point - p.position;
    p.position = p.last_position + correction_vector * (1 - this->friction);
  }
}

bool Sphere::set_incline_direction(Particle& p) {
    return false;
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
