#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "particle.h"

using namespace nanogui;
using namespace CGL;

void Particle::collide(Particle &p) {
  // TODO (Part 3): Handle collisions with spheres.

}

void Particle::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, position, radius * 0.92);
}
