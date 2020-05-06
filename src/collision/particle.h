#ifndef COLLISIONOBJECT_PARTICLE_H
#define COLLISIONOBJECT_PARTICLE_H

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "collisionObject.h"

using namespace CGL;
using namespace std;

struct Particle : public CollisionObject {
public:
  Particle(const Vector3D &position, double radius, double friction, int num_lat = 40, int num_lon = 40)
      : start_position(position), position(position), last_position(position), radius(radius), radius2(radius*radius),
        friction(friction), m_sphere_mesh(Misc::SphereMesh(num_lat, num_lon)) {
      on_incline_direction = Vector3D(0, 1, 0);
  }

  void render(GLShader &shader);
  bool set_incline_direction(Particle& p);
  void collide(Particle &p);
  
  Vector3D position;
  Vector3D start_position;
  Vector3D last_position;
  Vector3D delta_p;
  Vector3D velocity;
  Vector3D old_velocity;
  Vector3D omega;
  Vector3D forces;
  double radius;
  double radius2;
  Vector3D pos_temp; // x_star
  double lambda;
  vector<Particle*> neighbor_ptrs;
  double friction;
  Misc::SphereMesh m_sphere_mesh;

  int count;
  Vector3D on_incline_direction;
};

#endif /* COLLISIONOBJECT_PARTICLE_H */
