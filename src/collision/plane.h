#ifndef COLLISIONOBJECT_PLANE_H
#define COLLISIONOBJECT_PLANE_H

#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "collisionObject.h"

using namespace nanogui;
using namespace CGL;
using namespace std;

struct Plane : public CollisionObject {
public:
  Plane(const Vector3D &point, const Vector3D &normal, double friction) {
      this->point = point;
      this->normal = normal.unit();
      this->friction = friction;

      this->sPoint = Vector3f(this->point.x, this->point.y, this->point.z);
      this->sNormal = Vector3f(this->normal.x, this->normal.y, this->normal.z);

      Vector3D parallel = Vector3D(this->normal.y - this->normal.z, this->normal.z - this->normal.x, this->normal.x - this->normal.y);
      parallel.normalize();
      Vector3f sParallel = Vector3f(this->normal.y - this->normal.z, this->normal.z - this->normal.x, this->normal.x - this->normal.y);
      sParallel.normalize();
      Vector3D cross = CGL::cross(this->normal, parallel);
      Vector3f sCross = sNormal.cross(sParallel);

      this->corner0 = point + 2 * (cross + parallel);
      this->corner1 = point + 2 * (cross - parallel);
      this->corner2 = point + 2 * (-cross + parallel);
      this->corner3 = point + 2 * (-cross - parallel);

      this->sCorner0 = sPoint + 2 * (sCross + sParallel);
      this->sCorner1 = sPoint + 2 * (sCross - sParallel);
      this->sCorner2 = sPoint + 2 * (-sCross + sParallel);
      this->sCorner3 = sPoint + 2 * (-sCross - sParallel);

      defaultValues();
  }

  Plane(const Vector3D& point, const Vector3D& normal, const Vector3D& corner0,
      const Vector3D& corner1, const Vector3D& corner2, const Vector3D& corner3, double friction) {
      this->point = point;
      this->normal = normal.unit();
      this->friction = friction;

      this->corner0 = corner0;
      this->corner1 = corner1;
      this->corner2 = corner2;
      this->corner3 = corner3;

      this->sPoint = Vector3f(this->point.x, this->point.y, this->point.z);
      this->sNormal = Vector3f(this->normal.x, this->normal.y, this->normal.z);
      this->sCorner0 = Vector3f(corner0.x, corner0.y, corner0.z);
      this->sCorner1 = Vector3f(corner1.x, corner1.y, corner1.z);
      this->sCorner2 = Vector3f(corner2.x, corner2.y, corner2.z);
      this->sCorner3 = Vector3f(corner3.x, corner3.y, corner3.z);


      defaultValues();
  }

  void defaultValues() {
      this->incline_direction = Vector3D(0, 1, 0);
      this->minX = -numeric_limits<double>::infinity();
      this->maxX = numeric_limits<double>::infinity();
      this->minY = -numeric_limits<double>::infinity();
      this->maxY = numeric_limits<double>::infinity();
      this->minZ = -numeric_limits<double>::infinity();
      this->maxZ = numeric_limits<double>::infinity();
  }
  void render(GLShader &shader);
  bool Plane::set_incline_direction(Particle& pm);
  bool Plane::bounded(Particle& pm);
  void collide(Particle &p);

  Vector3D point;
  Vector3D normal;
  double friction;

  Vector3D corner0;
  Vector3D corner1;
  Vector3D corner2;
  Vector3D corner3;

  Vector3f sPoint;
  Vector3f sNormal;
  Vector3f sCorner0;
  Vector3f sCorner1;
  Vector3f sCorner2;
  Vector3f sCorner3;

  Vector3D incline_direction;
  double minX;
  double maxX;
  double minY;
  double maxY;
  double minZ;
  double maxZ;
};

#endif /* COLLISIONOBJECT_PLANE_H */
