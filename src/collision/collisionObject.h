#ifndef COLLISIONOBJECT
#define COLLISIONOBJECT

#include <nanogui/nanogui.h>

#include "../clothMesh.h"

using namespace CGL;
using namespace std;
using namespace nanogui;
struct Particle;
class CollisionObject {
public:
  virtual void render(GLShader &shader) = 0;
  virtual void collide(Particle &p) = 0;
  virtual bool set_incline_direction(Particle& pm) = 0;
  bool incline = false;
private:
  double friction;
};

#endif /* COLLISIONOBJECT */
