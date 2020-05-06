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
  Vector3D displacement_vector = pm.pos_temp - pm.position;
  Vector3D direction = displacement_vector.unit();
  double t_position = displacement_vector.norm();
  double t_plane = dot((this->point - pm.position), this->normal) / dot(direction, this->normal);
  
  //Adding maxY to plane with normal (0, 1, -1) makes points fall through. Why??? Particles start off above maxY, but should be less than maxY when they collide with plane.
  //if (pm.position.y > this->maxY&& abs(t_position) >= abs(t_plane)) {
  //    cout << "here\n";
  //}

  if (abs(t_position) >= abs(t_plane) && bounded(pm)) {
    Vector3D tangent_point =  pm.position + direction * t_plane;
    Vector3D correction_vector = (tangent_point - pm.position) * (1 - SURFACE_OFFSET);
    pm.pos_temp = pm.position + correction_vector * (1 - this->friction);
  } 
}

bool Plane::set_incline_direction(Particle& pm) {
    if (abs(dot(this->point - pm.position, this->normal)) <= SURFACE_OFFSET) {
        pm.on_incline_direction = this->incline_direction;
        return true;
    }
    return false;
}

bool Plane::bounded(Particle& pm) {
    Vector3D pos = pm.position;
    if (this->minX <= pos.x && pos.x <= this->maxX) {
        if (this->minY <= pos.y && pos.y <= this->maxY) {
            if (this->minZ <= pos.z && pos.z <= this->maxZ) {
                return true;
            }
        }
    }
    return false;
}

void Plane::render(GLShader &shader) {
    nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

    MatrixXf positions(3, 4);
    MatrixXf normals(3, 4);

    positions.col(0) << this->sCorner0;
    positions.col(1) << this->sCorner1;
    positions.col(2) << this->sCorner2;
    positions.col(3) << this->sCorner3;

    normals.col(0) << this->sNormal;
    normals.col(1) << this->sNormal;
    normals.col(2) << this->sNormal;
    normals.col(3) << this->sNormal;

    if (shader.uniform("u_color", false) != -1) {
     shader.setUniform("u_color", nanogui::Color(10,10,10,0));
    }
    shader.uploadAttrib("in_position", positions);
    if (shader.attrib("in_normal", false) != -1) {
     shader.uploadAttrib("in_normal", normals);
    }

    shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
