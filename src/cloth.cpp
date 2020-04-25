#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "collision/particle.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
}

Cloth::~Cloth() {
  particles.clear();
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  
  // num_particles for each axis
  for (int i = -num_particles/2; i < num_particles/2; i++) {
    for (int j = -num_particles/2; j < num_particles/2; j++) {
      for (int k = -num_particles/2; k < num_particles/2; k++) {
        Vector3D particle_origin = Vector3D(center_particles[0] + i*0.05,
                          center_particles[1] + j*0.05,
                          center_particles[2] + k*0.05);
        particles.push_back(Particle(particle_origin, 0.005, particle_friction));
      }
    }
  }
  printf("num particles generated - %lu\n", particles.size());
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = 1.0f;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  /*
  Implement PositionBasedFluids paper algorithm
  */
  // apply gravity (external_accelerations)
  // predict position pos_temp
  for (Particle& p : particles) {
    for (Vector3D external_accel : external_accelerations) {
      Vector3D external_force = mass * external_accel;
      p.velocity += delta_t * external_force;
    }
    p.last_position = p.position;
    p.pos_temp = p.position + delta_t * p.velocity;
  }

  build_neighbor_tree();
  for (Particle& p : particles) {
    // Find neighboring particles
  }
  for (int i = 0; i < solver_iterations; i++) {
    for (Particle& p : particles) {
      // calculate lambda for particle p
    }
    for (Particle& p : particles) {
      // Calculate delta p for particle p
      // perform collision detection and response
      for (Particle& p : particles) {
        for (CollisionObject* co : *collision_objects) {
          co->collide(p);
        }
      }
    }
    for (Particle& p : particles) {
      // update pos_temp = pos_temp + delta p
    }
  }
  for (Particle& p : particles) {
    // update velocity
    // apply vorticity confinement and xsph viscosity
    // update position = pos_temp
    p.position = p.pos_temp;
  }
}

void Cloth::build_neighbor_tree() {

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

  return 0.f; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  Particle *p = &particles[0];
  for (int i = 0; i < particles.size(); i++) {
    p->position = p->start_position;
    p->last_position = p->start_position;
    p++;
  }
}
