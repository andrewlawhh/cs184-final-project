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
  // TODO - clean this up
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
  populate_neighbors_fields();

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

// rebuild neighbor tree
void Cloth::build_neighbor_tree() {
  neighbors_list.clear();
  neighbors_list.resize(particles.size());
  // NAIVE APPROACH. REPLACE WITH KD TREE.
  for (int i = 0; i < particles.size(); i ++) {
    Particle& p = particles[i];
    for (int j = 0; j < particles.size(); j++) {
      if (i == j) continue;
      Particle& other = particles[j];
      Vector3D displacement_vector = other.pos_temp - p.pos_temp;
      if (displacement_vector.norm() < NN_RADIUS) {
        printf("neighbor added for %d\n", i);
        neighbors_list[i].push_back(&particles[j]);
      }
    }
  }
}

// Assign `neighbor_ptrs` in each particle using the list
void Cloth::populate_neighbors_fields() {
  for (int i = 0; i < particles.size(); i++) {
    particles[i].neighbor_ptrs = neighbors_list[i];
  }
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

// `W` kernel density function
// https://nccastaff.bournemouth.ac.uk/jmacey/MastersProjects/MSc15/06Burak/BurakErtekinMScThesis.pdf
double Cloth::poly6(Vector3D r) {
  double h = NN_RADIUS;
  double r_norm = r.norm();
  if (r_norm < 0 || r_norm > h) {
    return 0;
  }
  return (315 / 64 / M_PI / pow(h, 9)) * pow(h*h - r_norm*r_norm, 3);
}

// Spiky kernel for gradient calculation
// https://nccastaff.bournemouth.ac.uk/jmacey/MastersProjects/MSc15/06Burak/BurakErtekinMScThesis.pdf
Vector3D Cloth::spiky_gradient(Vector3D r) {
  double h = NN_RADIUS;
  double r_norm = r.norm();
  if (r_norm < 0 || r_norm > h) {
    return Vector3D(0);
  }
  return -(45 / M_PI / pow(h, 6)) * (r / r_norm) * pow(h - r_norm, 2);
}