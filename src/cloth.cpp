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
  double radius = 0.05;
  double offset = 2 * radius;
  // num_particles for each axis
  // TODO - clean this up
  for (int i = -num_particles/2; i < num_particles/2; i++) {
    for (int j = -num_particles/2; j < num_particles/2; j++) {
      for (int k = -num_particles/2; k < num_particles/2; k++) {
        Vector3D particle_origin = Vector3D(center_particles[0] + i*offset,
                          center_particles[1] + j*offset,
                          center_particles[2] + k*offset);
        particles.push_back(Particle(particle_origin, 0.02, particle_friction));
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
  // At this line, p.neighbor_ptrs should contain a vector of pointers
  // to a Particle `p`'s neighboring particles. This is important because
  // lambda and delta_p are calculated using neighbors.

  for (int i = 0; i < solver_iterations; i++) {
    for (Particle& p : particles) {
      p.lambda = get_particle_lambda(p);
    }
    for (Particle& p : particles) {
      p.delta_p = get_delta_p(p);
      p.pos_temp += p.delta_p;
    }
    for (Particle& p : particles) {
      for (CollisionObject* co : *collision_objects) {
        co->collide(p);
      }
    }
  }
  for (Particle& p : particles) {
    p.velocity = (1 / delta_t) * (p.pos_temp - p.position);
    apply_vorticity_confinement(p, delta_t);
    p.velocity += get_viscosity_correction(p);
    p.position = p.pos_temp;
  }
}

// rebuild neighbor tree
void Cloth::build_neighbor_tree() {
  neighbors_list.clear();
  neighbors_list.resize(particles.size());
  // NAIVE APPROACH. REPLACE WITH KD TREE OR SPATIAL MAP
  for (int i = 0; i < particles.size(); i ++) {
    Particle& p = particles[i];
    for (int j = 0; j < particles.size(); j++) {
      if (i == j) continue;
      Particle& other = particles[j];
      Vector3D displacement_vector = other.pos_temp - p.pos_temp;
      if (displacement_vector.norm() < NN_RADIUS) {
        //printf("neighbor added for %d\n", i);
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
///////////////////////////////////////////////////////

void Cloth::reset() {
  Particle *p = &particles[0];
  for (int i = 0; i < particles.size(); i++) {
    p->position = p->start_position;
    p->last_position = p->start_position;
    p->velocity = 0;
    p->delta_p = Vector3D(0);
    p->forces = Vector3D(0);
    p->lambda = 0;
    p->omega = Vector3D(0);
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
  if (r_norm == 0) {
    r_norm = EPS_D;
  }
  if (r_norm < 0 || r_norm > h) {
    return Vector3D(0);
  }
  return -(45 / M_PI / pow(h, 6)) * (r / r_norm) * pow(h - r_norm, 2);
}

// Density constraint
double Cloth::C(const Particle& p) {
  double rho_i = 0;
  for (Particle* neighbor_ptr: p.neighbor_ptrs) {
    rho_i += poly6(p.pos_temp - neighbor_ptr->pos_temp);
  }
  return rho_i / REST_DENSITY - 1;
}

// Gradient of density constraint with respect to pk
Vector3D Cloth::gradient_C(const Particle& p, const Particle& k) {
  Vector3D retval = Vector3D(0);
  for (Particle* neighbor_ptr: p.neighbor_ptrs) {
    retval += spiky_gradient(p.pos_temp - neighbor_ptr->pos_temp);
  }
  retval /= REST_DENSITY;
  return retval;
}

// Lambda_i = C / (delta C summed over all k)
double Cloth::get_particle_lambda(const Particle& p) {
  double retval = -C(p);
  double aggregate_gradient_norm2 = 0;
  for (Particle* neighbor_ptr : p.neighbor_ptrs) {
    aggregate_gradient_norm2 += gradient_C(p, *neighbor_ptr).norm2();
  }
  return - C(p) / (aggregate_gradient_norm2 + EPSILON);
}

Vector3D Cloth::get_delta_p(const Particle& p) {
  Vector3D retval = Vector3D(0);
  for (Particle* neighbor_ptr : p.neighbor_ptrs) {
    retval += (p.lambda + neighbor_ptr->lambda + tensile_instability_correction(p, *neighbor_ptr)) * 
      spiky_gradient(p.pos_temp - neighbor_ptr->pos_temp);
  }
  retval /= REST_DENSITY;
  return retval;
}

// Referenced as s_corr in the paper
double Cloth::tensile_instability_correction(const Particle& p, const Particle& neighbor) {
  double k = 0.1; // 0.1 suggested in the paper
  int n = 4;      // suggested in the paper
  Vector3D delta_q = Vector3D(0.11547) * NN_RADIUS; // magic number calculated using heuristic in the paper
  double numerator = poly6(p.pos_temp - neighbor.pos_temp);
  double denominator = poly6(delta_q);
  return -k * pow(numerator / denominator, 4);
}

void Cloth::apply_vorticity_confinement(Particle& p, double delta_t) {
  // calculate w
  Vector3D result_omega = Vector3D(0);
  for (Particle* neighbor_ptr : p.neighbor_ptrs) {
    Vector3D velocity_diff = neighbor_ptr->velocity - p.velocity;
    Vector3D spiky_result = spiky_gradient(p.pos_temp - neighbor_ptr->pos_temp);
    result_omega += cross(velocity_diff, spiky_result);
  }
  Vector3D N = result_omega / result_omega.norm();
  N.normalize();
  Vector3D corrective_force = cross(N, result_omega);
  p.velocity += delta_t * corrective_force;
}

Vector3D Cloth::get_viscosity_correction(const Particle& p) {
  double c = 0.0001; // 0.01 suggested in paper
  Vector3D retval = Vector3D(0);
  for (Particle* neighbor_ptr : p.neighbor_ptrs) {
    Vector3D velocity_diff = neighbor_ptr->velocity - p.velocity;
    double poly6_result = poly6(p.pos_temp - neighbor_ptr->pos_temp);
    retval += velocity_diff * poly6_result;
  }
  retval *= c;
  return retval;
}
