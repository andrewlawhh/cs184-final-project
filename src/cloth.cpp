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
        Vector3D particle_origin = Vector3D(center_particles[0] + i*INIT_OFFSET,
                          center_particles[1] + j*INIT_OFFSET,
                          center_particles[2] + k*INIT_OFFSET);
        Particle p = Particle(particle_origin, PARTICLE_RADIUS, particle_friction);
        particles.push_back(p);
      }
    }
  }
  printf("num particles generated - %lu\n", particles.size());
  double dimension = (num_particles - 1) * NN_RADIUS + 2 * PARTICLE_RADIUS;
  box_dimension = NN_RADIUS;
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {

  double mass = 1.0f;
  //double delta_t = 1.0f / frames_per_sec / simulation_steps;
  double delta_t = 0.0083;

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

  build_spatial_map();
  build_neighbor_tree();
  //populate_neighbors_fields();
  // At this line, p.neighbor_ptrs should contain a vector of pointers
  // to a Particle `p`'s neighboring particles. This is important because
  // lambda and delta_p are calculated using neighbors.

  for (int i = 0; i < solver_iterations; i++) {
    for (Particle& p : particles) {
      p.lambda = get_particle_lambda(p);
    }

    for (Particle& p : particles) {
      //printf("%f\n", p.lambda);
      p.delta_p = get_delta_p(p);
      p.pos_temp += p.delta_p;
    }

    for (Particle& p : particles) {
      for (CollisionObject* co : *collision_objects) {
        co->collide(p);
      }
      for (Particle* neighbor_ptr : p.neighbor_ptrs) {
        neighbor_ptr->collide(p);
      }
    }
  }
 
  for (Particle& p : particles) {
    p.velocity = (p.pos_temp - p.position) / delta_t;
    p.old_velocity = p.velocity;
    apply_vorticity_confinement(p, delta_t);
    p.velocity += get_viscosity_correction(p);
    p.position = p.pos_temp;
  }
}

// rebuild neighbor tree
void Cloth::build_neighbor_tree() {
  neighbors_list.clear();
  neighbors_list.resize(particles.size());
  for (int i = 0; i < particles.size(); i++) {
    Particle& p = particles[i];
    p.neighbor_ptrs = vector<Particle*>();

    tuple<double, double, double> contained = contained_box(p.pos_temp);
    double x = get<0>(contained);
    double y = get<1>(contained);
    double z = get<2>(contained);
    for (int diff_x = -1; diff_x <= 1; diff_x++) {
        for (int diff_y = -1; diff_y <= 1; diff_y++) {
            for (int diff_z = -1; diff_z <= 1; diff_z++) {
                tuple<double, double, double> neighbor_box = make_tuple(x + diff_x * box_dimension, y + diff_y * box_dimension, z + diff_z * box_dimension);
                float hash = hash_tuple(neighbor_box);
                std::unordered_map<float, vector<Particle*>*>::const_iterator res = map.find(hash);
                if (res != map.end()) {
                    vector<Particle*>* close = map[hash];
                    for (Particle* other : *close) {
                        if (&p == other) {
                            continue;
                        }
                        Vector3D displacement_vector = other->pos_temp - p.pos_temp;
                        if (displacement_vector.norm() < NN_RADIUS) {
                            p.neighbor_ptrs.push_back(other);
                        }
                    }
                }
            }
        }
    }
    
  } 
    // // BRUTE FORCE
    // for (int j = 0; j < particles.size(); j++) {
    //     Particle* other = &particles[j];
    //     if (&p == other) {
    //         continue;
    //     }
    //     Vector3D displacement_vector = other->pos_temp - p.pos_temp;
    //     if (displacement_vector.norm() < NN_RADIUS) {
    //         //printf("neighbor added for %d\n", i);
    //         p.neighbor_ptrs.push_back(other);
    //     }
    // }
  
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (int i = 0; i < particles.size(); i++) {
    Particle& p = particles[i];
    tuple<double, double, double> contained = contained_box(p.pos_temp);
    float hash = hash_tuple(contained);

    std::unordered_map<float, vector<Particle*>*>::const_iterator res = map.find(hash);
    if (res == map.end()) {
      map[hash] = new vector<Particle*>();
    }
    map[hash]->push_back(&p);
  }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

}

tuple<double, double, double> Cloth::contained_box(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  // double l = 3 * width / num_particles;
  // return make_tuple(floor(pos.x / l), floor(pos.y / l), floor(pos.z / l));
    double new_x = pos.x - fmod(pos.x, box_dimension);
    double new_y = pos.y - fmod(pos.y, box_dimension);
    double new_z = pos.z - fmod(pos.z, box_dimension);

    return make_tuple(new_x, new_y, new_z);
}

float Cloth::hash_tuple(tuple<double, double, double> t) {
  return (get<0>(t) * 223 + get<1>(t)) * 181 + get<2>(t);
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

void Cloth::reset() {
  Particle *p = &particles[0];
  for (int i = 0; i < particles.size(); i++) {
    p->position = p->start_position;
    p->last_position = p->start_position;
    p->pos_temp = p->start_position;
    p->velocity = Vector3D(0);
    p->delta_p = Vector3D(0);
    p->forces = Vector3D(0);
    p->lambda = 0;
    p->omega = Vector3D(0);
    p->forces = Vector3D(0);
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
  return (315 / 64 / PI / pow(h, 9)) * pow(h*h - r_norm*r_norm, 3);
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
  return -(45 / PI / pow(h, 6)) * (r / r_norm) * pow(h - r_norm, 2);
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
  return - retval / (aggregate_gradient_norm2 + EPSILON);
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
  return -k * pow(numerator / denominator, n);
}

void Cloth::apply_vorticity_confinement(Particle& p, double delta_t) {
  // calculate w
  Vector3D result_omega = Vector3D(0);
  for (Particle* neighbor_ptr : p.neighbor_ptrs) {
    Vector3D velocity_diff = neighbor_ptr->velocity - p.velocity;
    Vector3D spiky_result = spiky_gradient(p.pos_temp - neighbor_ptr->pos_temp);
    result_omega += cross(velocity_diff, spiky_result);
  }
  Vector3D N = result_omega;// / result_omega.norm();
  double N_norm = N.norm();
  if (N_norm > 0.00001) {
    N.x /= N_norm;
    N.y /= N_norm;
    N.z /= N_norm;
  } 
  Vector3D corrective_force = cross(N, result_omega);
  p.velocity += delta_t * corrective_force;
}

Vector3D Cloth::get_viscosity_correction(const Particle& p) {
  double c = 0.005; // 0.01 suggested in paper
  Vector3D retval = Vector3D(0);
  for (Particle* neighbor_ptr : p.neighbor_ptrs) {
    Vector3D velocity_diff = neighbor_ptr->old_velocity - p.old_velocity;
    double poly6_result = poly6(p.pos_temp - neighbor_ptr->pos_temp);
    retval += velocity_diff * poly6_result;
  }
  retval *= c;
  return retval;
}
