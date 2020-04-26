#ifndef CLOTH_H
#define CLOTH_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "clothMesh.h"
#include "collision/collisionObject.h"
#include "collision/particle.h"
#include "spring.h"

using namespace CGL;
using namespace std;

enum e_orientation { HORIZONTAL = 0, VERTICAL = 1 };

// Physics constants
const double PARTICLE_RADIUS = 0.05;
const double NN_RADIUS = 3.2 * PARTICLE_RADIUS;
const double REST_DENSITY = 25000;
const double EPSILON = 5000;
const double INIT_OFFSET = PARTICLE_RADIUS * 3;//pow(4 * pow(PARTICLE_RADIUS,3) * M_PI / 3 / 64, 1/3);

struct ClothParameters {
  ClothParameters() {}
  ClothParameters(bool enable_structural_constraints,
                  bool enable_shearing_constraints,
                  bool enable_bending_constraints, double damping,
                  double density, double ks)
      : enable_structural_constraints(enable_structural_constraints),
        enable_shearing_constraints(enable_shearing_constraints),
        enable_bending_constraints(enable_bending_constraints),
        damping(damping), density(density), ks(ks) {}
  ~ClothParameters() {}

  // Global simulation parameters

  bool enable_structural_constraints;
  bool enable_shearing_constraints;
  bool enable_bending_constraints;

  double damping;

  // Mass-spring parameters
  double density;
  double ks;
};

struct Cloth {
  Cloth() {}
  Cloth(double width, double height, int num_width_points,
        int num_height_points, float thickness);
  ~Cloth();

  void buildGrid();

  void simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();
  // void buildClothMesh();
  void build_neighbor_tree();
  void populate_neighbors_fields();
  void build_spatial_map();
  void self_collide(PointMass &pm, double simulation_steps);
  float hash_position(Vector3D pos);

  // Cloth properties
  double width;
  double height;
  int num_width_points;
  int num_height_points;
  double thickness;
  e_orientation orientation;

  // Cloth components
  int num_particles;
  Vector3D center_particles;
  double particle_friction;
  vector<Particle> particles;
  int solver_iterations;

  // kd neighbor_tree
  vector< vector<Particle*> > neighbors_list;
  // ClothMesh *clothMesh;

  // Spatial hashing
  unordered_map<float, vector<Particle *> *> map;


  // Physics functions
  double poly6(Vector3D r);
  Vector3D spiky_gradient(Vector3D r);
  double get_particle_lambda(const Particle& p);
  double C(const Particle& p);
  Vector3D gradient_C(const Particle& p, const Particle& k);
  Vector3D get_delta_p(const Particle& p);
  double tensile_instability_correction(const Particle& p, const Particle& neighbor);
  void apply_vorticity_confinement(Particle& p, double delta_t);
  Vector3D get_viscosity_correction(const Particle& p);
};

#endif /* CLOTH_H */
