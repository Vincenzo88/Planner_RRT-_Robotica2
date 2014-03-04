#ifndef __dubins_h__
#define __dubins_h__

#include <float.h>
#include <iostream>
#include "dynamical_system.h"
using namespace std;

class dubins_t : public dynamical_system_t<3>
{
  public:

    float delta_distance;
  
#define num_turning_radii   (3)
    float turning_radii[num_turning_radii];

    dubins_t() : delta_distance(0.07)
    {
      turning_radii[0] = 3.2;
      turning_radii[1] = 6;
      turning_radii[2] = 8;
      //turning_radii[3] = 9;
      //turning_radii[8] =11;
    };
    
    int extend_to(const state* si, const state* sf, trajectory_t& traj, optimization_data_t* opt_data);
    float evaluate_extend_cost(const state* si, const state* sf, optimization_data_t*& opt_data);

    float extend_dubins_spheres(const float si[3], const float sf[3], int comb_no, float turning_radius,
        bool return_trajectory, trajectory_t& traj);
    float extend_dubins_all(const float si[3], const float sf[3], bool return_trajectory,
        trajectory_t& traj, float turning_radius);
    
    int get_near_vertices(const state& s, vector<vertex*>& near_vertices, kdtree* kdt, float gamma, int num_vertices, region operating_region);
    
};

class dubins_optimization_data_t : public optimization_data_t
{
  public:
    int turning_radius;
    dubins_optimization_data_t() : turning_radius(-1){}
    ~dubins_optimization_data_t(){}
};
#endif
