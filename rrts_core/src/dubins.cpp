#include "dubins.h"


float dubins_t::extend_dubins_spheres(const float si[3], const float sf[3], int comb_no, float turning_radius,
    bool return_trajectory, trajectory_t& traj)
{
  float x_s1 = si[0], x_s2 = sf[0];
  float y_s1 = si[1], y_s2 = sf[1];
  float t_s1 = si[2], t_s2 = sf[2];

  float x_tr = x_s2 - x_s1;
  float y_tr = y_s2 - y_s1;
  float t_tr = atan2 (y_tr, x_tr);

  float distance = sqrt ( x_tr*x_tr + y_tr*y_tr );

  float x_start;
  float y_start;
  float t_start = 0;
  float x_end;
  float y_end;
  float t_end = 0;

  if (distance > 2 * turning_radius) 
  {  
    // disks do not intersect
    float t_balls = acos (2 * turning_radius / distance);
    switch (comb_no) 
    {
      case 1:
        t_start = t_tr - t_balls;
        t_end = t_tr + M_PI - t_balls;
        break;
      case 2:
        t_start = t_tr + t_balls;
        t_end = t_tr - M_PI + t_balls;
        break;
      case 3:
        t_start = t_tr - M_PI_2;
        t_end = t_tr - M_PI_2;
        break;
      case 4:
        t_start = t_tr + M_PI_2;
        t_end = t_tr + M_PI_2;
        break;
      default:
        return -1.0;
    }
  }
  else 
  { 
    // disks are intersecting
    switch (comb_no) 
    {
      case 1:
      case 2:
        // No solution
        return -1.0;
        break;

      case 3:
        t_start = t_tr - M_PI_2;
        t_end = t_tr - M_PI_2;
        break;
      case 4:
        t_start = t_tr + M_PI_2;
        t_end = t_tr + M_PI_2;
        break;
    }
  }

  x_start = x_s1 + turning_radius * cos (t_start);
  y_start = y_s1 + turning_radius * sin (t_start);
  x_end = x_s2 + turning_radius * cos (t_end);
  y_end = y_s2 + turning_radius * sin (t_end);

  int direction_s1 = 1;
  if ( (comb_no == 2) || (comb_no == 4) ) {
    direction_s1 = -1;
  }
  int direction_s2 = 1;
  if ( (comb_no == 1) || (comb_no == 4) ) {
    direction_s2 = -1;
  }

  float t_increment_s1 = direction_s1 * (t_start - t_s1);
  float t_increment_s2 = direction_s2 * (t_s2 - t_end);
  
  modulo_zero_2pi(t_increment_s1);
  modulo_zero_2pi(t_increment_s2);

  if  ( ( (t_increment_s1 > M_PI) && (t_increment_s2 > M_PI) ) 
      || ( (t_increment_s1 > 3*M_PI_2) || (t_increment_s2 > 3*M_PI_2) )  ){
    return -1.0;
  }

  // different costs
  float ts1c = t_increment_s1;
  float ts2c = t_increment_s2;
  modulo_mpi_pi(ts1c);
  modulo_mpi_pi(ts2c);
  
  // comment turning_cost for testing
  float time_cost = (( fabs(ts1c) + fabs(ts2c)) * turning_radius  + distance);
  float turning_cost = ( fabs(ts1c) + fabs(ts2c));
  //time_cost += turning_cost;

  if (return_trajectory) 
  {
    traj.clear();
    traj.total_variation = time_cost;
    traj.N = 3;
    traj.M = 1;

    // Generate states/inputs
    float del_d = delta_distance;
    float del_t = del_d/turning_radius;

    float t_inc_curr = 0.0;

    float state_curr[3] = {0};

    while (t_inc_curr < t_increment_s1) 
    {
      float t_inc_rel = del_t;
      t_inc_curr += del_t;
      if (t_inc_curr > t_increment_s1) 
      {
        t_inc_rel -= t_inc_curr - t_increment_s1;
        t_inc_curr = t_increment_s1;
      }

      state_curr[0] = x_s1 + turning_radius * cos (direction_s1 * t_inc_curr + t_s1);
      state_curr[1] = y_s1 + turning_radius * sin (direction_s1 * t_inc_curr + t_s1);
      state_curr[2] = direction_s1 * t_inc_curr + t_s1 + ( (direction_s1 == 1) ? M_PI_2 : 3.0*M_PI_2);

      modulo_mpi_pi(state_curr[2]);

      float* state_new = new float[3];
      float* control_new = new float[1];
      for (int i = 0; i < 3; i++) 
        state_new[i] = state_curr[i];
      *control_new = direction_s1*turning_radius;

      traj.states.push_back(state_new);
      traj.controls.push_back(control_new);
    }

    float d_inc_curr = 0.0;
    while (d_inc_curr < distance) 
    {
      float d_inc_rel = del_d;
      d_inc_curr += del_d;
      if (d_inc_curr > distance) {
        d_inc_rel -= d_inc_curr - distance;
        d_inc_curr = distance;
      }

      state_curr[0] = (x_end - x_start) * d_inc_curr / distance + x_start; 
      state_curr[1] = (y_end - y_start) * d_inc_curr / distance + y_start; 
      state_curr[2] = direction_s1 * t_inc_curr + t_s1 + ( (direction_s1 == 1) ? M_PI_2 : 3.0*M_PI_2);

      modulo_mpi_pi(state_curr[2]);

      float* state_new = new float[3];
      float* control_new = new float[1];
      for (int i = 0; i < 3; i++) 
        state_new[i] = state_curr[i];
      *control_new = 0;
      traj.states.push_back(state_new);
      traj.controls.push_back(control_new);
    }

    t_inc_curr = 0.0;
    while (t_inc_curr < t_increment_s2) 
    {
      float t_inc_rel = del_t;
      t_inc_curr += del_t;
      if (t_inc_curr > t_increment_s2)  {
        t_inc_rel -= t_inc_curr - t_increment_s2;
        t_inc_curr = t_increment_s2;
      }

      state_curr[0] = x_s2 + turning_radius * cos (direction_s2 * (t_inc_curr - t_increment_s2) + t_s2);
      state_curr[1] = y_s2 + turning_radius * sin (direction_s2 * (t_inc_curr - t_increment_s2) + t_s2);
      state_curr[2] = direction_s2 * (t_inc_curr - t_increment_s2) + t_s2 
        + ( (direction_s2 == 1) ?  M_PI_2 : 3.0*M_PI_2 );

      modulo_mpi_pi(state_curr[2]);

      float* state_new = new float [3];
      float* control_new = new float[1];
      for (int i = 0; i < 3; i++) 
        state_new[i] = state_curr[i];
      *control_new = turning_radius * direction_s2;
      traj.states.push_back(state_new);
      traj.controls.push_back(control_new);
    }
  }
  return time_cost;
}

float dubins_t::extend_dubins_all(const float si[3], const float sf[3], bool return_trajectory,
    trajectory_t& traj, float turning_radius)
{

  float ti = si[2];
  float tf = sf[2];
  float sin_ti = sin (-ti);
  float cos_ti = cos (-ti);
  float sin_tf = sin (-tf);
  float cos_tf = cos (-tf);

  float si_left[3] = {
    si[0] + turning_radius * sin_ti,
    si[1] + turning_radius * cos_ti,
    ti + (float)(3 * M_PI_2)
  };
  float si_right[3] = {
    si[0] - turning_radius * sin_ti,
    si[1] - turning_radius * cos_ti,
    ti + (float)M_PI_2
  };
  float sf_left[3] = {
    sf[0] + turning_radius * sin_tf,
    sf[1] + turning_radius * cos_tf,
    tf + (float)(3 * M_PI_2)
  };
  float sf_right[3] = {
    sf[0] - turning_radius * sin_tf,
    sf[1] - turning_radius * cos_tf,
    tf + (float)M_PI_2
  };

  // 2. extend all four spheres
  float times[4]; 
  times[0] = extend_dubins_spheres (si_left, sf_right, 1, turning_radius,
      return_trajectory, traj);
  times[1] = extend_dubins_spheres (si_right, sf_left, 2, turning_radius,
      return_trajectory, traj);
  times[2] = extend_dubins_spheres (si_left, sf_left, 3, turning_radius,
      return_trajectory, traj);
  times[3] = extend_dubins_spheres (si_right, sf_right, 4, turning_radius,
      return_trajectory, traj);

  float min_time = FLT_MAX/2;
  int comb_min = -1;
  for (int i = 0; i < 4; i++) 
  {
    if  ( (times[i] >= 0.0) && (times[i] < min_time) ) 
    {
      comb_min = i+1;
      min_time = times[i];
    }
  }

  if (comb_min == -1)
    return -1.0;

  return min_time;
}

int dubins_t::extend_to(const state* si, const state* sf, trajectory_t& traj, optimization_data_t* opt_data)
{
  bool return_trajectory = true;
  if(!opt_data)
  {
    if(!evaluate_extend_cost(si, sf, opt_data))
      return 1;
  }
  int& best_turning_radius = static_cast<dubins_optimization_data_t*>(opt_data)->turning_radius;
  extend_dubins_all(si->x, sf->x, return_trajectory, traj, turning_radii[best_turning_radius]);
  return 0;
}

float dubins_t::evaluate_extend_cost(const state* si, const state* sf, optimization_data_t*& opt_data)
{
  float min_cost = FLT_MAX;
  dubins_optimization_data_t* dubins_opt_data = new dubins_optimization_data_t();
  opt_data = dubins_opt_data;

  int& best_turning_radius = dubins_opt_data->turning_radius;
  best_turning_radius = -1;

  bool return_trajectory = false;
  trajectory_t traj;
  for(int i=num_turning_radii-1; i >=0; i--)
  {
    float tr = turning_radii[i];
    float cost = extend_dubins_all(si->x, sf->x, return_trajectory, traj, tr);
    if(cost > 0)
    {
      if(cost < min_cost)
      {
        min_cost = cost;
        best_turning_radius = i;
      }
    }
  }
  if((min_cost < 0) || (min_cost > FLT_MAX/2.0))
    return -1;
  return min_cost;
}

/*int dubins_t::get_near_vertices(const state& s, vector<vertex*>& near_vertices, kdtree* kdt, float gamma, int num_vertices, region operating_region){
  // s è lo stato di cui voglio determinare il vicinato
  // per la coordinate s[0] (x), s[1] (y), s[2], (theta rad)
  // 
}*/
    
