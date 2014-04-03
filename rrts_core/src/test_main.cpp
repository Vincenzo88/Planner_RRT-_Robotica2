#include <iostream>
#include <ctime>

#include "dubins.h"
#include "map.h"
#include "system.h"
#include "rrts.h"

using namespace std;

typedef state_t<3> state;
typedef vertex_t<3> vertex;
typedef region_t<3> region;


template<size_t N>
class my_map_t : public map_t<N>
{
  public:
    int sample_free_space(float[N])
    {
      return 1;
    }
    bool is_in_collision(const float s[N])
    {
      return false;
    }
    float get_state_cost(const float s[N])
    {
      return 0;
    }
};

int main() 
{  
  dubins_t dubins;
  my_map_t<3> obstacle_map;
  system_t<3> system;
  system.dynamical_system = &dubins;
  system.obstacle_map = &obstacle_map;

  float zero[3] = {0};
  float size[3] = {100,100,2*M_PI};
  system.operating_region = region(zero, size);

  float gc[3] = {10,10,0};
  float gs[3] = {1,1,0.1*M_PI};
  state goal_state(gc);
  system.goal_region = region(gc,gs);

  rrts_t<3> rrts;
  state* origin = new state(zero);
  rrts.initialize(&system, origin); 

  rrts.goal_sample_freq = 0.05;
  
  time_t ts=time(0), te;
  int max_iterations = 1000, diter=max_iterations/10;
  trajectory_t traj;
  for(int i=0; i<max_iterations; i++)
  {
    cout << rrts.num_iterations << " " << rrts.get_best_cost()->val<< " " << rrts.num_vertices<< endl; 
    //if(i%diter == 0)
      //cout<<i<<" "<<rrts.get_best_cost()->val<<endl;
    //cout<<"check_tree: "<< rrts.check_tree() << endl;
    rrts.iteration();
    
#if 0
    if(rrts.system->is_in_goal(*(rrts.root->state)))
      break;
    if(i % 100 == 0)
    { 
      cout<<i<<" "<<rrts.get_best_cost()->val<<endl;
      rrts.switch_root(25, traj);
      cout<<"is safe: "<< rrts.system->is_safe_trajectory(traj)<<endl;
      traj.print();
      traj.clear();
      cout<<"switched root: ";
      rrts.root->state->print(); 
      cout<<endl;
    }
#endif
  }
  traj.clear();
  cout<<rrts.get_best_cost()->val<<endl;
  rrts.get_best_trajectory(traj);
  traj.print();
  
  cout<<"time: "<< difftime(time(0), ts)<<endl;

  return 0;
}
