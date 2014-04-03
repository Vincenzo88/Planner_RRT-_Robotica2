#ifndef __dynamical_system_h__
#define __dynamical_system_h__

#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <algorithm>


#include "tree.h"
#include "kdtree.h"

using namespace std;

#define SQ(x)   ((x)*(x))

template<size_t N>
class state_t
{
  public:
    float x[N];
    state_t()
    {
      for(size_t i=0; i<N; i++)
        x[i] = 0;
    }
    virtual ~state_t(){};

    state_t(const state_t& s)
    {
      for(size_t i=0; i<N; i++)
        x[i] = s.x[i];
    }
    state_t(const float* sin)
    {
      for(size_t i=0; i<N; i++)
        x[i] = sin[i];
    }
    state_t& operator=(const state_t& sin)
    {
      if(this == &sin)
        return *this;
      for(size_t i=0; i<N; i++)
        x[i] = sin.x[i];
      return *this;
    }
    
    float operator[](const size_t i) const {return x[i];}
    
    virtual ostream& print(ostream& os=cout, const char* prefix=NULL, const char* suffix=NULL) const
    {
      if(prefix)
        os<<prefix<<" (";
      else
        os<<" (";
      for(size_t i=0; i<N-1; i++)
        os<<x[i]<<",";
      os<<x[N-1]<<")";
      if(suffix)
        os<<suffix;
      return os;
    }
    float dist(const state_t& s, bool only_xy=false) const
    {
      size_t len = N;
      if(only_xy)
        len = 2;
      float t=0;
      for(size_t i=0; i<len; i++)
        t = t + SQ(x[i]-s.x[i]);
      return sqrt(t);
    }

  const static size_t size = N;
};

class trajectory_t
{
  public:
    vector<float*> states;
    vector<float*> controls;
    float total_variation;
    size_t N;
    size_t M;
     
    trajectory_t() : total_variation(0), N(0), M(0){}
    
    // does not clear memory, explicitly
    // call trajectory.clear() to deallocate memory
    ~trajectory_t(){}
    int clear()
    {
      total_variation = 0;
      for(auto& i : states)
        delete[] i;
      for(auto& i : controls)
        delete[] i;
      states.clear();
      controls.clear();
      return 0;
    }
    int pop_front(int how_many)
    {
      int n=0;
      for(auto& ps : states)
      {
        if(n == how_many)
          break;
        delete[] ps;
        n++;
      }
      n=0;
      for(auto& pc : controls)
      {
        if(n == how_many)
          break;
        delete[] pc;  
        n++;
      }
      states = vector<float*>(states.begin()+how_many, states.end());
      controls = vector<float*>(controls.begin()+how_many, controls.end());
      return 0;
    }
    int append(trajectory_t& t2)
    {
      total_variation += t2.total_variation;
      N = t2.N;
      M = t2.M;
      states.insert(states.end(), t2.states.begin(), t2.states.end());
      controls.insert(controls.end(), t2.controls.begin(), t2.controls.end());
      
      // copy pointers into this object from t2
      t2.states.clear();
      t2.controls.clear();
      return 0;
    }
    void reverse()
    {
      std::reverse(states.begin(), states.end());
      std::reverse(controls.begin(), controls.end());
      return;
    }
    int print(const char* prefix="traj")
    {
      cout<<prefix<<endl;
      for(auto& ps : states)
      {
        for(size_t i=0; i<N-1; i++)
          cout << ps[i] <<",";
        cout<< ps[N-1]<<endl;
      }
      for(auto& pc : controls)
      {
        for(size_t i=0; i<M-1; i++)
          cout << pc[i] <<",";
        cout<< pc[M-1]<<endl;
      }
      return 0;
    }
};

class optimization_data_t
{
  public:
    virtual ~optimization_data_t(){};
};

template<size_t N>
class dynamical_system_t
{
  public:
    typedef state_t<N> state;
    typedef vertex_t<N> vertex;
    typedef region_t<N> region;

    int modulo_mpi_pi(float& th)
    {
      while(th < -M_PI)
        th = th + 2*M_PI;
      while(th > M_PI)
        th = th - 2*M_PI;
      return 0;
    }
    int modulo_zero_2pi(float& th)
    {
      while(th < 0)
        th = th + 2*M_PI;
      while(th > 2*M_PI)
        th = th - 2*M_PI;
      return 0;
    }
    
   virtual int get_near_vertices(const state& s, vector<vertex*>& near_vertices, kdtree* kdt, float gamma, int num_vertices, const region& operating_region){
      int toret = 0;
      
      cout << "sono arrivato qui" << endl;
      
      float* key = new float[N];
      get_key(s, key,	 operating_region);
      
      float rn = gamma*pow(log(num_vertices + 1.0)/(num_vertices+1.0), 1.0/(float)N);
      kdres* kdres = kd_nearest_rangef(kdt, key, rn);

      int num_near_vertices = kd_res_size(kdres);
      if(!num_near_vertices)
      {
        kd_res_free(kdres);

        // get nearest vertex
        kdres = kd_nearestf(kdt, key);
        if(kd_res_end(kdres))
          toret = 1;
        else
        {
	  
          vertex* vc = (vertex*)kd_res_item_data(kdres);
          near_vertices.push_back(vc);
        }
      }
      else
      {
        //kd_res_rewind(kdres);
        while(! kd_res_end(kdres))
        {
          vertex* vc = (vertex*)kd_res_item_data(kdres);
          near_vertices.push_back(vc);
          kd_res_next(kdres);
        }
      }

      delete[] key;
      kd_res_free(kdres);
      return toret;
    }
    
    int get_key(const state& s, float* key, const region& operating_region)
    {
      for(size_t i=0; i<N; i++)
        key[i] = (s.x[i] - operating_region.c[i])/operating_region.s[i] + 0.5;
      //cout<<"key: "<< key[0]<<" "<<key[1]<<" "<<key[2]<<endl;
      return 0;
    }
    
    virtual int extend_to(const state* si, const state* sf, trajectory_t& traj, optimization_data_t* opt_data)=0;
    virtual float evaluate_extend_cost(const state* si, const state* sf, optimization_data_t*& opt_data)=0;
      
};

#endif
