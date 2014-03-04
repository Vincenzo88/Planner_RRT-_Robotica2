#ifndef __tree_h__
#define __tree_h__

#include <list>
#include <set>
#include <vector>
#include <set>
#include <cfloat>
#include "kdtree.h"
#include <algorithm>

using namespace std;

class cost_t;

template<size_t N> class edge_t;
template<size_t N> class state_t;

template<size_t N>
class vertex_t
{
  public:
    typedef vertex_t<N> vertex;
    typedef edge_t<N> edge;

    vertex* parent;
    const state_t<N>* state;
    set<vertex*> children;
    
    int mark;
    
    cost_t* cost_from_root;
    cost_t* cost_from_parent;
    edge* edge_from_parent;

    vertex_t() : mark(0)
    {
      state = NULL;
      parent = NULL;
      edge_from_parent = NULL;
      cost_from_parent = NULL;
      cost_from_root = NULL;
    }
    ~vertex_t()
    {
      //static int debug_counter = 0;
      //debug_counter++;
      //cout<<"vdc: "<<debug_counter<<endl;
      if(state)
        delete state;
      if(edge_from_parent)
        delete edge_from_parent;
      if(cost_from_root)
        delete cost_from_root;
      /*if(cost_from_parent)
        delete cost_from_parent;*/ // already deleted when edge_from_parent is deleted!
    }

    vertex_t(const state_t<N>* sin) : mark(0)
    {
      parent = NULL;
      edge_from_parent = NULL;
      cost_from_parent = NULL;
      cost_from_root = NULL;
      state = sin;
    }
    string tostring() const
    {
      return state->tostring();
    }
    void print_branch(string prefix)
    {
      for(auto& pc : children)
        pc->print_branch(prefix+"   ");
    }
    const state_t<N>& get_state() const { return *state;};
    vertex& get_parent() const {return *parent;};
    cost_t* get_cost() const {return cost_from_root;};
};

template<size_t N>
class region_t
{
  public:
    float s[N];
    float c[N];

    region_t()
    {
      for(size_t i=0; i<N; i++)
        s[i] = c[i] = 0;
    }
    region_t(const float* cin, const float* sin)
    {
      for(size_t i=0; i< N; i++)
      {
        s[i] = sin[i];
        c[i] = cin[i];
      }
    }
    bool is_inside(const state_t<N>& sin) const
    {
      for(size_t i=0; i<N; i++)
      {
        if(fabs(sin.x[i] - c[i]) > s[i]/2.0)
          return false;
      }
      return true;
    }
};

#endif
