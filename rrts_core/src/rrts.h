#ifndef __rrts_h__
#define __rrts_h__

#include <list>
#include <set>
#include <vector>
#include <set>
#include <cfloat>
#include "kdtree.h"
#include <algorithm>

#include "tree.h"
#include "system.h"
using namespace std;

template<size_t N> class vertex_t;
template<size_t N> class edge_t;
template<size_t N> class rrts_t;


template<size_t N>
class edge_t
{
  public:
    typedef state_t<N> state;
    typedef edge_t<N> edge;

    const state* start_state;
    const state* end_state;

    cost_t* cost;
    optimization_data_t* opt_data;

    edge_t()
    {
      opt_data = NULL;
      end_state = NULL;
      cost = NULL;
    };

    edge_t(const state* si, const state* se, system_t<N>& sys, optimization_data_t* opt_data_in=NULL)
    {
      start_state = si;
      end_state = se;
      opt_data = opt_data_in;
      cost = sys.evaluate_extend_cost(si, se, opt_data);
    }

    ~edge_t()
    {
      if(opt_data)
        delete opt_data;
      if(cost)
        delete cost;
    }
};

template<size_t N>
class rrts_t
{
  public:
    typedef struct kdtree kdtree_t;
    typedef struct kdres kdres_t;

    typedef state_t<N> state;
    typedef vertex_t<N> vertex;
    typedef edge_t<N> edge;

    system_t<N>* system;

    int num_dim;
    int num_vertices;
    int num_iterations;
    list<vertex*> list_vertices;
    float gamma;

    float goal_sample_freq;
    
    static int debug_counter;

    cost_t* zero_cost;
    cost_t* infinite_cost;
    cost_t* root_cost;

    cost_t* lower_bound_cost;
    vertex* lower_bound_vertex;
    kdtree_t* kdtree;

    bool do_branch_and_bound;

    vertex* root;

    rrts_t()
    {
      system = NULL;

      gamma = 2.5;
      goal_sample_freq = 0.1;

      root = NULL;
      lower_bound_vertex = NULL;

      kdtree = NULL;
      num_vertices = 0;
      num_iterations = 0;
      num_dim = N;

      do_branch_and_bound = true;

    }
    ~rrts_t()
    {
      if(kdtree)
        kd_free(kdtree);
      for(auto& i : list_vertices)
        delete i;

      delete zero_cost;
      delete infinite_cost;
    }
    
    int initialize(system_t<N>* sys, const state* rs, bool do_branch_and_bound_=true)
    {
      if(!sys)
        return 1;
      else
        system = sys;
      if(! rs)
        return 2;

      for(auto& i : list_vertices)
        delete i;
      list_vertices.clear();
      num_vertices = 0;
      num_iterations = 0;

      lower_bound_cost = system->get_infinite_cost();
      lower_bound_vertex = NULL;

      if(kdtree)
        kd_free(kdtree);
      kdtree = kd_create(num_dim);
     
      root = new vertex(rs);
      root->cost_from_root = system->get_zero_cost();
      root->cost_from_parent = system->get_zero_cost();
      root->edge_from_parent = NULL;
      root->parent = NULL;
      root->children.clear();
      insert_into_kdtree(*root);
      root->state->print(cout,"set root to:", "\n");

      zero_cost = system->get_zero_cost();
      root_cost = system->get_zero_cost();
      infinite_cost = system->get_infinite_cost();
      do_branch_and_bound = do_branch_and_bound_;
            
      return 0;
    }
    
    int iteration()
    {
      num_iterations++;
      // 1. sample
      state* psr;
      float p = RANDF;
      if(p < goal_sample_freq || num_iterations==2){
        psr = system->sample_in_goal();
      }else{
        psr = system->sample_state();
      }

      if(!psr)
        return 1;

      //cout << "SAMPLE: "; psr->print(); cout << endl;

      state& sr = *psr;

      // 2. compute nearest vertices
      vector<vertex*> near_vertices;
      if(get_near_vertices(sr, near_vertices))
      {
        delete psr;
        return 2;
      }

      // 3. best parent
      vertex* best_parent = NULL;
      edge* edge_from_parent = NULL;
      if(find_best_parent(sr, near_vertices, best_parent, edge_from_parent))
      {
        //cout << "   Best Parent not found." << endl;
        delete psr;
        return 3;
      }

      //cout << "   Best Parent found: "; best_parent->state->print(); cout << endl;

      vertex* new_vertex = insert_edge(*best_parent, *edge_from_parent);
      if(!new_vertex)
      {
        //cout << "        not added!" << endl;
        delete psr;
        return 4;
      }
      //cout << "VERTEX: ";new_vertex->state->print();cout << endl;
      // 4. rewire
      if(near_vertices.size()){
        //cout << "     -- REWIRING --" << endl;
        rewire_vertices(*new_vertex, near_vertices);
      }
      return 0;
    }

    int insert_into_kdtree(vertex& v)
    {
      float* key = new float[N];
      system->get_key( *(v.state), key);
      kd_insertf(kdtree, key, &v);
      delete[] key;

      list_vertices.push_back(&v);
      num_vertices++;
      return 0;
    }
    
    vertex& get_root_vertex() { return *root;};
    cost_t* get_best_cost() { return lower_bound_cost;};
    vertex& get_best_vertex() { return *lower_bound_vertex;}
   
    float* copy_float_array(const float* src, size_t dim)
    {
      float* dest = new float[dim];
      for(size_t i=0; i<dim; i++)
        dest[i] = src[i];
      return dest;
    }

    int get_best_trajectory(trajectory_t& best_traj)
    {
      if(!lower_bound_vertex)
        return 1;
      
      best_traj.N = N;
      best_traj.clear();
      bool check_obstacles = false;
      vertex* vc = lower_bound_vertex;
      while(vc)
      {
        vertex* vparent = vc->parent;
        if(vparent)
        {
          trajectory_t traj_from_parent;
          system->extend_to(vparent->state, vc->state, check_obstacles, traj_from_parent, vc->edge_from_parent->opt_data);
          traj_from_parent.reverse();
          best_traj.append(traj_from_parent);
        }
        vc = vparent;
      }
      best_traj.reverse();
      return 0;
    }

    int get_near_vertices(const state& s, vector<vertex*>& near_vertices)
    {
      system->get_near_vertices(s, near_vertices, kdtree, gamma, num_vertices);
      /*
      int toret = 0;
      float* key = new float[N];
      system->get_key(s, key);

      float rn = gamma*pow(log(num_vertices + 1.0)/(num_vertices+1.0), 1.0/(float)num_dim);
      kdres_t* kdres = kd_nearest_rangef(kdtree, key, rn);

      int num_near_vertices = kd_res_size(kdres);
      if(!num_near_vertices)
      {
        kd_res_free(kdres);

        // get nearest vertex
        kdres = kd_nearestf(kdtree, key);
        if(kd_res_end(kdres))
          toret = 1;
        else
        {
          vertex* vc = (vertex*) kd_res_item_data(kdres);
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
      return toret;*/
    }

    int update_best_vertex(vertex& v)
    {
      if(system->is_in_goal(*(v.state)))
      {
        if( (!lower_bound_vertex) || (*(v.cost_from_root) < *(lower_bound_cost)))
        {
          cout << " --------------  NEW BEST: ";
          lower_bound_cost = v.cost_from_root;
          lower_bound_vertex = &v;
          lower_bound_cost->print();
          cout << endl<< flush;

        }
      }
      return 0;
    }

    vertex* insert_edge(vertex& vs, edge& e)
    {
      // branch and bound
      if(do_branch_and_bound){
        cost_t* new_cost = &(*(vs.cost_from_root) + *(e.cost)); 
        if(*new_cost > *lower_bound_cost){
          delete new_cost;
          return NULL;
        }

      delete new_cost;
      }

      // create new vertex
      vertex* new_vertex = new vertex(e.end_state);
      insert_into_kdtree(*new_vertex);

      insert_edge(vs, e, *new_vertex);
      return new_vertex;
    }
    
    int insert_edge(vertex& vs, edge& e, vertex& ve)
    {
      ve.cost_from_parent = e.cost;
      if(ve.cost_from_root)
        delete ve.cost_from_root;
      ve.cost_from_root = &(*vs.cost_from_root + *ve.cost_from_parent);
      update_best_vertex(ve);
      
      if(ve.edge_from_parent)
        delete ve.edge_from_parent;
      ve.edge_from_parent = &e;
      
      if(ve.parent)
        ve.parent->children.erase(&ve);
      ve.parent = &vs;
      vs.children.insert(&ve);
      return 0;
    }

    static bool compare_vertex_cost_pairs(const pair<vertex*, pair<cost_t*, optimization_data_t*> >& p1, const pair<vertex*, pair<cost_t*, optimization_data_t*> >& p2)
    {
      return (*(p1.second.first) < *(p2.second.first));
    }

    int find_best_parent(const state& sin, const vector<vertex*>& near_vertices, vertex*& best_parent,
        edge*& best_edge)
    {
      // 1. create vertex_cost_pairs
      vector<pair<vertex*, pair<cost_t*, optimization_data_t*> > > vertex_cost_pairs;
      for(auto& pv : near_vertices)
      {
        vertex& v = *pv;
        optimization_data_t* opt_data = NULL;

        cost_t* v_cost = system->evaluate_extend_cost(v.state, &sin, opt_data);
        if(v_cost->val > 0){
          vertex_cost_pairs.push_back(make_pair(pv, make_pair(v_cost, opt_data)));
        }else{
          delete v_cost;
          delete opt_data;
        }
      }
      
      // 2. sort using compare function of cost_t
      sort(vertex_cost_pairs.begin(), vertex_cost_pairs.end(), compare_vertex_cost_pairs);

      // 3. check obstacles in order of increasing cost
      bool check_obstacles = true;
      trajectory_t traj;
      bool found_best_parent = false;
      for(auto& p : vertex_cost_pairs)
      {
        vertex& v = *(p.first);
        optimization_data_t* opt_data = p.second.second;
        //cout << "                   + "; v.state->print(); cout << endl; 
        if( (!found_best_parent) && (system->extend_to(v.state, &sin, check_obstacles, traj, opt_data)==0) )
        {
          best_parent = &v;
          best_edge = new edge(v.state, &sin, *system, opt_data);
          found_best_parent = true;
        //   found_best_parent = best_edge->cost->val > 0;
        }
        else
        {
          //cout << "                                           COLLISION CHECKING FAILED !!" << endl << flush;
          // free opt_data
          delete opt_data;
        }
        traj.clear();
      }
      if(found_best_parent){
        return 0;
      }else{
        return 1;
      }
    }

    int update_all_costs()
    {
      if(lower_bound_cost)
        delete lower_bound_cost;
      lower_bound_cost = infinite_cost->clone();
      lower_bound_vertex = NULL;
      //cout << "Visiting ROOT, @ "; root->state->print(); cout << endl;
      update_branch_cost(*root,0); 
      return 0;
    }

    int update_branch_cost(const vertex& v, int depth)
    {
      //cout << "called update branch cost, depth " << depth << endl << flush;
      for(auto& pc : v.children)
      {
        if(depth>100) return 1;
        vertex& child = *pc;
        //cout << "asked to sum "; v.cost_from_root->print(); cout << "  +  " ; child.cost_from_parent->print(); 
        if(child.cost_from_root) 
          delete child.cost_from_root;
        child.cost_from_root = &(*v.cost_from_root + *child.cost_from_parent);
        //cout << " = "; child.cost_from_root->print(); cout << endl;
        update_best_vertex(child); 
        update_branch_cost(child, depth+1);
      }
      return 0;
    }
    
    int rewire_vertices(vertex& v, const vector<vertex*>& near_vertices)
    {
      bool check_obstacles = true;
      for(auto& pvn : near_vertices)
      {
        vertex& vn = *pvn;
        optimization_data_t* opt_data = NULL;
        trajectory_t traj;
        cost_t* cost_edge = system->evaluate_extend_cost(v.state, vn.state, opt_data);
        if(cost_edge->val < 0)
        {
          delete cost_edge;
          delete opt_data;
          continue;
        }

        cost_t* cvn = &(*v.cost_from_root + *cost_edge);
        if((*cvn) < (*vn.cost_from_root))
        {
          if(system->extend_to(v.state, vn.state, check_obstacles, traj, opt_data))
          {
            delete opt_data;
            continue;
          }
          edge* en = new edge(v.state, vn.state, *system, opt_data);
          // if(en->cost>0){
            insert_edge(v, *en, vn);
            update_branch_cost(vn,0);
          // }else{
          //   cout << "Dubin's COMPLAINING while rewiring!" << endl;
          //   traj.clear();
          //   delete opt_data;
          // }
        }
        else
        {
          traj.clear();
          delete opt_data;
        }
        delete cvn;
      }
      return 0;
    }

    int find_descendents(vertex& v)
    {
      v.mark = 1;
      for(auto& pc : v.children)
        find_descendents(*pc);
      return 0;
    }
    
    int recompute_cost(vertex& v)
    {
      update_branch_cost(v,0);  
      return 0;
    }

    // dynamic obstacles stuff
    bool is_safe_trajectory(const trajectory_t& traj)
    {
      return system->is_safe_trajectory(traj);
    }
   
    int mark_descendent_vertices(vertex& v)
    {
      v.mark = 1;
      for(auto& pc : v.children)
        mark_descendent_vertices(*pc);

      return 0;
    }
    int delete_unmarked_vertices(vector<vertex*>& surviving_vertices, vertex* except=NULL)
    {
      surviving_vertices.clear();
      for(auto& pv : list_vertices)
      {
        if(pv && pv->mark == 1)
        {
          pv->mark = 0;
          surviving_vertices.push_back(pv);
        }
        else
          if(pv && pv!=except)
            delete pv;
      }
      return 0;
    }
    int mark_vertex_and_remove_from_parent(vertex& v)
    {
      v.mark = 1;
      v.parent->children.erase(&v);
      for(auto& pc : v.children)
      {
        mark_vertex_and_remove_from_parent(*pc);
      }
      return 0;
    }

    int check_and_mark_children(vertex& v)
    {
      trajectory_t traj;
      bool check_obstacles = true;

      //v.state->print(cout,"v.state: ","\n");
      //v.parent->state->print(cout,"v.parent->state: ","\n");
      //cout<<v.edge_from_parent->opt_data<<endl;
      //cout<<v.edge_from_parent->opt_data->turning_radius<<endl;

      if(system->extend_to(v.parent->state, v.state, check_obstacles, traj, v.edge_from_parent->opt_data))
      { 
        traj.clear();
        mark_vertex_and_remove_from_parent(v);
      }
      else
      {
        traj.clear();
        for(auto& pc : v.children)
        {
          check_and_mark_children(*pc);
        }
      }
      return 0;
    }

    int check_tree()
    {
      if(system->is_in_collision(*(root->state)))
      {
        cout<<"root in collision"<<endl;
        return 1;
      }
      else if(!root->children.empty())
      {
        root->mark = 0;

        for(auto& prc : root->children)
            check_and_mark_children(*prc);
        
        list<vertex*> surviving_vertices;
        for(auto& pv : list_vertices)
        {
          if(!pv->mark)
            surviving_vertices.push_back(pv);
          else
            delete pv;
        }
        
        if(kdtree)
          kd_free(kdtree);
        kdtree = kd_create(N);

        list_vertices.clear();
        num_vertices = 0;
        for(auto& pv : surviving_vertices)
          insert_into_kdtree(*pv); 
        
        update_all_costs();
      }
      return 0;
    }

    int lazy_check_tree(const trajectory_t& committed_trajectory)
    {
      int ret = 0;
      if(!system->is_safe_trajectory(committed_trajectory))
        ret = check_tree();
      return ret;
    }

    int get_best_trajectory_vertices(list<vertex*>& best_trajectory_vertices)
    {
      if(!lower_bound_vertex)
        return 1;
      best_trajectory_vertices.clear();
      vertex* pvc = lower_bound_vertex;
      while(pvc)
      {
        best_trajectory_vertices.push_front(pvc);
        pvc = pvc->parent;
      }
      return 0;
    }
    int print_marks()
    {
      cout<<"marks: ";
      for(auto& pv : list_vertices)
        cout<<pv->mark<<" ";
      cout<<endl;
      return 0;
    }
    int switch_root(const float& distance, trajectory_t& committed_trajectory, vertex** last_committed_vertex_out = NULL)
    {
      if(!lower_bound_vertex)
        return 1;
      
      cout << "lbv cost: "; lower_bound_vertex->cost_from_root->print(); cout << "root cost:"; root_cost->print(); cout << endl;
      // 0. if root is inside goal
      if(system->is_in_goal(*(root->state)) && (*root_cost)<(*lower_bound_vertex->cost_from_root))
        return 0;

      // 1. find new root
      list<vertex*> best_trajectory_vertices;
      get_best_trajectory_vertices(best_trajectory_vertices);

      bool check_obstacles = false;
      float length = 0;
      
      state new_root_state;
      vertex* child_of_new_root_vertex = NULL;
      bool new_root_found = false;
      vertex* last_committed_vertex = NULL;
      for(auto& pv : best_trajectory_vertices)
      {
        vertex& vc = *pv;
        if(new_root_found)
          break;
        if(vc.parent)
        {
          //cout << "  going up to root" << endl << flush;
          vertex& parent = *(vc.parent);
          trajectory_t traj;
          // traj connects parent with vc!
          if(!system->extend_to(parent.state, vc.state, check_obstacles, traj, vc.edge_from_parent->opt_data))
          {
            // 1.a go ahead until reach the edge with the root
            if( (length + traj.total_variation) < distance)
            {
              length += traj.total_variation;
              committed_trajectory.append(traj);
            }
            // 1.b ----length----distance(new_root)----length+total_variation(new_child)
            else
            {
              state sc = state(traj.states.front()), sp = sc; // note: creating a new state here discards peculiar methods/fields of custom child classes of state, if used.
              auto pc = traj.controls.begin();
              bool only_xy = true;
              float t1 = 0;
              committed_trajectory.N = traj.N;
              for(auto& ps : traj.states)
              {
                sc = state(ps);
                t1 = sp.dist(sc, only_xy);
                sp = sc;
                if((t1+length)<distance)
                {
                  length = length + t1;
                  float* nps = copy_float_array(ps, N);
                  float* npc = copy_float_array(*pc, 1);
                  committed_trajectory.states.push_back(nps);
                  committed_trajectory.controls.push_back(npc);
                  committed_trajectory.total_variation += t1;
                }
                else
                {
                  new_root_state = sc;
                  child_of_new_root_vertex = pv;
                  new_root_found = true;
                  last_committed_vertex = vc.parent;
                  break;
                }
              }
              pc++;
            }
          }
          else
          {
            traj.clear();
            return 2;
          }
          traj.clear();
        }
      }
      // i.e., lower_bound_vertex = new root
      // no new child
      if(!new_root_found)
      {
        new_root_state = *(lower_bound_vertex->state);
        new_root_found = true;
        child_of_new_root_vertex = NULL;
        last_committed_vertex = lower_bound_vertex;
      }

      if(new_root_found)
      {
        if(last_committed_vertex_out && last_committed_vertex){
          (*last_committed_vertex_out) = last_committed_vertex;
        }
        // new_root is inside goal
        if(!child_of_new_root_vertex)
        {

          for(auto& pv : list_vertices){
            if(pv!=last_committed_vertex) 
              delete pv;
          }
          list_vertices.clear();
          num_vertices =  0;
          if(kdtree)
            kd_free(kdtree);
          kdtree = kd_create(N);
          
          root = new vertex( new state(new_root_state));
          root->cost_from_root = zero_cost->clone();
          root->cost_from_parent = zero_cost->clone();
          if(root_cost) delete root_cost;
          root_cost = last_committed_vertex->cost_from_root->clone();
          
          insert_into_kdtree(*root);
          update_all_costs();
          num_iterations = 0;
          return 0;
        }
        else
        {

          mark_descendent_vertices(*child_of_new_root_vertex);

          vector<vertex*> surviving_vertices;
          delete_unmarked_vertices(surviving_vertices, last_committed_vertex);

          list_vertices.clear();
          num_vertices = 0;
          if(kdtree)
            kd_free(kdtree);
          kdtree = kd_create(N);

          root = new vertex( new state(new_root_state));
          root->cost_from_root = zero_cost->clone();
          root->cost_from_parent = zero_cost->clone();
          if(root_cost) delete root_cost;
          root_cost = last_committed_vertex->cost_from_root->clone();

          if(child_of_new_root_vertex->edge_from_parent)
            delete child_of_new_root_vertex->edge_from_parent;

          trajectory_t new_root_traj;
          optimization_data_t* opt_data = NULL;
          if(system->extend_to(&new_root_state, child_of_new_root_vertex->state, check_obstacles, new_root_traj, opt_data))
          {
            new_root_traj.clear();
            delete opt_data;
            return 5;
          }
          child_of_new_root_vertex->edge_from_parent = new edge(&new_root_state, child_of_new_root_vertex->state, *system, opt_data);
          child_of_new_root_vertex->parent = root;
          child_of_new_root_vertex->cost_from_parent = child_of_new_root_vertex->edge_from_parent->cost;
          root->children.insert(child_of_new_root_vertex);
          new_root_traj.clear();

          insert_into_kdtree(*root);
          for(auto& pv : surviving_vertices)
            insert_into_kdtree(*pv);

          update_all_costs();
          num_iterations = 0;
          return 0;
        }
      }
      else
        return 3;
    }
};
#endif
