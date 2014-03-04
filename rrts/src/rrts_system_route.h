#ifndef __rrts_system_h__
#define __rrts_system_h__

#include <cmath>
#include <cfloat>
#include <vector>
#include <ostream>
using namespace std;

#include <system.h>
#include "local_map_route.h"

template < size_t N > class rrts_system_route_t : public system_t < N > {
    
public:
      
    typedef state_t<N> state;
    
    /** Y_map
     * ^      ^ Y_local
     * |      |
     * |      |
     * |      +(map->origin)----X_local
     * |
     * +(0,0)--------- X_map
     */
    
    int transform_local_to_map( const state& sl, state& sm) {  
      state origin = static_cast < local_map_route_t < N > * > ( this -> obstacle_map ) -> origin;
      float cth = cos( origin.x[2] ), sth = sin( origin.x[2] );
      sm.x[0] = origin.x[0]     + sl.x[0] * cth     - sl.x[1] * sth;
      sm.x[1] = origin.x[1]     + sl.x[0] * sth     + sl.x[1] * cth;
      sm.x[2] = origin.x[2]     + sl.x[2];
      this -> dynamical_system -> modulo_mpi_pi( sm.x[2] );
      return 0;
    }
    
    int transform_map_to_local( const state& sm, state& sl) {
      state origin = static_cast < local_map_route_t < N > * > ( this -> obstacle_map ) -> origin;
      float cth = cos( origin.x[2] ), sth = sin( origin.x[2] );
      sl.x[0] = ( sm.x[0] - origin.x[0] ) * cth + ( sm.x[1] - origin.x[1] ) * sth;
      sl.x[1] = -( sm.x[0] - origin.x[0] ) * sth + ( sm.x[1] - origin.x[1] ) * cth;
      sl.x[2] = sm.x[2] - origin.x[2];
      this -> dynamical_system -> modulo_mpi_pi( sl.x[2] );
      return 0;
    }
    
    int get_key( const state& s, float * key) {
      state sc;
      transform_map_to_local( s, sc);
      return system_t < N > :: get_key( sc, key);
    }
    
    /**
     * overwritten sample_state from system_t
     * sample_from_free is defaulted to false in system_t
     * do not change it to true here, default arguments of
     * polymorphic functions are resolved at compile time
     */
    
    float get_state_cost( state& s) {
      return static_cast<local_map_route_t<N>*>(this->obstacle_map)->get_state_cost(s.x);
    }
    
    state* sample_state( bool sample_from_free = false) {
      state* s;
      s = system_t<N>::sample_state(true);
      // if(sample_from_free)
        // s = system_t<N>::sample_state(true);
      // else
        // s = system_t<N>::sample_state(false);
      if (s) {
        state sc(*s);
        transform_local_to_map(sc, *s);
        return s;
      }
      
      return NULL;
      
    }
    
    bool is_in_collision( const state& s) {
      state sc;
      transform_map_to_local( s, sc);
      return static_cast < local_map_route_t < N > * > (this -> obstacle_map) -> is_in_collision(sc.x);
    }
    
};

#endif
