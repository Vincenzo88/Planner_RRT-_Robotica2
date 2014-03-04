#ifndef __map_h__
#define __map_h__

#define RANDF   (rand()/(RAND_MAX+1.0))

template < size_t N > class map_t {
public:
    
    map_t() {}
    virtual ~map_t() {}
    
    virtual int sample_free_space( float[N] ) = 0;
    virtual bool is_in_collision( const float[N]) = 0;
    virtual float get_state_cost( const float s[N]) { return 0;}
    virtual bool is_out_of_map(const float[N]){ return 0;}
};

#endif
