#ifndef __local_map_h__
#define __local_map_h__

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pnc_msgs/local_map.h>

#include <map.h>

template<size_t N>
class local_map_route_t : public map_t<N>
{
  public:
    typedef state_t<N> state;
    
    int height, width;
    float resolution;
    vector<int> free_cells;
    vector<int> centerline;
    vector<state> centerline_states;
    vector<signed char> map_data;
    state_t<N> origin;

    /**
     * safe_distance : car is blown up by this distance for obstacle checking
     * factor_reduce_size : reduce size of car for simulation sometimes
     * distance_rear_axis_rear : distance between center of rear axis and rear end of car
     */
    float car_width, car_height, safe_distance, 
          factor_reduce_size, distance_rear_axis_rear;
        
    local_map_route_t()
    {
      factor_reduce_size = 1.0;
      car_width = 1.2/factor_reduce_size;
      car_height = 2.28/factor_reduce_size;
      safe_distance = 0.08/factor_reduce_size; 
      distance_rear_axis_rear = 0.45/factor_reduce_size;
    }

    int map_to_local(const state& sm, state& sl) {
      float cth = cos( origin.x[2] ), sth = sin( origin.x[2] );
      sl.x[0] = ( sm.x[0] - origin.x[0] ) * cth + ( sm.x[1] - origin.x[1] ) * sth;
      sl.x[1] = -( sm.x[0] - origin.x[0] ) * sth + ( sm.x[1] - origin.x[1] ) * cth;
      sl.x[2] = sm.x[2] - origin.x[2];
      modulo_mpi_pi( sl.x[2] );
      return 0;
    }
    
    int copy_local_map(const pnc_msgs::local_map::ConstPtr lm)
    {
      free_cells = lm->free_cells;
      map_data = lm->occupancy.data;
      height = lm->occupancy.info.height;
      width = lm->occupancy.info.width;
      resolution = lm->occupancy.info.resolution;
      
      tf::Quaternion q;
      double roll=0, pitch=0, yaw=0;
      tf::quaternionMsgToTF(lm->occupancy.info.origin.orientation, q);
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
      float ros_orign_pose[3] = {(float)lm->occupancy.info.origin.position.x,
        (float)lm->occupancy.info.origin.position.y, (float)yaw};
      origin = state(ros_orign_pose);
      
      return 0;
    }

    void set_centerline(const sensor_msgs::PointCloud& pc){
      centerline_states.clear();
      for(vector<geometry_msgs::Point32>::const_iterator i=pc.points.begin(); i!=pc.points.end(); i++){
        float x = (*i).x;
        float y = (*i).y;
        float nx = (*++i).x;
        float ny = (*i--).y; 
        state am ,bm, al, bl;
        am.x[0] = x;
        am.x[1] = y;
        bm.x[0] = nx;
        bm.x[1] = ny;
        map_to_local(am, al);
        map_to_local(bm, bl);
        al.x[2] = atan2(bl.x[1]-al.x[1], bl.x[0]-al.x[0]);
        centerline_states.push_back(al);
      }
    }


    int getxy_from_index(float &x, float &y, const int index)
    {
      if( (index >= height*width/resolution/resolution) || (index < 0))
        return 1;

      x = (index%width)*resolution;
      y = (index/width)*resolution;
      return 0;
    }
    int get_cell_index(const float x, const float y, int& index)
    {
      // find cells corresponding to (x,y)
      // car is placed at (height/4, width/2) according to the local_map frame

      int cx = x/resolution;
      int cy = y/resolution;

      if( (cx >=0) && (cx < width) && 
          (cy >= 0) && (cy < height))
      {
        index = cy*width + cx;
        return 0;
      }
      else
      {
        index = -1;
        return 1;
      }
    }
    // checks a state in local cooridinates
    // for collision
    bool is_in_collision(const float sl[N])
    {
// change to 0 when testing!!
#if 1
      float cth = cos(sl[2]);
      float sth = sin(sl[2]);
      bool is_obstructed = false;
      float cxmax = car_height - distance_rear_axis_rear + safe_distance + 0.001;
      float cymax = car_width/2.0 + safe_distance + 0.001;
      float cy = -car_width/2.0 - safe_distance;

      while(cy < cymax)
      {
        float cx = -distance_rear_axis_rear - safe_distance;
        while(cx < cxmax)
        {
          // transform (cx,cy) from base_link to local_map frame
          // a negative rotation of angle sl[2]
          float x = sl[0] + cx*cth + cy*sth;
          float y = sl[1] + -cx*sth + cy*cth;
          int map_index = -1;
          if(get_cell_index(x, y, map_index))
            is_obstructed = false;
          else
          {
            int to_check = map_data[map_index];
            if(to_check == 0)
            {
              is_obstructed = true;
              return is_obstructed;
            }
          }
          cx = cx + resolution;
        }
        cy = cy + resolution;
      }
      return is_obstructed;
#endif
      return false;
    }

    float get_state_cost(float s[N])
    {
      float d = FLT_MAX/2;
      for(auto& sc : centerline_states)
        d = min(d, sc.dist(state(s)));
      return d;
    }
    float sample_univariate_gaussian(float mean, float stddev)
    {
      float p1 = RANDF;
      float p2 = RANDF;
      return (mean + stddev*sqrt(-2*log(p1)/p1)*cos(2*M_PI*p2));
    }
    int modulo_mpi_pi(float& th)
    {
      while(th < -M_PI)
        th = th + 2*M_PI;
      while(th > M_PI)
        th = th - 2*M_PI;
      return 0;
    }
    // returns a state in local coordinates
    int sample_free_space(float* z)
    {
      bool state_found = false;
      while(!state_found)
      {
        float x,y, theta;
        float sigma;
        int index = -1;
        float p = RANDF;
        if(p < 0.6 && !centerline_states.empty())
        {
          //cout << "Sampling from centerline!!" << endl;
          int q = (float)(RANDF*centerline_states.size());
          state chosen = centerline_states[q];
          x = chosen.x[0];
          y = chosen.x[1];
          theta = chosen.x[2];
          sigma = M_PI/12;
        }
        else
        {
          int q = (float)(RANDF*free_cells.size());
          index = free_cells[q];
          theta = 0; 
          sigma = M_PI/4;
          if(getxy_from_index(x, y, index))
            return 1;
        }
        z[0] = x + (RANDF-0.5)*resolution;
        z[1] = y + (RANDF-0.5)*resolution;
        z[2] = theta + sample_univariate_gaussian(0, sigma);
        modulo_mpi_pi(z[2]);

        state_found = !is_in_collision(z);
      }
      return 0;
    }
};

#endif
