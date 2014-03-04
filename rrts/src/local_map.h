#ifndef __local_map_h__
#define __local_map_h__

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pnc_msgs/local_map.h>

#include <map.h>

template<size_t N>
class local_map_t : public map_t<N>
{
  public:
    typedef state_t<N> state;
    
    int height, width;
    float resolution;
    vector<int> free_cells;
    /*vector<int> centerline;
    vector<state> centerline_states;*/
    vector<signed char> map_data;
    state_t<N> origin;

    /**
     * safe_distance : car is blown up by this distance for obstacle checking
     * factor_reduce_size : reduce size of car for simulation sometimes
     * distance_rear_axis_rear : distance between center of rear axis and rear end of car
     */
    float car_width, car_height, safe_distance, 
          factor_reduce_size, distance_rear_axis_rear;
        
    local_map_t()
    {
      factor_reduce_size = 1.0;
      car_width = 1.2/factor_reduce_size;
      car_height = 2.28/factor_reduce_size;
      safe_distance = 0.22/factor_reduce_size; 
      distance_rear_axis_rear = 0.45/factor_reduce_size;
    }
    
    int copy_local_map(const pnc_msgs::local_map::ConstPtr lm)
    {
      free_cells = lm->free_cells;
      //centerline = lm->centerline;
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
      
      // create centerline_xy
     /* centerline_states.clear();
      for(unsigned int i= 0; i<centerline.size(); i++)
      {
        float x=0, y=0;
        if(getxy_from_index(x,y, centerline[i]))
        {
          cout<<"bad index values for centerline"<<endl;
        }
        float theta = 0;
        if(i > 0)
          theta = atan2(y-centerline_states.back().x[1],  x-centerline_states.back().x[0]);
        float t1[3] = {x,y,theta};
        centerline_states.push_back(state(t1));
      }*/
      return 0;
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

    bool is_out_of_map(const float sl[N]){
      int cx = sl[0]/resolution;
      int cy = sl[1]/resolution;

      return ((cx <0 || cx > width) || (cy < 0 || cy > height));
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
            is_obstructed = false; // free space outside local map!
            //is_obstructed = true; // obstacle space outside local map!        
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

    /*float get_state_cost(float s[N])
    {
      float d = FLT_MAX/2;
      for(auto& sc : centerline_states)
        d = min(d, sc.dist(state(s)));
      return d;
    }*/
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
        int index = -1;
        float p = RANDF;
        /*if(p < 0.001)
        {
          int q = (float)(RANDF*centerline.size());
          //cout<<"p: "<< p<<endl;
          index = centerline[q];
        }
        else
        {*/
          if(free_cells.size()==0) return 1;
          int q = rand()%free_cells.size();
          
          //if(q>=free_cells.size() ) return 1;
          //cout<<"p: "<< p<<endl;
          index = free_cells[q];
        //}
        float x,y;
        if(getxy_from_index(x, y, index))
          return 1;

        z[0] = x + (RANDF-0.5)*resolution;
        z[1] = y + (RANDF-0.5)*resolution;
#if 1
        z[2] = sample_univariate_gaussian(M_PI/2, M_PI/4);
        modulo_mpi_pi(z[2]);
#else
        z[2] = (RANDF-0.5)*2*M_PI;
#endif
        state_found = !is_in_collision(z);
      }
      //cout<<"z: "<< z[0] <<" "<< z[1]<<" "<< z[2]<<endl;
      return 0;
    }

};

#endif
