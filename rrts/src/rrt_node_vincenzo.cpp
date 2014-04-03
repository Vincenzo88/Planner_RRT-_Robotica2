#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <fstream>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
//#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <pnc_msgs/local_map.h>


#include <dynamical_system.h>
#include <dubins.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <rrts.h>

using namespace std;
typedef state_t<3> state;
typedef vertex_t<3> vertex;
typedef region_t<3> region;

class my_map_t : public map_t<3>
{
  public:
    int rows;
    int cols;
    double mapresolution;
    nav_msgs::OccupancyGrid grid;
    
    int sample_free_space(float[3])
    {
      std::cout << "sample_free_space" << std::endl;
      return 1;
  
    }
    bool is_in_collision(const float s[3])
    {
       int m = round(s[0]/grid.info.resolution);
       int n = round(s[1]/grid.info.resolution);
  
       signed int height = grid.info.height;
       signed int width = grid.info.width;
       
       
       if( n < 0 || m < 0 || height < n || width < m)
	 return 1;
       
      return grid.data[n*cols+m]!=0;
      return false;
    }
    float get_state_cost(const float s[3])
    {
      return 0;
    }
    void init(const nav_msgs::OccupancyGridConstPtr mapptr)
    {
     grid = *mapptr;
     rows = grid.info.height;
     cols = grid.info.width;
     mapresolution = grid.info.resolution; 
    }
};


class planner_t
{
  public:
    
    ros::NodeHandle nh;
    ros::Timer planner_timer;
    planner_t();
    my_map_t map; 
    dubins_t dubins;
    system_t<3> system;
    rrts_t<3> rrts;

    //rrts_tree_pub_timer;
    //void on_rrts_tree_pub_timer(const ros::TimerEvent &e);
    void on_planner_timer(const ros::TimerEvent &e);    
    void on_map(const nav_msgs::OccupancyGridConstPtr); 
    bool map_received;
    bool goal_received;

    ros::Subscriber goal_sub, map_sub;
    void on_goal(const geometry_msgs::PoseStamped::ConstPtr p);
    bool collision_check(float* pos);
    
    int publish_rrts_tree();
    int max_iterations_dt;
    int max_iterations_rrt; 
    float size[3];
    int publish_committed_trajectory();
    time_t t_begin_planning;
    trajectory_t traj;
    
    void write_to_file(std::vector<time_t> time,std::vector<double> cost, std::vector<double> nedge);
    
    time_t ts;
    time_t tf;
    
    std::vector<time_t> t_iteration;
    std::vector<double> cost_iteration;
    std::vector<double> edge;

    visualization_msgs::Marker traj_marker;
    
    
    
#define NUM_RRTS_STATUS  (7)
    ros::Publisher rrts_tree_pub, rrts_vertex_pub, rrts_path_pub, rrts_trajectory_pub;
   
};




planner_t::planner_t()
{
  map_received = false;
  goal_received = false;
  max_iterations_dt = 100;
  max_iterations_rrt = 2000;
  

  rrts_tree_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_tree",1);
  rrts_vertex_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_vertex",1);
  
  rrts_path_pub = nh.advertise<visualization_msgs::Marker>("rrts_path",1);

  
  
  
  map_sub = nh.subscribe("map",2,&planner_t::on_map,this);
  goal_sub = nh.subscribe("pnc_nextpose",2, &planner_t::on_goal, this);

  system.dynamical_system = &dubins;
  system.obstacle_map = &map;
  
  planner_timer = nh.createTimer(ros::Duration(1), &planner_t::on_planner_timer, this);
 
  
   traj_marker.header.frame_id = "map";
   traj_marker.header.stamp = ros::Time();
   traj_marker.ns = "traj_path";
   traj_marker.id = 0;
   traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
   traj_marker.pose.position.x = 0;
   traj_marker.pose.position.y = 0;
   traj_marker.pose.position.z = 0;
   traj_marker.pose.orientation.x = 0.0;
   traj_marker.pose.orientation.y = 0.0;
   traj_marker.pose.orientation.z = 0.0;
   traj_marker.pose.orientation.w = 1.0;
   traj_marker.scale.x = 3;
   traj_marker.scale.y = 3;
   traj_marker.scale.z = 3;
   traj_marker.color.a = 1.0;
   traj_marker.color.r = 255.0;
   traj_marker.color.g = 127.0;
   traj_marker.color.b = 80.0;

  
  
}

bool planner_t::collision_check(float *posizione)
{
   return map.is_in_collision(posizione);
 }
  
  


void planner_t::on_goal(const geometry_msgs::PoseStamped::ConstPtr ps)
{
 
  double roll=0, pitch=0, yaw=0;
  tf::Quaternion q;
  tf::quaternionMsgToTF(ps->pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  float ros_goal_pose[3] = {(float)ps->pose.position.x, (float)ps->pose.position.y, (float)yaw};
  std::cout << "mi è arrivato un goal" << std::endl;
  bool b = collision_check(ros_goal_pose);
    if(!b){
      
    float gs[3] = {5,5,0.1*M_PI};
    system.goal_region = region(ros_goal_pose,gs);
    float init_state[3] = {4,14,0};
    state* origin = new state(init_state);
    rrts.initialize(&system, origin);
    goal_received = true;
    }
}

void planner_t::on_map(const nav_msgs::OccupancyGridConstPtr mapptr)
{
  map_received = true; 
  map.init(mapptr);
  
  float zero[3];
  zero[0] = map.mapresolution*map.cols/2;
  zero[1] = map.mapresolution*map.rows/2;
  zero[2] = 0;
  float size[3];
  size[0] = map.mapresolution*map.cols;
  size[1] = map.mapresolution*map.rows;
  size[2] = 2*M_PI;
  
  system.operating_region = region(zero, size);
  rrts.goal_sample_freq = 0.005;
  
}


void planner_t::on_planner_timer(const ros::TimerEvent &e)
{

  //ofstream myfile("/home/vincenzo/nuovo.txt", ios::out | ios::binary);
   //myfile << "Writing this to a file.\n";
   //myfile.close();
   
   if(goal_received){
  // Goal ricevuto
  if (rrts.num_iterations == 0){
    // All'inizio del planning salvo il tempo di sistema in secondi
    ts = time(0);
    // Cancello i dati relativi al goal precedente
    t_iteration.clear();
    cost_iteration.clear();
    edge.clear();
    
  }
  
  if(rrts.num_iterations == max_iterations_rrt){
    // Ultima iterazione, scrivo dati su file
    write_to_file(t_iteration,cost_iteration,edge);
    cout << "Ho finito il planning" << endl;
  }
    
    
  for(int i=0; i < max_iterations_dt && rrts.num_iterations < max_iterations_rrt ; i++)
  {
   
   rrts.iteration();
   
   t_iteration.push_back(time(0)-ts);
   cost_iteration.push_back(rrts.get_best_cost()->val);
   edge.push_back(rrts.num_edge);
   
   if(rrts.num_iterations%50 == 0){
      publish_rrts_tree();
      publish_committed_trajectory();
    }
  }
  
  }
  rrts.get_best_trajectory(traj);
}

int planner_t::publish_rrts_tree()
{
  
  bool check_obstacles = false;
  
  sensor_msgs::PointCloud pcv;
  pcv.header.stamp = ros::Time::now();
  pcv.header.frame_id = "map";

  sensor_msgs::PointCloud pct;
  pct.header.stamp = ros::Time::now();
  pct.header.frame_id = "map";

  if(rrts.num_vertices)
  {
    for(auto& pv : rrts.list_vertices)
    {
      vertex& v = *pv;
      float vs[3] = {v.state->x[0], v.state->x[1],0};
      
      // vertex
      geometry_msgs::Point32 p;
      p.x = vs[0];
      p.y = vs[1];
      p.z = vs[2];
      pcv.points.push_back(p);
      pct.points.push_back(p);
      
      // edge from parent
      if(v.parent)
      {
        vertex& parent = *(v.parent);
        trajectory_t traj_from_parent;
        if(system.extend_to(parent.state, v.state, check_obstacles, traj_from_parent, v.edge_from_parent->opt_data))
        {
          traj_from_parent.clear();
          cout<<"publish_tree: could not draw trajectory"<<endl;
          continue;
        }
        for(auto& ps : traj_from_parent.states)
        {
          geometry_msgs::Point32 p2;
          p2.x = ps[0];
          p2.y = ps[1];
          p2.z = 0.0;
          pct.points.push_back(p2);
        }
        traj_from_parent.clear();
      }
    }
  }
  rrts_tree_pub.publish(pct);
  rrts_vertex_pub.publish(pcv);

  return 0;
}

int planner_t::publish_committed_trajectory()
{
  
  // 2. publish to rviz
  geometry_msgs::Point p;
  traj_marker.points.clear();
  for(auto& ps : traj.states)
  {
   
    p.x = ps[0];
    p.y = ps[1];
    p.z = 0;
    traj_marker.points.push_back(p);
  }
  rrts_path_pub.publish(traj_marker);
  return 0;
}


void planner_t::write_to_file(std::vector<time_t> time, std::vector<double> cost, std::vector<double> nedge)
{
   ofstream mytime("/home/vincenzo/tempo_iteration.txt", ios::out);
   for(unsigned i = 0; i < time.size(); i++){
   mytime << time.at(i) <<  endl;
   }
   mytime.close();
   
   ofstream myiteration("/home/vincenzo/costo_iteration.txt", ios::out);
   for(unsigned i = 0; i < cost.size(); i++){
   myiteration << cost.at(i) <<  endl;
   }
   myiteration.close();
   
   
   ofstream myedge("/home/vincenzo/edge_iteration.txt", ios::out);
   for(unsigned i = 0; i < nedge.size(); i++){
   myedge << nedge.at(i) <<  endl;
   }
   myedge.close();  
}

int main(int argc, char **argv)
{
    
  ros::init(argc, argv, "rrts_node");
  planner_t planner;
  ros::spin();

  return 0;
}
