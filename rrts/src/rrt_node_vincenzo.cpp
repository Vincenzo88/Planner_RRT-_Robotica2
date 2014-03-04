#include <iostream>
#include <ctime>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pnc_msgs/local_map.h>


#include <dynamical_system.h>
#include <dubins.h>

#include <rrts.h>

typedef state_t<3> state;
typedef vertex_t<3> vertex;
typedef region_t<3> region;

class planner_t
{
  public:
    ros::NodeHandle nh;

    planner_t();

    dubins_t dubins;
    system_t<3> system;
    rrts_t<3> rrts;
    
    ros::Timer planner_timer, rrts_tree_pub_timer;
    //void on_rrts_tree_pub_timer(const ros::TimerEvent &e);
    void on_planner_timer(const ros::TimerEvent &e);    
        void on_map(const nav_msgs::OccupancyGridConstPtr); 
    bool map_received;

    nav_msgs::OccupancyGrid map;
    ros::Subscriber goal_sub, map_sub;
    void on_goal(const geometry_msgs::PoseStamped::ConstPtr p);
    bool collision_check(float* pos);
   




    
#define NUM_RRTS_STATUS  (7)
    ros::Publisher rrts_tree_pub, rrts_vertex_pub;
};

planner_t::planner_t()
{
  map_received = false;
  planner_timer = nh.createTimer(ros::Duration(0.5), &planner_t::on_planner_timer, this);
  //rrts_tree_pub_timer = nh.createTimer(ros::Duration(0.5), &planner_t::on_rrts_tree_pub_timer, this);

  rrts_tree_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_tree",1);
  rrts_vertex_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_vertex",1);

  map_sub = nh.subscribe("map",1,&planner_t::on_map,this);
  goal_sub = nh.subscribe("pnc_nextpose",2, &planner_t::on_goal, this);


  ros::spin();
}

bool planner_t::collision_check(float *posizione)
{
  std::cout << "sono arrivato nella funzione" << std::endl;
  return 1;
 }

void planner_t::on_goal(const geometry_msgs::PoseStamped::ConstPtr ps)
{
 
  double roll=0, pitch=0, yaw=0;
  tf::Quaternion q;
  tf::quaternionMsgToTF(ps->pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  float ros_goal_pose[3] = {(float)ps->pose.position.x, (float)ps->pose.position.y, (float)yaw};
  std::cout << "mi è arrivato un goal" << std::endl;
  std::cout << ros_goal_pose[0] << std::endl;
  std::cout << ros_goal_pose[1] << std::endl;
  std::cout << ros_goal_pose[2] << std::endl;
  bool b = collision_check(ros_goal_pose);
  std::cout << b << std::endl;
 }

void planner_t::on_map(const nav_msgs::OccupancyGridConstPtr mapptr)
{
  map = *mapptr;
  map_received = true;
  std::cout << "mappa ricevuta" << std::endl;
  std::cout << " sono qui" << std::endl;
  std::cout << " sono qui" << std::endl;
}


void planner_t::on_planner_timer(const ros::TimerEvent &e)
{
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrts_node");

  planner_t planner;

  return 0;
}
