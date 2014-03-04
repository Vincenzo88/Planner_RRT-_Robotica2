#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

#include <message_filters/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pnc_msgs/local_map.h>
#include <rrts/rrts_status.h>

#include <dynamical_system.h>
#include <dubins.h>
#include "local_map_route.h"
#include "rrts_system_route.h"
#include <rrts.h>

typedef state_t<3> state;
typedef vertex_t<3> vertex;
typedef region_t<3> region;

using namespace std;

class planner_t
{

    string base_link_;
    string global_frame_id_;

  public:

    ros::NodeHandle nh;

    planner_t();
    ~planner_t();

    dubins_t dubins;
    local_map_route_t<3> obstacle_map;
    rrts_system_route_t<3> system;
    rrts_t<3> rrts;
    int setup_rrts();
    float planner_dt;
    ros::Timer planner_timer, rrts_tree_pub_timer, 
      rrts_status_pub_timer;
    ros::Time last_found_best_path;
    ros::Time last_ct_unsafe;
    int planner_iteration();
    void on_planner_timer(const ros::TimerEvent &e);
    void on_rrts_tree_pub_timer(const ros::TimerEvent &e);
    
    state goal_state;
    bool is_first_goal, is_first_map;

    tf::TransformListener tfl;
    state robot_state;
    int get_robot_state();
    ros::Subscriber goal_sub, map_sub, centerline_sub;
    int change_goal_region();
    void on_goal(const geometry_msgs::PoseStamped::ConstPtr p);
    void on_map(const pnc_msgs::local_map::ConstPtr lm);

    trajectory_t committed_trajectory;
    int max_length_committed_trajectory;
    volatile bool is_updating_committed_trajectory;
    bool  should_send_new_committed_trajectory, 
         is_first_committed_trajectory;
    bool is_near_end_committed_trajectory();
    bool is_far_from_committed_trajectory();
    enum semaphore_member{TRAJ=0, TREE};
    int semaphore_set(int semaphore_member)
    {
      if(semaphore_member == TRAJ)
        is_updating_committed_trajectory = true;
      else if(semaphore_member == TREE)
      {
        is_updating_rrt_tree = true;
        is_updating_committed_trajectory = true;
      }
      return 0;
    }
    int semaphore_reset(int semaphore_member)
    {
      if(semaphore_member == TRAJ)
        is_updating_committed_trajectory = false;
      else if(semaphore_member == TREE)
      {
        is_updating_rrt_tree = false;
        is_updating_committed_trajectory = false;
      }
      return 0;
    }

    ros::Publisher committed_trajectory_pub, committed_trajectory_view_pub, control_trajectory_pub, best_trajectory_view_pub;
    ros::Timer committed_trajectory_pub_timer;
    int clear_committed_trajectory();
    int clear_committed_trajectory_length();
    int publish_committed_trajectory();
    int publish_control_trajectory();
    void on_committed_trajectory_pub_timer(const ros::TimerEvent &e);
    void on_centerline(const sensor_msgs::PointCloud&);
    ros::Time last_committed_time;

#define NUM_RRTS_STATUS  (7)
    enum status_def{rinc=0, ginc, ginf, ring, rnr, swr, tffc};
    bool rrts_status[NUM_RRTS_STATUS];
    ros::Publisher rrts_status_pub;
    void on_rrts_status_pub_timer(const ros::TimerEvent &e);
    bool is_root_in_goal();
    bool is_robot_in_collision();

    volatile bool is_updating_rrt_tree;
    int publish_rrts_tree();
    ros::Publisher rrts_tree_pub, rrts_vertex_pub;
};

planner_t::planner_t()
{

  ros::NodeHandle private_nh("~"); // NOTE the "~" argument is very important to properly get the parameters overridden in launchfile
  private_nh.param("robot_frame_id", base_link_, string("/base_link"));
  private_nh.param("global_frame_id", global_frame_id_, string("/map"));

  //ROS_ERROR("base_link_ is set to %s", base_link_.c_str());
  //ROS_ERROR("global_frame_id_ is set to %s", global_frame_id_.c_str());

  system.dynamical_system = &dubins;
  system.obstacle_map = &obstacle_map;

  ros::Rate wait_rate(10);
  while(get_robot_state())
  {
    cout<<"waiting for robot pose"<<endl;
    ros::spinOnce();
    wait_rate.sleep();
  }
  clear_committed_trajectory();
  is_updating_committed_trajectory = false;
  is_updating_rrt_tree = false;
  max_length_committed_trajectory = 10.0;

  planner_dt = 0.5;
  last_found_best_path = ros::Time::now();
  planner_timer = nh.createTimer(ros::Duration(planner_dt), &planner_t::on_planner_timer, this);
  rrts_status_pub_timer = nh.createTimer(ros::Duration(0.5), &planner_t::on_rrts_status_pub_timer, this);
  rrts_tree_pub_timer = nh.createTimer(ros::Duration(0.5), &planner_t::on_rrts_tree_pub_timer, this);
  committed_trajectory_pub_timer = nh.createTimer(ros::Duration(0.3), &planner_t::on_committed_trajectory_pub_timer, this);

  committed_trajectory_pub = nh.advertise<nav_msgs::Path>("pnc_trajectory",2);
  committed_trajectory_view_pub = nh.advertise<visualization_msgs::Marker>("pncview_trajectory",2);
  best_trajectory_view_pub = nh.advertise<visualization_msgs::Marker>("rrts_best_trajectory",2);
  rrts_tree_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_tree",1);
  rrts_vertex_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_vertex",1);
  rrts_status_pub = nh.advertise<rrts::rrts_status>("rrts_status",2);
  control_trajectory_pub = nh.advertise<std_msgs::Int16MultiArray>("control_trajectory", 2);

  map_sub = nh.subscribe("local_map",2, &planner_t::on_map, this);
  goal_sub = nh.subscribe("pnc_nextpose",2, &planner_t::on_goal, this);
  centerline_sub = nh.subscribe("route_centerline",2, &planner_t::on_centerline, this);
  is_first_goal = is_first_map = true;
  for(int i=0;i<NUM_RRTS_STATUS; i++)
    rrts_status[i] = false;

  ros::spin();
}
planner_t::~planner_t()
{
  clear_committed_trajectory();
}
int planner_t::clear_committed_trajectory()
{
  semaphore_set(TRAJ);
  committed_trajectory.clear();
  semaphore_reset(TRAJ);
  last_committed_time = ros::Time();
  //cout<<"cleared ct"<<endl;
  return 0;
}
int planner_t::clear_committed_trajectory_length()
{
  if(get_robot_state())
  {
    cout<<__LINE__<<" get_robot_pose failed"<<endl;
    return 0;
  }
  if(!committed_trajectory.states.empty())
  {
    int num_delete = 0;
    bool found_length_to_delete = false;
    state prev_state = state(committed_trajectory.states.front());
    float distance_deleted = 0;
    for(auto& ps : committed_trajectory.states)
    {
      state curr_state = state(ps);
      distance_deleted += curr_state.dist(prev_state);
      prev_state = curr_state;
      if(curr_state.dist(robot_state) < 2)
      {
        found_length_to_delete = true;
        break;
      }
      num_delete++;
    }
    if(found_length_to_delete)
    {
      semaphore_set(TRAJ);
      committed_trajectory.pop_front(num_delete);
      committed_trajectory.total_variation -= distance_deleted;
      semaphore_reset(TRAJ);
    }

  }
  return 0;
}

void planner_t::on_rrts_status_pub_timer(const ros::TimerEvent &e)
{
  rrts::rrts_status smsg;
  smsg.header.stamp = ros::Time::now();
  smsg.robot_in_collision = rrts_status[rinc];
  smsg.goal_in_collision = rrts_status[ginc];
  smsg.goal_infeasible = rrts_status[ginf];
  smsg.root_in_goal = rrts_status[ring];
  smsg.robot_near_root = rrts_status[rnr];

  smsg.switched_root = rrts_status[swr];
  smsg.too_far_from_committed = rrts_status[tffc];

  rrts_status_pub.publish(smsg);

  rrts_status[swr] = false;
}

int planner_t::change_goal_region()
{
  float size[3] = {1,1,30/180.0*M_PI};
  system.goal_region = region(goal_state.x, size);
  return 0;
}

void planner_t::on_goal(const geometry_msgs::PoseStamped::ConstPtr ps)
{
  double roll=0, pitch=0, yaw=0;
  tf::Quaternion q;
  tf::quaternionMsgToTF(ps->pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  float ros_goal_pose[3] = {(float)ps->pose.position.x, (float)ps->pose.position.y, (float)yaw};
  
  if(get_robot_state())
    cout<<"did not get robot state"<<endl;

  if(is_first_goal)
  {
    is_first_goal = false;
    cout<<"got first goal"<<endl;
    goal_state = state(ros_goal_pose);
    
    cout<<"got goal: "; goal_state.print(cout); cout<<endl;
    cout<<"robot_state: "; robot_state.print(cout); cout<<endl;
    if(is_first_map == false)
    {
      setup_rrts();
      if(rrts.system->is_in_collision(goal_state))
      {
        cout<<"goal in collision: stopping"<<endl;
        rrts_status[ginc] = true;
      }
      else
        rrts_status[ginc] = false;
    }
  }
  bool only_xy = true;
  // new goal than previous one, change sampling region
  if((!is_first_map) && (!is_first_goal))
  {
    state new_goal_state = state(ros_goal_pose);
    if( goal_state.dist(new_goal_state, only_xy) > 1)
    {
      goal_state = new_goal_state;
      cout<<"got goal: "; goal_state.print(cout); cout<<endl;
      if(rrts.system->is_in_collision(goal_state))
      {
        cout<<"goal in collision: sending collision"<<endl;
        rrts_status[ginc] = true;
      }
      else
      {  
        change_goal_region();
        rrts_status[ginc] = false;
      }
    }
  }
}


int planner_t::get_robot_state()
{
  //cout << "entering get_robot_state()" << endl << flush;
  tf::Stamped<tf::Pose> map_pose;
  map_pose.setIdentity();
  tf::Stamped<tf::Pose> robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = base_link_;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();

  bool transform_is_correct = true;
  try {
    tfl.transformPose(global_frame_id_, robot_pose, map_pose);
  }
  catch(tf::LookupException& ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    transform_is_correct = false;
  }
  catch(tf::ConnectivityException& ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    transform_is_correct = false;
  }
  catch(tf::ExtrapolationException& ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    transform_is_correct = false;
  }
  // check odom_pose timeout
  if (current_time.toSec() - map_pose.stamp_.toSec() > 0.6) {
    ROS_WARN("Get robot pose transform timeout. Current time: %.4f, odom_pose stamp: %.4f, tolerance: %.4f",
        current_time.toSec(), map_pose.stamp_.toSec(), 0.1);
    transform_is_correct = false;
  }

  if(transform_is_correct)
  {
    geometry_msgs::PoseStamped tmp;
    tf::poseStampedTFToMsg(map_pose, tmp);

    double roll=0, pitch=0, yaw=0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(tmp.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    float ros_robot_pose[3] = {(float)tmp.pose.position.x, (float)tmp.pose.position.y, (float)yaw};
    robot_state = state(ros_robot_pose);
    //cout << "exiting get_robot_state() successfully" << endl << flush;
    return 0;
  }
  //cout << "exiting get_robot_state() insuccessfully" << endl << flush;
  return 1;
}

void planner_t::on_map(const pnc_msgs::local_map::ConstPtr lm)
{
  obstacle_map.copy_local_map(lm);

  if(is_first_map)
  {
    is_first_map = false;
    cout<<"got first map"<<endl;
    if(is_first_goal == false)
    {
      setup_rrts();
    }
  }
  //cout<<"robot_state: "; robot_state.print(cout); cout<<endl;
  //cout<<"map_origin: "; obstacle_map.origin.print(cout); cout<<endl;
}

bool planner_t::is_root_in_goal()
{
  bool res = system.is_in_goal(*(rrts.root->state));
  rrts_status[ring] = res;
  return res;
}
bool planner_t::is_robot_in_collision()
{
  if(get_robot_state())
  {
    cout<<"get_robot_state failed"<<endl;
    return false;
  }
  bool res = false;
  if((!is_first_goal) && (!is_first_map))
      res = system.is_in_collision(robot_state);
  
  rrts_status[rinc] = res;
  return res;
}

int planner_t::setup_rrts()
{
  // 1. set root to current state
  if(get_robot_state())
    return 1;

  // 2. set operating region in local coordinates
  float h = obstacle_map.height*obstacle_map.resolution;
  float w = obstacle_map.width*obstacle_map.resolution;
  float center[3] = {w/2, h/2,0};
  float size[3] = {w, h, 2.0*M_PI};
  system.operating_region = region(center, size);
  
  // 3. set goal region
  change_goal_region();

  rrts.gamma = 2.5;
  rrts.goal_sample_freq = 0.3;
  
  const state* root_state = new state(robot_state);
  rrts.initialize(&system, root_state); 
  
  should_send_new_committed_trajectory = false;
  is_first_committed_trajectory = true;
  
  return 0;
}

int planner_t::planner_iteration()
{
  //cout << "iteration called" << endl << flush;
#if 1
  semaphore_set(TREE);
  if(rrts.lazy_check_tree(committed_trajectory))
  {
    cout<<"root in collision"<<endl;
    clear_committed_trajectory();
    setup_rrts();
  }
  semaphore_reset(TREE);
#endif

  // 2. no need to do anything if root is inside goal
  if(is_root_in_goal())
  {
    return 0;
  }

  // 3. sample states, get best trajectory
  bool found_best_path = false;
  cost_t best_cost = *rrts.get_best_cost();
  cost_t prev_best_cost;
  int num_samples = 0;
  ros::Time start_iteration = ros::Time::now();
  cout<<"s: "<< rrts.num_vertices<<" cost: "<<best_cost.val<<endl;
  while(!found_best_path)
  {
    // 3.a perform one iteration
    int ret = rrts.iteration();
    if(!ret)
    {
      num_samples++;
      publish_rrts_tree();
      publish_committed_trajectory();
    }
    // 3.b break if close to end of time slice
    ros::Duration dt = ros::Time::now() - start_iteration;
    if(dt.toSec() > 0.9*planner_dt)
    {
      //cout<<"broke out: 447"<<endl;
      break;
    }
  }
  prev_best_cost = best_cost;
  best_cost = *rrts.get_best_cost();
  
  cout<<"e: "<< rrts.num_vertices<<" cost: "<<best_cost.val<<endl;
  cost_t max_cost(FLT_MAX);
  ros::Duration delta_last_found_best_path = ros::Time::now() - last_found_best_path;
  if(best_cost < max_cost)
  {
    if((best_cost.difference(prev_best_cost) > 0.5) && (delta_last_found_best_path.toSec() > 2) || !found_best_path)
    {
      found_best_path = true;
      last_found_best_path = ros::Time::now();
    }
  }

  // 4. switch_root
  if(found_best_path)
  {

    // cout<<" ##################################################################" << endl;
    // cout<<" #################     FOUND BEST PATH        #####################" << endl;
    // cout<<" ##################################################################" << endl;
    // cout<<" #######  should_send = " << should_send_new_committed_trajectory << "   ####################" << endl;
    // cout<<" #######  is_first = " << is_first_committed_trajectory << "   ####################" << endl;
    // cout<<" #######  delta_last_found_best_path.toSec() = " << delta_last_found_best_path.toSec() << "  ##################" << endl;

    rrts_status[ginf] = false;
    if(should_send_new_committed_trajectory || is_first_committed_trajectory)
    {
      semaphore_set(TREE);
      int res = rrts.switch_root(max_length_committed_trajectory, committed_trajectory);
      if(res)
      {

        // cout<<" ##################################################################" << endl;
        // cout<<" ##################################################################" << endl;
        // cout<<" ##################################################################" << endl;
        // cout<<" ############################      ################################" << endl;
        // cout<<" #####################                     ########################" << endl;
        // cout<<" #################     CANNOT SWITCH ROOOT    #####################" << endl;
        // cout<<" #####################                     ########################" << endl;
        // cout<<" ############################      ################################" << endl;
        // cout<<" ##################################################################" << endl;
        // cout<<" ##################################################################" << endl;
        // cout<<" ##################################################################" << endl;

        //committed_trajectory.clear();
        #if 0
        //exit(1); // not nice to stop randomly!
        #else
          cout << "replanning..." << endl;
          // just replan      
          clear_committed_trajectory();
          setup_rrts();
          planner_iteration();
        #endif
      }
      else
      {
        rrts_status[swr] = true;
        cout<<"switched root successfully: "<< committed_trajectory.total_variation<<endl;
        cout<<"esw: "<< rrts.num_vertices<<" cost: "<<best_cost.val<<endl;
        last_committed_time = ros::Time();
      }
      semaphore_reset(TREE);
      publish_rrts_tree();

      is_first_committed_trajectory = false;
      should_send_new_committed_trajectory = false;
      return 0;
    }
  }
  
  if( (rrts.num_vertices > 500    // cout<<" ##################################################################" << endl;
    // cout<<" #################     FOUND BEST PATH        #####################" << endl;
    // cout<<" ##################################################################" << endl;
    // cout<<" #######  should_send = " << should_send_new_committed_trajectory << "   ####################" << endl;
    // cout<<" #######  is_first = " << is_first_committed_trajectory << "   ####################" << endl;
    // cout<<" #######  delta_last_found_best_path.toSec() = " << delta_last_found_best_path.toSec() << "  ##################" << endl;
) || (delta_last_found_best_path.toSec() > 20))
  {
    last_found_best_path = ros::Time::now();

    rrts_status[ginf] = true;
    cout<<"did not find best path: reinitializing"<<endl;
    should_send_new_committed_trajectory = true;
    clear_committed_trajectory();
    setup_rrts();

    return 1;
  }
  return 0;
}

bool planner_t::is_near_end_committed_trajectory()
{
  bool ret = false;
  bool only_xy = true;
  if(get_robot_state())
    return false;
  if(!committed_trajectory.states.empty())
  {
    state last_state = state(committed_trajectory.states.back());
    float t = last_state.dist(robot_state, only_xy);
    if(t< 10.0)
      ret = true;
  }
  rrts_status[rnr] = ret;
  return ret;
}

bool planner_t::is_far_from_committed_trajectory()
{
  if(committed_trajectory.states.empty())
    return false;
  if(get_robot_state())
  {
    cout<<__LINE__<<": did not get robot state"<<endl;
    return false;
  }
  bool ret = true, only_xy = true;
  state sc;

  for(auto& ps : committed_trajectory.states)
  {
    sc = state(ps);
    if(sc.dist(robot_state, only_xy) < 2.0)
    {
      ret = false;
      break;
    }
  }
  rrts_status[tffc] = ret;
  return ret;
}

void planner_t::on_planner_timer(const ros::TimerEvent &e)
{
  //cout<<endl<<"timer del: "<< e.profile.last_duration<<endl;
  //cout<<endl;

  // 1. failsafes 
  if(is_robot_in_collision())
  {
    cout<<"1. robot in collision"<<endl;
    clear_committed_trajectory();
    return;
  }
  // 2. is committed trajectory safe
  if(!system.is_safe_trajectory(committed_trajectory))
  {
    cout<<"2. is it safe?"<<endl;
    #if 0
    cout<<"committed_trajectory unsafe, waiting for clearance..."<<endl; 
    ros::Duration(2).sleep();
    cout << "waking up..." << endl;
    if(!system.is_safe_trajectory(committed_trajectory)){
      clear_committed_trajectory();
      setup_rrts();
    }
    #else
      clear_committed_trajectory();
      setup_rrts();
      should_send_new_committed_trajectory = true;
    #endif
  }
  // 3. is at end of trajectory
  else if(is_near_end_committed_trajectory() && (!is_root_in_goal()))
  {
    cout<<"3. near end of committed trajectory"<<endl;
    should_send_new_committed_trajectory = true;
    planner_iteration();
    return;
  }
  // 4. if far from trajectory, emergency replan
  else if(is_far_from_committed_trajectory())
  {
    cout<<"4. far from committed trajectory: emergency replan"<<endl;
    clear_committed_trajectory();
    setup_rrts();
    planner_iteration();
    return;
  }
  // 5. nominal case
  else
  {
    //cout << "5. nominal" << endl;
    if((!is_first_map) && (!is_first_goal))
      planner_iteration();
  }
}

int planner_t::publish_control_trajectory()
{
  std_msgs::Int16MultiArray tmp;

  for(auto& pc : committed_trajectory.controls)
  {
    tmp.data.push_back((int)pc[0]);
  }
  control_trajectory_pub.publish(tmp);
  return 0;
}

int planner_t::publish_committed_trajectory()
{
  if(is_updating_committed_trajectory)
    return 1;

  clear_committed_trajectory_length();

  // 1. publish to golfcar_pp
  nav_msgs::Path traj_msg;
  traj_msg.header.stamp = ros::Time(); //last_committed_time;
  traj_msg.header.frame_id = global_frame_id_;

  auto pc = committed_trajectory.controls.begin();
  for(auto& ps : committed_trajectory.states)
  {
    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time(); //last_committed_time;
    p.header.frame_id = global_frame_id_;

    p.pose.position.x = ps[0];
    p.pose.position.y = ps[1];
    p.pose.position.z = (*pc)[0];        // send control as the third state
    p.pose.orientation.w = 1.0;
    //printf(" [%f, %f, %f]", p.pose.position.x, p.pose.position.y, p.pose.position.z);

    traj_msg.poses.push_back(p);
    pc++;
  }
  committed_trajectory_pub.publish(traj_msg);

// 2. publish to rviz
  visualization_msgs::Marker ls_msg;
  ls_msg.header.frame_id = global_frame_id_;
  ls_msg.header.stamp = ros::Time::now();
  ls_msg.ns = "commtraj";
  ls_msg.id = 0;
  ls_msg.type = visualization_msgs::Marker::LINE_STRIP;
  ls_msg.action = visualization_msgs::Marker::ADD;

  ls_msg.color.r = 0;
  ls_msg.color.g = 1;
  ls_msg.color.b = 0;
  ls_msg.color.a = 0.8;
  ls_msg.scale.x = 0.15;

  ls_msg.lifetime = ros::Duration(200*planner_dt);

  geometry_msgs::Point p;
  //if(!committed_trajectory.states.empty()){ // wanna see when there's no ct!
    for(auto& ps : committed_trajectory.states){
      p.x = ps[0];
      p.y = ps[1];
      p.z = 0;

      ls_msg.points.push_back(p);
    }
    committed_trajectory_view_pub.publish(ls_msg);
  //}

  // 2. publish best trajectory to rviz

  ls_msg.points.clear();

  ls_msg.ns = "bestTraj";  
  ls_msg.color.r = 0;
  ls_msg.color.g = 0.9;
  ls_msg.color.b = 1;
  ls_msg.color.a = 0.2;
  ls_msg.scale.x = 0.3;

  trajectory_t best_traj;
  rrts.get_best_trajectory(best_traj);

  for(auto& ps : best_traj.states)
  {
    geometry_msgs::Point p;

    p.x = ps[0];
    p.y = ps[1];
    p.z = 0;

    ls_msg.points.push_back(p);

  }
  
  best_traj.clear();
  best_trajectory_view_pub.publish(ls_msg);

  return 0;
}

void planner_t::on_committed_trajectory_pub_timer(const ros::TimerEvent &e)
{
  publish_committed_trajectory();
  publish_control_trajectory();
  return;
}

int planner_t::publish_rrts_tree()
{
  if(is_updating_rrt_tree)
    return 1;
  
  bool check_obstacles = false;
  
  sensor_msgs::PointCloud pcv;
  pcv.header.stamp = ros::Time::now();
  pcv.header.frame_id = global_frame_id_;

  sensor_msgs::PointCloud pct;
  pct.header.stamp = ros::Time::now();
  pct.header.frame_id = global_frame_id_;

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

void planner_t::on_rrts_tree_pub_timer(const ros::TimerEvent& e)
{
  //publish_rrts_tree();
}


void planner_t::on_centerline(const sensor_msgs::PointCloud& pc){
  obstacle_map.set_centerline(pc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrts_node");

  planner_t planner;

  return 0;
}
