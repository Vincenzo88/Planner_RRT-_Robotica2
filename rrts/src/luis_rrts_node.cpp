#include <ctime>
#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <pnc_msgs/local_map.h>

#include <sensor_msgs/PointCloud.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <rrts.h>

#include <rrts/rrts_status.h>

#include <dynamical_system.h>

#include <dubins.h>

#include "local_map.h"

#include "rrts_system.h"

#define NUM_RRTS_STATUS  (7)

typedef state_t<3> state;

typedef vertex_t<3> vertex;

typedef region_t<3> region;

using namespace std;

class planner_t {

  public:
      
    // --------------------------------------------------------------------------------------------
    // MEMBERS: ROS-RELATED TYPES
    // --------------------------------------------------------------------------------------------
    
    ros::NodeHandle nh;
    
    ros::Publisher  committed_trajectory_pub, 
                    
                    committed_trajectory_view_pub, 
                    
                    control_trajectory_pub;
    
    ros::Publisher rrts_status_pub;
    
    ros::Publisher rrts_tree_pub, rrts_vertex_pub;
    
    ros::Subscriber goal_sub, map_sub;
    
    ros::Time last_found_best_path;
    
    ros::Timer planner_timer, committed_trajectory_pub_timer, rrts_tree_pub_timer, rrts_status_pub_timer;
    
    tf::TransformListener tfl;
    
    // --------------------------------------------------------------------------------------------
    // MEMBERS: RRT*-RELATED TYPES
    // --------------------------------------------------------------------------------------------
    
    dubins_t dubins;

    local_map_t<3> obstacle_map;

    rrts_system_t<3> system;
    
    rrts_t<3> rrts;
    
    state robot_state, goal_state;
    
    trajectory_t committed_trajectory;
      
    // --------------------------------------------------------------------------------------------
    // MEMBERS: PRIMITIVE TYPES
    // --------------------------------------------------------------------------------------------
    
    enum semaphore_member { TRAJ = 0, TREE };
    
    enum status_def{ rinc=0, ginc, ginf, ring, rnr, swr, tffc};
    
    float planner_dt;
    
    int max_length_committed_trajectory;
    
    bool is_first_committed_trajectory;
    
    bool is_first_goal, is_first_map;
    
    bool rrts_status[NUM_RRTS_STATUS];
    
    bool  should_send_new_committed_trajectory;
    
    volatile bool is_updating_rrt_tree;
    
    volatile bool is_updating_committed_trajectory;
    
    // --------------------------------------------------------------------------------------------
    // CONSTRUCTOR AND DESTRUCTOR
    // --------------------------------------------------------------------------------------------
    
    planner_t();

    ~planner_t();
    
    // --------------------------------------------------------------------------------------------
    // METHODS: FUNCTIONS USED WHEN LISTENING
    // --------------------------------------------------------------------------------------------
    
    int get_robot_state();
    
    void on_committed_trajectory_pub_timer(const ros::TimerEvent &e);
    
    void on_goal(const geometry_msgs::PoseStamped::ConstPtr p);
    
    void on_map(const pnc_msgs::local_map::ConstPtr lm);
    
    void on_planner_timer(const ros::TimerEvent &e);
    
    void on_rrts_status_pub_timer(const ros::TimerEvent &e);
    
    void on_rrts_tree_pub_timer(const ros::TimerEvent &e);
    
    // --------------------------------------------------------------------------------------------
    // METHODS: FUNCTIONS USED WHEN PUBLISHING
    // --------------------------------------------------------------------------------------------
    
    int publish_committed_trajectory();
    
    int publish_control_trajectory();
    
    int publish_rrts_tree();
    
    // --------------------------------------------------------------------------------------------
    // METHODS: ALL OTHER
    // --------------------------------------------------------------------------------------------
    
    int change_goal_region();
    
    int clear_committed_trajectory();
    
    int clear_committed_trajectory_length();
    
    bool is_near_end_committed_trajectory();
    
    bool is_far_from_committed_trajectory();
    
    bool is_robot_in_collision();
    
    bool is_root_in_goal();
    
    int planner_iteration();
    
    int semaphore_set( int semaphore_member) {
        
      if (semaphore_member == TRAJ) {
          
        is_updating_committed_trajectory = true;
        
      }
      
      else if (semaphore_member == TREE) {
          
        is_updating_rrt_tree = true;

        is_updating_committed_trajectory = true;
        
      }
      
      return 0;
      
    }
    
    int semaphore_reset( int semaphore_member) {
        
      if( semaphore_member == TRAJ )
          
        is_updating_committed_trajectory = false;
      
      else if ( semaphore_member == TREE ) {
          
        is_updating_rrt_tree = false;
        
        is_updating_committed_trajectory = false;
        
      }
      
      return 0;
      
    }
    
    int setup_rrts();
    
};

// ------------------------------------------------------------------------------------------------
// CONSTRUCTOR AND DESTRUCTOR
// ------------------------------------------------------------------------------------------------

planner_t::planner_t() {
    
  system.dynamical_system = &dubins;
  
  system.obstacle_map = &obstacle_map;
  
  planner_dt = 0.5;
  
  max_length_committed_trajectory = 25.0;

  ros::Rate wait_rate(10);
  
  while( get_robot_state() ) {
      
    cout << "Waiting for robot pose" << endl;
    
    ros::spinOnce();
    
    wait_rate.sleep();
    
  }
  
  clear_committed_trajectory();
  
  is_updating_committed_trajectory = false;
  
  is_updating_rrt_tree = false;
  
  last_found_best_path = ros::Time::now();
  
  planner_timer = 
          
          nh.createTimer( ros::Duration(planner_dt), 
          
          &planner_t::on_planner_timer, this);
  
  rrts_status_pub_timer = 
          
          nh.createTimer( ros::Duration(0.5), 
          
          &planner_t::on_rrts_status_pub_timer, this);
  
  rrts_tree_pub_timer = 
          
          nh.createTimer( ros::Duration(0.5), 
          
          &planner_t::on_rrts_tree_pub_timer, this);
  
  committed_trajectory_pub_timer = 
          
          nh.createTimer( ros::Duration(0.3), 
          
          &planner_t::on_committed_trajectory_pub_timer, this);
  
  committed_trajectory_pub = 
          
          nh.advertise < nav_msgs::Path > ( "pnc_trajectory", 2);
  
  committed_trajectory_view_pub = 
          
          nh.advertise < nav_msgs::Path > ( "pncview_trajectory", 2);
  
  rrts_tree_pub = 
          
          nh.advertise < sensor_msgs::PointCloud > ("rrts_tree", 1);
  
  rrts_vertex_pub = 
          
          nh.advertise < sensor_msgs::PointCloud > ("rrts_vertex", 1);
  
  rrts_status_pub = 
          
          nh.advertise < rrts::rrts_status > ( "rrts_status", 2);
  
  control_trajectory_pub = 
          
          nh.advertise < std_msgs::Int16MultiArray > ( "control_trajectory", 2);

  map_sub = 
          
          nh.subscribe( "local_map", 2, &planner_t::on_map, this);
  
  goal_sub = 
          
          nh.subscribe( "pnc_nextpose", 2, &planner_t::on_goal, this);
  
  is_first_goal = is_first_map = true;
  
  for ( int i = 0; i < NUM_RRTS_STATUS; i++) { rrts_status[i] = false; }
  
  ros::spin();
  
}

planner_t::~planner_t() {
    
  clear_committed_trajectory();
  
}

// ------------------------------------------------------------------------------------------------
// METHODS: FUNCTIONS USED WHEN LISTENING
// ------------------------------------------------------------------------------------------------

int planner_t::get_robot_state() {
    
  // Declares and initializes map and robot poses
    
  tf::Stamped < tf::Pose > map_pose; map_pose.setIdentity();
  
  tf::Stamped < tf::Pose > robot_pose; robot_pose.setIdentity();
  
  robot_pose.frame_id_ = "base_link";
  
  robot_pose.stamp_ = ros::Time();
  
  ros::Time current_time = ros::Time::now();
  
  bool transform_is_correct = true;
  
  try {
      
    tfl.transformPose( "map", robot_pose, map_pose);
    
  }
  
  catch( tf::LookupException& ex ) {
      
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    
    transform_is_correct = false;
    
  }
  
  catch( tf::ConnectivityException& ex ) {
      
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    
    transform_is_correct = false;
    
  }
  
  catch( tf::ExtrapolationException& ex ) {
      
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    
    transform_is_correct = false;
    
  }
  
  // Issues warning if computation of odometry pose takes too long
  
  if ( current_time.toSec() - map_pose.stamp_.toSec() > 0.4 ) {
      
    ROS_WARN( "Get robot pose transform timeout. Current time: %.4f, odom_pose stamp: %.4f, tolerance: %.4f", 
            
            current_time.toSec(), map_pose.stamp_.toSec(), 0.1);
    
    transform_is_correct = false;
    
  }
  
  if (transform_is_correct) {
      
    geometry_msgs::PoseStamped tmp;
    
    tf::poseStampedTFToMsg( map_pose, tmp);
    
    double roll = 0, pitch = 0, yaw = 0;
    
    tf::Quaternion q;
    
    tf::quaternionMsgToTF( tmp.pose.orientation, q);
    
    tf::Matrix3x3(q).getRPY( roll, pitch, yaw);
    
    float ros_robot_pose[3] = { (float) tmp.pose.position.x, 
                                (float) tmp.pose.position.y, 
                                (float) yaw };
    
    robot_state = state(ros_robot_pose);

    return 0;
  }
  
  return 1;
  
}

void planner_t::on_committed_trajectory_pub_timer( const ros::TimerEvent &e) {
    
  publish_committed_trajectory();
  
  publish_control_trajectory();
  
  return;
  
}


void planner_t::on_goal( const geometry_msgs::PoseStamped::ConstPtr ps) {
    
  double roll = 0, pitch = 0, yaw = 0;
  
  tf::Quaternion q;
  
  tf::quaternionMsgToTF( ps->pose.orientation, q);
  
  tf::Matrix3x3(q).getRPY( roll, pitch, yaw);
  
  float ros_goal_pose[3] = { (float) ps->pose.position.x, 
                             (float) ps->pose.position.y, 
                             (float) yaw };
  
  if ( get_robot_state() ) { cout << "Method get_robot_state failed" << endl; }
  
  if (is_first_goal) {
      
    is_first_goal = false;
    
    cout << "Got first goal" << endl;
    
    goal_state = state(ros_goal_pose);
    
    cout << "Goal: "; goal_state.print(cout); cout << endl;
    
    cout << "robot_state: "; robot_state.print(cout); cout << endl;
    
    if ( is_first_map == false ) {
        
      setup_rrts();
      
      if ( rrts.system->is_in_collision(goal_state) ) {
          
        cout << "Goal in collision: stopping" << endl;
        
        rrts_status[ginc] = true;
        
      }
      
      else
          
        rrts_status[ginc] = false;
      
    }
    
  }
  
  bool only_xy = true;
  
  // new goal than previous one, change sampling region
  
  if( ( !is_first_map ) && ( !is_first_goal ) ) {
      
    state new_goal_state = state(ros_goal_pose);
    
    if( goal_state.dist( new_goal_state, only_xy) > 1 ) {
        
      goal_state = new_goal_state;
      
      cout << "Got goal: "; goal_state.print(cout); cout << endl;
      
      if ( rrts.system -> is_in_collision(goal_state) ) {
          
        cout << "Goal in collision: sending collision" << endl;
        
        rrts_status[ginc] = true;
        
      }
      
      else {
          
        change_goal_region();
        
        rrts_status[ginc] = false;
        
      }
      
    }
    
  }
  
}

void planner_t::on_map( const pnc_msgs::local_map::ConstPtr lm) {
    
  obstacle_map.copy_local_map(lm);
  
  if ( is_first_map ) {
      
    is_first_map = false;
    
    cout << "Got first map" << endl;
    
    if ( is_first_goal == false ) {
        
      setup_rrts();
      
    }
    
  }
  
}

void planner_t::on_planner_timer( const ros::TimerEvent &e) {
    
  cout << endl;

  // 1. failsafes 
  if( is_robot_in_collision() ) {
      
    cout << "Robot in collision" << endl;
    
    clear_committed_trajectory();
    
    return;
    
  }
  
  // 2. is committed trajectory safe
  if( !system.is_safe_trajectory(committed_trajectory) ) {
      
    cout << "Committed_trajectory unsafe" << endl;
    
    clear_committed_trajectory();
    
    setup_rrts();
    
  }
  
  // 3. is at end of trajectory
  else if ( is_near_end_committed_trajectory() && (!is_root_in_goal()) ) {
      
    cout << "Near end of committed trajectory" << endl;
    
    should_send_new_committed_trajectory = true;
    
    planner_iteration();
    
    return;
    
  }
  
  // 4. if far from trajectory, emergency replan
  else if ( is_far_from_committed_trajectory() ) {
      
    cout << "Far from committed trajectory: emergency replan" << endl;
    
    clear_committed_trajectory();
    
    setup_rrts();
    
    planner_iteration();
    
    return;
    
  }
  
  // 5. nominal case
  else if ( (!is_first_map) && (!is_first_goal) ) { planner_iteration(); }
  
}

void planner_t::on_rrts_status_pub_timer( const ros::TimerEvent &e) {
    
  rrts::rrts_status smsg;
  
  smsg.header.stamp             = ros::Time::now();
  
  smsg.robot_in_collision       = rrts_status[rinc];
  
  smsg.goal_in_collision        = rrts_status[ginc];
  
  smsg.goal_infeasible          = rrts_status[ginf];
  
  smsg.root_in_goal             = rrts_status[ring];
  
  smsg.robot_near_root          = rrts_status[rnr];
  
  smsg.switched_root            = rrts_status[swr];
  
  smsg.too_far_from_committed   = rrts_status[tffc];
  
  rrts_status_pub.publish(smsg);
  
  rrts_status[swr] = false;
  
}

void planner_t::on_rrts_tree_pub_timer( const ros::TimerEvent& e) {}

// ------------------------------------------------------------------------------------------------
// METHODS: FUNCTIONS USED WHEN PUBLISHING
// ------------------------------------------------------------------------------------------------

int planner_t::publish_committed_trajectory() {
    
  if ( is_updating_committed_trajectory ) { return 1; }
  
  clear_committed_trajectory_length();
  
  // 1. publish to golfcar_pp
  
  nav_msgs::Path traj_msg;
  
  traj_msg.header.stamp = ros::Time::now();
  
  traj_msg.header.frame_id = "map";
  
  auto pc = committed_trajectory.controls.begin();
  
  for ( auto& ps : committed_trajectory.states ) {
      
    geometry_msgs::PoseStamped p;
    
    p.header.stamp = ros::Time::now();
    
    p.header.frame_id = "map";
    
    p.pose.position.x = ps[0];
    
    p.pose.position.y = ps[1];
    
    p.pose.position.z = (*pc)[0]; // send control as the third state
    
    p.pose.orientation.w = 1.0;
    
    traj_msg.poses.push_back(p);
    
    pc++;
    
  }
  
  committed_trajectory_pub.publish(traj_msg);
  
  // 2. publish to rviz
  
  traj_msg.poses.clear();
  
  for ( auto& ps : committed_trajectory.states ) {
      
    geometry_msgs::PoseStamped p;
    
    p.header.stamp = ros::Time::now();
    
    p.header.frame_id = "map";
    
    p.pose.position.x = ps[0];
    
    p.pose.position.y = ps[1];
    
    p.pose.position.z = 0;
    
    p.pose.orientation.w = 1.0;
    
    traj_msg.poses.push_back(p);
    
  }
  
  committed_trajectory_view_pub.publish(traj_msg);
  
  return 0;
  
}

int planner_t::publish_control_trajectory() {
    
  std_msgs::Int16MultiArray tmp;
  
  for ( auto& pc : committed_trajectory.controls) {
      
    tmp.data.push_back((int)pc[0]);
    
  }
  
  control_trajectory_pub.publish(tmp);
  
  return 0;
  
}

int planner_t::publish_rrts_tree() {
    
  if ( is_updating_rrt_tree ) {return 1;}
  
  bool check_obstacles = false;
  
  sensor_msgs::PointCloud pcv;
  
  pcv.header.stamp = ros::Time::now();
  
  pcv.header.frame_id = "map";
  
  sensor_msgs::PointCloud pct;
  
  pct.header.stamp = ros::Time::now();
  
  pct.header.frame_id = "map";
  
  if ( rrts.num_vertices ) {
      
    for ( auto& pv : rrts.list_vertices ) {
        
      vertex& v = *pv;
      
      float vs[3] = { v.state -> x[0], v.state -> x[1], 0 };
      
      // vertex
      
      geometry_msgs::Point32 p;
      
      p.x = vs[0];
      p.y = vs[1];
      p.z = vs[2];
      
      pcv.points.push_back(p);
      
      pct.points.push_back(p);

      // edge from parent
      
      if( v.parent ) {
          
        vertex& parent = *(v.parent);
        
        trajectory_t traj_from_parent;
        
        if ( system.extend_to( parent.state, v.state, 
                               check_obstacles, 
                               traj_from_parent, 
                               v.edge_from_parent -> opt_data ) ) {
            
          traj_from_parent.clear();
          
          cout << "Publish_tree: could not draw trajectory" << endl;
          
          continue;
          
        }
        
        for( auto& ps : traj_from_parent.states ) {
            
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

// ------------------------------------------------------------------------------------------------
// METHODS: ALL OTHER
// ------------------------------------------------------------------------------------------------

int planner_t::change_goal_region() {
    
  float size[3] = { 2, 2, 30 / 180.0 * M_PI};
  
  system.goal_region = region( goal_state.x, size);
  
  return 0;
  
}

int planner_t::clear_committed_trajectory() {
    
  semaphore_set(TRAJ);
  
  committed_trajectory.clear();
  
  semaphore_reset(TRAJ);
  
  cout << "Cleared committed trajectory" << endl;
  
  return 0;
  
}

int planner_t::clear_committed_trajectory_length() {
    
  if( get_robot_state() ) {
      
    cerr << __LINE__ << "Method get_robot_state failed" << endl;
    
    return 0;
    
  }
  
  if( !committed_trajectory.states.empty() ) {
      
    int num_delete = 0;
    
    bool found_length_to_delete = false;
    
    state prev_state = state( committed_trajectory.states.front() );
    
    float distance_deleted = 0;
    
    for( auto& ps : committed_trajectory.states) {
        
      state curr_state = state(ps);
      
      distance_deleted += curr_state.dist(prev_state);
      
      prev_state = curr_state;
      
      if ( curr_state.dist(robot_state) < 2 ) {
          
        found_length_to_delete = true;
        
        break;
        
      }
      
      num_delete++;
      
    }
    
    if (found_length_to_delete) {
        
      semaphore_set(TRAJ);
      
      committed_trajectory.pop_front(num_delete);
      
      committed_trajectory.total_variation -= distance_deleted;
      
      semaphore_reset(TRAJ);
      
    }
    
  }
  
  return 0;
  
}

bool planner_t::is_near_end_committed_trajectory() {
    
  bool ret = false;
  
  bool only_xy = true;
  
  if ( get_robot_state() ) { return false; }
  
  if( !committed_trajectory.states.empty() ) {
      
    state last_state = state(committed_trajectory.states.back());
    
    float t = last_state.dist(robot_state, only_xy);
    
    if ( t < 10.0 ) { ret = true; }
    
  }
  
  rrts_status[rnr] = ret;
  
  return ret;
  
}

bool planner_t::is_far_from_committed_trajectory() {
    
  if ( committed_trajectory.states.empty() ) { return false; }
  
  if ( get_robot_state() ) {
      
    cout << __LINE__ <<": Method get_robot_state failed" << endl;
    
    return false;
    
  }
  
  bool ret = true, only_xy = true;
  
  state sc;
  
  for ( auto& ps : committed_trajectory.states) {
      
    sc = state(ps);
    
    if ( sc.dist(robot_state, only_xy) < 2.0 ) {
        
      ret = false; break;
      
    }
    
  }
  
  rrts_status[tffc] = ret;
  
  return ret;
  
}

bool planner_t::is_robot_in_collision() {
    
  if ( get_robot_state() ) {
      
    cout << "Method get_robot_state failed" << endl;
    
    return false;
    
  }
  
  bool res = false;
  
  if ( (!is_first_goal) && (!is_first_map) ) {
      
      res = system.is_in_collision(robot_state);
      
  }
  
  rrts_status[rinc] = res;
  
  return res;
  
}

bool planner_t::is_root_in_goal() {
    
  bool res = system.is_in_goal( *(rrts.root->state) );
  
  rrts_status[ring] = res;
  
  return res;
  
}

int planner_t::planner_iteration() {
    
  semaphore_set(TREE);
  
  if( rrts.lazy_check_tree(committed_trajectory) ) {
      
    cout << "Root in collision" << endl;
    
    clear_committed_trajectory();
    
    setup_rrts();
    
  }
  
  semaphore_reset(TREE);
  
  // 2. no need to do anything if root is inside goal
  
  if ( is_root_in_goal() ) { return 0; }
  
  // 3. sample states, get best trajectory
  
  bool found_best_path = false;
  
  cost_t best_cost = * rrts.get_best_cost();
  
  cost_t prev_best_cost(best_cost);
  
  int num_samples = 0;
  
  ros::Time start_iteration = ros::Time::now();
  
  cout << "size: " << rrts.num_vertices << " cost: " << best_cost.val << endl;
  
  while( !found_best_path ) {
      
    // 3.a perform one iteration
      
    int ret = rrts.iteration();
    if ( !ret ) {
        
      num_samples++;
      
      publish_rrts_tree();
      
      publish_committed_trajectory();
      
    }
    
    // 3.b break if close to end of time slice
    
    ros::Duration dt = ros::Time::now() - start_iteration;
    
    if ( dt.toSec() > 0.8 * planner_dt ) { break; }
    
  }
  
  best_cost = * rrts.get_best_cost();
  
  prev_best_cost = best_cost;
  
  cout << "size: " << rrts.num_vertices << " cost: " << best_cost.val << endl;
  
  cost_t max_cost(FLT_MAX);
  
  if ( best_cost < max_cost ) {
      
    if ( ( best_cost.difference(prev_best_cost) < 0.1 ) && 
         ( best_cost.val < FLT_MAX/2 ) && 
         ( rrts.num_vertices > 5 ) ) {
        
      found_best_path = true;
      
      last_found_best_path = ros::Time::now();
      
    }
    
  }
  
  ros::Duration delta_last_found_best_path = ros::Time::now() - last_found_best_path;
  
  // 4. switch_root
  
  if (found_best_path) {
      
    rrts_status[ginf] = false;
    
    if ( should_send_new_committed_trajectory || is_first_committed_trajectory ) {
        
      semaphore_set(TREE);
      
      int res = rrts.switch_root(max_length_committed_trajectory, committed_trajectory);
      
      if ( res ) {
          
        cout << "Cannot switch_root: "<< res << endl;
        
        committed_trajectory.clear();
        
        exit(1);
        
      }
      
      else {
          
        rrts_status[swr] = true;
        
        cout << "Switched root successfully: " << committed_trajectory.total_variation << endl;
        
        cout << "size: "<< rrts.num_vertices << " cost: " << best_cost.val << endl;
        
      }
      
      semaphore_reset(TREE);
      
      publish_rrts_tree();
      
      is_first_committed_trajectory = false;
      
      should_send_new_committed_trajectory = false;
      
      return 0;
      
    }
    
  }
  
  else if ( (rrts.num_vertices > 500) || (delta_last_found_best_path.toSec() > 20) ) {
      
    last_found_best_path = ros::Time::now();
    
    rrts_status[ginf] = true;
    
    cout << "Did not find best path: reinitializing" << endl;
    
    clear_committed_trajectory();
    
    setup_rrts();
    
    return 1;
    
  }
  
  return 0;
  
}

int planner_t::setup_rrts() {
    
  // 1. set root to current state
    
  if( get_robot_state() ) { return 1; }
  
  // 2. set operating region in local coordinates
  
  float h = obstacle_map.height * obstacle_map.resolution;
  
  float w = obstacle_map.width * obstacle_map.resolution;
  
  float center[3] = { w/2, h/2, 0};
  
  float size[3] = {w, h, 2.0*M_PI};
  
  system.operating_region = region( center, size);
  
  // 3. set goal region
  
  change_goal_region();
  
  rrts.gamma = 2.5;
  
  rrts.goal_sample_freq = 0.15;
  
  const state * root_state = new state(robot_state);
  
  rrts.initialize( &system, root_state);
  
  should_send_new_committed_trajectory = false;
  
  is_first_committed_trajectory = true;
  
  return 0;
  
}

// ------------------------------------------------------------------------------------------------
// MAIN FUNCTION
// ------------------------------------------------------------------------------------------------

int main( int argc, char **argv) {
    
  ros::init( argc, argv, "rrts_node");
  
  planner_t planner;
  
  return 0;
  
}
