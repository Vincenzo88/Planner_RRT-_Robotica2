#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
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

#include <dynamical_system.h>
#include <dubins.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <rrts.h>


#include <tf/tf.h>

using namespace std;


class SimpleGoal
{
  public:

    ros::Subscriber rviz_goal_listener;
    ros::Publisher goal_pub;
    SimpleGoal()
    {
      ros::NodeHandle n;
      rviz_goal_listener = n.subscribe("move_base_simple/goal", 2, &SimpleGoal::on_rviz_goal_listener, this);
      goal_pub = n.advertise<geometry_msgs::PoseStamped>("pnc_nextpose", 2);
    }
    void on_rviz_goal_listener(const geometry_msgs::PoseStamped::ConstPtr p)
    {
      geometry_msgs::PoseStamped pose;
      pose = *p;
      goal_pub.publish(pose);
      ROS_INFO("published pose");
      cout<<pose<<endl;
    }
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_goal");

  SimpleGoal sg;

  ros::spin();

  return 0;
}
