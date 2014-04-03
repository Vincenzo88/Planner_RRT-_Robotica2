#include "ros/ros.h"

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <ctime>

class pcl2_broadcast{

  public:
    ros::NodeHandle n;
    ros::Timer timer_;

    pcl2_broadcast(){
	timer_ = n.createTimer(ros::Duration(1), &pcl2_broadcast::fake_kinect, this);
    }

    void fake_kinect(const ros::TimerEvent&){
	std::cout << "hello" << std::endl;
    }

};

int main(int argc, char **argv){
  
  ros::init(argc, argv, "pcl2_broadcast");
  
  pcl2_broadcast P;
  
  ros::spin();
  
  return 0;
}
