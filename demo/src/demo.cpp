#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_cpp");
  ros::NodeHandle nh;
  std::cout << "Demo" << std::endl;
  while (ros::ok())
  {
  }
  std::cout << "End demo" << std::endl;
  return 1;
} 