#include <iostream>
#include <ros/ros.h>
#include "FrustumProjector.h"

int main(int argc, char **argv) {
  std::cout << std::fixed;
  std::cout << std::setprecision(10);

  std::string node_name{"frustum_projector"};
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  ros::AsyncSpinner async_spinner(9);
  async_spinner.start();

  FrustumProjector frustum_projector(nh);
  std::cout << node_name + " instance has started!" << std::endl;

  ros::waitForShutdown();
  return 0;
}