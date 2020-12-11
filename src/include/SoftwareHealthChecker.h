//
// Created by kaan on 15.09.2020.
//

#ifndef SRC_SOFTWAREHEALTHCHECKER_H
#define SRC_SOFTWAREHEALTHCHECKER_H

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8.h>

class SoftwareHealthChecker {
public:
  static void SoftwareHealthCheckerCallback(const ros::Publisher& publisher) {
    std_msgs::Int8 health_message;
    health_message.data = 1;
    publisher.publish(health_message);
  }

private:
};

#endif //SRC_SOFTWAREHEALTHCHECKER_H
