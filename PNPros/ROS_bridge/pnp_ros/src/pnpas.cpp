#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <pnp_ros/names.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pnpas");

  PNPActionServer pnpas;
  pnpas.start();
  ros::spin();

  return 0;
}

