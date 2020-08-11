#include <ros/ros.h>
#include <signal.h>
#include "loam_velodyne/LaserOdometry.h"

std::unique_ptr<loam::LaserOdometry> laser_odom;

void mySigIntHandler(int sig)
{
	laser_odom->saveFrameRelPoses("../Documents/rel_poses.txt");
}

/** Main node entry point. */
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "laserOdometry");
	ros::init(argc, argv, "laserOdometry", ros::init_options::NoSigintHandler);
	  signal(SIGINT, mySigIntHandler);
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

//  loam::LaserOdometry laserOdom(0.1);
//
//  if (laserOdom.setup(node, privateNode)) {
//    // initialization successful
//    laserOdom.spin();
//  }

  laser_odom.reset(new loam::LaserOdometry(0.1));
  if (laser_odom->setup(node, privateNode)) {
      // initialization successful
	  laser_odom->spin();
    }

  return 0;
}
