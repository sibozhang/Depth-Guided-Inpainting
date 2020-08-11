#include <ros/ros.h>
#include <signal.h>
#include "loam_velodyne/LaserMapping.h"

std::unique_ptr<loam::LaserMapping> laser_mapper;

void mySigIntHandler(int sig)
{
	laser_mapper->savePoses("../Documents/global_poses.txt");
}

/** Main node entry point. */
int main(int argc, char **argv)
{
//	ros::init(argc, argv, "laserMapping");
  ros::init(argc, argv, "laserMapping", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigIntHandler);
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  //loam::LaserMapping laserMapping(0.1);
//  if (laserMapping.setup(node, privateNode)) {
//      // initialization successful
//      laserMapping.spin();
//    }

  laser_mapper.reset(new loam::LaserMapping(0.1));

  if (laser_mapper->setup(node, privateNode)) {
  //      // initialization successful
	  laser_mapper->spin();
      }



  return 0;
}
