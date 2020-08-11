cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  pcl_conversions)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED )
find_package(opengm REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf pcl_conversions
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES loam
)

## Compile as C++14, supported in ROS Kinetic and newer
# set_property(TARGET invz_player PROPERTY CXX_STANDARD 17)
add_compile_options(-std=c++14)

add_definitions( -march=native )


add_subdirectory(src/lib)

add_executable(multiScanRegistration src/multi_scan_registration_node.cpp)
target_link_libraries(multiScanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserOdometry src/laser_odometry_node.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserMapping src/laser_mapping_node.cpp)
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(transformMaintenance src/transform_maintenance_node.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(mapStitcher src/MapStitcher.cpp)
target_link_libraries(mapStitcher ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(rosbagParser src/RosbagParser.cpp)
target_link_libraries(rosbagParser ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam ${OpenCV_LIBRARIES})

add_executable(videoInpainting src/VideoInpainting.cpp)
target_link_libraries(videoInpainting ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam ${OpenCV_LIBRARIES})

SET(MRF_LABEL_VALUE "int" CACHE STRING "selected label type for MRF library")
add_definitions(-DMRFLABELVALUE=${MRF_LABEL_VALUE})
SET(MRF_ENERGY_VALUE "double" CACHE STRING "selected energy type for MRF library")
add_definitions(-DMRFENERGYVALUE=${MRF_ENERGY_VALUE})
SET(MRF_COST_VALUE "double" CACHE STRING "selected cost type for MRF library")
add_definitions(-DMRFCOSTVALUE=${MRF_COST_VALUE})
include_directories("../code/opengm/src/external/MRF-v2.1.src-patched")
link_directories("../code/opengm/build/src/external")
link_directories("/usr/lib/external-library-mrf")
add_executable(videoInpaintingTexSynth src/VideoInpaintingTexSynth.cpp src/cnpy.cpp)
target_link_libraries(videoInpaintingTexSynth ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam ${OpenCV_LIBRARIES} ${opengm_LIBRARIES} external-library-mrf)

add_executable(videoInpaintingTexSynthFusion src/VideoInpaintingTexSynthFusion.cpp src/cnpy.cpp)
target_link_libraries(videoInpaintingTexSynthFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam ${OpenCV_LIBRARIES} ${opengm_LIBRARIES} external-library-mrf)


add_executable(loamMapper src/LoamMapper.cpp src/lib/MultiScanRegistrationRosFree.cpp src/lib/LaserOdometryRosFree.cpp src/lib/LaserMappingRosFree.cpp)
target_link_libraries(loamMapper ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam ${OpenCV_LIBRARIES})

add_executable(pointcloudProcessing src/PointCloudProcessing.cpp)
target_link_libraries(pointcloudProcessing ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam ${OpenCV_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # TODO: Download test data
  catkin_download_test_data(${PROJECT_NAME}_test_data.tar.gz
    https://dl.dropboxusercontent.com/s/y4hn486461tfmpm/velodyne_loam_test_data.tar.gz
    MD5 3d5194e6981975588b7a93caebf79ba4)
  add_custom_target(${PROJECT_NAME}_test_data
    COMMAND ${CMAKE_COMMAND} -E tar -xzf velodyne_loam_test_data.tar.gz
    DEPENDS ${PROJECT_NAME}_test_data.tar.gz)
  configure_file(tests/loam.test.in
                 ${PROJECT_BINARY_DIR}/test/loam.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/loam.test
    DEPENDENCIES
      ${PROJECT_NAME}_test_data
      multiScanRegistration
      laserOdometry
      laserMapping
      transformMaintenance)
endif()


