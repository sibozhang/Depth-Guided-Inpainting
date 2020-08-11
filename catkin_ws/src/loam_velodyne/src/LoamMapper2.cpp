#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/io/ply_io.h>
#include <fstream>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include "../include/loam_velodyne/Twist.h"
#include "loam_velodyne/LaserMapping.h"
#include "lib/math_utils.h"
#include "../include/loam_velodyne/common.h"
#include "../include/loam_velodyne/MultiScanRegistrationRosFree.h"
#include "../include/loam_velodyne/LaserOdometryRosFree.h"
#include "../include/loam_velodyne/LaserMappingRosFree.h"

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */

struct PointXYZIS {
	PointXYZIS() {
	}
	PointXYZIS(float ix, float iy, float iz, float inten, float rp) :
			x(ix), y(iy), z(iz), intensity(inten), rotate_percentage(rp) {
	}
	float x, y, z;
	float intensity;
	float rotate_percentage;
};

template<class M>
class BagSubscriber: public message_filters::SimpleFilter<M> {
public:
	void newMessage(const boost::shared_ptr<M const> &msg) {
		signalMessage(msg);
	}
};

void ReadKittiPointCloud(const std::string &bin_file,
		std::vector<pcl::PointXYZI> &cloud) {
	// load point cloud
	std::fstream input(bin_file.c_str(), std::ios::in | std::ios::binary);
	if (!input.good()) {
		std::cerr << "Could not read file: " << bin_file << std::endl;
		exit (EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);

	int i;
	for (i = 0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *) &point.x, 3 * sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		cloud.push_back(point);

		//std::cout<<point.x<<","<<point.y<<","<<point.z<<","<<point.intensity<<std::endl;
	}
	input.close();
}

void WriteToPLY(const std::vector<PointXYZIS> &pc,
		const std::string &file_name) {

	pcl::PointCloud < pcl::PointXYZI > cloud;
	// Fill in the cloud data
	cloud.width = pc.size();
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (int i = 0; i < cloud.size(); i++) {
		cloud.points[i].x = pc[i].x;
		cloud.points[i].y = pc[i].y;
		cloud.points[i].z = pc[i].z;
		cloud.points[i].intensity = pc[i].intensity;
	}

	pcl::PLYWriter writer;
	writer.write(file_name, cloud);
}

std::map<unsigned long long, loam::Twist> ReadPose(
		const std::string &file_name) {
	std::map<unsigned long long, loam::Twist> time_pose;
	std::ifstream is(file_name);
	if (is.is_open()) {
		unsigned long long ts;
		float rx, ry, rz, x, y, z;
		while (is >> ts >> rx >> ry >> rz >> x >> y >> z) {
			loam::Twist transform;
			transform.rot_x = rx;
			transform.rot_y = ry;
			transform.rot_z = rz;
			transform.pos.x() = x;
			transform.pos.y() += y;
			transform.pos.z() += z;

			time_pose[ts] = transform;
		}
		is.close();
	}

	return time_pose;
}

std::vector<PointXYZIS> ScanRegistration(
		const std::vector<PointXYZIS> &laserCloudIn) {

	std::vector<PointXYZIS> laserCloudOut;

	size_t cloudSize = laserCloudIn.size();

	// determine scan start and end orientations
	float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
	float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
			laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
	if (endOri - startOri > 3 * M_PI) {
		endOri -= 2 * M_PI;
	} else if (endOri - startOri < M_PI) {
		endOri += 2 * M_PI;
	}

	bool halfPassed = false;
	PointXYZIS point;

	// extract valid points from input cloud
	for (int i = 0; i < cloudSize; i++) {
		point.x = laserCloudIn[i].y;
		point.y = laserCloudIn[i].z;
		point.z = laserCloudIn[i].x;
		point.intensity = laserCloudIn[i].intensity;

		// skip NaN and INF valued points
		if (!std::isfinite(point.x) || !std::isfinite(point.y)
				|| !std::isfinite(point.z)) {
			continue;
		}

		// skip zero valued points
		if (point.x * point.x + point.y * point.y + point.z * point.z
				< 0.0001) {
			continue;
		}

//	    // calculate vertical point angle and scan ID
//	    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
//	    int scanID = _scanMapper.getRingForAngle(angle);
//	    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 ){
//	      continue;
//	    }

// calculate horizontal point angle
		float ori = -std::atan2(point.x, point.z);
		if (!halfPassed) {
			if (ori < startOri - M_PI / 2) {
				ori += 2 * M_PI;
			} else if (ori > startOri + M_PI * 3 / 2) {
				ori -= 2 * M_PI;
			}

			if (ori - startOri > M_PI) {
				halfPassed = true;
			}
		} else {
			ori += 2 * M_PI;

			if (ori < endOri - M_PI * 3 / 2) {
				ori += 2 * M_PI;
			} else if (ori > endOri + M_PI / 2) {
				ori -= 2 * M_PI;
			}
		}

		point.rotate_percentage = (ori - startOri) / (endOri - startOri);

		laserCloudOut.push_back(point);
		// calculate relative scan time based on point orientation
		//float relTime = config().scanPeriod * (ori - startOri) / (endOri - startOri);
		//point.intensity = scanID + relTime;

		//projectPointToStartOfSweep(point, relTime);

		//_laserCloudScans[scanID].push_back(point);
	}

	return laserCloudOut;
}

int getdir(const std::string &dir, std::vector<std::string> &files) {
	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(dir.c_str())) == NULL) {
		std::cout << "Error(" << errno << ") opening " << dir << std::endl;
		return errno;
	}

	while ((dirp = readdir(dp)) != NULL) {
		files.push_back(std::string(dirp->d_name));
	}
	closedir(dp);
	return 0;
}

void transformToEnd(std::vector<PointXYZIS>& cloud,
		const loam::Twist &transform) {

	size_t cloudSize = cloud.size();

	for (size_t i = 0; i < cloudSize; i++) {
		PointXYZIS& point = cloud[i];

		float s = point.rotate_percentage;

		point.x -= s * transform.pos.x();
		point.y -= s * transform.pos.y();
		point.z -= s * transform.pos.z();
		//point.intensity = int(point.intensity);

		loam::Angle rx = -s * transform.rot_x.rad();
		loam::Angle ry = -s * transform.rot_y.rad();
		loam::Angle rz = -s * transform.rot_z.rad();
		rotateZXY(point, rz, rx, ry);
		rotateYXZ(point, transform.rot_y, transform.rot_x, transform.rot_z);

		point.x += transform.pos.x();
		point.y += transform.pos.y();
		point.z += transform.pos.z();
	}
}

void pointAssociateToMap(std::vector<PointXYZIS>& cloud,
		const loam::Twist &transform) {
	for (auto &po : cloud) {
		rotateZXY(po, transform.rot_z, transform.rot_x, transform.rot_y);

		po.x += transform.pos.x();
		po.y += transform.pos.y();
		po.z += transform.pos.z();
	}
}

std::vector<std::string> StringSplit(const std::string &input_str,
		const std::string &delimiter) {
	std::vector < std::string > output;

	std::string s = input_str;
	size_t pos = 0;
	std::string token;
	while ((pos = s.find(delimiter)) != std::string::npos) {
		token = s.substr(0, pos);
		output.push_back(token);
		s.erase(0, pos + delimiter.length());
	}

	output.push_back(s);

	return output;
}

/** Main node entry point. */
int main(int argc, char **argv) {


	loam_ros_free::MultiScanRegistrationRosFree multiScan;
		loam_ros_free::LaserOdometryRosFree laser_odom(0.1);
		loam_ros_free::LaserMappingRosFree laser_mapper(0.1);

//		cv::Mat K =
//				(cv::Mat_<double>(3, 3) << 1455.59396770496, 0, 607.3026371775167, 0, 1442.022869616021, 337.4925841715398, 0, 0, 1);
//		cv::Mat D =
//				(cv::Mat_<double>(5, 1) << -0.4198050958980115, -0.1198815507748308, 0.003039318819037075, 0.003714130187390825, 0.4054623390709889);
//		cv::Mat T =
//				(cv::Mat_<double>(3, 1) << 0.01594753344139972, -0.1080142986036267, -0.09012356233462081);
//		Eigen::Quaterniond q(0.01891882509940872, -0.01497411297655385,
//				0.7065780729576551, -0.7072236427134418);
//		auto rot = q.normalized().toRotationMatrix();
//		cv::Mat R = (cv::Mat_<double>(3, 3) << rot(0, 0), rot(0, 1), rot(0, 2), rot(
//				1, 0), rot(1, 1), rot(1, 2), rot(2, 0), rot(2, 1), rot(2, 2));
//		R = R.inv();
//		T = -R * T;

		if (!multiScan.setup("Hesai-Pandora")) {
			printf("MultiScanRegistrationRosFree failed to setup");
			return -1;
		}

		double min_time=1534312784.0, max_time=1534312819.0;
		std::string working_folder =std::string("../data/pandora_liang/set1");

		char output_folder[512];
		sprintf(output_folder, "%s/%llu-%llu", working_folder.c_str(), (long long)(min_time), (long long)(max_time));
		mkdir(output_folder, 0777);

//		std::map<double, std::string> img_time_to_path;
//		std::string img_folder = "../data/pandora_liang/set2/image/";
//		std::vector < std::string > img_file_list;
//		getdir(img_folder, img_file_list);
//		for (auto const &file_name : img_file_list) {
//			int str_len = file_name.length();
//			if (str_len > 10) {
//				std::string time_str = file_name.substr(0, 17);
//				double ts = std::stod(time_str);
//				if (min_time < ts && ts < max_time){
//					img_time_to_path[ts] = img_folder + file_name;
//				}
//			}
//		}

		std::string lidar_folder = working_folder + "/lidar_bg/";
		std::vector < std::string > file_list;
		getdir(lidar_folder, file_list);
		std::sort(file_list.begin(), file_list.end());
		std::reverse(file_list.begin(), file_list.end());
		std::map<double, std::string> lidar_time_to_path;


		for (auto const &file_name : file_list) {
			if (file_name.length()>10 &&  file_name[0] == 'b') {
				double lidar_start_time = std::stod(file_name.substr(3, 17));
				if (min_time < lidar_start_time && lidar_start_time < max_time){
					lidar_time_to_path[lidar_start_time]=lidar_folder + file_name;
				}
			}
		}

		for (auto const &lidar_data : lidar_time_to_path) {

				std::vector < pcl::PointXYZI > cloud;
				pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(
						new pcl::PointCloud<pcl::PointXYZI>);

				std::cout<<lidar_data.second<<std::endl;
				if (pcl::io::loadPLYFile<pcl::PointXYZI>(lidar_data.second, *tmp) == -1) //* load the file
						{
					PCL_ERROR("Couldn't read file pcd \n");
					return (-1);
				}

				cloud.resize(tmp->width * tmp->height);
				for (size_t i = 0; i < tmp->points.size(); ++i) {
					cloud[i].x = tmp->points[i].x;
					cloud[i].y = tmp->points[i].y;
					cloud[i].z = tmp->points[i].z;
					cloud[i].intensity = tmp->points[i].intensity;
				}

				loam::Time t;


				multiScan.process(cloud, t);


				laser_odom.laserCloudSharpHandler(multiScan.cornerPointsSharp());
				laser_odom.laserCloudLessSharpHandler(
						multiScan.cornerPointsLessSharp());
				laser_odom.laserCloudFlatHandler(multiScan.surfacePointsFlat());
				laser_odom.laserCloudLessFlatHandler(
						multiScan.surfacePointsLessFlat());
				laser_odom.laserCloudFullResHandler(multiScan.laserCloud());

				laser_odom.process(lidar_data.second);



				laser_mapper.laserCloudCornerLastHandler(
						*laser_odom.lastCornerCloud());
				laser_mapper.laserCloudSurfLastHandler(
						*laser_odom.lastSurfaceCloud());
				laser_mapper.laserCloudFullResHandler(*laser_odom.laserCloud());
				laser_mapper.laserOdometryHandler(
						laser_odom.transformSum().rot_x.rad(),
						laser_odom.transformSum().rot_y.rad(),
						laser_odom.transformSum().rot_z.rad(),
						laser_odom.transformSum().pos.x(),
						laser_odom.transformSum().pos.y(),
						laser_odom.transformSum().pos.z());
				laser_mapper.process(t, lidar_data.second);


		}

		pcl::PointCloud < pcl::PointXYZI > output_cloud;
		laser_mapper.downSizeFullResMap(0.2, output_cloud);
		pcl::PLYWriter writer;

		if (argc == 1)
		    writer.write(std::string(output_folder)+"/ds_map.ply", output_cloud);
		else
            writer.write(std::string(output_folder)+"/" + std::string(argv[1]), output_cloud);

		laser_odom.saveFrameRelPoses(std::string(output_folder)+"/rel_poses.txt");
		laser_mapper.saveNamePoses(std::string(output_folder)+"/global_poses.txt");



	//hesai data from Jiang He
	///////////////////////////////////////////////////////////////////////
//	loam_ros_free::MultiScanRegistrationRosFree multiScan;
//	loam_ros_free::LaserOdometryRosFree laser_odom(0.1);
//	loam_ros_free::LaserMappingRosFree laser_mapper(0.1);
//
//	cv::Mat K =
//			(cv::Mat_<double>(3, 3) << 1455.59396770496, 0, 607.3026371775167, 0, 1442.022869616021, 337.4925841715398, 0, 0, 1);
//	cv::Mat D =
//			(cv::Mat_<double>(5, 1) << -0.4198050958980115, -0.1198815507748308, 0.003039318819037075, 0.003714130187390825, 0.4054623390709889);
//	cv::Mat T =
//			(cv::Mat_<double>(3, 1) << 0.01594753344139972, -0.1080142986036267, -0.09012356233462081);
//	Eigen::Quaterniond q(0.01891882509940872, -0.01497411297655385,
//			0.7065780729576551, -0.7072236427134418);
//	auto rot = q.normalized().toRotationMatrix();
//	cv::Mat R = (cv::Mat_<double>(3, 3) << rot(0, 0), rot(0, 1), rot(0, 2), rot(
//			1, 0), rot(1, 1), rot(1, 2), rot(2, 0), rot(2, 1), rot(2, 2));
//	R = R.inv();
//	T = -R * T;
//
//	if (!multiScan.setup("Hesai-Pandora")) {
//		printf("MultiScanRegistrationRosFree failed to setup");
//		return -1;
//	}
//
//
//	std::map<double, std::string> img_time_to_path;
//	std::string img_folder = "../data/pandora/camera/";
//	std::vector < std::string > img_file_list;
//	getdir(img_folder, img_file_list);
//	for (auto const &file_name : img_file_list) {
//		int str_len = file_name.length();
//		if (str_len > 20 && file_name.at(str_len - 5) == '0') {
//			std::string time_str = file_name.substr(10, 17);
//			double ts = std::stod(time_str);
//			img_time_to_path[ts] = img_folder + file_name;
////			std::cout<<std::setprecision(20)<<ts<<std::endl;
//		}
//	}
//
//	std::string folder = "../data/pandora/lidar/";
//	std::vector < std::string > file_list;
//	getdir(folder, file_list);
//	std::sort(file_list.begin(), file_list.end());
//
//	for (auto const &file_name : file_list) {
//		if (file_name.at(0) == 'l') {
//			std::vector < pcl::PointXYZI > cloud;
//			pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(
//					new pcl::PointCloud<pcl::PointXYZI>);
//
//			if (pcl::io::loadPCDFile < pcl::PointXYZI
//					> (folder + file_name, *tmp) == -1) //* load the file
//					{
//				PCL_ERROR("Couldn't read file pcd \n");
//				return (-1);
//			}
//
//			cloud.resize(tmp->width * tmp->height);
//			for (size_t i = 0; i < tmp->points.size(); ++i) {
//				cloud[i].x = tmp->points[i].x;
//				cloud[i].y = tmp->points[i].y;
//				cloud[i].z = tmp->points[i].z;
//				cloud[i].intensity = tmp->points[i].intensity;
//			}
//
//			loam::Time t;
//
//
//			multiScan.process(cloud, t);
//
//			double lidar_start_time = std::stod(file_name.substr(6, 17));
//
//			laser_odom.laserCloudSharpHandler(multiScan.cornerPointsSharp());
//			laser_odom.laserCloudLessSharpHandler(
//					multiScan.cornerPointsLessSharp());
//			laser_odom.laserCloudFlatHandler(multiScan.surfacePointsFlat());
//			laser_odom.laserCloudLessFlatHandler(
//					multiScan.surfacePointsLessFlat());
//			laser_odom.laserCloudFullResHandler(multiScan.laserCloud());
//
//			laser_odom.process(file_name);
//
//			///////////////////////////////////////////////////////////////////
//
//			auto ptr = img_time_to_path.upper_bound(lidar_start_time);
//			if (ptr != img_time_to_path.end()
//					&& ptr->first < lidar_start_time + 0.1) {
//				pcl::PointCloud < pcl::PointXYZI > laserCloudAtImgTime;
//				pcl::copyPointCloud(
//						multiScan.laserCloud() /**laser_odom.laserCloud()*/,
//						laserCloudAtImgTime);
//				laser_odom.transformToTime(laserCloudAtImgTime,
//						ptr->first - lidar_start_time);
//
//				std::cout << "rel time: " << ptr->first - lidar_start_time
//						<< std::endl;
//
////				pcl::PLYWriter writer_tmp;
////				writer_tmp.write("../Documents/pandora_raw_frame.ply",
////							*tmp);
////				writer_tmp.write("../Documents/pandora_tran_frame.ply",
////						laserCloudAtImgTime);
////				exit(1);
//
//				cv::Mat img = cv::imread(ptr->second);
//				cv::Mat img_undist;
//				cv::undistort(img, img_undist, K, D);
//
//				for (auto &p : /*tmp->points*/laserCloudAtImgTime.points) {
//					cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.z, p.x, p.y); //order changed in MultiScanRegistration
//
//					cv::Mat uv = K * (R * v3 + T);
//					int u = uv.at<double>(0) / uv.at<double>(2);
//					int v = uv.at<double>(1) / uv.at<double>(2);
//
//					if (uv.at<double>(2) > 0 && 0 <= u && u < img_undist.cols
//							&& 0 <= v && v < img_undist.rows) {
//						cv::circle(img_undist, cv::Point(u, v), 2,
//								cv::Scalar(0, 255, 0), -1);
//					}
//				}
//
//				cv::imshow("img", img_undist);
//				cv::imwrite("../Documents/atimgtime_" + file_name + ".jpg",
//						img_undist);
//				cv::waitKey(1);
//
//
//
//				cv::Mat img_undist_new_new;
//				cv::undistort(img, img_undist_new_new, K, D);
//
//				for (auto &p : tmp->points) {
//					cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);
//
//					cv::Mat uv = K * (R * v3 + T);
//					int u = uv.at<double>(0) / uv.at<double>(2);
//					int v = uv.at<double>(1) / uv.at<double>(2);
//
//					if (uv.at<double>(2) > 0 && 0 <= u
//							&& u < img_undist_new_new.cols && 0 <= v
//							&& v < img_undist_new_new.rows) {
//						cv::circle(img_undist_new_new, cv::Point(u, v), 2,
//								cv::Scalar(0, 255, 0), -1);
//					}
//				}
//
//				cv::imwrite("../Documents/raw_" + file_name + ".jpg",
//						img_undist_new_new);
//			}
//			///////////////////////////////////////////////////////////////////
//
//			laser_mapper.laserCloudCornerLastHandler(
//					*laser_odom.lastCornerCloud());
//			laser_mapper.laserCloudSurfLastHandler(
//					*laser_odom.lastSurfaceCloud());
//			laser_mapper.laserCloudFullResHandler(*laser_odom.laserCloud());
//			laser_mapper.laserOdometryHandler(
//					laser_odom.transformSum().rot_x.rad(),
//					laser_odom.transformSum().rot_y.rad(),
//					laser_odom.transformSum().rot_z.rad(),
//					laser_odom.transformSum().pos.x(),
//					laser_odom.transformSum().pos.y(),
//					laser_odom.transformSum().pos.z());
//			laser_mapper.process(t, file_name);
//		}
//
//	}
//
//	pcl::PointCloud < pcl::PointXYZI > output_cloud;
//	laser_mapper.downSizeFullResMap(0.1, output_cloud);
//	pcl::PLYWriter writer;
//	writer.write("../Documents/ds_map_pandora_allframes_new.ply",
//			output_cloud);
//
//	laser_odom.saveFrameRelPoses("../data/pandora/rel_poses.txt");
//	laser_mapper.saveNamePoses("../data/pandora/global_poses.txt");

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//	for (int i = 0; i < 433; i++) {
//		std::cout<<i<<std::endl;
//		std::vector < pcl::PointXYZI > cloud;
//		char name[256];
//		sprintf(name,
//				"../data/2011_09_26/2011_09_26_drive_0093_sync/velodyne_points/data/%010d.bin",
//				i);
//		ReadKittiPointCloud(std::string(name), cloud);
//
//		pcl::PointCloud < pcl::PointXYZI > output_cloud;
//		output_cloud.width = cloud.size();
//		output_cloud.height = 1;
//		output_cloud.points.resize(cloud.size());
//		for (int j = 0; j < cloud.size(); j++) {
//			pcl::PointXYZI p;
//			p.x = cloud[j].x;
//			p.y = cloud[j].y;
//			p.z = cloud[j].z;
//			output_cloud.points[j] = p;
//		}
//		pcl::PLYWriter writer;
//		sprintf(name, "../Documents/%010d.ply", i);
//		writer.write(name, output_cloud);
//	}

	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////
//	if (!multiScan.setup("HDL-64E")) {
//		printf("MultiScanRegistrationRosFree failed to setup");
//		return -1;
//	}
//
//	for (int i = 0; i < 144; i+=1) {
//
//		std::vector < pcl::PointXYZI > cloud;
//		char name[256];
//		sprintf(name,
//				"../data/2011_09_26/2011_09_26_drive_0013_sync/velodyne_points/data/%010d.bin",
//				i);
//		//std::cout<<name<<std::endl;
//		ReadKittiPointCloud(std::string(name), cloud);
//
//		loam::Time t;
//
//		multiScan.process(cloud, t);
//
////		std::cout<<multiScan.cornerPointsSharp().points.size()<<std::endl;
////		std::cout<<multiScan.cornerPointsLessSharp().points.size()<<std::endl;
////		std::cout<<multiScan.surfacePointsFlat().points.size()<<std::endl;
////		std::cout<<multiScan.surfacePointsLessFlat().points.size()<<std::endl;
//
//		laser_odom.laserCloudSharpHandler(multiScan.cornerPointsSharp());
//		laser_odom.laserCloudLessSharpHandler(
//				multiScan.cornerPointsLessSharp());
//		laser_odom.laserCloudFlatHandler(multiScan.surfacePointsFlat());
//		laser_odom.laserCloudLessFlatHandler(multiScan.surfacePointsLessFlat());
//		laser_odom.laserCloudFullResHandler(multiScan.laserCloud());
//		laser_odom.process();
//
//		laser_mapper.laserCloudCornerLastHandler(*laser_odom.lastCornerCloud());
//		laser_mapper.laserCloudSurfLastHandler(*laser_odom.lastSurfaceCloud());
//		laser_mapper.laserCloudFullResHandler(*laser_odom.laserCloud());
//		laser_mapper.laserOdometryHandler(laser_odom.transformSum().rot_x.rad(),
//				laser_odom.transformSum().rot_y.rad(),
//				laser_odom.transformSum().rot_z.rad(),
//				laser_odom.transformSum().pos.x(),
//				laser_odom.transformSum().pos.y(),
//				laser_odom.transformSum().pos.z());
//		laser_mapper.process(t);
//
////		pcl::PointCloud < pcl::PointXYZI > output_cloud;
////			laser_mapper.downSizeFullResMap(0.1, output_cloud);
////			pcl::PLYWriter writer;
////			sprintf(name, "../Documents/kitti0005_%010d.ply", i);
////			writer.write(name, output_cloud);
//			std::cout<<i<<std::endl;
//
//	}
//
////	auto angle_stats=multiScan.getAngleStats();
////	for (int i=0; i<64; i++){
////	std::vector<float> arr=angle_stats[i];
////	std::sort(arr.begin(), arr.end());
////	if (arr.size()>0)
////	std::cout<<i<<" ==> "<<arr[arr.size()/2]<<std::endl;
////	}
//
//	pcl::PointCloud < pcl::PointXYZI > output_cloud;
//	laser_mapper.downSizeFullResMap(0.1, output_cloud);
//	pcl::PLYWriter writer;
//	writer.write("../Documents/ds_map_kitti_0013_allframes.ply", output_cloud);

	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////
//	if (!multiScan.setup("Hesai-Pandora")) {
//		printf("MultiScanRegistrationRosFree failed to setup");
//		return -1;
//	}
//
//	std::string folder = "../data/pandora/lidar/";
//	std::vector < std::string > file_list;
//	getdir(folder, file_list);
//	std::sort(file_list.begin(), file_list.end());
//
//	for (auto const &file_name : file_list) {
//		if (file_name.at(0) == 'l') {
//			std::vector < pcl::PointXYZI > cloud;
//			pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(
//					new pcl::PointCloud<pcl::PointXYZI>);
//
//			if (pcl::io::loadPCDFile < pcl::PointXYZI
//					> (folder + file_name, *tmp) == -1) //* load the file
//					{
//				PCL_ERROR("Couldn't read file pcd \n");
//				return (-1);
//			}
//
//			cloud.resize(tmp->width * tmp->height);
//			for (size_t i = 0; i < tmp->points.size (); ++i){
//				cloud[i].x = tmp->points[i].x;
//				cloud[i].y = tmp->points[i].y;
//				cloud[i].z = tmp->points[i].z;
//				cloud[i].intensity = tmp->points[i].intensity;
//			}
//
//
//			loam::Time t;
//
//			multiScan.process(cloud, t);
//
//
//
//			laser_odom.laserCloudSharpHandler(multiScan.cornerPointsSharp());
//			laser_odom.laserCloudLessSharpHandler(
//					multiScan.cornerPointsLessSharp());
//			laser_odom.laserCloudFlatHandler(multiScan.surfacePointsFlat());
//			laser_odom.laserCloudLessFlatHandler(multiScan.surfacePointsLessFlat());
//			laser_odom.laserCloudFullResHandler(multiScan.laserCloud());
//			laser_odom.process();
//
//			laser_mapper.laserCloudCornerLastHandler(*laser_odom.lastCornerCloud());
//			laser_mapper.laserCloudSurfLastHandler(*laser_odom.lastSurfaceCloud());
//			laser_mapper.laserCloudFullResHandler(*laser_odom.laserCloud());
//			laser_mapper.laserOdometryHandler(laser_odom.transformSum().rot_x.rad(),
//					laser_odom.transformSum().rot_y.rad(),
//					laser_odom.transformSum().rot_z.rad(),
//					laser_odom.transformSum().pos.x(),
//					laser_odom.transformSum().pos.y(),
//					laser_odom.transformSum().pos.z());
//			laser_mapper.process(t);
//		}
//
//	}
//
//		pcl::PointCloud < pcl::PointXYZI > output_cloud;
//		laser_mapper.downSizeFullResMap(0.1, output_cloud);
//		pcl::PLYWriter writer;
//		writer.write("../Documents/ds_map_pandora_allframes_new.ply", output_cloud);

	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////
//	std::vector < std::string > bag_files = StringSplit(argv[1], ",");
//	if (!multiScan.setup("HDL-64ES3")) {
//		printf("MultiScanRegistrationRosFree failed to setup");
//		return -1;
//	}
//
//	int frame_cnt = -1;
//	std::vector<PointXYZIS> stitched_map;
//
//	for (auto const &f : bag_files) {
//
//		std::cout << f << std::endl;
//		rosbag::Bag bag;
//		bag.open(f, rosbag::bagmode::Read);
//
//
//		//rosbag::View view(bag, rosbag::TopicQuery(topics));
//		rosbag::View view(bag);
//
//		// Set up fake subscribers to capture images
//		//BagSubscriber<sensor_msgs::PointCloud2> lidar_sub;
//
//		int frame_cnt = 0;
//		for (rosbag::MessageInstance const m : view) {
//			if (m.getTopic() == /*"/velodyne_points"*/ "/sensor/velodyne64/PointCloud2") {
//				frame_cnt++;
//
//				//if (frame_cnt<1400) continue;
//
//				sensor_msgs::PointCloud2::ConstPtr pr = m.instantiate<
//						sensor_msgs::PointCloud2>();
//				if (pr != NULL) {
//					sensor_msgs::PointCloud2 mm = *pr;
//
//					sensor_msgs::PointCloud2Iterator<float> iter_x(mm, "x");
//					sensor_msgs::PointCloud2Iterator<float> iter_y(mm, "y");
//					sensor_msgs::PointCloud2Iterator<float> iter_z(mm, "z");
//					sensor_msgs::PointCloud2Iterator < uint8_t
//							> iter_intensity(mm, "intensity");
//
//					int i = 0;
//					std::vector < pcl::PointXYZI > cloud;
//					for (; iter_x != iter_x.end();
//							++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
//						pcl::PointXYZI p;
//						p.x = *iter_x;
//						p.y = *iter_y;
//						p.z = *iter_z;
//						p.intensity = *iter_intensity;
//						cloud.push_back(p);
//					}
//
//					loam::Time t = loam::fromROSTime(pr->header.stamp);
//
//					multiScan.process(cloud, t);
//
//					laser_odom.laserCloudSharpHandler(
//							multiScan.cornerPointsSharp());
//					laser_odom.laserCloudLessSharpHandler(
//							multiScan.cornerPointsLessSharp());
//					laser_odom.laserCloudFlatHandler(
//							multiScan.surfacePointsFlat());
//					laser_odom.laserCloudLessFlatHandler(
//							multiScan.surfacePointsLessFlat());
//					laser_odom.laserCloudFullResHandler(multiScan.laserCloud());
//					laser_odom.process();
//
//					laser_mapper.laserCloudCornerLastHandler(
//							*laser_odom.lastCornerCloud());
//					laser_mapper.laserCloudSurfLastHandler(
//							*laser_odom.lastSurfaceCloud());
//					laser_mapper.laserCloudFullResHandler(
//							*laser_odom.laserCloud());
//					laser_mapper.laserOdometryHandler(
//							laser_odom.transformSum().rot_x.rad(),
//							laser_odom.transformSum().rot_y.rad(),
//							laser_odom.transformSum().rot_z.rad(),
//							laser_odom.transformSum().pos.x(),
//							laser_odom.transformSum().pos.y(),
//							laser_odom.transformSum().pos.z());
//					laser_mapper.process(t);
//
//				}
//
//
//				std::cout<<frame_cnt<<std::endl;
//				if (frame_cnt>2200)
//					break;
//			}
//		}
//
//		pcl::PointCloud < pcl::PointXYZI > output_cloud;
//		laser_mapper.downSizeFullResMap(0.1, output_cloud);
//		pcl::PLYWriter writer;
//		writer.write("../Documents/ds_map_neighborhood_1400-2200.ply", output_cloud);
//	}

//WriteToPLY(stitched_map, "../Documents/ds_map.ply");

	return 0;
}
