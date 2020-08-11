#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>
#include "../include/loam_velodyne/Twist.h"
#include "loam_velodyne/LaserMapping.h"
#include "lib/math_utils.h"
#include "../include/loam_velodyne/common.h"

//std::unique_ptr<loam::LaserMapping> laser_mapper;
//
//void mySigIntHandler(int sig)
//{
//	laser_mapper->savePoses("../Documents/global_poses.txt");
//}

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

template<typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}

/** Main node entry point. */
int main(int argc, char **argv) {

	rosbag::Bag bag;
	bag.open("../Downloads/2018-08-15-14-12-50_3.bag",
			rosbag::bagmode::Read);

	rosbag::View view(bag);

	for (rosbag::MessageInstance const m : view) {
		if (m.getTopic() == "/apollo/sensor/pandora/hesai40/PointCloud2") {
			sensor_msgs::PointCloud2::ConstPtr pr = m.instantiate<
					sensor_msgs::PointCloud2>();
			if (pr != NULL) {
				sensor_msgs::PointCloud2 mm = *pr;

				sensor_msgs::PointCloud2Iterator<float> iter_x(mm, "x");
				sensor_msgs::PointCloud2Iterator<float> iter_y(mm, "y");
				sensor_msgs::PointCloud2Iterator<float> iter_z(mm, "z");
				sensor_msgs::PointCloud2Iterator < uint8_t
						> iter_intensity(mm, "intensity");
				sensor_msgs::PointCloud2Iterator<double> iter_time(mm, "timestamp");


				pcl::PointCloud < pcl::PointXYZI > cloud;

				// Fill in the cloud data

				cloud.is_dense = false;
				int i = 0;
				//double mint=DBL_MAX, maxt=0;
				for (; iter_x != iter_x.end();
						++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
					pcl::PointXYZI p;
					p.x = *iter_x;
					p.y = *iter_y;
					p.z = *iter_z;
					p.intensity = *iter_intensity;

					//mint=*iter_time<mint?*iter_time:mint;
					//maxt=*iter_time>maxt?*iter_time:maxt;
					//std::cout<<std::setprecision(20)<<*iter_time<<std::endl;

					cloud.points.push_back(p);
				}


				//std::cout<<std::setprecision(20)<<"min time: "<<mint<<std::endl;
				//std::cout<<std::setprecision(20)<<"max time: "<<maxt<<std::endl;


				cloud.width = cloud.points.size();
				cloud.height = 1;


				std::chrono::system_clock::time_point t = loam::fromROSTime(
						pr->header.stamp);
				unsigned long long time_ms = std::chrono::duration_cast
						< std::chrono::microseconds
						> (t.time_since_epoch()).count();

				pcl::io::savePCDFileBinary(
						"../data/pandora_liang/set2/lidar/"
								+ to_string_with_precision(time_ms / 1000000.0,
										6) + ".pcd", cloud);

				//std::cout<<std::setprecision(20)<<time_ms / 1000000.0<<std::endl;
				//exit(1);

//				pcl::PLYWriter writer;
//					writer.write("../data/pandora_liang/set2/test.ply", cloud);
			}
		}

		if (m.getTopic() == "/apollo/sensor/pandora/camera/front_color") {
			sensor_msgs::Image::ConstPtr pr =
					m.instantiate<sensor_msgs::Image>();
			if (pr != NULL) {
				cv::Mat img(pr->height, pr->width, CV_8UC3);
				for (int i=0; i<img.rows; i++){
					for (int j=0; j<img.cols; j++){
						int index=(i*img.cols + j) * 3;
						img.data[index]=pr->data[index];
						img.data[index+1]=pr->data[index+1];
						img.data[index+2]=pr->data[index+2];
					}
				}


				std::chrono::system_clock::time_point t = loam::fromROSTime(
						pr->header.stamp);
				unsigned long long time_ms = std::chrono::duration_cast
						< std::chrono::microseconds
						> (t.time_since_epoch()).count();

				cv::imwrite(
						"../data/pandora_liang/set2/image/"
								+ to_string_with_precision(time_ms / 1000000.0,
										6) + ".jpg", img);

			}
		}
	}

	return 0;
}
