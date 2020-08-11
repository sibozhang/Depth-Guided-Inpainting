#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/io/ply_io.h>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include "../include/loam_velodyne/Twist.h"
#include "loam_velodyne/LaserMapping.h"
#include "lib/math_utils.h"
#include "../include/loam_velodyne/common.h"
#include "delaunay-triangulation/vector2.h"
#include "delaunay-triangulation/triangle.h"
#include "delaunay-triangulation/delaunay.h"

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
	double lidar_frame_time = 0;

	cv::Mat* img_ptr = nullptr;
	int u, v;
	double img_frame_time = 0;
};

template<class M>
class BagSubscriber: public message_filters::SimpleFilter<M> {
public:
	void newMessage(const boost::shared_ptr<M const> &msg) {
		signalMessage(msg);
	}
};

std::vector<Triangle<float> > DelaunayTriangulation(
		std::vector<cv::Point2f> const &pts) {

	std::vector<Vector2<float> > vertices;
	for (int i = 0; i < pts.size(); i++) {
		vertices.emplace_back(pts[i].x, pts[i].y, i);
	}

	Delaunay<float> triangulation;
	std::vector<Triangle<float> > triangles = triangulation.triangulate(
			vertices);

	return triangles;
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

std::map<std::string, loam::Twist> ReadPose(const std::string &file_name) {
	std::map<std::string, loam::Twist> time_pose;
	std::ifstream is(file_name);
	if (is.is_open()) {
		std::string filename;
		float rx, ry, rz, x, y, z;
		while (is >> filename >> rx >> ry >> rz >> x >> y >> z) {
			loam::Twist transform;
			transform.rot_x = rx;
			transform.rot_y = ry;
			transform.rot_z = rz;
			transform.pos.x() = x;
			transform.pos.y() += y;
			transform.pos.z() += z;

			time_pose[filename] = transform;
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
		point = laserCloudIn[i];

		point.x = laserCloudIn[i].y;
		point.y = laserCloudIn[i].z;
		point.z = laserCloudIn[i].x;

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

void transformToStart(std::vector<PointXYZIS>& cloud,
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
	}
}

void transformToStartFromEnd(std::vector<PointXYZIS>& cloud,
		const loam::Twist &transform) {

	size_t cloudSize = cloud.size();

	for (size_t i = 0; i < cloudSize; i++) {
		PointXYZIS& point = cloud[i];

		point.x -= transform.pos.x();
		point.y -= transform.pos.y();
		point.z -= transform.pos.z();
		//point.intensity = int(point.intensity);

		loam::Angle rx = -transform.rot_x.rad();
		loam::Angle ry = -transform.rot_y.rad();
		loam::Angle rz = -transform.rot_z.rad();
		rotateZXY(point, rz, rx, ry);
	}
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

void TransformToLocalFrame(std::vector<PointXYZIS>& cloud,
		const loam::Twist &local_pose) {
	for (auto &po : cloud) {
		po.x -= local_pose.pos.x();
		po.y -= local_pose.pos.y();
		po.z -= local_pose.pos.z();

		rotateYXZ(po, -local_pose.rot_y, -local_pose.rot_x, -local_pose.rot_z);
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

template<class T>
auto FindClosest(std::map<double, T> const &img_times, double const val) {
	auto low = img_times.lower_bound(val);
	if (low == img_times.end()) {
		return img_times.rbegin()->first;
	} else if (low == img_times.begin()) {
		return low->first;
	} else {
		auto prev = std::prev(low);
		if ((val - prev->first) < (low->first - val))
			return prev->first;
		else
			return low->first;
	}
}

const cv::Vec3f near_color(0, 20, 255);
const cv::Vec3f far_color(255, 0, 0);
const float near_dist = 5, far_dist = 50;

cv::Scalar ColorMapping(float const dist) {
	auto ratio_near = (dist - near_dist) / (far_dist - near_dist);
	if (ratio_near < 0)
		ratio_near = 0;
	if (ratio_near > 1)
		ratio_near = 1;

	cv::Vec3f c = near_color * (1 - ratio_near) + far_color * ratio_near;

	return cv::Scalar(c[0], c[1], c[2]);
}

cv::Mat CloudImageProjection(std::vector<PointXYZIS> &cloud, cv::Mat const &R,
		cv::Mat const &T, cv::Mat const &K, cv::Mat &img, bool draw_pts = false,
		bool record_uv = false) {

	cv::Mat output_img = img.clone();

	for (auto &p : cloud) {
		cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

		cv::Mat uv = K * (R * v3 + T);
		int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
		int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;

		if (uv.at<double>(2) > 0 && 0 <= u && u < output_img.cols && 0 <= v
				&& v < output_img.rows) {
			float p_dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);

			if (draw_pts) {
				cv::circle(output_img, cv::Point(u, v), 2, ColorMapping(p_dist),
						-1);
			}

			if (record_uv) {
				p.img_ptr = &img;
				p.u = u;
				p.v = v;
			}
		}
	}

	return output_img;
}

cv::Mat CloudImageProjectionWithMask(std::vector<PointXYZIS> &cloud,
		cv::Mat const &R, cv::Mat const &T, cv::Mat const &K, cv::Mat &img,
		bool draw_pts = false, bool record_uv = false, cv::Mat const& mask =
				cv::Mat()) {

	cv::Mat output_img = img.clone();

	if (mask.empty()) {
		for (auto &p : cloud) {
			cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

			cv::Mat uv = K * (R * v3 + T);
			int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
			int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;

			if (uv.at<double>(2) > 0 && 0 <= u && u < output_img.cols && 0 <= v
					&& v < output_img.rows) {
				float p_dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);

				if (draw_pts) {
					cv::circle(output_img, cv::Point(u, v), 2,
							ColorMapping(p_dist), -1);
				}

				if (record_uv) {
					p.img_ptr = &img;
					p.u = u;
					p.v = v;
				}
			}
		}
	} else {
		for (auto &p : cloud) {
			cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

			cv::Mat uv = K * (R * v3 + T);
			int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
			int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;

			if (uv.at<double>(2) > 0 && 0 <= u && u < output_img.cols && 0 <= v
					&& v < output_img.rows) {
				float p_dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);

				if (draw_pts) {
					cv::circle(output_img, cv::Point(u, v), 2,
							ColorMapping(p_dist), -1);
				}

				if (record_uv && mask.at < uchar > (v, u) == 0) {
					p.img_ptr = &img;
					p.u = u;
					p.v = v;
				}
			}
		}

	}

	return output_img;
}

cv::Mat CloudImageProjectionZBuffer(std::vector<PointXYZIS> &cloud,
		cv::Mat const &R, cv::Mat const &T, cv::Mat const &K, cv::Mat &img) {

	cv::Mat output_img = img.clone();
	output_img = 0;
	cv::Mat z_buffer(img.size(), CV_32F, cv::Scalar(1000000));

	for (auto &p : cloud) {
		cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

		cv::Mat uv = K * (R * v3 + T);
		int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
		int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;

		if (uv.at<double>(2) > 0 && 0 <= u && u < output_img.cols && 0 <= v
				&& v < output_img.rows) {
			float p_dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);

			if (p_dist < z_buffer.at<float>(v, u)) {
				z_buffer.at<float>(v, u) = p_dist;
				cv::circle(output_img, cv::Point(u, v), 2, ColorMapping(p_dist),
						-1);
			}

		}
	}

	output_img = output_img + img / 3;

	return output_img;
}

cv::Mat CloudImageProjectionDepthOrder(std::vector<PointXYZIS> &cloud,
		cv::Mat const &R, cv::Mat const &T, cv::Mat const &K, cv::Mat &img) {

	cv::Mat output_img = img.clone();
	output_img = 0;

	std::map<float, PointXYZIS> depth_points;

	for (auto &p : cloud) {
		cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

		cv::Mat uv = K * (R * v3 + T);
		int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
		int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;

		if (uv.at<double>(2) > 0 && 0 <= u && u < output_img.cols && 0 <= v
				&& v < output_img.rows) {
			float p_dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
			depth_points[p_dist] = p;
		}
	}

	for (auto ptr = depth_points.rbegin(); ptr != depth_points.rend(); ++ptr) {

		cv::Mat v3 =
				(cv::Mat_<double>(3, 1) << ptr->second.x, ptr->second.y, ptr->second.z);

		cv::Mat uv = K * (R * v3 + T);
		int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
		int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;

		cv::circle(output_img, cv::Point(u, v), 2, ColorMapping(ptr->first),
				-1);
	}

	output_img = output_img + img / 2;

	return output_img;
}

cv::Mat CloudImageProjectionInpaint(std::vector<PointXYZIS> &cloud,
		cv::Mat const &R, cv::Mat const &T, cv::Mat const &K, cv::Mat &img,
		const cv::Rect &roi) {

	cv::Mat output_img = img.clone();
	output_img(roi) = 0;
	cv::Mat z_buffer(output_img.size(), CV_32F, cv::Scalar(100000));

	for (auto &p : cloud) {
		cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

		cv::Mat uv = K * (R * v3 + T);
		int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
		int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;

		if (uv.at<double>(2) > 0 && roi.contains(cv::Point(u, v))) {
			float p_dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
			if (p_dist < z_buffer.at<float>(v, u) && p.img_ptr) {
				z_buffer.at<float>(v, u) = p_dist;
				output_img.at < cv::Vec3b > (v, u) = p.img_ptr->at < cv::Vec3b
						> (p.v, p.u);
				//std::cout<<output_img.at<cv::Vec3b>(v, u)<<std::endl;
			}

		}
	}

	return output_img;
}

void SparseDenseInterpolation(cv::Mat space_time_coords) {
	cv::Vec3d blank_coord(0, 0, 0);

	bool last_pix_valid;
	int dist = 0;
	cv::Mat dist_to_up_down(space_time_coords.size(), CV_32SC2, cv::Scalar(0)); //up, bottom
	cv::Mat dist_to_left_right(space_time_coords.size(), CV_32SC2,
			cv::Scalar(0)); //left, right

	for (int v = 0; v < space_time_coords.rows; v++) {
		last_pix_valid = false;
		dist = 0;
		for (int u = 0; u < space_time_coords.cols; u++) {
			if (space_time_coords.at < cv::Vec3d > (v, u) == blank_coord) {
				dist++;
				if (last_pix_valid) {
					dist_to_left_right.at < cv::Vec2i > (v, u)[0] = dist;
				}
			} else {
				last_pix_valid = true;
				dist = 0;
			}
		}

		last_pix_valid = false;
		dist = 0;
		for (int u = space_time_coords.cols - 1; u >= 0; u--) {
			if (space_time_coords.at < cv::Vec3d > (v, u) == blank_coord) {
				dist++;
				if (last_pix_valid) {
					dist_to_left_right.at < cv::Vec2i > (v, u)[1] = dist;
				}
			} else {
				last_pix_valid = true;
				dist = 0;
			}
		}
	}

	//horizontal interpolation
	for (int v = 0; v < space_time_coords.rows; v++) {
		for (int u = 0; u < space_time_coords.cols; u++) {
			int dist_to_left = dist_to_left_right.at < cv::Vec2i > (v, u)[0];
			int dist_to_right = dist_to_left_right.at < cv::Vec2i > (v, u)[1];
			if (space_time_coords.at < cv::Vec3d > (v, u) == blank_coord
					&& dist_to_left > 0 && dist_to_right > 0) {

				float left_w = (float) dist_to_right
						/ (float) (dist_to_left + dist_to_right);
				float right_w = (float) dist_to_left
						/ (float) (dist_to_left + dist_to_right);

				cv::Vec3d left_coord = space_time_coords.at < cv::Vec3d
						> (v, u - dist_to_left);
				cv::Vec3d right_coord = space_time_coords.at < cv::Vec3d
						> (v, u + dist_to_right);
				space_time_coords.at < cv::Vec3d > (v, u) = left_coord * left_w
						+ right_coord * right_w;
			}
		}
	}

	for (int u = 0; u < space_time_coords.cols; u++) {
		last_pix_valid = false;
		dist = 0;
		for (int v = 0; v < space_time_coords.rows; v++) {
			if (space_time_coords.at < cv::Vec3d > (v, u) == blank_coord) {
				dist++;
				if (last_pix_valid) {
					dist_to_up_down.at < cv::Vec2i > (v, u)[0] = dist;
				}
			} else {
				last_pix_valid = true;
				dist = 0;
			}
		}

		last_pix_valid = false;
		dist = 0;
		for (int v = space_time_coords.rows - 1; v >= 0; v--) {
			if (space_time_coords.at < cv::Vec3d > (v, u) == blank_coord) {
				dist++;
				if (last_pix_valid) {
					dist_to_up_down.at < cv::Vec2i > (v, u)[1] = dist;
				}
			} else {
				last_pix_valid = true;
				dist = 0;
			}
		}
	}

	//vertical interpolation
	for (int v = 0; v < space_time_coords.rows; v++) {
		for (int u = 0; u < space_time_coords.cols; u++) {
			int dist_to_up = dist_to_up_down.at < cv::Vec2i > (v, u)[0];
			int dist_to_down = dist_to_up_down.at < cv::Vec2i > (v, u)[1];
			if (space_time_coords.at < cv::Vec3d > (v, u) == blank_coord
					&& dist_to_up > 0 && dist_to_down > 0) {

				float up_w = (float) dist_to_down
						/ (float) (dist_to_up + dist_to_down);
				float down_w = (float) dist_to_up
						/ (float) (dist_to_up + dist_to_down);

				cv::Vec3d up_coord = space_time_coords.at < cv::Vec3d
						> (v - dist_to_up, u);
				cv::Vec3d down_coord = space_time_coords.at < cv::Vec3d
						> (v + dist_to_down, u);
				space_time_coords.at < cv::Vec3d > (v, u) = up_coord * up_w
						+ down_coord * down_w;
			}
		}
	}

}

void SpaceTimeSampling(cv::Mat img, cv::Mat space_time_coords,
		std::map<double, cv::Mat> const &time_to_images) {

	cv::Vec3b blank_pix(0, 0, 0);
	for (int v = 0; v < img.rows; v++) {
		for (int u = 0; u < img.cols; u++) {
			if (img.at < cv::Vec3b > (v, u) == blank_pix) {
				cv::Vec3d st_coord = space_time_coords.at < cv::Vec3d > (v, u);
				int x = int(st_coord[0] + 0.5);
				int y = int(st_coord[1] + 0.5);

				double inter_time = st_coord[2];
				double closest_img_time = FindClosest(time_to_images,
						inter_time);
				const cv::Mat &inter_img = time_to_images.at(closest_img_time);

				if (0 <= x && x < inter_img.cols && 0 <= y
						&& y < inter_img.rows) {
					img.at < cv::Vec3b > (v, u) = inter_img.at < cv::Vec3b
							> (y, x);
				}

			}
		}
	}
}

cv::Mat CloudImageProjectionInpaintWithMask(std::vector<PointXYZIS> &cloud,
		cv::Mat const &R, cv::Mat const &T, cv::Mat const &K, cv::Mat &img,
		const cv::Mat &mask, cv::Mat &space_time_coords) {

	cv::Mat output_img = img.clone();
	output_img.setTo(0, mask);
	cv::Mat z_buffer(output_img.size(), CV_32F, cv::Scalar(100000));
	space_time_coords = cv::Mat(output_img.size(), CV_64FC3, cv::Scalar(0));

	for (auto &p : cloud) {
		cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

		cv::Mat uv = K * (R * v3 + T);
		int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
		int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;

		if (uv.at<double>(2) > 0 && 0 <= u && u < img.cols && 0 <= v
				&& v < img.rows && mask.at < uchar > (v, u) > 0) {
			float p_dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
			if (p_dist < z_buffer.at<float>(v, u) && p.img_ptr) {
				z_buffer.at<float>(v, u) = p_dist;
				output_img.at < cv::Vec3b > (v, u) = p.img_ptr->at < cv::Vec3b
						> (p.v, p.u);
				space_time_coords.at < cv::Vec3d > (v, u) = cv::Vec3b(p.u, p.v,
						p.img_frame_time);
				//std::cout<<output_img.at<cv::Vec3b>(v, u)<<std::endl;
			}

		}
	}

	return output_img;
}

void Barycentric(cv::Point2f const &p, cv::Point2f const &a,
		cv::Point2f const &b, cv::Point2f const &c, float &u, float &v,
		float &w) {
	cv::Point2f v0 = b - a, v1 = c - a, v2 = p - a;
	float d00 = v0.dot(v0);
	float d01 = v0.dot(v1);
	float d11 = v1.dot(v1);
	float d20 = v2.dot(v0);
	float d21 = v2.dot(v1);
	float denom = d00 * d11 - d01 * d01;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
}

void TriangleBoundary(const Triangle<float> &t, float &l, float &r, float &u,
		float &b) {
	l = t.p1.x < t.p2.x ? t.p1.x : t.p2.x;
	l = t.p3.x < l ? t.p3.x : l;

	r = t.p1.x > t.p2.x ? t.p1.x : t.p2.x;
	r = t.p3.x > r ? t.p3.x : r;

	u = t.p1.y < t.p2.y ? t.p1.y : t.p2.y;
	u = t.p3.y < u ? t.p3.y : u;

	b = t.p1.y > t.p2.y ? t.p1.y : t.p2.y;
	b = t.p3.y > b ? t.p3.y : b;
}

cv::Mat ImageProjectionWithMask(std::vector<PointXYZIS> &cloud,
		cv::Mat const &R, cv::Mat const &T, cv::Mat const &K, cv::Mat &img,
		const cv::Mat &mask, const double img_time) {

	cv::Mat output_img = img.clone();
	output_img.setTo(0, mask);
	std::vector < cv::Point2f > pts;
	std::vector<cv::Point2f> tex_coords;
	cv::Mat *texture=NULL;

	for (auto &p : cloud) {

		if (p.img_frame_time != img_time || p.img_ptr == nullptr)
			continue;

		cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

		cv::Mat uv = K * (R * v3 + T);
		float x = uv.at<double>(0) / uv.at<double>(2);
		float y = uv.at<double>(1) / uv.at<double>(2);
		int u = x + 0.5;
		int v = y + 0.5;

		if (uv.at<double>(2) > 0 && 0 <= u && u < img.cols && 0 <= v
				&& v < img.rows) {
			float p_dist = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
			cv::circle(output_img, cv::Point(u, v), 2, ColorMapping(p_dist),
					-1);

			if (mask.at < uchar > (v, u) > 0){
				pts.emplace_back(x, y);
				tex_coords.emplace_back(p.u, p.v);
				texture=p.img_ptr;
			}
		}
	}

	if (texture == NULL) return output_img;


	std::vector<Triangle<float> > triags = DelaunayTriangulation(pts);


	for (auto &t : triags) {
//		cv::line(output_img, cv::Point2f(t.p1.x, t.p1.y),
//				cv::Point2f(t.p2.x, t.p2.y), cv::Scalar(255, 255, 255), 1);
//		cv::line(output_img, cv::Point2f(t.p1.x, t.p1.y),
//				cv::Point2f(t.p3.x, t.p3.y), cv::Scalar(255, 255, 255), 1);
//		cv::line(output_img, cv::Point2f(t.p3.x, t.p3.y),
//				cv::Point2f(t.p2.x, t.p2.y), cv::Scalar(255, 255, 255), 1);

		if (t.p1.idx >= 0 && t.p2.idx >= 0 && t.p3.idx >= 0) {
			float l, r, u, b;
			TriangleBoundary(t, l, r, u, b);
			int lx = int(l + 0.9999);
			int rx = int(r);
			int uy = int(u + 0.9999);
			int by = int(b);

			lx=lx>=0?lx:0;
			rx=rx<img.cols?rx:img.cols-1;
			uy=uy>=0?uy:0;
			by=by<img.rows?by:img.rows-1;


			for (int y = uy; y <= by; y++) {
				for (int x = lx; x <= rx; x++) {

					float w1, w2, w3;
					Barycentric(cv::Point2f(x, y), cv::Point2f(t.p1.x, t.p1.y),
							cv::Point2f(t.p2.x, t.p2.y),
							cv::Point2f(t.p3.x, t.p3.y), w1, w2, w3);
					if (w1>0 && w2>0 && w3>0){
						cv::Point2f tc=w1 * tex_coords[t.p1.idx] + w2 * tex_coords[t.p2.idx] + w3 * tex_coords[t.p3.idx];
						int tex_u=int(tc.x+0.5);
						int tex_v=int(tc.y+0.5);
						if (0<=tex_u && tex_u<texture->cols && 0<=tex_v && tex_v<texture->rows){
							output_img.at<cv::Vec3b>(y, x)=texture->at<cv::Vec3b>(tex_v, tex_u);
						}
					}
				}
			}
		}
	}

	return output_img;
}

void SearchAreaAssociation(const cv::Mat &mask, const std::vector<PointXYZIS> &cloud,
		cv::Mat const &R, cv::Mat const &T, cv::Mat const &K){}

/** Main node entry point. */
int main(int argc, char **argv) {

//	cv::Mat mask=cv::imread("../data/pandora_liang/set2/mask/1534313593.029519.png", cv::IMREAD_GRAYSCALE);
//	while(1){
//		cv::imshow("img", mask);
//		cv::waitKey(1);
//	}

	std::map<std::string, loam::Twist> rel_poses =
			ReadPose(
					"../data/pandora_liang/set2/1534313590-1534313598/rel_poses.txt");
	std::map<std::string, loam::Twist> glb_poses =
			ReadPose(
					"../data/pandora_liang/set2/1534313590-1534313598/global_poses.txt");

	cv::Mat K =
			(cv::Mat_<double>(3, 3) << 1477.504983559334, 0, 602.1210058502093, 0, 1464.34160012337, 342.1011293729023, 0, 0, 1);
	cv::Mat D =
			(cv::Mat_<double>(5, 1) << -0.4545587733996991, 0.0930592980092097, -0.001296660233384058, 0.0006516672999316331, 0.08511338611655252);
	cv::Mat T =
			(cv::Mat_<double>(3, 1) << 0.001476393536454348, -0.03503004858621427, -0.1188303720275153);
	Eigen::Quaterniond q(-0.007226328651958486, 0.00177378227135296,
			-0.7087850442558459, 0.705385281183203);
	auto rot = q.normalized().toRotationMatrix();
	cv::Mat R = (cv::Mat_<double>(3, 3) << rot(0, 0), rot(0, 1), rot(0, 2), rot(
			1, 0), rot(1, 1), rot(1, 2), rot(2, 0), rot(2, 1), rot(2, 2));
	R = R.inv();
	T = -R * T;

	int frame_cnt = -1;
	std::vector<PointXYZIS> stitched_map;

	std::map<double, std::string> img_time_to_path;
	std::map<double, cv::Mat> img_time_to_cvmat;
	std::string img_folder = "../data/pandora_liang/set2/image/";
	std::vector < std::string > img_file_list;
	getdir(img_folder, img_file_list);
	for (auto const &file_name : img_file_list) {
		int str_len = file_name.length();
		if (str_len > 10) {
			std::string time_str = file_name.substr(0, 17);
			double ts = std::stod(time_str);
			img_time_to_path[ts] = img_folder + file_name;
		}
	}

	std::map<double, cv::Mat> mask_time_to_cvmat;
	std::string mask_folder = "../data/pandora_liang/set2/mask/";
	std::vector < std::string > mask_file_list;
	getdir(mask_folder, mask_file_list);
	for (auto const &file_name : mask_file_list) {
		int str_len = file_name.length();
		if (str_len > 10 && file_name.substr(str_len - 3, 3) == "png") {
			std::string time_str = file_name.substr(0, 17);
			double ts = std::stod(time_str);
			mask_time_to_cvmat[ts] = imread(mask_folder + file_name,
					cv::IMREAD_GRAYSCALE);
		}
	}

	std::string lidar_folder = "../data/pandora_liang/set2/lidar_bg/";
	std::vector < std::string > file_list;
	getdir(lidar_folder, file_list);
	std::sort(file_list.begin(), file_list.end());

	std::map<double, loam::Twist> lidar_endtime_to_pose;
	std::map<double, loam::Twist> lidar_endtime_to_relpose;
	std::map<double, loam::Twist> lidar_starttime_to_pose;
	std::map<double, loam::Twist> lidar_starttime_to_relpose;
	double min_time = 1534313590.0, max_time = 1534313598.0;
	int frm_cnt = 0;
	for (auto const &file_name : file_list) {
		if (file_name.at(0) == 'b') {

			double lidar_end_time = std::stod(file_name.substr(3, 17));
			double lidar_start_time = lidar_end_time - 0.1;

			if (min_time < lidar_end_time && lidar_end_time < max_time) {

				frm_cnt++;

				auto closest_img_time = FindClosest(img_time_to_path,
						lidar_start_time);
				std::cout << "time diff: "
						<< std::fabs(closest_img_time - lidar_start_time)
						<< std::endl;
				cv::Mat img = cv::imread(img_time_to_path[closest_img_time]);
				cv::Mat img_undist;
				cv::undistort(img, img_undist, K, D);
				img_time_to_cvmat[closest_img_time] = img;		//img_undist;
				//cv::imwrite("img"+std::to_string(frm_cnt)+"_raw.jpg", img);
				//cv::imwrite("img"+std::to_string(frm_cnt)+"_und.jpg", img_undist);

				std::vector<PointXYZIS> cloud;

				pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(
						new pcl::PointCloud<pcl::PointXYZI>);
				if (pcl::io::loadPLYFile < pcl::PointXYZI
						> (lidar_folder + file_name, *tmp) == -1) //* load the file
						{
					PCL_ERROR("Couldn't read file pcd \n");
					return (-1);
				}
				for (auto const &p : tmp->points) {
					cloud.emplace_back(p.x, p.y, p.z, p.intensity, 0);
					cloud.back().lidar_frame_time = lidar_start_time;
					cloud.back().img_frame_time = closest_img_time;
				}

				auto show_img = CloudImageProjectionWithMask(cloud, R, T, K,
						img_time_to_cvmat[closest_img_time], true, true,
						mask_time_to_cvmat[closest_img_time]);
				//cv::imshow("img", show_img);
				//cv::waitKey(1);

				auto rel_ptr = rel_poses.find(lidar_folder + file_name);
				auto glb_ptr = glb_poses.find(lidar_folder + file_name);

				if (rel_ptr != rel_poses.end() && glb_ptr != glb_poses.end()) {
					++frame_cnt;
					std::cout << frame_cnt << std::endl;
					//WriteToPLY(cloud, "../Documents/motion_orig.ply");
					std::vector<PointXYZIS> filtered_cloud = ScanRegistration(
							cloud);

					/////////////////////////////////////////////////////////////
//					std::vector<PointXYZIS> tmp1 = filtered_cloud;
//					transformToStart(tmp1, rel_ptr->second);
//					WriteToPLY(tmp1,
//							"../data/pandora_liang/set2/results/"
//									+ std::to_string(lidar_start_time)
//									+ "_s1.ply");
					/////////////////////////////////////////////////////////////

					transformToEnd(filtered_cloud, rel_ptr->second);
					lidar_endtime_to_relpose[lidar_end_time] = rel_ptr->second;
					lidar_starttime_to_relpose[lidar_start_time] =
							rel_ptr->second;

					///////////////////////////////////////////////////////////////////////////
//					std::vector<PointXYZIS> tmp2 = filtered_cloud;
//					transformToStartFromEnd(tmp2, rel_ptr->second);
//					WriteToPLY(tmp2,
//							"../data/pandora_liang/set2/results/"
//									+ std::to_string(lidar_start_time)
//									+ "_s2.ply");
//
////					transformToStart(filtered_cloud, rel_ptr->second);
//					std::vector<PointXYZIS> show_cloud;
//					for (auto const &p : tmp2) {
//						PointXYZIS show_p;
//						show_p.x = p.z;
//						show_p.y = p.x;
//						show_p.z = p.y;
//						show_cloud.push_back(show_p);
//					}
//					auto proj_img = CloudImageProjection(show_cloud, R, T, K,
//							img_time_to_cvmat[closest_img_time], true, false);
//					cv::imwrite(
//							"../data/pandora_liang/set2/results/"
//									+ std::to_string(lidar_start_time)
//									+ "_s2.jpg", proj_img);
					////////////////////////////////////////////////////////////////////////////

					pointAssociateToMap(filtered_cloud, glb_ptr->second);
					lidar_endtime_to_pose[lidar_end_time] = glb_ptr->second;
					lidar_starttime_to_pose[lidar_start_time] = glb_ptr->second;
					//stitched_map.insert(stitched_map.end(), filtered_cloud.begin(), filtered_cloud.end());
					for (int j = 0; j < filtered_cloud.size(); j += 1) {
						stitched_map.push_back(filtered_cloud[j]);
					}
				}
			}
		}
	}

	WriteToPLY(stitched_map,
			"../data/pandora_liang/set2/results/stitched_map.ply");

	for (auto &img : img_time_to_cvmat) {

		//if (fabs(img.first - 1534313591.4320321083)<0.0001) continue; //this frame cause denaulay algorithm stuck
		if (img.first< 1534313593.0) continue;

		std::cout<<std::setprecision(20)<<img.first<<std::endl;

		if (mask_time_to_cvmat[img.first].empty())
			continue;

		double closest_lidar_start_time = FindClosest(lidar_starttime_to_pose,
				img.first);
		double time_diff = fabs(closest_lidar_start_time - img.first);

		if (time_diff < 0.05) {

			std::vector<PointXYZIS> whole_map = stitched_map;
			TransformToLocalFrame(whole_map,
					lidar_starttime_to_pose[closest_lidar_start_time]);
			transformToStartFromEnd(whole_map,
					lidar_starttime_to_relpose[closest_lidar_start_time]);

			std::vector<PointXYZIS> proj_map;
			for (auto &p : whole_map) {
				auto tmp_p = p;
				tmp_p.x = p.z;
				tmp_p.y = p.x;
				tmp_p.z = p.y;
				proj_map.push_back(tmp_p);
			}
			//auto show_img = CloudImageProjection(whole_map, R, T, K, img.second, true);

//			cv::Mat space_time_coords;
//			cv::Mat &mask = mask_time_to_cvmat[img.first];
//			auto show_img = CloudImageProjectionInpaintWithMask(proj_map, R, T,
//					K, img.second, mask, space_time_coords);
//
//			int l = mask.cols, r = 0, t = mask.rows, b = 0;
//			for (int v = 0; v < mask.rows; v++) {
//				for (int u = 0; u < mask.cols; u++) {
//					if (mask.at < uchar > (v, u) > 0) {
//						l = u < l ? u : l;
//						r = u > r ? u : r;
//						t = v < t ? v : t;
//						b = v > b ? v : b;
//					}
//				}
//			}
//			cv::Rect mask_rect(l, t, r - l + 1, b - t + 1);
//
//			SparseDenseInterpolation(space_time_coords(mask_rect));
//
//			SpaceTimeSampling(show_img(mask_rect), space_time_coords(mask_rect),
//					img_time_to_cvmat);

			cv::Mat &mask = mask_time_to_cvmat[img.first];

			for (auto const &img_t : img_time_to_cvmat) {
				auto show_img = ImageProjectionWithMask(proj_map, R, T, K,
						img.second, mask, img_t.first);


				cv::imshow("img", show_img);
				cv::imwrite(
						"../data/pandora_liang/set2/results/inpainted_"
								+ std::to_string(img.first) + "_"
								+ std::to_string(img_t.first) + ".jpg",
						show_img);

				cv::waitKey(1);

			}

			//////////////////////////////////////////////////////////////////
//			std::vector<PointXYZIS> tmp3;
//			for (auto const &p : whole_map) {
//				if (p.frame_time == closest_lidar_start_time) {
//					tmp3.push_back(p);
//				}
//			}
////			WriteToPLY(tmp3,
////					"../data/pandora_liang/set2/results/"
////							+ std::to_string(closest_lidar_start_time)
////							+ "_s3.ply");
//
//			std::vector<PointXYZIS> show_cloud;
//			for (auto const &p : tmp3) {
//				PointXYZIS show_p = p;
//				show_p.x = p.z;
//				show_p.y = p.x;
//				show_p.z = p.y;
//				show_cloud.push_back(show_p);
//			}
//			auto proj_img = CloudImageProjectionDepthOrder(show_cloud, R, T, K,
//					img.second);
//			cv::imwrite(
//					"../data/pandora_liang/set2/results/"
//							+ std::to_string(closest_lidar_start_time)
//							+ "_s3.jpg", proj_img);
//
//			show_cloud.clear();
//			for (auto const &p : whole_map) {
//				PointXYZIS show_p = p;
//				show_p.x = p.z;
//				show_p.y = p.x;
//				show_p.z = p.y;
//				show_cloud.push_back(show_p);
//			}
//			auto proj_img1 = CloudImageProjectionDepthOrder(show_cloud, R, T, K,
//					img.second);
//			cv::imwrite(
//					"../data/pandora_liang/set2/results/"
//							+ std::to_string(closest_lidar_start_time)
//							+ "_s4.jpg", proj_img1);

//			WriteToPLY(whole_map,
//								"../data/pandora_liang/set2/results/"
//										+ std::to_string(closest_lidar_start_time)
//										+ "_s4.ply");
			//////////////////////////////////////////////////////////////////

//			for (auto &p : whole_map) {
//				auto tmp_p = p;
//				p.x = tmp_p.z;
//				p.y = tmp_p.x;
//				p.z = tmp_p.y;
//			}
//			auto show_img = CloudImageProjectionZBuffer(whole_map, R, T, K,
//					img.second);
//			auto show_img = CloudImageProjectionInpaint(whole_map, R, T, K,
//					img.second, cv::Rect(600, 400, 200, 200));
//			cv::imshow("img", show_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/results/proj_"
//							+ std::to_string(img.first) + ".jpg", show_img);
//			cv::waitKey(1);
		}

	}

	std::cout<<"done"<<std::endl;

//	std::string folder = argv[1];
//	std::map<std::string, loam::Twist> rel_poses = ReadPose(argv[2]);
//	std::map<std::string, loam::Twist> glb_poses = ReadPose(argv[3]);
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
//	int frame_cnt = -1;
//	std::vector<PointXYZIS> stitched_map;
//
//	std::map<double, std::string> img_time_to_path;
//	std::map<double, cv::Mat> img_time_to_cvmat;
//	std::string img_folder = "../data/pandora/camera/";
//	std::vector < std::string > img_file_list;
//	getdir(img_folder, img_file_list);
//	for (auto const &file_name : img_file_list) {
//		int str_len = file_name.length();
//		if (str_len > 20 && file_name.at(str_len - 5) == '0') {
//			std::string time_str = file_name.substr(10, 17);
//			double ts = std::stod(time_str);
//			img_time_to_path[ts] = img_folder + file_name;
//		}
//	}
//
//	std::vector < std::string > file_list;
//	getdir(folder, file_list);
//	std::sort(file_list.begin(), file_list.end());
//
//	std::map<double, loam::Twist> lidar_endtime_to_pose;
//
//	for (auto const &file_name : file_list) {
//		if (file_name.at(0) == 'l') {
//
//			double lidar_start_time = std::stod(file_name.substr(6, 17));
//
//			auto closest_img_time = FindClosest(img_time_to_path,
//					lidar_start_time);
//			cv::Mat img = cv::imread(img_time_to_path[closest_img_time]);
//			cv::Mat img_undist;
//			cv::undistort(img, img_undist, K, D);
//			img_time_to_cvmat[closest_img_time]=img_undist;
//
//			std::vector<PointXYZIS> cloud;
//
//			pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(
//					new pcl::PointCloud<pcl::PointXYZI>);
//			if (pcl::io::loadPCDFile < pcl::PointXYZI
//					> (folder + "/" + file_name, *tmp) == -1) //* load the file
//					{
//				PCL_ERROR("Couldn't read file pcd \n");
//				return (-1);
//			}
//			for (auto const &p : tmp->points) {
//				cloud.emplace_back(p.x, p.y, p.z, p.intensity, 0);
//			}
//
//			auto show_img = CloudImageProjection(cloud, R, T, K, img_time_to_cvmat[closest_img_time], true, true);
//			cv::imshow("img", show_img);
//			cv::waitKey(1);
//
//			auto rel_ptr = rel_poses.find(file_name);
//			auto glb_ptr = glb_poses.find(file_name);
//
//			if (rel_ptr != rel_poses.end() && glb_ptr != glb_poses.end()) {
//				++frame_cnt;
//				std::cout << frame_cnt << std::endl;
//				//WriteToPLY(cloud, "../Documents/motion_orig.ply");
//				std::vector<PointXYZIS> filtered_cloud = ScanRegistration(
//						cloud);
//				//WriteToPLY(filtered_cloud, "../Documents/motion_before"+std::to_string(time_ms)+".ply");
//				transformToEnd(filtered_cloud, rel_ptr->second);
//				//WriteToPLY(filtered_cloud, "../Documents/motion_after"+std::to_string(time_ms)+".ply");
//
//				pointAssociateToMap(filtered_cloud, glb_ptr->second);
//				lidar_endtime_to_pose[lidar_start_time + 0.1] = glb_ptr->second;
//				//stitched_map.insert(stitched_map.end(), filtered_cloud.begin(), filtered_cloud.end());
//				for (int j = 0; j < filtered_cloud.size(); j += 1) {
//					stitched_map.push_back(filtered_cloud[j]);
//				}
//			}
//		}
//	}
//
//	for (auto &img : img_time_to_cvmat){
//		double closest_lidar_time=FindClosest(lidar_endtime_to_pose, img.first);
//		double time_diff=fabs(closest_lidar_time - img.first);
//		std::cout<<time_diff<<std::endl;
//		if (time_diff< 0.05){
//			std::vector<PointXYZIS> whole_map=stitched_map;
//			TransformToLocalFrame(whole_map, lidar_endtime_to_pose[closest_lidar_time]);
//			for (auto &p : whole_map){
//				auto tmp_p=p;
//				p.x=tmp_p.z;
//				p.y=tmp_p.x;
//				p.z=tmp_p.y;
//			}
//			//auto show_img = CloudImageProjection(whole_map, R, T, K, img.second, true);
//			auto show_img = CloudImageProjectionInpaint(whole_map, R, T, K, img.second, cv::Rect(600, 400, 200, 200));
//			cv::imshow("img", show_img);
//			cv::imwrite("../Documents/wholemap/inpainted_"+std::to_string(img.first)+".jpg", show_img);
//			cv::waitKey(1);
//		}
//
//
//	}
//
//
//
//	WriteToPLY(stitched_map, "../Documents/stitched_map_new.ply");

	return 0;
}
