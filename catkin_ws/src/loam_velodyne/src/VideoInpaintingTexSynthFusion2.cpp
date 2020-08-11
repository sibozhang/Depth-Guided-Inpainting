#include "utils.hpp"
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
#include <thread>

#include <opengm/opengm.hxx>
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/functions/potts.hxx>

#include <opengm/inference/messagepassing/messagepassing.hxx>
#include <opengm/operations/minimizer.hxx>
#include <opengm/inference/icm.hxx>




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

			std::size_t found = filename.find_last_of('/');
			std::string basefilename = filename.substr(found+1, filename.length()-1-found);


			time_pose[basefilename] = transform;
		}
		is.close();
	}

	return time_pose;
}

struct PixLabelData{
	std::vector<double> label_cost;
	std::vector<cv::Vec3b> label_color;
	std::vector<double> label_depth;
	std::vector<cv::Vec3b> left_color, right_color, top_color, bot_color;
};

//typedef double                                                                 ValueType;          // type used for values
//typedef size_t                                                                 IndexType;          // type used for indexing nodes and factors (default : size_t)
//typedef size_t                                                                 LabelType;          // type used for labels (default : size_t)
//typedef opengm::Adder                                                          OpType;             // operation used to combine terms
//typedef opengm::ExplicitFunction<ValueType,IndexType,LabelType>                ExplicitFunction;   // shortcut for explicit function
//typedef opengm::PottsFunction<ValueType,IndexType,LabelType>                   PottsFunction;      // shortcut for Potts function
//typedef opengm::meta::TypeListGenerator<ExplicitFunction,PottsFunction>::type  FunctionTypeList;   // list of all function the model can use (this trick avoids virtual methods) - here only one
//typedef opengm::DiscreteSpace<IndexType, LabelType>                            SpaceType;          // type used to define the feasible state-space
//typedef opengm::GraphicalModel<ValueType,OpType,FunctionTypeList,SpaceType>    Model;              // type of the model
//typedef Model::FunctionIdentifier                                              FunctionIdentifier; // type of the function identifier

const double color_weight = 100.0;
const double depth_weight = 100.0;

void GlobalOpt(std::vector<PixLabelData> &pll, int width, int height, int num_label, std::vector<int> &res){


	std::vector<ValueType> numbersOfLabels(pll.size(),num_label);
	Model gm(SpaceType(numbersOfLabels.begin(), numbersOfLabels.end()));

	// Add functions
	LabelType nl = num_label;
//	std::vector < FunctionIdentifier > unaryids(256);
	ExplicitFunction f1(&nl, &nl + 1);


	for (int i = 0; i < pll.size(); ++i) {
		for (int j=0; j<num_label; j++){
			f1(j) = pll[i].label_cost[j];
		}
//		FunctionIdentifier fid = gm.addFunction(f1);
		gm.addFactor(gm.addFunction(f1), &i, &i + 1);
	}

//	PottsFunction pottsfunction(numLabel, numLabel, 0.0, lambda);
//	FunctionIdentifier pottsid = gm.addFunction(pottsfunction);
//
//	// Add factor
//	for (IndexType n = 0; n < height; ++n) {
//		for (IndexType m = 0; m < width; ++m) {
//			IndexType var = n + m * height;
//			gm.addFactor(unaryids[image(n, m)], &var, &var + 1);
//		}
//	}
//	{
//		IndexType vars[] = { 0, 1 };
//		for (IndexType n = 0; n < height; ++n) {
//			for (IndexType m = 0; m < width; ++m) {
//				vars[0] = n + m * height;
//				if (n + 1 < height) { //check for right neighbor
//					vars[1] = (n + 1) + (m) * height;
//					OPENGM_ASSERT(vars[0] < vars[1]); // variables need to be ordered!
//					gm.addFactor(pottsid, vars, vars + 2);
//				}
//				if (m + 1 < width) { //check for lower neighbor
//					vars[1] = (n) + (m + 1) * height;
//					OPENGM_ASSERT(vars[0] < vars[1]); // variables need to be ordered!
//					gm.addFactor(pottsid, vars, vars + 2);
//				}
//			}
//		}
//	}

	LabelType l2l[] = {num_label, num_label};
	ExplicitFunction f2(l2l, l2l + 2);
	IndexType vars[] = { 0, 1 };
	for (IndexType n = 0; n < height; ++n) {
		for (IndexType m = 0; m < width; ++m) {
			vars[0] = n * width + m;
			if (n + 1 < height) { //check for lower neighbor
				vars[1] = (n + 1) * width + m;
				OPENGM_ASSERT(vars[0] < vars[1]); // variables need to be ordered!
				auto &p1 = pll[vars[0]];
				auto &p2 = pll[vars[1]];
				for (LabelType i1=0; i1<num_label; i1++){
					for (LabelType i2=0; i2<num_label; i2++){
						double cost = cv::norm(p1.label_color[i1], p2.top_color[i2], CV_L2) * color_weight+
								cv::norm(p1.bot_color[i1], p2.label_color[i2], CV_L2) * color_weight
								+std::fabs(p1.label_depth[i1] - p2.label_depth[i2]) * depth_weight;
						f2(i1, i2) = cost;
					}
				}
				gm.addFactor(gm.addFunction(f2), vars, vars + 2);
			}
			if (m + 1 < width) { //check for right neighbor
				vars[1] = n * width + (m + 1);
				OPENGM_ASSERT(vars[0] < vars[1]); // variables need to be ordered!
				auto &p1 = pll[vars[0]];
				auto &p2 = pll[vars[1]];
				for (LabelType i1=0; i1<num_label; i1++){
					for (LabelType i2=0; i2<num_label; i2++){
						double cost = cv::norm(p1.label_color[i1], p2.left_color[i2], CV_L2) * color_weight +
								cv::norm(p1.right_color[i1], p2.label_color[i2], CV_L2) * color_weight +
								std::fabs(p1.label_depth[i1] - p2.label_depth[i2]) * depth_weight;
						f2(i1, i2) = cost;
					}
				}
				gm.addFactor(gm.addFunction(f2), vars, vars + 2);
			}
		}
	}


	// Inference
	typedef opengm::BeliefPropagationUpdateRules<Model, opengm::Minimizer> UpdateRules;
	typedef opengm::MessagePassing<Model, opengm::Minimizer, UpdateRules,
			opengm::MaxDistance> LBP;
	typedef opengm::ICM<Model, opengm::Minimizer> ICM;

	LBP::Parameter parameter(2, 0.01, 0.8);
	LBP lbp(gm, parameter);

	LBP::VerboseVisitorType visitor;
	lbp.infer(visitor);

	std::vector < LabelType > labeling(gm.numberOfVariables());
	lbp.arg(labeling);

	res.resize(width * height);
	for(IndexType n=0; n<height;++n){
	      for(IndexType m=0; m<width;++m){
	         IndexType var = n*width + m ;
	         res[var] = labeling[var];
	      }
	}
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

cv::Mat transformToStartFromEndMat(const loam::Twist &transform) {
	//R(X - T)
	cv::Mat T =
			(cv::Mat_<double>(3, 1) << transform.pos.x(), transform.pos.y(), transform.pos.z());

	loam::Angle ay = -transform.rot_y;
	loam::Angle ax = -transform.rot_x;
	loam::Angle az = -transform.rot_z;

	cv::Mat rotY =
			(cv::Mat_<double>(3, 3) << ay.cos(), 0, ay.sin(), 0, 1, 0, -ay.sin(), 0, ay.cos());

	cv::Mat rotX =
			(cv::Mat_<double>(3, 3) << 1, 0, 0, 0, ax.cos(), -ax.sin(), 0, ax.sin(), ax.cos());

	cv::Mat rotZ =
			(cv::Mat_<double>(3, 3) << az.cos(), -az.sin(), 0, az.sin(), az.cos(), 0, 0, 0, 1);

	cv::Mat R = rotY * rotX * rotZ;

	cv::Mat M4x4 = cv::Mat::eye(4, 4, CV_64F);
	R.copyTo(M4x4(cv::Rect(0, 0, 3, 3)));
	M4x4(cv::Rect(3, 0, 1, 3)) = -R * T;

	return M4x4;
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

cv::Mat TransformToLocalFrameMat(const loam::Twist &local_pose) {

	//R(X - T)
	cv::Mat T =
			(cv::Mat_<double>(3, 1) << local_pose.pos.x(), local_pose.pos.y(), local_pose.pos.z());

	loam::Angle ay = -local_pose.rot_y;
	loam::Angle ax = -local_pose.rot_x;
	loam::Angle az = -local_pose.rot_z;

	cv::Mat rotY =
			(cv::Mat_<double>(3, 3) << ay.cos(), 0, ay.sin(), 0, 1, 0, -ay.sin(), 0, ay.cos());

	cv::Mat rotX =
			(cv::Mat_<double>(3, 3) << 1, 0, 0, 0, ax.cos(), -ax.sin(), 0, ax.sin(), ax.cos());

	cv::Mat rotZ =
			(cv::Mat_<double>(3, 3) << az.cos(), -az.sin(), 0, az.sin(), az.cos(), 0, 0, 0, 1);

	cv::Mat R = rotZ * rotX * rotY;

	cv::Mat M4x4 = cv::Mat::eye(4, 4, CV_64F);
	R.copyTo(M4x4(cv::Rect(0, 0, 3, 3)));
	M4x4(cv::Rect(3, 0, 1, 3)) = -R * T;

	return M4x4;

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
					p.to_img_distance = fabs(uv.at<double>(2));
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
					p.to_img_distance = fabs(uv.at<double>(2));
				} else if (mask.at < uchar > (v, u) > 0){ //set point to 0 in order to remove it from stitched pointcloud later
				    p.x = 0;
				    p.y = 0;
				    p.z = 0;
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

	output_img = output_img/2 + img / 2;

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

//cv::Mat ImageLocalDepth(const std::vector<PointXYZIS> &cloud,
//		const cv::Mat &proj_mat, const cv::Mat &img) {
//
//	cv::Mat dmap(img.size(), CV_32F, cv::Scalar(FLT_MAX));
//	cv::Mat z_buffer(img.size(), CV_32F, cv::Scalar(1000000));
//
//	for (auto &p : cloud) {
//		cv::Mat v3 = (cv::Mat_<double>(4, 1) << p.x, p.y, p.z, 1);
//
//		cv::Mat uv = proj_mat * v3;
//		int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
//		int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;
//
//		if (uv.at<double>(2) > 0 && 0 <= u && u < img.cols && 0 <= v
//				&& v < img.rows) {
//			float p_dist = uv.at<double>(2);
//
//			if (p_dist < z_buffer.at<float>(v, u)) {
//				z_buffer.at<float>(v, u) = p_dist;
//				dmap.at<float>(v, u) = p_dist;
//			}
//		}
//	}
//
//	cv::Mat_<float> valid_pts(0, 2);
//	for (int x = 0; x < dmap.cols; x++) {
//		for (int y = 0; y < dmap.rows; y++) {
//			if (dmap.at<float>(y, x) < FLT_MAX) {
//				cv::Mat row = (cv::Mat_<float>(1, 2) << x, y);
//				valid_pts.push_back(row);
//			}
//		}
//	}
//	cv::flann::Index flann_index(valid_pts, cv::flann::KDTreeIndexParams(),
//			cvflann::FLANN_DIST_EUCLIDEAN);
//
//	cv::Mat indices, dists;
//	int K = 10;
//	for (int x = 0; x < dmap.cols; x++) {
//		for (int y = 0; y < dmap.rows; y++) {
//			if (dmap.at<float>(y, x) == FLT_MAX) {
//				cv::Mat query = (cv::Mat_<float>(1, 2) << x, y);
//
//				flann_index.knnSearch(query, indices, dists, K);
//
//				float ave_d = 0;
//				float sum_dist = 0;
//				for (int i = 0; i < K; i++) {
//					int ind = indices.at<int>(i);
//					float dist = sqrt(dists.at<float>(i));
//					int sx = valid_pts.at<float>(ind, 0);
//					int sy = valid_pts.at<float>(ind, 1);
//					ave_d += dmap.at<float>(sy, sx) * (1.f / dist);
//					sum_dist += 1.f / dist;
//				}
//
//				ave_d /= sum_dist;
//				dmap.at<float>(y, x) = ave_d;
//			}
//		}
//	}
//
//	cv::medianBlur(dmap, dmap, 5);
//
//	return dmap;
//}

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

void UpdateInpainting(cv::Mat &img, const std::vector<cv::Point2i> &filling_pix,
		const std::vector<cv::Mat *> &img_ptrs,
		const std::vector<double> &img_scales,
		const std::vector<cv::Rect> &img_rois) {

	cv::Mat color_sum(img.size(), CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat color_cnt(img.size(), CV_32S, cv::Scalar(0));
	int cnt = 0;
	cv::Rect img_rect(0, 0, img.cols, img.rows);
	for (int i = 0; i < filling_pix.size(); i++) {

		cv::Point2i pix = filling_pix[i];

		double img_scaling = img_scales[i];
		cv::Mat scaled_img;
		cv::resize(*(img_ptrs[i]), scaled_img, cv::Size(), img_scaling,
				img_scaling);

		cv::Mat sub_img = scaled_img(img_rois[i]);

		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
		temp_img_rect = temp_img_rect & img_rect;
		cv::Mat temp_img = img(temp_img_rect);

		cv::Mat result;
		cv::matchTemplate(sub_img, temp_img, result, CV_TM_SQDIFF);

		double minVal, maxVal;
		cv::Point minLoc, maxLoc;
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

		cv::Rect matching_rect(minLoc.x, minLoc.y, temp_img_rect.width,
				temp_img_rect.height);
		cv::Mat matching_patch;
		sub_img(matching_rect).convertTo(matching_patch, CV_64F);
		color_sum(temp_img_rect) = color_sum(temp_img_rect) + matching_patch;
		color_cnt(temp_img_rect) = color_cnt(temp_img_rect) + 1;
	}

	for (int i = 0; i < filling_pix.size(); i++) {
		cv::Point2i pix = filling_pix[i];
		cv::Vec3d c = color_sum.at < cv::Vec3d > (pix.y, pix.x);
		int cnt = color_cnt.at<int>(pix.y, pix.x);
		if (cnt > 0)
			img.at < cv::Vec3b > (pix.y, pix.x) = cv::Vec3b(c[0] / cnt,
					c[1] / cnt, c[2] / cnt);
	}
}

void UpdateInpaintingVotingWidth(cv::Mat &img,
		const std::vector<cv::Point2i> &filling_pix,
		const std::vector<cv::Mat *> &img_ptrs,
		const std::vector<double> &img_scales,
		const std::vector<cv::Rect> &img_rois, const int vw) {

	cv::Mat color_sum(img.size(), CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat color_cnt(img.size(), CV_32S, cv::Scalar(0));
	int cnt = 0;
	cv::Rect img_rect(0, 0, img.cols, img.rows);
	for (int i = 0; i < filling_pix.size(); i++) {

		cv::Point2i pix = filling_pix[i];

		double img_scaling = img_scales[i];
		cv::Mat scaled_img;
		cv::resize(*(img_ptrs[i]), scaled_img, cv::Size(), img_scaling,
				img_scaling);

		cv::Mat sub_img = scaled_img(img_rois[i]);

		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
		temp_img_rect = temp_img_rect & img_rect;
		cv::Mat temp_img = img(temp_img_rect);

		cv::Rect voting_rect(pix.x - vw, pix.y - vw, vw * 2 + 1, vw * 2 + 1);
		voting_rect = voting_rect & img_rect;
		cv::Rect rel_voting_rect(voting_rect.x - temp_img_rect.x,
				voting_rect.y - temp_img_rect.y, voting_rect.width,
				voting_rect.height);

		cv::Mat result;
		cv::matchTemplate(sub_img, temp_img, result, CV_TM_SQDIFF);

		double minVal, maxVal;
		cv::Point minLoc, maxLoc;
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

		cv::Rect matching_rect(minLoc.x, minLoc.y, temp_img_rect.width,
				temp_img_rect.height);
		cv::Mat matching_patch;
		sub_img(matching_rect).convertTo(matching_patch, CV_64F);
		color_sum(voting_rect) = color_sum(voting_rect)
				+ matching_patch(rel_voting_rect);
		color_cnt(voting_rect) = color_cnt(voting_rect) + 1;

//		if (pix.x == 590 && pix.y == 439) {
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313648-1534313657_results/scaled_img.jpg",
//					scaled_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313648-1534313657_results/sub_img.jpg",
//					sub_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313648-1534313657_results/temp_img.jpg",
//					temp_img);
//
//			cv::rectangle(sub_img,
//					cv::Rect(minLoc.x, minLoc.y, temp_img.cols, temp_img.rows),
//					cv::Scalar(0, 255, 0));
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313648-1534313657_results/det_img.jpg",
//					sub_img);
//			exit(0);
//		}
	}

	for (int i = 0; i < filling_pix.size(); i++) {
		cv::Point2i pix = filling_pix[i];
		cv::Vec3d c = color_sum.at < cv::Vec3d > (pix.y, pix.x);
		int cnt = color_cnt.at<int>(pix.y, pix.x);
		if (cnt > 0)
			img.at < cv::Vec3b > (pix.y, pix.x) = cv::Vec3b(c[0] / cnt,
					c[1] / cnt, c[2] / cnt);
	}
}

void UpdateInpaintingVotingWidthDepth(cv::Mat &img, cv::Mat &dmap,
		const std::vector<cv::Point2i> &filling_pix,
		const std::vector<cv::Mat *> &img_ptrs,
		const std::vector<double> &img_scales,
		const std::vector<cv::Rect> &img_rois, const int vw) {

	cv::Mat color_sum(img.size(), CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat color_cnt(img.size(), CV_32S, cv::Scalar(0));
	int cnt = 0;
	cv::Rect img_rect(0, 0, img.cols, img.rows);
	for (int i = 0; i < filling_pix.size(); i++) {

		cv::Point2i pix = filling_pix[i];

		double img_scaling = img_scales[i];
		cv::Mat scaled_img, scaled_dmap;
		cv::resize(*(img_ptrs[i]), scaled_img, cv::Size(), img_scaling,
				img_scaling);
		cv::resize(img2depth[img_ptrs[i]], scaled_dmap, cv::Size(), img_scaling,
				img_scaling);

		cv::Mat sub_img = scaled_img(img_rois[i]);
		cv::Mat sub_dmap = scaled_dmap(img_rois[i]).clone();

		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
		temp_img_rect = temp_img_rect & img_rect;
		cv::Mat temp_img = img(temp_img_rect);
		cv::Mat temp_dmap = dmap(temp_img_rect).clone();

		cv::Rect voting_rect(pix.x - vw, pix.y - vw, vw * 2 + 1, vw * 2 + 1);
		voting_rect = voting_rect & img_rect;
		cv::Rect rel_voting_rect(voting_rect.x - temp_img_rect.x,
				voting_rect.y - temp_img_rect.y, voting_rect.width,
				voting_rect.height);

		cv::Mat result;
//		cv::matchTemplate(sub_img, temp_img, result, CV_TM_SQDIFF);

		cv::Mat sub_img_float, temp_img_float;
		sub_img.convertTo(sub_img_float, CV_32F);
		temp_img.convertTo(temp_img_float, CV_32F);
		int from_to[] = { 0, 2 };
		sub_dmap = sub_dmap * 0.3f;
		temp_dmap = temp_dmap * 0.3f;

		cv::mixChannels(&sub_dmap, 1, &sub_img_float, 1, from_to, 1);
		cv::mixChannels(&temp_dmap, 1, &temp_img_float, 1, from_to, 1);
		cv::matchTemplate(sub_img_float, temp_img_float, result, CV_TM_SQDIFF);

		double minVal, maxVal;
		cv::Point minLoc, maxLoc;
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

		cv::Rect matching_rect(minLoc.x, minLoc.y, temp_img_rect.width,
				temp_img_rect.height);
		cv::Mat matching_patch;
		sub_img(matching_rect).convertTo(matching_patch, CV_64F);
		color_sum(voting_rect) = color_sum(voting_rect)
				+ matching_patch(rel_voting_rect);
		color_cnt(voting_rect) = color_cnt(voting_rect) + 1;

//		if (pix.x == 590 && pix.y == 439) {
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313648-1534313657_results/scaled_img.jpg",
//					scaled_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313648-1534313657_results/sub_img.jpg",
//					sub_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313648-1534313657_results/temp_img.jpg",
//					temp_img);
//
//			cv::rectangle(sub_img,
//					cv::Rect(minLoc.x, minLoc.y, temp_img.cols, temp_img.rows),
//					cv::Scalar(0, 255, 0));
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313648-1534313657_results/det_img.jpg",
//					sub_img);
//			exit(0);
//		}
	}

	for (int i = 0; i < filling_pix.size(); i++) {
		cv::Point2i pix = filling_pix[i];
		cv::Vec3d c = color_sum.at < cv::Vec3d > (pix.y, pix.x);
		int cnt = color_cnt.at<int>(pix.y, pix.x);
		if (cnt > 0)
			img.at < cv::Vec3b > (pix.y, pix.x) = cv::Vec3b(c[0] / cnt,
					c[1] / cnt, c[2] / cnt);
	}
}

//K need to be shifted to sub image
cv::Mat WarpedImageByDepth(const cv::Mat &input_img, const cv::Mat &local_depth,
		const cv::Mat inv_K, const cv::Mat &inv_R, const cv::Mat &T,
		const cv::Mat &proj_mat, cv::Mat &mask, cv::Mat &xymap, int cx, int cy) {

	cv::Mat output_img = input_img.clone();
	output_img = cv::Scalar(0, 0, 0, 0);
	mask = cv::Mat(input_img.size(), CV_32FC4, cv::Scalar(0, 0, 0, 0));
	xymap = cv::Mat(input_img.size(), CV_32FC2, cv::Scalar(-1, -1));

//	int cx = input_img.cols/2, cy = input_img.rows/2;
	float d = local_depth.at<float>(cy, cx);
	cv::Mat v3 = (cv::Mat_<double>(3, 1) << cx * d, cy *d , d);
	cv::Mat p3 = inv_R * (inv_K * v3 - T);
	cv::Mat v4 =(cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(1), p3.at<double>(2), 1);
	cv::Mat uv = proj_mat * v4;
	float dcx = uv.at<double>(0)/uv.at<double>(2), dcy = uv.at<double>(1)/uv.at<double>(2);
	float shift_x = cx - dcx, shift_y = cy - dcy;

	double upsample_scaling = 2.0;
	cv::Mat input_img_upsampled, local_depth_upsampled;
	cv::resize(input_img, input_img_upsampled, cv::Size(), upsample_scaling,
			upsample_scaling);
	cv::resize(local_depth, local_depth_upsampled, cv::Size(), upsample_scaling,
			upsample_scaling);


	for (int y = 0; y < input_img_upsampled.rows; y++) {
		for (int x = 0; x < input_img_upsampled.cols; x++) {
			float d = local_depth_upsampled.at<float>(y, x);
			cv::Mat v3 = (cv::Mat_<double>(3, 1) << x / upsample_scaling * d, y
					/ upsample_scaling * d, d);
			cv::Mat p3 = inv_R * (inv_K * v3 - T);
			cv::Mat v4 = (cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<
					double>(1), p3.at<double>(2), 1);
			cv::Mat uv = proj_mat * v4;
			float dcx = uv.at<double>(0)/uv.at<double>(2), dcy = uv.at<double>(1)/uv.at<double>(2);

			dcx += shift_x;
			dcy += shift_y;


			int ix = dcx + 0.5f;
			int iy = dcy + 0.5f;

			if (0<=ix && ix<output_img.cols && 0<=iy && iy<output_img.rows){
				output_img.at<cv::Vec4f>(iy, ix) = input_img_upsampled.at<cv::Vec4f>(y, x);
				mask.at<cv::Vec4f>(iy, ix) = cv::Vec4f(1, 1, 1, 1);
				xymap.at<cv::Vec2f>(int(y/upsample_scaling+0.01f), int(x/upsample_scaling+0.01f)) = cv::Vec2f(ix, iy);
			}
		}
	}

	return output_img;
}

void UpdateInpaintingVotingWidthDepthWarpedPatch(cv::Mat &img, cv::Mat &dmap,
		cv::Mat &local_depth, const std::vector<cv::Point2i> &filling_pix,
		const std::vector<cv::Mat *> &img_ptrs,
		const std::vector<double> &img_scales,
		const std::vector<cv::Rect> &img_rois,
		const std::vector<cv::Mat> &img_proj_mats, const int vw,
		const cv::Mat &K, const cv::Mat &R, const cv::Mat &T, cv::Rect &filling_rect) {

	cv::Mat color_sum(img.size(), CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat color_cnt(img.size(), CV_32S, cv::Scalar(0));
	int cnt = 0;
	cv::Rect img_rect(0, 0, img.cols, img.rows);

	cv::Mat inv_R = R.inv();
	std::vector<PixLabelData> pll(filling_pix.size());
	const int ws = 5;
	for (int i = 0; i < filling_pix.size(); i++) {

		cv::Point2i pix = filling_pix[i];

//		if (!(pix.x == 324 && pix.y == 344))
//			continue;

		double img_scaling = img_scales[i];
		cv::Mat scaled_img, scaled_dmap;
		cv::resize(*(img_ptrs[i]), scaled_img, cv::Size(), img_scaling,
				img_scaling);
		cv::resize(img2depth[img_ptrs[i]], scaled_dmap, cv::Size(), img_scaling,
				img_scaling);

		cv::Mat sub_img = scaled_img(img_rois[i]);
		cv::Mat sub_dmap = scaled_dmap(img_rois[i]).clone();

		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
		temp_img_rect = temp_img_rect & img_rect;
		cv::Mat temp_img = img(temp_img_rect);
		cv::Mat temp_dmap = dmap(temp_img_rect).clone();
		cv::Mat temp_local_depth = local_depth(temp_img_rect).clone();
		int local_center_x = pix.x - temp_img_rect.x;
		int local_center_y = pix.y - temp_img_rect.y;

		cv::Mat rect_offset=(cv::Mat_<double>(3, 3)<<1, 0, -temp_img_rect.x, 0, 1, -temp_img_rect.y, 0, 0, 1);
		cv::Mat shifted_K = rect_offset * K;
		cv::Mat inv_K = shifted_K.inv();

		cv::Rect voting_rect(pix.x - vw, pix.y - vw, vw * 2 + 1, vw * 2 + 1);
		voting_rect = voting_rect & img_rect;
		cv::Rect rel_voting_rect(voting_rect.x - temp_img_rect.x,
				voting_rect.y - temp_img_rect.y, voting_rect.width,
				voting_rect.height);

		cv::Mat result;
//		cv::matchTemplate(sub_img, temp_img, result, CV_TM_SQDIFF);

		cv::Mat sub_img_float, temp_img_float;
		cv::Mat sub_imgdepth_float(sub_img.size(), CV_32FC4), temp_imgdepth_float(temp_img.size(), CV_32FC4);
		sub_img.convertTo(sub_img_float, CV_32F);
		temp_img.convertTo(temp_img_float, CV_32F);
		int from_to[] = { 0, 0, 1, 1, 2, 2, 3, 3 };
		sub_dmap = sub_dmap * 3.f;
		temp_dmap = temp_dmap * 3.f;

		cv::Mat in1[] = { sub_img_float, sub_dmap };
		cv::Mat in2[] ={temp_img_float, temp_dmap};
//		cv::mixChannels(&sub_dmap, 1, &sub_img_float, 1, from_to, 1);
//		cv::mixChannels(&temp_dmap, 1, &temp_img_float, 1, from_to, 1);
		cv::mixChannels(in1, 2, &sub_imgdepth_float, 1, from_to, 4);
		cv::mixChannels(in2, 2, &temp_imgdepth_float, 1, from_to, 4);

		cv::Mat scaled_proj_mat = img_proj_mats[i].clone();
		scaled_proj_mat.row(0) = scaled_proj_mat.row(0) * img_scaling;
		scaled_proj_mat.row(1) = scaled_proj_mat.row(1) * img_scaling;
		cv::Mat warped_mask, xymap;

		cv::Mat warped_templ=WarpedImageByDepth(temp_imgdepth_float, temp_local_depth,
				inv_K, inv_R, T,
				scaled_proj_mat, warped_mask, xymap, local_center_x, local_center_y);


//		cv::matchTemplate(sub_img_float, temp_img_float, result, CV_TM_SQDIFF);
		cv::matchTemplate(sub_imgdepth_float, warped_templ, result, CV_TM_SQDIFF, warped_mask);

//		cv::Mat tmp1(sub_imgdepth_float.size(), CV_32FC3);
//		cv::Mat tmp2(warped_templ.size(), CV_32FC3);
//		cv::mixChannels(&sub_imgdepth_float, 1, &tmp1, 1, from_to, 3);
//		cv::mixChannels(&warped_templ, 1, &tmp2, 1, from_to, 3);
//		cv::Mat tmp3, tmp4;


		double minVal, maxVal;
		cv::Point minLoc, maxLoc;
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

		cv::Mat warped_mask_1c(warped_mask.size(), CV_32F);
		cv::mixChannels(&warped_mask, 1, &warped_mask_1c, 1, from_to, 1);
		int npix = cv::countNonZero(warped_mask_1c);
		double ave_minv;
		if (npix > 0)
			ave_minv=minVal / npix;
		else{
			ave_minv=100000000.0;
			npix = 1;
		}

//		std::cout<<pix.x<<", "<<pix.y<<std::endl;
//		std::cout<<ave_minv<<std::endl;

//		if (ave_minv< 5000.0){
//			cv::Rect matching_rect(minLoc.x, minLoc.y, temp_img_rect.width,
//					temp_img_rect.height);
//			cv::Mat matching_patch;
//			sub_img(matching_rect).convertTo(matching_patch, CV_64F);
//			color_sum(voting_rect) = color_sum(voting_rect)
//					+ matching_patch(rel_voting_rect);
//			color_cnt(voting_rect) = color_cnt(voting_rect) + 1;
//	    }


		cv::Rect matching_rect(minLoc.x, minLoc.y, temp_img_rect.width,
				temp_img_rect.height);
		cv::Mat inv_warped_patch, inv_warped_mask;
		cv::remap(sub_img(matching_rect), inv_warped_patch, xymap, cv::Mat(),
				cv::INTER_NEAREST);
		cv::remap(cv::Mat(temp_img_rect.size(), CV_32S, cv::Scalar(1)),
				inv_warped_mask, xymap, cv::Mat(), cv::INTER_NEAREST);
//		if (ave_minv < 50000.0) {
//			cv::Mat matching_patch;
//			inv_warped_patch.convertTo(matching_patch, CV_64F);
//			color_sum(voting_rect) = color_sum(voting_rect)
//					+ matching_patch(rel_voting_rect);
//			color_cnt(voting_rect) = color_cnt(voting_rect)
//					+ inv_warped_mask(rel_voting_rect);
//		}

		int lx = pix.x - filling_rect.x;
		int ly = pix.y - filling_rect.y;
		PixLabelData p;

		float left_pix_offset_x = -1, left_pix_offset_y =0;
		if (local_center_x - 1>=0 && xymap.at<cv::Vec2f>(local_center_y, local_center_x)[0]>0 &&
				xymap.at<cv::Vec2f>(local_center_y, local_center_x-1)[0]>0 ){
			left_pix_offset_x = xymap.at<cv::Vec2f>(local_center_y, local_center_x-1)[0] - xymap.at<cv::Vec2f>(local_center_y, local_center_x)[0];
			left_pix_offset_y = xymap.at<cv::Vec2f>(local_center_y, local_center_x-1)[1] - xymap.at<cv::Vec2f>(local_center_y, local_center_x)[1];
		}
		float right_pix_offset_x = 1, right_pix_offset_y =0;
		if (local_center_x + 1 < xymap.cols && xymap.at<cv::Vec2f>(local_center_y, local_center_x)[0]>0 &&
				xymap.at<cv::Vec2f>(local_center_y, local_center_x+1)[0]>0 ){
			right_pix_offset_x = xymap.at<cv::Vec2f>(local_center_y, local_center_x+1)[0] - xymap.at<cv::Vec2f>(local_center_y, local_center_x)[0];
			right_pix_offset_y = xymap.at<cv::Vec2f>(local_center_y, local_center_x+1)[1] - xymap.at<cv::Vec2f>(local_center_y, local_center_x)[1];
		}
		float top_pix_offset_x = 0, top_pix_offset_y =-1;
		if (local_center_y - 1 >= 0 && xymap.at<cv::Vec2f>(local_center_y, local_center_x)[0]>0 &&
				xymap.at<cv::Vec2f>(local_center_y-1, local_center_x)[0]>0 ){
			top_pix_offset_x = xymap.at<cv::Vec2f>(local_center_y-1, local_center_x)[0] - xymap.at<cv::Vec2f>(local_center_y, local_center_x)[0];
			top_pix_offset_y = xymap.at<cv::Vec2f>(local_center_y-1, local_center_x)[1] - xymap.at<cv::Vec2f>(local_center_y, local_center_x)[1];
		}
		float bot_pix_offset_x = 0, bot_pix_offset_y = 1;
		if (local_center_y + 1 < xymap.rows && xymap.at<cv::Vec2f>(local_center_y, local_center_x)[0]>0 &&
				xymap.at<cv::Vec2f>(local_center_y+1, local_center_x)[0]>0 ){
			bot_pix_offset_x = xymap.at<cv::Vec2f>(local_center_y+1, local_center_x)[0] - xymap.at<cv::Vec2f>(local_center_y, local_center_x)[0];
			bot_pix_offset_y = xymap.at<cv::Vec2f>(local_center_y+1, local_center_x)[1] - xymap.at<cv::Vec2f>(local_center_y, local_center_x)[1];
		}

		const float color_2_proj_weight = 1000000.0;
		for (int yy=minLoc.y-ws; yy<=minLoc.y+ws; yy++){
			for (int xx=minLoc.x-ws; xx<=minLoc.x+ws; xx++){
				if (0<=xx && xx<result.cols && 0<=yy && yy<result.rows){
					int matching_x = xx + local_center_x;
					int matching_y = yy + local_center_y;

//					p.label_cost.push_back(sqrt(result.at<float>(yy, xx)/npix));
					float diff2proj_cost = cv::norm(sub_img.at<cv::Vec3b>(matching_y, matching_x), img.at<cv::Vec3b>(pix.y, pix.x), CV_L2) * color_2_proj_weight;
					p.label_cost.push_back(diff2proj_cost);

					p.label_color.push_back(sub_img.at<cv::Vec3b>(matching_y, matching_x));
					p.label_depth.push_back((double)(sub_dmap.at<float>(matching_y, matching_x)));

					int u = matching_x + left_pix_offset_x, v = matching_y + left_pix_offset_y;
					if (0<=u && u<sub_img.cols && 0<=v && v<sub_img.rows){
						p.left_color.push_back(sub_img.at<cv::Vec3b>(v, u));
					}else{
						p.left_color.push_back(sub_img.at<cv::Vec3b>(matching_y, matching_x));
					}

					u = matching_x + right_pix_offset_x, v = matching_y + right_pix_offset_y;
					if (0<=u && u<sub_img.cols && 0<=v && v<sub_img.rows){
						p.right_color.push_back(sub_img.at<cv::Vec3b>(v, u));
					}else{
						p.right_color.push_back(sub_img.at<cv::Vec3b>(matching_y, matching_x));
					}

					u = matching_x + top_pix_offset_x, v = matching_y + top_pix_offset_y;
					if (0<=u && u<sub_img.cols && 0<=v && v<sub_img.rows){
						p.top_color.push_back(sub_img.at<cv::Vec3b>(v, u));
					}else{
						p.top_color.push_back(sub_img.at<cv::Vec3b>(matching_y, matching_x));
					}

					u = matching_x + bot_pix_offset_x, v = matching_y + bot_pix_offset_y;
					if (0<=u && u<sub_img.cols && 0<=v && v<sub_img.rows){
						p.bot_color.push_back(sub_img.at<cv::Vec3b>(v, u));
					}else{
						p.bot_color.push_back(sub_img.at<cv::Vec3b>(matching_y, matching_x));
					}
				}
				else{
					p.label_cost.push_back(100000000000000.0);
					p.label_color.push_back(cv::Vec3b(0, 0, 0));
					p.label_depth.push_back(0);
					p.left_color.push_back(cv::Vec3b(0, 0, 0));
					p.right_color.push_back(cv::Vec3b(0, 0, 0));
					p.top_color.push_back(cv::Vec3b(0, 0, 0));
					p.bot_color.push_back(cv::Vec3b(0, 0, 0));
				}
			}
		}

		const double boundary_consistent_weight = 100000.0;
		if (lx == 0){
			if (pix.x > 0){
				cv::Vec3b boundary_color = img.at<cv::Vec3b>(pix.y, pix.x-1);
				for (int c=0; c<p.label_cost.size(); c++){
					cv::Vec3b lc = p.label_color[c];
					double c_dist = cv::norm(lc, boundary_color, CV_L2);
					p.label_cost[c]+=c_dist * boundary_consistent_weight;
				}
			}
		}
		if (lx == filling_rect.width-1){
			if (pix.x < img.cols -1 ){
				cv::Vec3b boundary_color = img.at<cv::Vec3b>(pix.y, pix.x+1);
				for (int c=0; c<p.label_cost.size(); c++){
					cv::Vec3b lc = p.label_color[c];
					double c_dist = cv::norm(lc, boundary_color, CV_L2);
					if (pix.x == 353 && pix.y == 395) {
						std::cout<<c<<", "<<lc<<", "<<boundary_color<<" ==> "<<c_dist<<"   left color: "<<p.left_color[c]<<std::endl;
					}
					p.label_cost[c]+=c_dist * boundary_consistent_weight;
				}
			}
		}
		if (ly == 0){
			if (pix.y > 0 ){
				cv::Vec3b boundary_color = img.at<cv::Vec3b>(pix.y-1, pix.x);
				for (int c=0; c<p.label_cost.size(); c++){
					cv::Vec3b lc = p.label_color[c];
					double c_dist = cv::norm(lc, boundary_color, CV_L2);
					p.label_cost[c]+=c_dist * boundary_consistent_weight;
				}
			}
		}
		if (ly == filling_rect.height-1){
			if (pix.y < img.rows-1 ){
				cv::Vec3b boundary_color = img.at<cv::Vec3b>(pix.y+1, pix.x);
				for (int c=0; c<p.label_cost.size(); c++){
					cv::Vec3b lc = p.label_color[c];
					double c_dist = cv::norm(lc, boundary_color, CV_L2);
					p.label_cost[c]+=c_dist * boundary_consistent_weight;
				}
			}
		}

		pll[ly * filling_rect.width + lx] = p;


//		if (pix.x == 312 && pix.y == 374) {
//		if (pix.x == 276 && pix.y == 386) {
//		if (pix.x == 282 && pix.y == 343) {
//		if (pix.x == 351 && pix.y == 395) {
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/scaled_img.jpg",
//					scaled_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/sub_img.jpg",
//					sub_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/temp_img.jpg",
//					temp_img);
//
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/inv_warped_patch.jpg",
//					inv_warped_patch);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/inv_warped_mask.jpg",
//					inv_warped_mask * 255);
//
//
//			cv::Mat disp_img;
//			cv::Mat c3(warped_templ.size(), CV_32FC3);
//			cv::mixChannels(&warped_templ, 1, &c3, 1, from_to, 3);
//			warped_templ.convertTo(disp_img, CV_8U);
//			cv::imwrite("../data/pandora_liang/set2/1534313570-1534313581_results/temp_img_2c_warped.jpg",
//								disp_img);
//
//			cv::imwrite("../data/pandora_liang/set2/1534313570-1534313581_results/warped_mask.jpg",
//					warped_mask*255);
//
//			cv::rectangle(sub_img,
//					cv::Rect(minLoc.x, minLoc.y, temp_img.cols, temp_img.rows),
//					cv::Scalar(0, 255, 0));
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/det_img.jpg",
//					sub_img);
//			exit(0);
//		}
	}

	std::vector<int> res;
	GlobalOpt(pll, filling_rect.width, filling_rect.height, (ws*2+1)*(ws*2+1), res);
	std::cout<<"done !!!"<<std::endl;

	char outname[512];
	sprintf(outname, "../data/pandora_liang/set2/1534313570-1534313581_results/img_c%f_d%f_input.jpg", color_weight, depth_weight);
	cv::imwrite(outname, img);
	int center_idx = (ws*2+1)*(ws*2+1)/2;
	for (int r = 0; r < filling_rect.height; r++) {
			for (int c = 0; c < filling_rect.width; c++) {
				int img_x = c + filling_rect.x;
				int img_y = r + filling_rect.y;
				int pix_idx = r * filling_rect.width + c;
				img.at < cv::Vec3b > (img_y, img_x) =
						pll[pix_idx].label_color[center_idx];
			}
		}

	sprintf(outname, "../data/pandora_liang/set2/1534313570-1534313581_results/img_c%f_d%f_patchmatch.jpg", color_weight, depth_weight);
	cv::imwrite(outname, img);


	for (int r = 0; r < filling_rect.height; r++) {
		for (int c = 0; c < filling_rect.width; c++) {
			int img_x = c + filling_rect.x;
			int img_y = r + filling_rect.y;
			int pix_idx = r * filling_rect.width + c;
			int label_idx = res[pix_idx];
			img.at < cv::Vec3b > (img_y, img_x) =
					pll[pix_idx].label_color[label_idx];
			if (img_x == 353 && img_y == 395){
				std::cout<<353<<", "<<label_idx<<", "<<pll[pix_idx].label_color[label_idx]<<std::endl;
			}
			if (img_x == 352 && img_y == 395){
				std::cout<<352<<", "<<label_idx<<", "<<pll[pix_idx].label_color[label_idx]<<std::endl;
			}
			if (img_x == 351 && img_y == 395){
				std::cout<<351<<", "<<label_idx<<", "<<pll[pix_idx].label_color[label_idx]<<std::endl;
			}
		}
	}

	sprintf(outname, "../data/pandora_liang/set2/1534313570-1534313581_results/img_c%f_d%f_bf.jpg", color_weight, depth_weight);
	cv::imwrite(outname, img);
	exit(1);

//	for (int i = 0; i < filling_pix.size(); i++) {
//		cv::Point2i pix = filling_pix[i];
//		cv::Vec3d c = color_sum.at < cv::Vec3d > (pix.y, pix.x);
//		int cnt = color_cnt.at<int>(pix.y, pix.x);
//		if (cnt > 0)
//			img.at < cv::Vec3b > (pix.y, pix.x) = cv::Vec3b(c[0] / cnt,
//					c[1] / cnt, c[2] / cnt);
//	}
}

cv::Mat WarpedImageBy3DPoints(const cv::Mat &input_img, const cv::Mat &proj_pts,
		const cv::Mat &proj_mat, cv::Mat &mask) {
	cv::Mat inter_proj_pts = proj_pts.clone();

	/////////////////////////////////////////////////////////////////
//	cv::Mat mask_before = cv::Mat::zeros(inter_proj_pts.size(), CV_8U);
//	for (int c = 0; c < inter_proj_pts.cols; c++) {
//		for (int r = 0; r < inter_proj_pts.rows; r++) {
//			if (inter_proj_pts.at < cv::Vec4f > (r, c)[0] > 0) {
//				mask_before.at < uchar > (r, c) = 255;
//			}
//		}
//	}
//	cv::imwrite(
//			"../data/pandora_liang/set2/1534313648-1534313657_results/mask_before.png",
//			mask_before);
	/////////////////////////////////////////////////////////////////

	for (int c = 0; c < inter_proj_pts.cols; c++) {
		std::vector<float> to_up_dist(inter_proj_pts.rows, -1);
		std::vector < cv::Vec3f > up_pts(inter_proj_pts.rows);

		bool found_pt = false;
		cv::Vec3f last_pt;
		float dist = 0;
		for (int r = 0; r < inter_proj_pts.rows; r++) {
			cv::Vec4f pt = inter_proj_pts.at < cv::Vec4f > (r, c);
			//valid point
			if (pt[0] > 0) {
				found_pt = true;
				last_pt[0] = pt[1];
				last_pt[1] = pt[2];
				last_pt[2] = pt[3];
				dist = 0;
			} else {
				dist += 1.f;
				if (found_pt) {
					to_up_dist[r] = dist;
					up_pts[r] = last_pt;
				}
			}
		}

		found_pt = false;
		dist = 0;
		for (int r = inter_proj_pts.rows - 1; r >= 0; r--) {
			cv::Vec4f pt = inter_proj_pts.at < cv::Vec4f > (r, c);
			if (pt[0] > 0) {
				found_pt = true;
				last_pt[0] = pt[1];
				last_pt[1] = pt[2];
				last_pt[2] = pt[3];
				dist = 0;
			} else {
				dist += 1.f;
				if (found_pt && to_up_dist[r] > 0) {
					cv::Vec3f inter_pt = (up_pts[r] * dist
							+ last_pt * to_up_dist[r]) / (dist + to_up_dist[r]);
					cv::Vec4f new_pt(1.f, inter_pt[0], inter_pt[1],
							inter_pt[2]);
					inter_proj_pts.at < cv::Vec4f > (r, c) = new_pt;
				}
			}
		}
	}

	for (int r = 0; r < inter_proj_pts.rows; r++) {
		std::vector<float> to_left_dist(inter_proj_pts.cols, -1);
		std::vector < cv::Vec3f > left_pts(inter_proj_pts.cols);

		bool found_pt = false;
		cv::Vec3f last_pt;
		float dist = 0;
		for (int c = 0; c < inter_proj_pts.cols; c++) {
			cv::Vec4f pt = inter_proj_pts.at < cv::Vec4f > (r, c);
			//valid point
			if (pt[0] > 0) {
				found_pt = true;
				last_pt[0] = pt[1];
				last_pt[1] = pt[2];
				last_pt[2] = pt[3];
				dist = 0;
			} else {
				dist += 1.f;
				if (found_pt) {
					to_left_dist[c] = dist;
					left_pts[c] = last_pt;
				}
			}
		}

		found_pt = false;
		dist = 0;
		for (int c = inter_proj_pts.cols - 1; c >= 0; c--) {
			cv::Vec4f pt = inter_proj_pts.at < cv::Vec4f > (r, c);
			if (pt[0] > 0) {
				found_pt = true;
				last_pt[0] = pt[1];
				last_pt[1] = pt[2];
				last_pt[2] = pt[3];
				dist = 0;
			} else {
				dist += 1.f;
				if (found_pt && to_left_dist[c] > 0) {
					cv::Vec3f inter_pt = (left_pts[c] * dist
							+ last_pt * to_left_dist[c])
							/ (dist + to_left_dist[c]);
					cv::Vec4f new_pt(1.f, inter_pt[0], inter_pt[1],
							inter_pt[2]);
					inter_proj_pts.at < cv::Vec4f > (r, c) = new_pt;
				}
			}
		}
	}

	/////////////////////////////////////////////////////////////////
//	cv::Mat mask_after = cv::Mat::zeros(inter_proj_pts.size(), CV_8U);
//	for (int c = 0; c < inter_proj_pts.cols; c++) {
//		for (int r = 0; r < inter_proj_pts.rows; r++) {
//			if (inter_proj_pts.at < cv::Vec4f > (r, c)[0] > 0) {
//				mask_after.at < uchar > (r, c) = 255;
//			}
//		}
//	}
//	cv::imwrite(
//			"../data/pandora_liang/set2/1534313648-1534313657_results/mask_after.png",
//			mask_after);
//
//	exit(1);
	/////////////////////////////////////////////////////////////////
	int cx = inter_proj_pts.cols / 2, cy = inter_proj_pts.rows / 2;
	int anchor_x = 0, anchor_y = 0;
	int max_dist = 10000000;
	for (int ax = 0; ax < inter_proj_pts.cols; ax++) {
		for (int ay = 0; ay < inter_proj_pts.rows; ay++) {
			if (inter_proj_pts.at < cv::Vec4f > (ay, ax)[0] > 0) {
				int dist = abs(cx - ax) + abs(cy - ay);
				if (dist < max_dist) {
					max_dist = dist;
					anchor_x = ax;
					anchor_y = ay;
				}
			}
		}
	}
//	int cx = inter_proj_pts.cols / 2, cy = inter_proj_pts.rows / 2;
//	int dy=0, dx =0;
//	for (int dy = 0; dy <= cy; dy++) {
//		for (int dx = 0; dx <= std::min(cx, dy); dx++) {
//			if (inter_proj_pts.at < cv::Vec4f > (cy + dy, cx + dx)[0] > 0) {
//				anchor_x = cx + dx;
//				anchor_y = cy + dy;
//				break;
//			}
//
//			if (inter_proj_pts.at < cv::Vec4f > (cy + dy, cx - dx)[0] > 0) {
//				anchor_x = cx + dx;
//				anchor_y = cy + dy;
//				break;
//			}
//
//			if (inter_proj_pts.at < cv::Vec4f > (cy - dy, cx + dx)[0] > 0) {
//				anchor_x = cx + dx;
//				anchor_y = cy + dy;
//				break;
//			}
//
//			if (inter_proj_pts.at < cv::Vec4f > (cy - dy, cx - dx)[0] > 0) {
//				anchor_x = cx + dx;
//				anchor_y = cy + dy;
//				break;
//			}
//		}
//	}

//	std::cout << inter_proj_pts.size() << std::endl;
//	std::cout << "anchor x, y: " << anchor_x << ", " << anchor_y << std::endl;

	cv::Vec4f anchor_pt = inter_proj_pts.at < cv::Vec4f > (anchor_y, anchor_x);
	cv::Mat v4 =
			(cv::Mat_<double>(4, 1) << anchor_pt[1], anchor_pt[2], anchor_pt[3], 1);
	cv::Mat v3 = proj_mat * v4;
	float proj_x = v3.at<double>(0) / v3.at<double>(2);
	float proj_y = v3.at<double>(1) / v3.at<double>(2);

	float shift_x = anchor_x - proj_x, shift_y = anchor_y - proj_y;

//	std::cout << "shift_x_y: " << shift_x << ", " << shift_y << std::endl;

	cv::Mat output_img = input_img.clone();
//	cv::Mat xmap(input_img.size(), CV_32F, cv::Scalar(-1));
//	cv::Mat ymap(input_img.size(), CV_32F, cv::Scalar(-1));
	mask = cv::Mat::zeros(input_img.size(), CV_8UC3);
	output_img = cv::Scalar(0, 0, 0);
	for (int r = 0; r < inter_proj_pts.rows; r++) {
		for (int c = 0; c < inter_proj_pts.cols; c++) {
			cv::Vec4f pt = inter_proj_pts.at < cv::Vec4f > (r, c);
			if (pt[0] > 0) {
				cv::Mat v4 = (cv::Mat_<double>(4, 1) << pt[1], pt[2], pt[3], 1);
				cv::Mat v3 = proj_mat * v4;
				float proj_x = v3.at<double>(0) / v3.at<double>(2);
				float proj_y = v3.at<double>(1) / v3.at<double>(2);

				int u = int(proj_x + shift_x + 0.5f);
				int v = int(proj_y + shift_y + 0.5f);

//				std::cout<<"c, r:"<<c<<", "<<r<<std::endl;
//				std::cout<<v4<<std::endl;
//				std::cout<<proj_mat<<std::endl;
//				std::cout<<u<<", "<<v<<std::endl;

				if (0 <= u && u < output_img.cols && 0 <= v
						&& v < output_img.rows) {
					output_img.at < cv::Vec3b > (v, u) = input_img.at
							< cv::Vec3b > (r, c);
					mask.at < cv::Vec3b > (v, u) = cv::Vec3b(255, 255, 255);
				}
			}
		}
	}

//	cv::imwrite(
//			"../data/pandora_liang/set2/1534313648-1534313657_results/input_img.png",
//			input_img);
//	cv::imwrite(
//			"../data/pandora_liang/set2/1534313648-1534313657_results/output_img.png",
//			output_img);
//	exit(1);

	return output_img;
}

//void UpdateInpaintingByDenseSampling(cv::Mat &img, cv::Mat &dmap,
//		cv::Mat &local_depth, const std::vector<cv::Point2i> &filling_pix,
//		const std::vector<cv::Mat *> &img_ptrs,
//		const std::vector<double> &img_scales,
//		const std::vector<cv::Rect> &img_rois,
//		const std::vector<cv::Mat> &img_proj_mats, const int vw,
//		const cv::Mat &K, const cv::Mat &R, const cv::Mat &T, cv::Rect &filling_rect) {
//
//	cv::Mat color_sum(img.size(), CV_64FC3, cv::Scalar(0, 0, 0));
//	cv::Mat color_cnt(img.size(), CV_32S, cv::Scalar(0));
//	int cnt = 0;
//	cv::Rect img_rect(0, 0, img.cols, img.rows);
//
//	cv::Mat inv_R = R.inv();
//	cv::Mat inv_K = K.inv();
//	std::vector<PixLabelData> pll(filling_pix.size());
//	const int ws = 5;
//	for (int i = 0; i < filling_pix.size(); i++) {
//
//		cv::Point2i pix = filling_pix[i];
//
//		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
//				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
//		temp_img_rect = temp_img_rect & img_rect;
//		cv::Mat temp_img = img(temp_img_rect);
//		cv::Mat temp_dmap = dmap(temp_img_rect).clone();
//		cv::Mat temp_local_depth = local_depth(temp_img_rect).clone();
//		int local_center_x = pix.x - temp_img_rect.x;
//		int local_center_y = pix.y - temp_img_rect.y;
//
//
//		cv::Mat proj_mat = img_proj_mats[i].clone();
//		float d = local_depth.at<float>(cy, cx);
//		cv::Mat v3 = (cv::Mat_<double>(3, 1) << pix.x * d, pix.y *d , d);
//		cv::Mat p3 = inv_R * (inv_K * v3 - T);
//		cv::Mat v4 =(cv::Mat_<double>(4, 1) << p3.at<double>(0), p3.at<double>(1), p3.at<double>(2), 1);
//		cv::Mat uv = proj_mat * v4;
//		float dx = uv.at<double>(0)/uv.at<double>(2), dy = uv.at<double>(1)/uv.at<double>(2);
//
//	}
//
//}

cv::Mat WarpedImageBy3DPointsHomo(const cv::Mat &input_img,
		const cv::Mat &proj_pts, const cv::Mat &proj_mat, cv::Mat &mask,
		cv::Mat &homo) {

	std::vector < cv::Point2f > src_pts;
	std::vector < cv::Point2f > dst_pts;

	for (int r = 0; r < proj_pts.rows; r++) {
		for (int c = 0; c < proj_pts.cols; c++) {
			cv::Vec4f pt = proj_pts.at < cv::Vec4f > (r, c);
			if (pt[0] > 0) {
				cv::Mat v4 = (cv::Mat_<double>(4, 1) << pt[1], pt[2], pt[3], 1);
				cv::Mat v3 = proj_mat * v4;
				float proj_x = v3.at<double>(0) / v3.at<double>(2);
				float proj_y = v3.at<double>(1) / v3.at<double>(2);

				src_pts.push_back(cv::Point2f(c, r));
				dst_pts.push_back(cv::Point2f(proj_x, proj_y));

			}
		}
	}

	cv::Mat H = cv::findHomography(src_pts, dst_pts, cv::RANSAC);

	std::vector<cv::Point2f> center_pt, warped_center_pt;
	center_pt.push_back(cv::Point2f(proj_pts.cols / 2, proj_pts.rows / 2));
	cv::perspectiveTransform(center_pt, warped_center_pt, H);
	cv::Point2f shift = center_pt[0] - warped_center_pt[0];

	cv::Mat shifted_M =
			(cv::Mat_<double>(3, 3) << 1, 0, shift.x, 0, 1, shift.y, 0, 0, 1);
	cv::Mat shifted_H = shifted_M * H;

//	std::cout<<"shifted_H: "<<std::endl;
//	std::cout<<shifted_H.size()<<std::endl;
//	std::cout<<input_img.size()<<std::endl;
	cv::Mat output_img;
	cv::warpPerspective(input_img, output_img, shifted_H, input_img.size());
	cv::warpPerspective(
			cv::Mat(input_img.size(), CV_8UC3, cv::Scalar(255, 255, 255)), mask,
			shifted_H, input_img.size(), cv::INTER_NEAREST);

	homo = shifted_H.clone();

	return output_img;
}

cv::Mat HomoBy3DPointsProj(const cv::Mat &proj_pts, const cv::Mat &proj_mat) {

	std::vector < cv::Point2f > src_pts;
	std::vector < cv::Point2f > dst_pts;

	for (int r = 0; r < proj_pts.rows; r++) {
		for (int c = 0; c < proj_pts.cols; c++) {
			cv::Vec4f pt = proj_pts.at < cv::Vec4f > (r, c);
			if (pt[0] > 0) {
				cv::Mat v4 = (cv::Mat_<double>(4, 1) << pt[1], pt[2], pt[3], 1);
				cv::Mat v3 = proj_mat * v4;
				float proj_x = v3.at<double>(0) / v3.at<double>(2);
				float proj_y = v3.at<double>(1) / v3.at<double>(2);

				src_pts.push_back(cv::Point2f(c, r));
				dst_pts.push_back(cv::Point2f(proj_x, proj_y));

			}
		}
	}

//	cv::Mat H = cv::findHomography(src_pts, dst_pts, cv::RANSAC);
	cv::Mat H = cv::findHomography(src_pts, dst_pts);

	return H;
}

void UpdateInpaintingVotingWidthWarpedTempl(cv::Mat &img,
		const cv::Mat &proj_pts, const std::vector<cv::Point2i> &filling_pix,
		const std::vector<cv::Mat *> &img_ptrs,
		const std::vector<cv::Mat> &img_proj_mats,
		const std::vector<double> &img_scales,
		const std::vector<cv::Rect> &img_rois, const int vw) {

	cv::Mat color_sum(img.size(), CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat color_cnt(img.size(), CV_32S, cv::Scalar(0));
	int cnt = 0;
	cv::Rect img_rect(0, 0, img.cols, img.rows);
	for (int i = 0; i < filling_pix.size(); i++) {
		cv::Point2i pix = filling_pix[i];

		double img_scaling = img_scales[i];
		cv::Mat scaled_img;
		cv::resize(*(img_ptrs[i]), scaled_img, cv::Size(), img_scaling,
				img_scaling);

		cv::Mat sub_img = scaled_img(img_rois[i]);

		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
		temp_img_rect = temp_img_rect & img_rect;
		cv::Mat temp_img = img(temp_img_rect);
		cv::Mat temp_pts = proj_pts(temp_img_rect);

		cv::Mat scaled_proj_mat = img_proj_mats[i].clone();
		scaled_proj_mat.row(0) = scaled_proj_mat.row(0) * img_scaling;
		scaled_proj_mat.row(1) = scaled_proj_mat.row(1) * img_scaling;

		cv::Mat warped_mask, homo;
		cv::Mat warped_templ = WarpedImageBy3DPointsHomo(temp_img, temp_pts,
				scaled_proj_mat, warped_mask, homo);
		cv::Mat inv_homo = homo.inv();

		cv::Rect voting_rect(pix.x - vw, pix.y - vw, vw * 2 + 1, vw * 2 + 1);
		voting_rect = voting_rect & img_rect;
		cv::Rect rel_voting_rect(voting_rect.x - temp_img_rect.x,
				voting_rect.y - temp_img_rect.y, voting_rect.width,
				voting_rect.height);
		cv::Mat result;
		cv::matchTemplate(sub_img, warped_templ, result, CV_TM_SQDIFF,
				warped_mask);

		double minVal, maxVal;
		cv::Point minLoc(0, 0), maxLoc(0, 0);
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

		cv::Mat mask1c;
		cv::cvtColor(warped_mask, mask1c, CV_BGR2GRAY);
		int num_pix = cv::countNonZero(mask1c);
		double ave_pixel_diff = minVal / num_pix;
//		std::cout<<"ave_pixel_diff: "<<ave_pixel_diff<<std::endl;

		if (ave_pixel_diff < 0.01f) {
			cv::Rect matching_rect(minLoc.x, minLoc.y, temp_img_rect.width,
					temp_img_rect.height);
			cv::Mat matching_patch, warped_back_patch;

			cv::warpPerspective(sub_img(matching_rect), warped_back_patch,
					inv_homo, matching_rect.size());
			cv::Mat inv_mask1c;
			cv::warpPerspective(
					cv::Mat(matching_rect.size(), CV_32S, cv::Scalar(1)),
					inv_mask1c, inv_homo, matching_rect.size(),
					cv::INTER_NEAREST);

			warped_back_patch.convertTo(matching_patch, CV_64F);
			color_sum(voting_rect) = color_sum(voting_rect)
					+ matching_patch(rel_voting_rect);
			color_cnt(voting_rect) = color_cnt(voting_rect)
					+ inv_mask1c(rel_voting_rect);
		}

//		if (pix.x == 282 && pix.y == 396) {
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/temp_img.jpg",
//					temp_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/warped_templ.jpg",
//					warped_templ);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/warped_mask.jpg",
//					warped_mask);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/scaled_img.jpg",
//					scaled_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/sub_img.jpg",
//					sub_img);
//
//			cv::rectangle(sub_img,
//					cv::Rect(minLoc.x, minLoc.y, temp_img.cols, temp_img.rows),
//					cv::Scalar(0, 255, 0));
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/det_img.jpg",
//					sub_img);
//			exit(0);
//		}
	}

	for (int i = 0; i < filling_pix.size(); i++) {
		cv::Point2i pix = filling_pix[i];
		cv::Vec3d c = color_sum.at < cv::Vec3d > (pix.y, pix.x);
		int cnt = color_cnt.at<int>(pix.y, pix.x);
		if (cnt > 0)
			img.at < cv::Vec3b > (pix.y, pix.x) = cv::Vec3b(c[0] / cnt,
					c[1] / cnt, c[2] / cnt);
	}
}

void UpdateInpaintingVotingWidthWarpedTemplFixPix(cv::Mat &img,
		const cv::Mat &proj_pts, std::vector<cv::Point2i> &filling_pix,
		std::vector<cv::Mat *> &img_ptrs, std::vector<cv::Mat> &img_proj_mats,
		std::vector<double> &img_scales, std::vector<cv::Rect> &img_rois,
		const int vw, cv::Rect &inp_rect, double matching_thresh) {

	cv::Mat color_sum(img.size(), CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat color_cnt(img.size(), CV_32S, cv::Scalar(0));
	cv::Mat color_cost(img.size(), CV_64F, cv::Scalar(DBL_MAX));
	int cnt = 0;
	cv::Rect img_rect(0, 0, img.cols, img.rows);

	auto filling_pix_iter = filling_pix.begin();
	auto img_ptrs_iter = img_ptrs.begin();
	auto img_proj_mats_iter = img_proj_mats.begin();
	auto img_scales_iter = img_scales.begin();
	auto img_rois_iter = img_rois.begin();
	for (; filling_pix_iter != filling_pix.end();) {
		cv::Point2i pix = *filling_pix_iter;

		double img_scaling = *img_scales_iter;
		cv::Mat scaled_img;
		cv::resize(*(*img_ptrs_iter), scaled_img, cv::Size(), img_scaling,
				img_scaling);

		cv::Mat sub_img = scaled_img(*img_rois_iter);

		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
		temp_img_rect = temp_img_rect & img_rect;
		cv::Mat temp_img = img(temp_img_rect);
		cv::Mat temp_pts = proj_pts(temp_img_rect);

		cv::Mat scaled_proj_mat = (*img_proj_mats_iter).clone();
		scaled_proj_mat.row(0) = scaled_proj_mat.row(0) * img_scaling;
		scaled_proj_mat.row(1) = scaled_proj_mat.row(1) * img_scaling;

		cv::Mat warped_mask, homo;
		cv::Mat warped_templ = WarpedImageBy3DPointsHomo(temp_img, temp_pts,
				scaled_proj_mat, warped_mask, homo);
		cv::Mat inv_homo = homo.inv();

		cv::Rect voting_rect(pix.x - vw, pix.y - vw, vw * 2 + 1, vw * 2 + 1);
		voting_rect = voting_rect & img_rect;
		cv::Rect rel_voting_rect(voting_rect.x - temp_img_rect.x,
				voting_rect.y - temp_img_rect.y, voting_rect.width,
				voting_rect.height);
		cv::Mat result;
		cv::matchTemplate(sub_img, warped_templ, result, CV_TM_SQDIFF,
				warped_mask);

//		cv::Mat sub_img_float, warped_templ_float;
//		sub_img.convertTo(sub_img_float, CV_32F);
//		warped_templ.convertTo(warped_templ_float, CV_32F);
//
//		cv::Mat sub_img_depth(sub_img_float.size(), CV_32FC4,
//				cv::Scalar(0, 0, 0, 0));
//		cv::Mat warped_templ_depth(warped_templ_float.size(), CV_32FC4,
//				cv::Scalar(0, 0, 0, 0));
//		cv::Mat warped_mask_4c(warped_mask.size(), CV_8UC4);
//		int from_to[] = { 0, 0, 1, 1, 2, 2, 2, 3 };
//		std::cout << "sub_img_float" << std::endl;
//		cv::mixChannels(&sub_img_float, 1, &sub_img_depth, 1, from_to, 3);
//		std::cout << "warped_templ_float" << std::endl;
//		cv::mixChannels(&warped_templ_float, 1, &warped_templ_depth, 1, from_to,
//				3);
//		std::cout << "warped_mask" << std::endl;
//		cv::mixChannels(&warped_mask, 1, &warped_mask_4c, 1, from_to, 4);
//		std::cout << "matchTemplate" << std::endl;
//		cv::matchTemplate(sub_img_depth, warped_templ_depth, result,
//				CV_TM_SQDIFF, warped_mask);
//		std::cout << "done" << std::endl;

		double minVal, maxVal;
		cv::Point minLoc(0, 0), maxLoc(0, 0);
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

		cv::Mat mask1c;
		cv::cvtColor(warped_mask, mask1c, CV_BGR2GRAY);
		int num_pix = cv::countNonZero(mask1c);
		double ave_pixel_diff = minVal / num_pix;

//		cv::Mat temp_img_mask(temp_img.size(), CV_8UC3,
//				cv::Scalar(255, 255, 255));
//		cv::matchTemplate(sub_img, temp_img, result, CV_TM_SQDIFF,
//				temp_img_mask);
//		double minVal1, maxVal1;
//		cv::Point minLoc1(0, 0), maxLoc1(0, 0);
//		cv::minMaxLoc(result, &minVal1, &maxVal1, &minLoc1, &maxLoc1);
//		double ave_pixel_diff1 = minVal1 / (temp_img.rows * temp_img.cols);

//		std::cout<<"ave_pixel_diff vs ave_pixel_diff1: "<<ave_pixel_diff<<"    "<<ave_pixel_diff1<<std::endl;

		if (ave_pixel_diff < matching_thresh) {
			cv::Rect matching_rect(minLoc.x, minLoc.y, temp_img_rect.width,
					temp_img_rect.height);
			cv::Mat matching_patch, warped_back_patch;

			cv::warpPerspective(sub_img(matching_rect), warped_back_patch,
					inv_homo, matching_rect.size());
			cv::Mat inv_mask1c;
			cv::warpPerspective(
					cv::Mat(matching_rect.size(), CV_32S, cv::Scalar(1)),
					inv_mask1c, inv_homo, matching_rect.size(),
					cv::INTER_NEAREST);

			warped_back_patch.convertTo(matching_patch, CV_64F);
			color_sum(voting_rect) = color_sum(voting_rect)
					+ matching_patch(rel_voting_rect);
			color_cnt(voting_rect) = color_cnt(voting_rect)
					+ inv_mask1c(rel_voting_rect);

//			cv::Mat sub_color_sum = color_sum(voting_rect);
//			cv::Mat sub_color_cost = color_cost(voting_rect);
//			cv::Mat sub_inv_mask1c = inv_mask1c(rel_voting_rect);
//			cv::Mat sub_matching_patch = matching_patch(rel_voting_rect);
//			for (int c=0; c<sub_color_sum.cols; c++){
//				for (int r=0; r<sub_color_sum.rows; r++){
//					if (sub_inv_mask1c.at<int>(r, c)>0 && ave_pixel_diff<sub_color_cost.at<double>(r, c)){
//						sub_color_sum.at<cv::Vec3d>(r, c) = sub_matching_patch.at<cv::Vec3d>(r, c);
//						sub_color_cost.at<double>(r, c) = ave_pixel_diff;
//					}
//				}
//			}

//			filling_pix_iter = filling_pix.erase(filling_pix_iter);
//			img_ptrs_iter = img_ptrs.erase(img_ptrs_iter);
//			img_proj_mats_iter = img_proj_mats.erase(img_proj_mats_iter);
//			img_scales_iter = img_scales.erase(img_scales_iter);
//			img_rois_iter = img_rois.erase(img_rois_iter);
		} else {
//			filling_pix_iter++;
//			img_ptrs_iter++;
//			img_proj_mats_iter++;
//			img_scales_iter++;
//			img_rois_iter++;
		}

		filling_pix_iter++;
		img_ptrs_iter++;
		img_proj_mats_iter++;
		img_scales_iter++;
		img_rois_iter++;

//		if (pix.x == 301 && pix.y == 346) {
////			std::cout << "ave_pixel_diff vs ave_pixel_diff1: " << ave_pixel_diff
////					<< "    " << ave_pixel_diff1 << std::endl;
//
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/temp_img.jpg",
//					temp_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/warped_templ.jpg",
//					warped_templ);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/warped_mask.jpg",
//					warped_mask);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/scaled_img.jpg",
//					scaled_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/sub_img.jpg",
//					sub_img);
//
//			cv::rectangle(sub_img,
//					cv::Rect(minLoc.x, minLoc.y, temp_img.cols, temp_img.rows),
//					cv::Scalar(0, 255, 0));
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/det_img.jpg",
//					sub_img);
////			exit(0);
//		}
	}

//	for (int x = inp_rect.x; x < inp_rect.x + inp_rect.width; x++) {
//		for (int y = inp_rect.y; y < inp_rect.y + inp_rect.height; y++) {
//			cv::Vec3d c = color_sum.at < cv::Vec3d > (y, x);
//			if (color_cost.at<double>(y, x) < DBL_MAX)
//				img.at < cv::Vec3b > (y, x) = cv::Vec3b(c[0],
//						c[1], c[2]);
//		}
//	}

	filling_pix_iter = filling_pix.begin();
	img_ptrs_iter = img_ptrs.begin();
	img_proj_mats_iter = img_proj_mats.begin();
	img_scales_iter = img_scales.begin();
	img_rois_iter = img_rois.begin();
	for (; filling_pix_iter != filling_pix.end();) {
		cv::Point2i pix = *filling_pix_iter;
		int cnt = color_cnt.at<int>(pix.y, pix.x);
		if (cnt > 0) {
			cv::Vec3d c = color_sum.at < cv::Vec3d > (pix.y, pix.x);
			img.at < cv::Vec3b > (pix.y, pix.x) = cv::Vec3b(c[0] / cnt,
					c[1] / cnt, c[2] / cnt);

			filling_pix_iter = filling_pix.erase(filling_pix_iter);
			img_ptrs_iter = img_ptrs.erase(img_ptrs_iter);
			img_proj_mats_iter = img_proj_mats.erase(img_proj_mats_iter);
			img_scales_iter = img_scales.erase(img_scales_iter);
			img_rois_iter = img_rois.erase(img_rois_iter);

//			filling_pix_iter++;
//			img_ptrs_iter++;
//			img_proj_mats_iter++;
//			img_scales_iter++;
//			img_rois_iter++;
		} else {
			filling_pix_iter++;
			img_ptrs_iter++;
			img_proj_mats_iter++;
			img_scales_iter++;
			img_rois_iter++;
		}
	}

}

void UpdateInpaintingInvWarpedTemplFixPix(cv::Mat &img, const cv::Mat &proj_pts,
		std::vector<cv::Point2i> &filling_pix, std::vector<cv::Mat *> &img_ptrs,
		std::vector<cv::Mat> &img_proj_mats, std::vector<double> &img_scales,
		std::vector<cv::Rect> &img_rois, const int vw, cv::Rect &inp_rect,
		double matching_thresh) {

	cv::Mat color_sum(img.size(), CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat color_cnt(img.size(), CV_32S, cv::Scalar(0));
	cv::Mat color_cost(img.size(), CV_64F, cv::Scalar(DBL_MAX));
	int cnt = 0;
	cv::Rect img_rect(0, 0, img.cols, img.rows);

	auto filling_pix_iter = filling_pix.begin();
	auto img_ptrs_iter = img_ptrs.begin();
	auto img_proj_mats_iter = img_proj_mats.begin();
	auto img_scales_iter = img_scales.begin();
	auto img_rois_iter = img_rois.begin();
	for (; filling_pix_iter != filling_pix.end();) {
		cv::Point2i pix = *filling_pix_iter;

		double img_scaling = *img_scales_iter;
		cv::Mat scaled_img;
		cv::resize(*(*img_ptrs_iter), scaled_img, cv::Size(), img_scaling,
				img_scaling);

//		cv::Mat sub_img = scaled_img(*img_rois_iter);

		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
		temp_img_rect = temp_img_rect & img_rect;
		cv::Mat temp_img = img(temp_img_rect);
		cv::Mat temp_pts = proj_pts(temp_img_rect);

		cv::Mat scaled_proj_mat = (*img_proj_mats_iter).clone();
		scaled_proj_mat.row(0) = scaled_proj_mat.row(0) * img_scaling;
		scaled_proj_mat.row(1) = scaled_proj_mat.row(1) * img_scaling;

		cv::Mat homo = HomoBy3DPointsProj(temp_pts, scaled_proj_mat);
		cv::Mat inv_homo = homo.inv();

		cv::Rect voting_rect(pix.x - vw, pix.y - vw, vw * 2 + 1, vw * 2 + 1);
		voting_rect = voting_rect & img_rect;
		cv::Rect rel_voting_rect(voting_rect.x - temp_img_rect.x,
				voting_rect.y - temp_img_rect.y, voting_rect.width,
				voting_rect.height);

		std::vector<cv::Point2f> center_pt, inv_warped_center_pt;
		center_pt.push_back(
				cv::Point2f(img_rois_iter->x + img_rois_iter->width / 2,
						img_rois_iter->y + img_rois_iter->height / 2));
		cv::perspectiveTransform(center_pt, inv_warped_center_pt, inv_homo);
		cv::Point2f shift = center_pt[0] - inv_warped_center_pt[0];

		cv::Mat shifted_M =
				(cv::Mat_<double>(3, 3) << 1, 0, shift.x, 0, 1, shift.y, 0, 0, 1);
		cv::Mat shifted_inv_homo = shifted_M * inv_homo;
		cv::Mat inv_scaled_img;
		cv::warpPerspective(scaled_img, inv_scaled_img, shifted_inv_homo,
				scaled_img.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
		cv::Mat sub_img = inv_scaled_img(*img_rois_iter);

		cv::Mat result;
		cv::matchTemplate(sub_img, temp_img, result, CV_TM_SQDIFF);

		double minVal, maxVal;
		cv::Point minLoc(0, 0), maxLoc(0, 0);
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

		double ave_pixel_diff = minVal / temp_img_rect.area();
//		std::cout<<"minVal: "<<minVal<<std::endl;
//		std::cout<<"ave_pixel_diff: "<<ave_pixel_diff<<std::endl;

		if (ave_pixel_diff < matching_thresh) {
			cv::Rect matching_rect(minLoc.x, minLoc.y, temp_img_rect.width,
					temp_img_rect.height);
			cv::Mat matching_patch, warped_back_patch;

			warped_back_patch = sub_img(matching_rect);

			cv::Mat warped_back_mask(warped_back_patch.size(), CV_32S,
					cv::Scalar(0));
			for (int r = 0; r < warped_back_mask.rows; r++) {
				for (int c = 0; c < warped_back_mask.cols; c++) {
					if (warped_back_patch.at < cv::Vec3b > (r, c)
							!= cv::Vec3b(0, 0, 0)) {
						warped_back_mask.at<int>(r, c) = 1;
					}
				}
			}

			warped_back_patch.convertTo(matching_patch, CV_64F);
			color_sum(voting_rect) = color_sum(voting_rect)
					+ matching_patch(rel_voting_rect);
			color_cnt(voting_rect) = color_cnt(voting_rect)
					+ warped_back_mask(rel_voting_rect);

//			cv::Mat sub_color_sum = color_sum(voting_rect);
//			cv::Mat sub_color_cost = color_cost(voting_rect);
//			cv::Mat sub_inv_mask1c = inv_mask1c(rel_voting_rect);
//			cv::Mat sub_matching_patch = matching_patch(rel_voting_rect);
//			for (int c=0; c<sub_color_sum.cols; c++){
//				for (int r=0; r<sub_color_sum.rows; r++){
//					if (sub_inv_mask1c.at<int>(r, c)>0 && ave_pixel_diff<sub_color_cost.at<double>(r, c)){
//						sub_color_sum.at<cv::Vec3d>(r, c) = sub_matching_patch.at<cv::Vec3d>(r, c);
//						sub_color_cost.at<double>(r, c) = ave_pixel_diff;
//					}
//				}
//			}

//			filling_pix_iter = filling_pix.erase(filling_pix_iter);
//			img_ptrs_iter = img_ptrs.erase(img_ptrs_iter);
//			img_proj_mats_iter = img_proj_mats.erase(img_proj_mats_iter);
//			img_scales_iter = img_scales.erase(img_scales_iter);
//			img_rois_iter = img_rois.erase(img_rois_iter);
		} else {
//			filling_pix_iter++;
//			img_ptrs_iter++;
//			img_proj_mats_iter++;
//			img_scales_iter++;
//			img_rois_iter++;
		}

		filling_pix_iter++;
		img_ptrs_iter++;
		img_proj_mats_iter++;
		img_scales_iter++;
		img_rois_iter++;

//		if (pix.x==180 && pix.y==218) {
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/temp_img.jpg",
//					temp_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/scaled_img.jpg",
//					scaled_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/inv_scaled_img.jpg",
//					inv_scaled_img);
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/sub_img.jpg",
//					sub_img);
//
//			cv::rectangle(sub_img,
//					cv::Rect(minLoc.x, minLoc.y, temp_img.cols, temp_img.rows),
//					cv::Scalar(0, 255, 0));
//			cv::imwrite(
//					"../data/pandora_liang/set2/1534313570-1534313581_results/det_img.jpg",
//					sub_img);
//			exit(0);
//		}
	}

//	for (int x = inp_rect.x; x < inp_rect.x + inp_rect.width; x++) {
//		for (int y = inp_rect.y; y < inp_rect.y + inp_rect.height; y++) {
//			cv::Vec3d c = color_sum.at < cv::Vec3d > (y, x);
//			if (color_cost.at<double>(y, x) < DBL_MAX)
//				img.at < cv::Vec3b > (y, x) = cv::Vec3b(c[0],
//						c[1], c[2]);
//		}
//	}

	filling_pix_iter = filling_pix.begin();
	img_ptrs_iter = img_ptrs.begin();
	img_proj_mats_iter = img_proj_mats.begin();
	img_scales_iter = img_scales.begin();
	img_rois_iter = img_rois.begin();
	for (; filling_pix_iter != filling_pix.end();) {
		cv::Point2i pix = *filling_pix_iter;
		int cnt = color_cnt.at<int>(pix.y, pix.x);
		if (cnt > 0) {
			cv::Vec3d c = color_sum.at < cv::Vec3d > (pix.y, pix.x);
			img.at < cv::Vec3b > (pix.y, pix.x) = cv::Vec3b(c[0] / cnt,
					c[1] / cnt, c[2] / cnt);

			filling_pix_iter = filling_pix.erase(filling_pix_iter);
			img_ptrs_iter = img_ptrs.erase(img_ptrs_iter);
			img_proj_mats_iter = img_proj_mats.erase(img_proj_mats_iter);
			img_scales_iter = img_scales.erase(img_scales_iter);
			img_rois_iter = img_rois.erase(img_rois_iter);
		} else {
			filling_pix_iter++;
			img_ptrs_iter++;
			img_proj_mats_iter++;
			img_scales_iter++;
			img_rois_iter++;
		}
	}

}

struct SourceImgMeta {
	SourceImgMeta() {
	}
	SourceImgMeta(cv::Mat *ip, cv::Mat ipm, double is, cv::Rect ir,
			cv::Vec3b ic) :
			img_ptr(ip), img_proj_mat(ipm), img_scale(is), img_roi(ir), color(
					ic) {
	}
	cv::Mat *img_ptr;
	cv::Mat img_proj_mat;
	double img_scale;
	cv::Rect img_roi;
	cv::Vec3b color;
};

double FindNearestNeighbor(cv::Mat &img, cv::Mat &proj_pts, int x, int y,
		const cv::Mat *img_ptr, const cv::Mat &img_proj_mat,
		const double img_scaling, const cv::Rect &img_roi, cv::Vec3b &color) {

	cv::Rect img_rect(0, 0, img.cols, img.rows);

	cv::Mat scaled_img;
	cv::resize(*img_ptr, scaled_img, cv::Size(), img_scaling, img_scaling);

	cv::Mat sub_img = scaled_img(img_roi);

	cv::Rect temp_img_rect(x - PATCH_SIZE, y - PATCH_SIZE, PATCH_SIZE * 2 + 1,
			PATCH_SIZE * 2 + 1);
	temp_img_rect = temp_img_rect & img_rect;
	cv::Mat temp_img = img(temp_img_rect);
//	cv::Mat temp_pts = proj_pts(temp_img_rect);

//	cv::Mat scaled_proj_mat = img_proj_mat.clone();
//	scaled_proj_mat.row(0) = scaled_proj_mat.row(0) * img_scaling;
//	scaled_proj_mat.row(1) = scaled_proj_mat.row(1) * img_scaling;

//	cv::Mat warped_mask, homo;
//	cv::Mat warped_templ = WarpedImageBy3DPointsHomo(temp_img, temp_pts,
//			scaled_proj_mat, warped_mask, homo);
//	cv::Mat inv_homo = homo.inv();
//	cv::Mat result;
//	cv::matchTemplate(sub_img, warped_templ, result, CV_TM_SQDIFF, warped_mask);
//
//	double minVal, maxVal;
//	cv::Point minLoc(0, 0), maxLoc(0, 0);
//	cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
//
//	cv::Mat mask1c;
//	cv::cvtColor(warped_mask, mask1c, CV_BGR2GRAY);
//	int num_pix = cv::countNonZero(mask1c);
//	double ave_pixel_diff = minVal / num_pix;

	cv::Mat result;
	cv::matchTemplate(sub_img, temp_img, result, CV_TM_SQDIFF);

	double minVal, maxVal;
	cv::Point minLoc(0, 0), maxLoc(0, 0);
	cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

	int num_pix = temp_img_rect.area();
	double ave_pixel_diff = minVal / num_pix;

	int templ_center_x = x - temp_img_rect.x;
	int templ_center_y = y - temp_img_rect.y;
	int matching_x = minLoc.x + templ_center_x;
	int matching_y = minLoc.y + templ_center_y;
	color = sub_img.at < cv::Vec3b > (matching_y, matching_x);

//	if (x == 178 && y == 217) {
//		char oname[512];
//		sprintf(oname, "../data/pandora_liang/set2/1534313570-1534313581_results/%f_temp_img.jpg", ave_pixel_diff);
//		cv::imwrite(oname, temp_img);
//
//		sprintf(oname, "../data/pandora_liang/set2/1534313570-1534313581_results/%f_scaled_img.jpg", ave_pixel_diff);
//		cv::imwrite(oname,	scaled_img);
//
//		sprintf(oname, "../data/pandora_liang/set2/1534313570-1534313581_results/%f_sub_img.jpg", ave_pixel_diff);
//		cv::imwrite(oname, sub_img);
//
//		sprintf(oname, "../data/pandora_liang/set2/1534313570-1534313581_results/%f_det_img.jpg", ave_pixel_diff);
//		cv::rectangle(sub_img,
//				cv::Rect(minLoc.x, minLoc.y, temp_img.cols, temp_img.rows),
//				cv::Scalar(0, 255, 0));
//		cv::imwrite(oname,	sub_img);
//	}

	return ave_pixel_diff;
}

void InpaintingNNFUpdateWarpedTempl(cv::Mat &img, cv::Mat &proj_pts,
		const std::vector<cv::Point2i> &filling_pix,
		const std::vector<cv::Mat *> &img_ptrs,
		const std::vector<cv::Mat> &img_proj_mats,
		const std::vector<double> &img_scales,
		const std::vector<cv::Rect> &img_rois, const int vw) {

	char oname[512];
	sprintf(oname,
			"../data/pandora_liang/set2/1534313570-1534313581_results/input.jpg");
	cv::imwrite(oname, img);

	cv::Mat color_sum(img.size(), CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat color_cnt(img.size(), CV_32S, cv::Scalar(0));
	int cnt = 0;
	cv::Rect img_rect(0, 0, img.cols, img.rows);

	int lx = INT_MAX, rx = -1, ty = INT_MAX, by = -1;
	int in_w, in_h;
	for (int i = 0; i < filling_pix.size(); i++) {
		cv::Point2i pix = filling_pix[i];
		lx = lx < pix.x ? lx : pix.x;
		rx = rx > pix.x ? rx : pix.x;
		ty = ty < pix.y ? ty : pix.y;
		by = by > pix.y ? by : pix.y;
	}
	in_w = rx - lx + 1;
	in_h = by - ty + 1;

	cv::Vec3b color;

	std::vector < std::vector
			< SourceImgMeta
					>> pix_source_meta(in_h,
							std::vector < SourceImgMeta > (in_w));
	std::vector<std::vector<double>> pix_source_diff(in_h,
			std::vector<double>(in_w));
	for (int i = 0; i < filling_pix.size(); i++) {
		cv::Point2i pix = filling_pix[i];
		double min_diff = FindNearestNeighbor(img, proj_pts, pix.x, pix.y,
				img_ptrs[i], img_proj_mats[i], img_scales[i], img_rois[i],
				color);

//		cv::Point2i pix = filling_pix[i];
//
//		double img_scaling = img_scales[i];
//		cv::Mat scaled_img;
//		cv::resize(*(img_ptrs[i]), scaled_img, cv::Size(), img_scaling,
//				img_scaling);
//
//		cv::Mat sub_img = scaled_img(img_rois[i]);
//
//		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
//				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
//		temp_img_rect = temp_img_rect & img_rect;
//		cv::Mat temp_img = img(temp_img_rect);
//		cv::Mat temp_pts = proj_pts(temp_img_rect);
//
//		cv::Mat scaled_proj_mat = img_proj_mats[i].clone();
//		scaled_proj_mat.row(0) = scaled_proj_mat.row(0) * img_scaling;
//		scaled_proj_mat.row(1) = scaled_proj_mat.row(1) * img_scaling;
//
//		cv::Mat warped_mask, homo;
//		cv::Mat warped_templ = WarpedImageBy3DPointsHomo(temp_img, temp_pts,
//				scaled_proj_mat, warped_mask, homo);
//		cv::Mat inv_homo = homo.inv();
//
//		cv::Rect voting_rect(pix.x - vw, pix.y - vw, vw * 2 + 1, vw * 2 + 1);
//		voting_rect = voting_rect & img_rect;
//		cv::Rect rel_voting_rect(voting_rect.x - temp_img_rect.x,
//				voting_rect.y - temp_img_rect.y, voting_rect.width,
//				voting_rect.height);
//		cv::Mat result;
//		cv::matchTemplate(sub_img, warped_templ, result, CV_TM_SQDIFF,
//				warped_mask);
//
//		double minVal, maxVal;
//		cv::Point minLoc(0, 0), maxLoc(0, 0);
//		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
//
//		cv::Mat mask1c;
//		cv::cvtColor(warped_mask, mask1c, CV_BGR2GRAY);
//		int num_pix = cv::countNonZero(mask1c);
//		double ave_pixel_diff = minVal / num_pix;

		pix_source_meta[pix.y - ty][pix.x - lx] = SourceImgMeta(img_ptrs[i],
				img_proj_mats[i], img_scales[i], img_rois[i], color);
		pix_source_diff[pix.y - ty][pix.x - lx] = min_diff;
	}

	for (int iter = 0; iter < 10; iter++) {
		if (iter % 2 == 0) {
			for (int y = ty; y <= by; y++) {
				for (int x = lx; x <= rx; x++) {
					if (x > lx) {
						SourceImgMeta neighbor = pix_source_meta[y - ty][x - 1
								- lx];
						double min_diff = FindNearestNeighbor(img, proj_pts, x,
								y, neighbor.img_ptr, neighbor.img_proj_mat,
								neighbor.img_scale, neighbor.img_roi, color);
						if (min_diff < pix_source_diff[y - ty][x - lx]) {
							pix_source_diff[y - ty][x - lx] = min_diff;
							neighbor.color = color;
							pix_source_meta[y - ty][x - lx] = neighbor;

							if (x == 178 && y == 217)
								std::cout << "updated by left pix: " << min_diff
										<< std::endl;
						}
					}

					if (y > ty) {
						SourceImgMeta neighbor = pix_source_meta[y - 1 - ty][x
								- lx];
						double min_diff = FindNearestNeighbor(img, proj_pts, x,
								y, neighbor.img_ptr, neighbor.img_proj_mat,
								neighbor.img_scale, neighbor.img_roi, color);
						if (min_diff < pix_source_diff[y - ty][x - lx]) {
							pix_source_diff[y - ty][x - lx] = min_diff;
							neighbor.color = color;
							pix_source_meta[y - ty][x - lx] = neighbor;

							if (x == 178 && y == 217)
								std::cout << "updated by top pix: " << min_diff
										<< std::endl;
						}
					}
				}
			}
		} else {
			for (int y = by; y >= ty; y--) {
				for (int x = rx; x >= lx; x--) {
					if (x < rx) {
						SourceImgMeta neighbor = pix_source_meta[y - ty][x + 1
								- lx];
						double min_diff = FindNearestNeighbor(img, proj_pts, x,
								y, neighbor.img_ptr, neighbor.img_proj_mat,
								neighbor.img_scale, neighbor.img_roi, color);
						if (min_diff < pix_source_diff[y - ty][x - lx]) {
							pix_source_diff[y - ty][x - lx] = min_diff;
							neighbor.color = color;
							pix_source_meta[y - ty][x - lx] = neighbor;

							if (x == 178 && y == 217)
								std::cout << "updated by right pix: "
										<< min_diff << std::endl;
						}
					}

					if (y < by) {
						SourceImgMeta neighbor = pix_source_meta[y + 1 - ty][x
								- lx];
						double min_diff = FindNearestNeighbor(img, proj_pts, x,
								y, neighbor.img_ptr, neighbor.img_proj_mat,
								neighbor.img_scale, neighbor.img_roi, color);
						if (min_diff < pix_source_diff[y - ty][x - lx]) {
							pix_source_diff[y - ty][x - lx] = min_diff;
							neighbor.color = color;
							pix_source_meta[y - ty][x - lx] = neighbor;

							if (x == 178 && y == 217)
								std::cout << "updated by bottom pix: "
										<< min_diff << std::endl;
						}
					}
				}
			}
		}

//		for (int y = ty; y <= by; y++) {
//			for (int x = lx; x <= rx; x++) {
//				img.at < cv::Vec3b > (y, x) =
//						pix_source_meta[y - ty][x - lx].color;
//			}
//		}

//		sprintf(oname,
//				"../data/pandora_liang/set2/1534313570-1534313581_results/iter%04d.jpg",
//				iter);
//		cv::imwrite(oname, img);

	}
}

cv::Mat CloudImageProjectionGuidedInpainting(std::vector<PointXYZIS> &cloud,
		cv::Mat const &R, cv::Mat const &T, cv::Mat const &K, cv::Mat &img,
		const cv::Mat &mask, cv::Mat &inpainted_init) {

	cv::Mat output_img = img.clone();
	output_img.setTo(0, mask);

	cv::Mat debug_img = output_img.clone();
//std::vector<cv::Point2f> inbound_pts;
	std::vector<PointXYZIS *> pts_ptr;
	std::vector<float> pts_dist;
	cv::Mat_<float> inbound_pts(0, 2);
	for (auto &p : cloud) {
		cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

		cv::Mat uv = K * (R * v3 + T);
		float fu = uv.at<double>(0) / uv.at<double>(2);
		float fv = uv.at<double>(1) / uv.at<double>(2);
		int u = fu + 0.5;
		int v = fv + 0.5;

		if (uv.at<double>(2) > 0 && 0 <= u && u < img.cols && 0 <= v
				&& v < img.rows && mask.at < uchar > (v, u) > 0) {

			if (p.img_ptr) {
				//inbound_pts.emplace_back(fu, fv);
				cv::Mat row = (cv::Mat_<float>(1, 2) << fu, fv);
				inbound_pts.push_back(row);
				pts_ptr.push_back(&p);
				pts_dist.push_back(fabs(uv.at<double>(2)));

				debug_img.at < cv::Vec3b > (v, u) = p.img_ptr->at < cv::Vec3b
						> (p.v, p.u);
			}
		}
	}

	cv::imwrite("/disk1/tmp/proj.jpg", debug_img);

	cv::Mat mask_cpy = mask.clone();
	int cu = -1, cv = -1;

	for (int i = 0; i < mask_cpy.rows; i++) {
		for (int j = 0; j < mask_cpy.cols; j++) {
			if (mask_cpy.at < uchar > (i, j) > 0) {
				cu = j;
				cv = i;
				break;
			}
		}
		if (cv == i)
			break;
	}

	std::vector < cv::Point > dir_offset;
	dir_offset.emplace_back(1, 0);
	dir_offset.emplace_back(0, 1);
	dir_offset.emplace_back(-1, 0);
	dir_offset.emplace_back(0, -1);
	int dir_idx = 0;
	std::vector < cv::Point2i > filling_pix;
	while (mask_cpy.at < uchar > (cv, cu) > 0) {
		bool pix_stored = false;
		if ((dir_idx == 0 && cv > PATCH_SIZE)
				|| (dir_idx == 1 && cu + PATCH_SIZE < mask_cpy.cols)
				|| (dir_idx == 2 && cv + PATCH_SIZE < mask_cpy.rows)
				|| (dir_idx == 3 && cu > PATCH_SIZE)) {
			filling_pix.emplace_back(cu, cv);
			mask_cpy.at < uchar > (cv, cu) = 0;
			pix_stored = true;
		}
		cv::Point2i next_pix = cv::Point2i(cu, cv) + dir_offset[dir_idx];
		if (next_pix.x < 0 || next_pix.x >= mask_cpy.cols || next_pix.y < 0
				|| next_pix.y >= mask_cpy.rows
				|| mask_cpy.at < uchar > (next_pix.y, next_pix.x) == 0) {
			dir_idx = (dir_idx + 1) % 4;
			if (pix_stored)
				next_pix = cv::Point2i(cu, cv) + dir_offset[dir_idx];
			else
				next_pix = cv::Point2i(cu, cv);
			if (next_pix.x < 0 || next_pix.x >= mask_cpy.cols || next_pix.y < 0
					|| next_pix.y >= mask_cpy.rows
					|| mask_cpy.at < uchar > (next_pix.y, next_pix.x) == 0)
				break;
		}
		cu = next_pix.x;
		cv = next_pix.y;
	}

	cv::flann::Index flann_index(inbound_pts, cv::flann::KDTreeIndexParams(),
			cvflann::FLANN_DIST_EUCLIDEAN);
	cv::Rect img_rect(0, 0, img.cols, img.rows);
	cv::Mat mask_inv = ~mask;
	cv::cvtColor(mask_inv, mask_inv, cv::COLOR_GRAY2BGR);

	int cnt = 0;
	std::vector<cv::Mat *> img_ptrs;
	std::vector<double> img_scales;
	std::vector < cv::Rect > img_rois;
	for (auto pix : filling_pix) {

//		printf("pix loc: %d   %d\n", pix.x, pix.y);

		cv::Mat query = (cv::Mat_<float>(1, 2) << pix.x, pix.y);
		cv::Mat indices, dists;
		int K = 10;
		flann_index.knnSearch(query, indices, dists, K);

		int closest_idx = 0;
		float min_diff_to_1 = FLT_MAX;
		for (int n = 0; n < K; n++) {
			int cur_ind = indices.at<int>(n);
			float cur_dist = pts_dist[cur_ind];
			auto &pt_ptr = pts_ptr[cur_ind];
			float diff_to_1 = std::max(cur_dist,
					(float) pt_ptr->to_img_distance)
					/ std::min(cur_dist, (float) pt_ptr->to_img_distance);
			if (diff_to_1 < min_diff_to_1) {
				min_diff_to_1 = diff_to_1;
				closest_idx = n;
			}
		}

		int p_ind = indices.at<int>(closest_idx);
		float pix_dist = sqrt(dists.at<float>(closest_idx));

		auto &pt_ptr = pts_ptr[p_ind];
		float pt_dist = pts_dist[p_ind];
		double img_scaling = pt_ptr->to_img_distance / pt_dist;
		cv::Mat scaled_img;
		cv::resize(*(pt_ptr->img_ptr), scaled_img, cv::Size(), img_scaling,
				img_scaling);

		double center_u = pt_ptr->u * img_scaling;
		double center_v = pt_ptr->v * img_scaling;

		//std::cout<<"pix_dist: "<<pix_dist<<std::endl;

		int left = center_u - pix_dist * 3 - PATCH_SIZE * 5;
		int right = center_u + pix_dist * 3 + PATCH_SIZE * 5;
		int top = center_v - pix_dist * 3 - PATCH_SIZE * 5;
		int bottom = center_v + pix_dist * 3 + PATCH_SIZE * 5;

		cv::Rect sub_img_rect(left, top, right - left + 1, bottom - top + 1);
		sub_img_rect = sub_img_rect
				& cv::Rect(0, 0, scaled_img.cols, scaled_img.rows);
		//std::cout<<"sub_img_rect:"<<sub_img_rect<<std::endl;
		cv::Mat sub_img = scaled_img(sub_img_rect);

		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
		temp_img_rect = temp_img_rect & img_rect;
		//std::cout<<"temp_img_rect:"<<temp_img_rect<<std::endl;
		cv::Mat temp_img = output_img(temp_img_rect);
		cv::Mat temp_mask = mask_inv(temp_img_rect);

		cv::Mat result;
		cv::matchTemplate(sub_img, temp_img, result, CV_TM_SQDIFF, temp_mask);

		double minVal, maxVal;
		cv::Point minLoc, maxLoc;
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

//		  if (pix.x==192 && pix.y==551){
//			  std::cout<<sub_img_rect<<std::endl;
//			  std::cout<<temp_img_rect<<std::endl;
//
//			  cv::imwrite("/disk1/tmp/mask_inv.jpg", mask_inv);
//			  cv::imwrite("/disk1/tmp/output_img.jpg", output_img);
//			  cv::imwrite("/disk1/tmp/scaled_img.jpg", scaled_img);
//			  cv::imwrite("/disk1/tmp/sub_img.jpg", sub_img);
//			  cv::imwrite("/disk1/tmp/temp_img.jpg", temp_img);
//			  cv::imwrite("/disk1/tmp/temp_mask.jpg", temp_mask);
//			  std::cout<<minLoc<<std::endl;
//			  exit(0);
//		  }

		int src_u = minLoc.x + PATCH_SIZE;
		int src_v = minLoc.y + PATCH_SIZE;

		output_img.at < cv::Vec3b > (pix.y, pix.x) = sub_img.at < cv::Vec3b
				> (src_v, src_u);
		mask_inv.at < cv::Vec3b > (pix.y, pix.x) = cv::Vec3b(255, 255, 255);

		img_ptrs.push_back(pt_ptr->img_ptr);
		img_scales.push_back(img_scaling);
		img_rois.push_back(sub_img_rect);

//		if (cnt % 500 == 0 || cnt == filling_pix.size() - 1) {
//			char oname[256];
//			sprintf(oname, "/disk1/tmp/img%08d.jpg", cnt);
//			cv::imwrite(oname, output_img);
//			sprintf(oname, "/disk1/tmp/mask%04d.jpg", cnt);
//			cv::imwrite(oname, mask_inv);
//		}

		cnt++;
	}

	inpainted_init = output_img.clone();
	char oname[256];
//cv::imwrite("/disk1/tmp/inpainting_init.jpg", output_img);
	for (int iter = 0; iter < 10; iter++) {
		UpdateInpainting(output_img, filling_pix, img_ptrs, img_scales,
				img_rois);
		//sprintf(oname, "/disk1/tmp/inpainting_iter%04d.jpg", iter);
		//cv::imwrite(oname, output_img);
		//std::cout<<oname<<std::endl;
	}

//exit(0);
	return output_img;
}

//std::vector<cv::Point2i> GetBoundaryPixelInward(const cv::Mat &mask) {
////find top left mask pixel
//	cv::Mat mask_cpy = mask.clone();
//	int cu = -1, cv = -1;
//	for (int i = 0; i < mask_cpy.rows; i++) {
//		for (int j = 0; j < mask_cpy.cols; j++) {
//			if (mask_cpy.at < uchar > (i, j) > 0) {
//				cu = j;
//				cv = i;
//				break;
//			}
//		}
//		if (cv == i)
//			break;
//	}
//
//	std::vector < cv::Point > dir_offset;
//	dir_offset.emplace_back(1, 0);
//	dir_offset.emplace_back(0, 1);
//	dir_offset.emplace_back(-1, 0);
//	dir_offset.emplace_back(0, -1);
//	int dir_idx = 0;
//	std::vector < cv::Point2i > filling_pix;
//	while (mask_cpy.at < uchar > (cv, cu) > 0) {
//		bool pix_stored = false;
//		if ((dir_idx == 0 && cv > PATCH_SIZE)
//				|| (dir_idx == 1 && cu + PATCH_SIZE < mask_cpy.cols)
//				|| (dir_idx == 2 && cv + PATCH_SIZE < mask_cpy.rows)
//				|| (dir_idx == 3 && cu > PATCH_SIZE)) {
//			filling_pix.emplace_back(cu, cv);
//			mask_cpy.at < uchar > (cv, cu) = 0;
//			pix_stored = true;
//		}
//		cv::Point2i next_pix = cv::Point2i(cu, cv) + dir_offset[dir_idx];
//		if (next_pix.x < 0 || next_pix.x >= mask_cpy.cols || next_pix.y < 0
//				|| next_pix.y >= mask_cpy.rows
//				|| mask_cpy.at < uchar > (next_pix.y, next_pix.x) == 0) {
//			dir_idx = (dir_idx + 1) % 4;
//			if (pix_stored)
//				next_pix = cv::Point2i(cu, cv) + dir_offset[dir_idx];
//			else
//				next_pix = cv::Point2i(cu, cv);
//			if (next_pix.x < 0 || next_pix.x >= mask_cpy.cols || next_pix.y < 0
//					|| next_pix.y >= mask_cpy.rows
//					|| mask_cpy.at < uchar > (next_pix.y, next_pix.x) == 0)
//				break;
//		}
//		cu = next_pix.x;
//		cv = next_pix.y;
//	}
//
//	return filling_pix;
//}

//cv::Rect GetMaskDim(const cv::Mat &mask) {
////find top left mask pixel
//	int cu = -1, cv = -1;
//	for (int i = 0; i < mask.rows; i++) {
//		for (int j = 0; j < mask.cols; j++) {
//			if (mask.at < uchar > (i, j) > 0) {
//				cu = j;
//				cv = i;
//				break;
//			}
//		}
//		if (cv == i)
//			break;
//	}
//
//	int width = 0, height = 0;
//	for (int i = cv; i < mask.rows; i++) {
//		if (mask.at < uchar > (i, cu) > 0)
//			height++;
//		else
//			break;
//	}
//
//	for (int j = cu; j < mask.cols; j++) {
//		if (mask.at < uchar > (cv, j) > 0)
//			width++;
//		else
//			break;
//	}
//
//	return cv::Rect(cu, cv, width, height);
//}

cv::Mat CloudImageProjectionGuidedInpaintingMultiScale(
		std::vector<PointXYZIS> &cloud, cv::Mat const &R, cv::Mat const &T,
		cv::Mat const &K, cv::Mat &img, const cv::Mat &mask,
		cv::Mat &inpainted_init) {

	cv::Mat output_img = img.clone();
	output_img.setTo(0, mask);

//project pointcloud into inpainting area
	cv::Mat debug_img = output_img.clone();
	std::vector<PointXYZIS *> pts_ptr;
	std::vector<float> pts_dist;
	cv::Mat_<float> inbound_pts(0, 2);
	for (auto &p : cloud) {
		cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

		cv::Mat uv = K * (R * v3 + T);
		float fu = uv.at<double>(0) / uv.at<double>(2);
		float fv = uv.at<double>(1) / uv.at<double>(2);
		int u = fu + 0.5;
		int v = fv + 0.5;

		if (uv.at<double>(2) > 0 && 0 <= u && u < img.cols && 0 <= v
				&& v < img.rows && mask.at < uchar > (v, u) > 0) {

			if (p.img_ptr) {
				cv::Mat row = (cv::Mat_<float>(1, 2) << fu, fv);
				inbound_pts.push_back(row);
				pts_ptr.push_back(&p);
				pts_dist.push_back(fabs(uv.at<double>(2)));

				debug_img.at < cv::Vec3b > (v, u) = p.img_ptr->at < cv::Vec3b
						> (p.v, p.u);
			}
		}
	}
	cv::imwrite("/disk1/tmp/proj.jpg", debug_img);

//downsize images
	cv::Rect mask_dim = GetMaskDim(mask);
	float ds_scale = float(PATCH_SIZE * 5)
			/ std::max(mask_dim.width, mask_dim.height);
	if (ds_scale > 1.f)
		ds_scale = 1.f;
	cv::Mat ds_mask, ds_output_img;
	cv::resize(mask, ds_mask, cv::Size(), ds_scale, ds_scale,
			cv::INTER_NEAREST);
	cv::resize(output_img, ds_output_img, cv::Size(), ds_scale, ds_scale,
			cv::INTER_NEAREST);
//ds_output_img.setTo(0, ds_mask);

	cv::Mat ds_inbound_pts = inbound_pts * ds_scale;
	cv::flann::Index flann_index(ds_inbound_pts, cv::flann::KDTreeIndexParams(),
			cvflann::FLANN_DIST_EUCLIDEAN);
	cv::Rect ds_img_rect(0, 0, ds_output_img.cols, ds_output_img.rows);
	cv::Mat ds_mask_inv = ~ds_mask;
	cv::cvtColor(ds_mask_inv, ds_mask_inv, cv::COLOR_GRAY2BGR);

//inpaint downsized image by patch searching
	std::vector < cv::Point2i > filling_pix = GetBoundaryPixelInward(ds_mask);
	int cnt = 0;
	std::vector<cv::Mat *> img_ptrs;
	std::vector<double> img_scales;
	std::vector < cv::Rect > img_rois;
	for (auto pix : filling_pix) {

		cv::Mat query = (cv::Mat_<float>(1, 2) << pix.x, pix.y);
		cv::Mat indices, dists;
		int K = 10;
		flann_index.knnSearch(query, indices, dists, K);

		int closest_idx = 0;
		float min_diff_to_1 = FLT_MAX;
		for (int n = 0; n < K; n++) {
			int cur_ind = indices.at<int>(n);
			float cur_dist = pts_dist[cur_ind];
			auto &pt_ptr = pts_ptr[cur_ind];
			float diff_to_1 = std::max(cur_dist,
					(float) pt_ptr->to_img_distance)
					/ std::min(cur_dist, (float) pt_ptr->to_img_distance);
			if (diff_to_1 < min_diff_to_1) {
				min_diff_to_1 = diff_to_1;
				closest_idx = n;
			}
		}

		int p_ind = indices.at<int>(closest_idx);
		float pix_dist = sqrt(dists.at<float>(closest_idx));

		auto &pt_ptr = pts_ptr[p_ind];
		float pt_dist = pts_dist[p_ind];
		double img_scaling = pt_ptr->to_img_distance / pt_dist * ds_scale;
		cv::Mat scaled_img;
		cv::resize(*(pt_ptr->img_ptr), scaled_img, cv::Size(), img_scaling,
				img_scaling);

		double center_u = pt_ptr->u * img_scaling;
		double center_v = pt_ptr->v * img_scaling;

		int left = center_u - pix_dist * 3 - PATCH_SIZE * 5;
		int right = center_u + pix_dist * 3 + PATCH_SIZE * 5;
		int top = center_v - pix_dist * 3 - PATCH_SIZE * 5;
		int bottom = center_v + pix_dist * 3 + PATCH_SIZE * 5;

		cv::Rect sub_img_rect(left, top, right - left + 1, bottom - top + 1);
		sub_img_rect = sub_img_rect
				& cv::Rect(0, 0, scaled_img.cols, scaled_img.rows);
		cv::Mat sub_img = scaled_img(sub_img_rect);

		cv::Rect temp_img_rect(pix.x - PATCH_SIZE, pix.y - PATCH_SIZE,
				PATCH_SIZE * 2 + 1, PATCH_SIZE * 2 + 1);
		temp_img_rect = temp_img_rect & ds_img_rect;
		cv::Mat temp_img = ds_output_img(temp_img_rect);
		cv::Mat temp_mask = ds_mask_inv(temp_img_rect);

		cv::Mat result;
		cv::matchTemplate(sub_img, temp_img, result, CV_TM_SQDIFF, temp_mask);

		double minVal, maxVal;
		cv::Point minLoc, maxLoc;
		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

		int src_u = minLoc.x + PATCH_SIZE;
		int src_v = minLoc.y + PATCH_SIZE;

		ds_output_img.at < cv::Vec3b > (pix.y, pix.x) = sub_img.at < cv::Vec3b
				> (src_v, src_u);
		ds_mask_inv.at < cv::Vec3b > (pix.y, pix.x) = cv::Vec3b(255, 255, 255);

		img_ptrs.push_back(pt_ptr->img_ptr);
		img_scales.push_back(img_scaling);
		img_rois.push_back(sub_img_rect);

		if (pix.x == 148 && pix.y == 167) {

			cv::imwrite("/disk1/tmp/mask_inv.jpg", ds_mask_inv);
			cv::imwrite("/disk1/tmp/output_img.jpg", ds_output_img);
			cv::imwrite("/disk1/tmp/scaled_img.jpg", scaled_img);
			cv::imwrite("/disk1/tmp/sub_img.jpg", sub_img);
			cv::imwrite("/disk1/tmp/temp_img.jpg", temp_img);
			cv::imwrite("/disk1/tmp/temp_mask.jpg", temp_mask);
			cv::rectangle(sub_img,
					cv::Rect(minLoc.x, minLoc.y, temp_img.cols, temp_img.rows),
					cv::Scalar(0, 255, 0));
			cv::imwrite("/disk1/tmp/det_img.jpg", sub_img);
			exit(0);
		}

		cnt++;
	}
//cv::imwrite("/disk1/tmp/output.jpg", output_img);
	cv::imwrite("/disk1/tmp/ds_output.jpg", ds_output_img);
	char oname[256];
	for (int iter = 0; iter < 10; iter++) {
		UpdateInpaintingVotingWidth(ds_output_img, filling_pix, img_ptrs,
				img_scales, img_rois, 9 - iter);
		sprintf(oname, "/disk1/tmp/inpainting_iter%04d.jpg", iter);
		cv::imwrite(oname, ds_output_img);
	}

//	filling_pix = GetBoundaryPixelInward(mask);
//	std::vector<cv::Mat *> img_ptrs;
//	std::vector<double> img_scales;
//	std::vector < cv::Rect > img_rois;
//	for (auto pix : filling_pix) {
//		cv::Mat query = (cv::Mat_<float>(1, 2) << pix.x, pix.y);
//		cv::Mat indices, dists;
//		int K = 10;
//		flann_index.knnSearch(query, indices, dists, K);
//
//		int closest_idx = 0;
//		float min_diff_to_1 = FLT_MAX;
//		for (int n = 0; n < K; n++) {
//			int cur_ind = indices.at<int>(n);
//			float cur_dist = pts_dist[cur_ind];
//			auto &pt_ptr = pts_ptr[cur_ind];
//			float diff_to_1 = std::max(cur_dist,
//					(float) pt_ptr->to_img_distance)
//					/ std::min(cur_dist, (float) pt_ptr->to_img_distance);
//			if (diff_to_1 < min_diff_to_1) {
//				min_diff_to_1 = diff_to_1;
//				closest_idx = n;
//			}
//		}
//
//		int p_ind = indices.at<int>(closest_idx);
//		float pix_dist = sqrt(dists.at<float>(closest_idx));
//
//		auto &pt_ptr = pts_ptr[p_ind];
//		float pt_dist = pts_dist[p_ind];
//		double img_scaling = pt_ptr->to_img_distance / pt_dist;
//		int scaled_img_cols = round(img_scaling * pt_ptr->img_ptr->cols);
//		int scaled_img_rows = round(img_scaling * pt_ptr->img_ptr->rows);
//
//		double center_u = pt_ptr->u * img_scaling;
//		double center_v = pt_ptr->v * img_scaling;
//		int left = center_u - pix_dist * 3 - PATCH_SIZE * 5;
//		int right = center_u + pix_dist * 3 + PATCH_SIZE * 5;
//		int top = center_v - pix_dist * 3 - PATCH_SIZE * 5;
//		int bottom = center_v + pix_dist * 3 + PATCH_SIZE * 5;
//
//		cv::Rect sub_img_rect(left, top, right - left + 1, bottom - top + 1);
//		sub_img_rect = sub_img_rect
//				& cv::Rect(0, 0, scaled_img_cols, scaled_img_rows);
//
//		img_ptrs.push_back(pt_ptr->img_ptr);
//		img_scales.push_back(img_scaling);
//		img_rois.push_back(sub_img_rect);
//	}
//
//	cv::Mat us_inpainting;
//	cv::resize(ds_output_img, us_inpainting, output_img.size());
//	us_inpainting.copyTo(output_img, mask);
//
//	inpainted_init = output_img.clone();
//	char oname[256];
//	cv::imwrite("/disk1/tmp/inpainting_init.jpg", output_img);
//	for (int iter = 0; iter < 10; iter++) {
//		UpdateInpaintingVotingWidth(output_img, filling_pix, img_ptrs, img_scales,
//				img_rois, 3);
//	sprintf(oname, "/disk1/tmp/inpainting_iter%04d.jpg", iter);
//	cv::imwrite(oname, output_img);
//	std::cout<<oname<<std::endl;
//	}

	exit(0);
	return output_img;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R) {
	float sy = sqrt(
			R.at<double>(0, 0) * R.at<double>(0, 0)
					+ R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular) {
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	} else {
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return cv::Vec3f(x, y, z);

}

// Calculates rotation matrix given euler angles.
cv::Mat eulerAnglesToRotationMatrix(const cv::Vec3f &theta) {
// Calculate rotation about x axis
	cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta[0]), -sin(
			theta[0]), 0, sin(theta[0]), cos(theta[0]));

// Calculate rotation about y axis
	cv::Mat R_y =
			(cv::Mat_<double>(3, 3) << cos(theta[1]), 0, sin(theta[1]), 0, 1, 0, -sin(
					theta[1]), 0, cos(theta[1]));

// Calculate rotation about z axis
	cv::Mat R_z =
			(cv::Mat_<double>(3, 3) << cos(theta[2]), -sin(theta[2]), 0, sin(
					theta[2]), cos(theta[2]), 0, 0, 0, 1);

// Combined rotation matrix
	cv::Mat R = R_z * R_y * R_x;

	return R;

}

cv::Mat RotationOptimizer(std::vector<PointXYZIS> &cloud, cv::Mat const &R,
		cv::Mat const &T, cv::Mat const &K, cv::Mat &img, const cv::Mat &mask, const float step = 0.001f) {

	int dilation_size = 100;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
			cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
			cv::Point(dilation_size, dilation_size));
	cv::Mat dilated_mask;
	cv::dilate(mask, dilated_mask, element);

	cv::Mat counting_mask = dilated_mask ^ mask; //~mask;
//	const float step = 0.001f;

//	cv::imwrite("../data/pandora_liang/set2/1534313648-1534313657_results/mask.jpg", mask);
//	cv::imwrite("../data/pandora_liang/set2/1534313648-1534313657_results/counting_mask.jpg", counting_mask);

//select useful pts to save time
	std::vector<PointXYZIS> useful_pts;
	for (auto &p : cloud) {
		cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

		cv::Mat uv = K * (R * v3 + T);
		float fu = uv.at<double>(0) / uv.at<double>(2);
		float fv = uv.at<double>(1) / uv.at<double>(2);
		int u = fu + 0.5;
		int v = fv + 0.5;

		if (uv.at<double>(2) > 0 && 0 <= u && u < img.cols && 0 <= v
				&& v < img.rows) {
			if (p.img_ptr) {
				useful_pts.push_back(p);
			}
		}
	}

	cv::Vec3f angles = rotationMatrixToEulerAngles(R);
	double min_cost = DBL_MAX;
	cv::Mat opt_R;
	int mx, my, mz;
	for (int ix = -10; ix <= 10; ix++) {
		for (int iy = -10; iy <= 10; iy++) {
			int iz = 0;
			/*for (int iz = -5; iz <= 5; iz++)*/{

//				std::cout << "ix iy iz: " << ix << ", " << iy << ", " << iz
//						<< std::endl;

				cv::Vec3f new_angles;
				new_angles[0] = angles[0] + ix * step;
				new_angles[1] = angles[1] + iy * step;
				new_angles[2] = angles[2] + iz * step;

				cv::Mat newR = eulerAnglesToRotationMatrix(new_angles);

				cv::Mat proj_img = img.clone();
				proj_img.setTo(cv::Scalar(0, 0, 0), mask);
				cv::Mat zbuf = cv::Mat(img.size(), CV_32F,
						cv::Scalar(10000000000.f));
				cv::Mat zbuf_cost = cv::Mat(img.size(), CV_64F, cv::Scalar(-1));
				for (auto &p : useful_pts) {
					cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

					cv::Mat uv = K * (newR * v3 + T);
					float fu = uv.at<double>(0) / uv.at<double>(2);
					float fv = uv.at<double>(1) / uv.at<double>(2);
					int u = fu + 0.5;
					int v = fv + 0.5;
					float z = uv.at<double>(2);

					if (z > 0 && 0 <= u && u < img.cols && 0 <= v
							&& v < img.rows) {

						bool is_closer = false;
						if (z < zbuf.at<float>(v, u)) {
							zbuf.at<float>(v, u) = z;
							is_closer = true;
						}

						if (is_closer && counting_mask.at < uchar > (v, u) > 0
								&& p.img_ptr) {
							cv::Vec3b c1 = img.at < cv::Vec3b > (v, u);
							cv::Vec3b c2 = p.img_ptr->at < cv::Vec3b
									> (p.v, p.u);

							double cdiff = (fabs(
									(double) c1[0] - (double) c2[0])
									+ fabs((double) c1[1] - (double) c2[1])
									+ fabs((double) c1[2] - (double) c2[2]))
									/ 3;

							zbuf_cost.at<double>(v, u) = cdiff;
						}

//						if (is_closer && mask.at < uchar > (v, u) > 0 && p.img_ptr){
//							proj_img.at < cv::Vec3b > (v, u) = p.img_ptr->at
//																< cv::Vec3b > (p.v, p.u);
//						}
					}
				}

				double ave_cost = 0;
				int pix_cnt = 0;
				for (int r = 0; r < zbuf_cost.rows; r++) {
					for (int c = 0; c < zbuf_cost.cols; c++) {
						if (zbuf_cost.at<double>(r, c) > 0) {
							ave_cost += zbuf_cost.at<double>(r, c);
							pix_cnt++;
						}
					}
				}
				ave_cost /= pix_cnt;

				if (ave_cost < min_cost) {
					min_cost = ave_cost;
					opt_R = newR.clone();

//					mx = ix;
//					my = iy;
//					mz = iz;
//					std::cout << "ave cost: " << ave_cost << std::endl;
//					char name[512];
//					sprintf(name,
//							"../data/pandora_liang/set2/1534313648-1534313657_results/img%f_%d_%d_%d.jpg", ave_cost,
//							ix, iy, iz);
//					cv::imwrite(name, proj_img);
				}
			}
		}
	}

	return opt_R;
}

cv::Mat CloudImageProjectionGuidedInpaintingMultiScale1(
		std::vector<PointXYZIS> &cloud, cv::Mat const &R, cv::Mat const &T,
		cv::Mat const &K, cv::Mat &img, const cv::Mat &mask,
		cv::Mat &inpainted_init, std::map<double, cv::Mat> &pts2img_proj_mats,
		double ts) {

	cv::Mat PMat;
	cv::hconcat(R, T, PMat);
	PMat = K * PMat;
	cv::Mat full_local_depth = ImageLocalDepth(cloud, PMat, img);

	cv::Mat output_img = img.clone();
//output_img.setTo(0, mask);

//project pointcloud into inpainting area
	cv::Mat debug_img = output_img.clone();
	debug_img.setTo(0, mask);
	std::vector<PointXYZIS *> pts_ptr;
	std::vector<float> pts_dist;
	cv::Mat_<float> inbound_pts(0, 2);

	std::vector<PointXYZIS *> inimage_pts_ptr;
	std::vector<float> inimage_pts_dist;
	cv::Mat_<float> inimage_pts(0, 2);
	for (auto &p : cloud) {
		cv::Mat v3 = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);

		cv::Mat uv = K * (R * v3 + T);
		float fu = uv.at<double>(0) / uv.at<double>(2);
		float fv = uv.at<double>(1) / uv.at<double>(2);
		int u = fu + 0.5;
		int v = fv + 0.5;

		bool within_img = uv.at<double>(2) > 0 && 0 <= u && u < img.cols
				&& 0 <= v && v < img.rows;

		if (within_img) {
			if (p.img_ptr) {
				cv::Mat row = (cv::Mat_<float>(1, 2) << fu, fv);
				inimage_pts.push_back(row);
				inimage_pts_ptr.push_back(&p);
				inimage_pts_dist.push_back(fabs(uv.at<double>(2)));
			}
		}

		if (within_img && mask.at < uchar > (v, u) > 0) {
			if (p.img_ptr) {
				cv::Mat row = (cv::Mat_<float>(1, 2) << fu, fv);
				inbound_pts.push_back(row);
				pts_ptr.push_back(&p);
				pts_dist.push_back(fabs(uv.at<double>(2)));

				debug_img.at < cv::Vec3b > (v, u) = p.img_ptr->at < cv::Vec3b
						> (p.v, p.u);
			}
		}
	}

//cv::imwrite("/disk1/tmp/proj.jpg", debug_img);

//iterate all inpainting rects
	inpainted_init = img.clone();
	cv::Mat remaining_mask = mask.clone();
	int idx_part = -1;
	while (true) {
		cv::Mat last_inpainted;
		bool looping = true;
		int iter = -1;

		cv::Rect mask_dim = GetMaskDim(remaining_mask);

		if (mask_dim.x == -1) {
			break;
		}

		idx_part++;

		//iterate all scales
		while (looping) {
			iter++;
			//downsize images

			float ds_scale;
			ds_scale = float(20 * 5 * pow(1.5, iter))
					/ std::max(mask_dim.width, mask_dim.height);
			if (ds_scale >= 1.f) {
				ds_scale = 1.f;
				looping = false;
			}
			cv::Mat ds_mask, ds_output_img, ds_dmap, ds_local_depth;
			cv::resize(remaining_mask, ds_mask, cv::Size(), ds_scale, ds_scale,
					cv::INTER_NEAREST);
			cv::resize(output_img, ds_output_img, cv::Size(), ds_scale,
					ds_scale, cv::INTER_LINEAR);
			cv::resize(time2depth[ts], ds_dmap, cv::Size(), ds_scale, ds_scale,
					cv::INTER_LINEAR);
			cv::resize(full_local_depth, ds_local_depth, cv::Size(), ds_scale, ds_scale,
								cv::INTER_LINEAR);
			ds_output_img.setTo(0, ds_mask);
			cv::Mat proj_pts_array(ds_output_img.size(), CV_32FC4,
					cv::Scalar(-1, -1, -1, -1));

			cv::Mat ds_inbound_pts = inbound_pts * ds_scale;
			cv::Mat ds_inimage_pts = inimage_pts * ds_scale;
			cv::flann::Index flann_index(ds_inbound_pts,
					cv::flann::KDTreeIndexParams(),
					cvflann::FLANN_DIST_EUCLIDEAN);
			cv::Rect ds_img_rect(0, 0, ds_output_img.cols, ds_output_img.rows);
			cv::Mat ds_mask_inv = ~ds_mask;

			cv::Rect ds_mask_dim = GetMaskDim(ds_mask);
			if (iter > 0) {
				cv::Mat tmp;
				cv::resize(last_inpainted, tmp, ds_output_img.size());
				tmp(ds_mask_dim).copyTo(ds_output_img(ds_mask_dim));
				ds_mask_inv = 255;
			}

			//////////////////////////////////////////////////////////////////////////
//			char oname100[512];
//			sprintf(oname100,
//					"../data/pandora_liang/set2/1534313648-1534313657_results/part%04d_scale%04d_upscale.jpg",
//					idx_part, iter);
//			cv::imwrite(oname100, ds_output_img);
			//////////////////////////////////////////////////////////////////////////

//			if (iter == 0) {
//				cv::Mat color_sum(ds_output_img.size(), CV_32FC3,
//						cv::Scalar(0));
//				cv::Mat cnt_sum(ds_output_img.size(), CV_32SC1, cv::Scalar(0));

			cv::Mat zbuf_img(proj_pts_array.size(), CV_32F,
					cv::Scalar(FLT_MAX));
			for (int r = 0; r < ds_inimage_pts.rows; r++) {
				float fu = ds_inimage_pts.at<float>(r, 0);
				float fv = ds_inimage_pts.at<float>(r, 1);

				int u = fu + 0.5;
				int v = fv + 0.5;
				if (0 <= u && u < proj_pts_array.cols && 0 <= v
						&& v < proj_pts_array.rows) {
					if (inimage_pts_dist[r] < zbuf_img.at<float>(v, u)) {
						zbuf_img.at<float>(v, u) = inimage_pts_dist[r];
						proj_pts_array.at < cv::Vec4f > (v, u) = cv::Vec4f(1,
								inimage_pts_ptr[r]->x, inimage_pts_ptr[r]->y,
								inimage_pts_ptr[r]->z);
					}
				}
			}

			cv::Mat zbuf(ds_output_img.size(), CV_64F, cv::Scalar(DBL_MAX));
			for (int r = 0; r < ds_inbound_pts.rows; r++) {
				float fu = ds_inbound_pts.at<float>(r, 0);
				float fv = ds_inbound_pts.at<float>(r, 1);
				int u = fu + 0.5;
				int v = fv + 0.5;

				if (0 <= u && u < ds_mask.cols && 0 <= v && v < ds_mask.rows
						&& ds_mask.at < uchar > (v, u) > 0) {

					if (pts_dist[r] < zbuf.at<double>(v, u)) {
						auto &img_ptr = pts_ptr[r]->img_ptr;
						ds_output_img.at < cv::Vec3b > (v, u) = img_ptr->at
								< cv::Vec3b > (pts_ptr[r]->v, pts_ptr[r]->u);
						ds_mask_inv.at < uchar > (v, u) = 255;
						zbuf.at<double>(v, u) = pts_dist[r];

//						proj_pts_array.at < cv::Vec4f > (v, u) = cv::Vec4f(1,
//								pts_ptr[r]->x, pts_ptr[r]->y, pts_ptr[r]->z);
					}

//						color_sum.at < cv::Vec3f > (v, u) =
//								color_sum.at < cv::Vec3f> (v, u) +
//								cv::Vec3f(img_ptr->at < cv::Vec3b> (pts_ptr[r]->v, pts_ptr[r]->u));
//						cnt_sum.at<int>(v, u) = cnt_sum.at<int>(v, u) + 1;
				}
			}

			//////////////////////////////////////////////////////////////////////////
//			sprintf(oname100,
//					"../data/pandora_liang/set2/1534313648-1534313657_results/part%04d_scale%04d_ptproj.jpg",
//					idx_part, iter);
//			cv::imwrite(oname100, ds_output_img);
			//////////////////////////////////////////////////////////////////////////

//			if (ds_scale == 1.f){
//				ds_output_img(ds_mask_dim).copyTo(inpainted_init(ds_mask_dim));
//			}

//				for (int r = 0; r < ds_output_img.rows; r++) {
//					for (int c = 0; c < ds_output_img.cols; c++) {
//						if (cnt_sum.at<int>(r, c) > 0) {
//							ds_output_img.at < cv::Vec3b > (r, c)[0] =
//									color_sum.at < cv::Vec3f
//											> (r, c)[0] / cnt_sum.at<int>(r, c);
//							ds_output_img.at < cv::Vec3b > (r, c)[1] =
//									color_sum.at < cv::Vec3f
//											> (r, c)[1] / cnt_sum.at<int>(r, c);
//							ds_output_img.at < cv::Vec3b > (r, c)[2] =
//									color_sum.at < cv::Vec3f
//											> (r, c)[2] / cnt_sum.at<int>(r, c);
//						}
//					}
//				}
//			} else {
//				cv::Mat tmp;
//				cv::resize(last_inpainted, tmp, ds_output_img.size());
//				cv::Rect ds_mask_dim = GetMaskDim(ds_mask);
//				tmp(ds_mask_dim).copyTo(ds_output_img(ds_mask_dim));
//				ds_mask_inv = 255;
//			}

			//char oname1[256];
			//sprintf(oname1, "/disk1/tmp1/proj_s%04d.jpg", iter);
			//cv::imwrite(oname1, ds_output_img);
			//cv::imwrite("/disk1/tmp/ds_mask_inv.jpg", ds_mask_inv);
			//cv::imwrite("/disk1/tmp/ds_output_img.jpg", ds_output_img);
			//exit(0);

			cv::cvtColor(ds_mask_inv, ds_mask_inv, cv::COLOR_GRAY2BGR);

			//inpaint downsized image by patch searching
			std::vector < cv::Point2i > filling_pix = GetBoundaryPixelInward(
					ds_mask);
			int cnt = 0;
			std::vector<cv::Mat *> img_ptrs;
			std::vector<double> img_scales;
			std::vector < cv::Rect > img_rois;
			std::vector < cv::Mat > img_proj_mats;

			//for (int l = 0; l < 10; l++) {
			//	cv::Mat ds_output_img_holder(ds_output_img.size(), CV_8UC3);
			for (auto pix : filling_pix) {
				cv::Mat query = (cv::Mat_<float>(1, 2) << pix.x, pix.y);
				cv::Mat indices, dists;
				int K = 20;
				flann_index.knnSearch(query, indices, dists, K);

				std::map<cv::Mat *, int> unique_imgs;
//				for (int n = 0; n < K; n++) {
//					int cur_ind = indices.at<int>(n);
//					auto &pt_ptr = pts_ptr[cur_ind];
//					if (unique_imgs.find(pt_ptr->img_ptr)
//							== unique_imgs.end()) {
//						unique_imgs[pt_ptr->img_ptr] = n;
//					}
//				}
				float min_dist_diff = FLT_MAX;
				int min_n =0;
				PointXYZIS *min_pt_ptr;
				for (int n = 0; n < K; n++) {
					int cur_ind = indices.at<int>(n);
					auto &pt_ptr = pts_ptr[cur_ind];
					float dist_cur_img = pts_dist[cur_ind];
					float dist_diff = fabs(pt_ptr->to_img_distance - dist_cur_img);
					if (dist_diff < min_dist_diff){
						min_dist_diff = dist_diff;
						min_n = n;
						min_pt_ptr = pt_ptr;
					}
				}
				unique_imgs[min_pt_ptr->img_ptr] = min_n;


				double min_matching_val = DBL_MAX;
				double max_matching_val = -1e20;
				cv::Mat *best_ptr;
				cv::Mat best_proj_mat;
				double best_scale;
				cv::Rect best_rect;
				cv::Mat best_sub_img, best_temp_img, best_temp_mask;
				cv::Point best_minLoc;
				for (auto const &ui : unique_imgs) {

					int p_ind = indices.at<int>(ui.second);
					float pix_dist = sqrt(dists.at<float>(ui.second));

					auto &pt_ptr = pts_ptr[p_ind];
					float pt_dist = pts_dist[p_ind];
					double img_scaling = pt_ptr->to_img_distance / pt_dist
							* ds_scale;
					cv::Mat scaled_img;
					cv::resize(*(pt_ptr->img_ptr), scaled_img, cv::Size(),
							img_scaling, img_scaling);

					double center_u = pt_ptr->u * img_scaling;
					double center_v = pt_ptr->v * img_scaling;

					const float PATCH_TIMES = 2.2 /*5*/, DIST_TIMES = 1 /*3*/;
					int left = center_u - pix_dist * DIST_TIMES
							- PATCH_SIZE * PATCH_TIMES;
					int right = center_u + pix_dist * DIST_TIMES
							+ PATCH_SIZE * PATCH_TIMES;
					int top = center_v - pix_dist * DIST_TIMES
							- PATCH_SIZE * PATCH_TIMES;
					int bottom = center_v + pix_dist * DIST_TIMES
							+ PATCH_SIZE * PATCH_TIMES;

					cv::Rect sub_img_rect(left, top, right - left + 1,
							bottom - top + 1);
					sub_img_rect = sub_img_rect
							& cv::Rect(0, 0, scaled_img.cols, scaled_img.rows);
					cv::Mat sub_img = scaled_img(sub_img_rect);

					cv::Rect temp_img_rect(pix.x - PATCH_SIZE,
							pix.y - PATCH_SIZE, PATCH_SIZE * 2 + 1,
							PATCH_SIZE * 2 + 1);
					temp_img_rect = temp_img_rect & ds_img_rect;
					cv::Mat temp_img = ds_output_img(temp_img_rect);
					cv::Mat temp_mask = ds_mask_inv(temp_img_rect);

					cv::Mat result;
					cv::matchTemplate(sub_img, temp_img, result, CV_TM_SQDIFF,
							temp_mask);

					double minVal, maxVal;
					cv::Point minLoc, maxLoc;
					cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc,
							cv::Mat());

					if (minVal < min_matching_val) {
						//std::cout<<min_matching_val<<" replaced with "<<minVal<<std::endl;
						min_matching_val = minVal;

						int src_u = minLoc.x + PATCH_SIZE;
						int src_v = minLoc.y + PATCH_SIZE;

						//if (l == 0){
						if (ds_mask_inv.at < cv::Vec3b > (pix.y, pix.x)[0]
								== 0) {
							ds_output_img.at < cv::Vec3b > (pix.y, pix.x) =
									sub_img.at < cv::Vec3b > (src_v, src_u);
							ds_mask_inv.at < cv::Vec3b > (pix.y, pix.x) =
									cv::Vec3b(255, 255, 255);
						}
						//}else{
						//	ds_output_img_holder.at < cv::Vec3b > (pix.y, pix.x) =
						//									sub_img.at < cv::Vec3b > (src_v, src_u);
						//}

						best_ptr = pt_ptr->img_ptr;
						best_scale = img_scaling;
						best_rect = sub_img_rect;
						best_proj_mat =
								pts2img_proj_mats[pt_ptr->img_frame_time];

						best_sub_img = sub_img.clone();
						best_temp_img = temp_img.clone();
						best_temp_mask = temp_mask.clone();
						best_minLoc = minLoc;
					}
				}

				img_ptrs.push_back(best_ptr);
				img_scales.push_back(best_scale);
				img_rois.push_back(best_rect);
				img_proj_mats.push_back(best_proj_mat);

//		if (pix.x == 53 && pix.y == 91) {
//
//			cv::imwrite("/disk1/tmp/mask_inv.jpg", ds_mask_inv);
//			cv::imwrite("/disk1/tmp/output_img.jpg", ds_output_img);
//			//		cv::imwrite("/disk1/tmp/scaled_img.jpg", scaled_img);
//			cv::imwrite("/disk1/tmp/sub_img.jpg", best_sub_img);
//			cv::imwrite("/disk1/tmp/temp_img.jpg", best_temp_img);
//			cv::imwrite("/disk1/tmp/temp_mask.jpg", best_temp_mask);
//			cv::rectangle(best_sub_img,
//					cv::Rect(best_minLoc.x, best_minLoc.y, best_temp_img.cols,
//							best_temp_img.rows), cv::Scalar(0, 255, 0));
//			cv::imwrite("/disk1/tmp/det_img.jpg", best_sub_img);
//			exit(0);
//		}
			}

			//////////////////////////////////////////////////////////////////////////
//			sprintf(oname100,
//					"../data/pandora_liang/set2/1534313648-1534313657_results/part%04d_scale%04d_ptfill.jpg",
//					idx_part, iter);
//			cv::imwrite(oname100, ds_output_img);
			//////////////////////////////////////////////////////////////////////////

			if (ds_scale == 1.f) {
				ds_output_img(ds_mask_dim).copyTo(inpainted_init(ds_mask_dim));
			}

			//if (l > 0){
			//for (auto pix : filling_pix){
			//	ds_output_img.at < cv::Vec3b > (pix.y, pix.x)=ds_output_img_holder.at < cv::Vec3b > (pix.y, pix.x);
			//}
			//}

//			char oname[256];
//			inpainted_init = ds_output_img.clone();
//			sprintf(oname, "/disk1/tmp1/inpainting_init_s%04d.jpg", iter);
			//cv::imwrite(oname, inpainted_init);
			//}

			//exit(1);
//	cv::imwrite("/disk1/tmp/ds_output.jpg", ds_output_img);

			char oname0[512];
			sprintf(oname0,
					"../data/pandora_liang/set2/1534313570-1534313581_results/input_part%04d_scale%04d.jpg",
					idx_part, iter);
			cv::imwrite(oname0, ds_output_img);
			cv::Mat scaled_K = K.clone();
			scaled_K.row(0) = scaled_K.row(0) * ds_scale;
			scaled_K.row(1) = scaled_K.row(1) * ds_scale;
			for (int it = 0; it < 10; it++) {
//				UpdateInpaintingVotingWidth(ds_output_img, filling_pix,
//						img_ptrs, img_scales, img_rois, 1);
//				UpdateInpaintingVotingWidthDepth(ds_output_img, ds_dmap,
//						filling_pix, img_ptrs, img_scales, img_rois, 1);
//				UpdateInpaintingVotingWidthDepthWarpedPatch(ds_output_img, ds_dmap,
//						ds_local_depth, filling_pix, img_ptrs, img_scales, img_rois,
//						img_proj_mats, 1, scaled_K, R, T, ds_mask_dim);
				UpdateInpaintingByDenseSampling(ds_output_img, ds_dmap,
						ds_local_depth, filling_pix, img_ptrs, img_scales, img_rois,
						img_proj_mats, 1, scaled_K, R, T, ds_mask_dim);
//				UpdateInpaintingVotingWidthWarpedTemplFixPix(ds_output_img,
//						proj_pts_array, filling_pix, img_ptrs, img_proj_mats,
//						img_scales, img_rois, 10, ds_mask_dim, 0.01 * (it + 1));
//				UpdateInpaintingVotingWidthWarpedTemplByDMapFixPix(ds_output_img,
//						proj_pts_array, filling_pix, img_ptrs, img_proj_mats,
//						img_scales, img_rois, 10, ds_mask_dim, 0.01 * (it + 1));
//				UpdateInpaintingInvWarpedTemplFixPix(ds_output_img,
//										proj_pts_array, filling_pix, img_ptrs, img_proj_mats,
//										img_scales, img_rois, 1, ds_mask_dim, 1000.0 * (it + 1));
				char oname[512];
				sprintf(oname,
						"../data/pandora_liang/set2/1534313570-1534313581_results/part%04d_scale%04d_iter%04d.jpg",
						idx_part, iter, it);
				cv::imwrite(oname, ds_output_img);
			}
			exit(1);

//			InpaintingNNFUpdateWarpedTempl(ds_output_img, proj_pts_array,
//					filling_pix, img_ptrs, img_proj_mats, img_scales, img_rois,
//					10);

			last_inpainted = ds_output_img.clone();

			//////////////////////////////////////////////////////////////////////////
//			sprintf(oname100,
//					"../data/pandora_liang/set2/1534313648-1534313657_results/part%04d_scale%04d_voteave.jpg",
//					idx_part, iter);
//			cv::imwrite(oname100, ds_output_img);
			//////////////////////////////////////////////////////////////////////////
		}

		remaining_mask(mask_dim) = 0;
		output_img = last_inpainted;
	}

//	exit(1);
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
	std::vector < cv::Point2f > tex_coords;
	cv::Mat *texture = NULL;

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

			if (mask.at < uchar > (v, u) > 0) {
				pts.emplace_back(x, y);
				tex_coords.emplace_back(p.u, p.v);
				texture = p.img_ptr;
			}
		}
	}

	if (texture == NULL)
		return output_img;

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

			lx = lx >= 0 ? lx : 0;
			rx = rx < img.cols ? rx : img.cols - 1;
			uy = uy >= 0 ? uy : 0;
			by = by < img.rows ? by : img.rows - 1;

			for (int y = uy; y <= by; y++) {
				for (int x = lx; x <= rx; x++) {

					float w1, w2, w3;
					Barycentric(cv::Point2f(x, y), cv::Point2f(t.p1.x, t.p1.y),
							cv::Point2f(t.p2.x, t.p2.y),
							cv::Point2f(t.p3.x, t.p3.y), w1, w2, w3);
					if (w1 > 0 && w2 > 0 && w3 > 0) {
						cv::Point2f tc = w1 * tex_coords[t.p1.idx]
								+ w2 * tex_coords[t.p2.idx]
								+ w3 * tex_coords[t.p3.idx];
						int tex_u = int(tc.x + 0.5);
						int tex_v = int(tc.y + 0.5);
						if (0 <= tex_u && tex_u < texture->cols && 0 <= tex_v
								&& tex_v < texture->rows) {
							output_img.at < cv::Vec3b > (y, x) = texture->at
									< cv::Vec3b > (tex_v, tex_u);
						}
					}
				}
			}
		}
	}

	return output_img;
}

void SearchAreaAssociation(const cv::Mat &mask,
		const std::vector<PointXYZIS> &cloud, cv::Mat const &R,
		cv::Mat const &T, cv::Mat const &K) {
}

cv::Mat ImageGlobalDepth(const std::vector<PointXYZIS> &cloud,
		const cv::Mat &proj_mat, const cv::Mat &img) {

	cv::Mat dmap(img.size(), CV_32F, cv::Scalar(FLT_MAX));

//	cv::Mat output_img = img.clone();
//	output_img = 0;
	cv::Mat z_buffer(img.size(), CV_32F, cv::Scalar(1000000));

	for (auto &p : cloud) {
		cv::Mat v3 = (cv::Mat_<double>(4, 1) << p.x, p.y, p.z, 1);

		cv::Mat uv = proj_mat * v3;
		int u = uv.at<double>(0) / uv.at<double>(2) + 0.5;
		int v = uv.at<double>(1) / uv.at<double>(2) + 0.5;

		if (uv.at<double>(2) > 0 && 0 <= u && u < img.cols && 0 <= v
				&& v < img.rows) {
			float p_dist = uv.at<double>(2);

			if (p_dist < z_buffer.at<float>(v, u)) {
				z_buffer.at<float>(v, u) = p_dist;
//				cv::circle(output_img, cv::Point(u, v), 2, ColorMapping(p_dist),
//						-1);
				dmap.at<float>(v, u) = -p.x;
			}
		}
	}

//	cv::Mat output_img0(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
//	for (int x = 0; x < dmap.cols; x++) {
//		for (int y = 0; y < dmap.rows; y++) {
//			if (dmap.at<float>(y, x) < FLT_MAX) {
//				cv::Scalar c = ColorMapping(dmap.at<float>(y, x));
//				output_img0.at < cv::Vec3b > (y, x) = cv::Vec3b(c[0], c[1],
//						c[2]);
//			}
//		}
//	}
//	imwrite(
//			"../data/pandora_liang/set2/1534313570-1534313581_results/oimg_sparse.jpg",
//			output_img0);

	//cv::Mat output_img;
	//cv::normalize(dmap, output_img, 0, 255, cv::NORM_MINMAX, CV_8U);
	//	cv::cvtColor(output_img, output_img, CV_GRAY2BGR);
	//applyColorMap(output_img, output_img, cv::COLORMAP_JET);
	//imwrite("../data/pandora_liang/set2/1534313570-1534313581_results/oimg_sparse.jpg", output_img);

	cv::Mat_<float> valid_pts(0, 2);
	for (int x = 0; x < dmap.cols; x++) {
		for (int y = 0; y < dmap.rows; y++) {
			if (dmap.at<float>(y, x) < FLT_MAX) {
				cv::Mat row = (cv::Mat_<float>(1, 2) << x, y);
				valid_pts.push_back(row);
			}
		}
	}
	cv::flann::Index flann_index(valid_pts, cv::flann::KDTreeIndexParams(),
			cvflann::FLANN_DIST_EUCLIDEAN);

	cv::Mat indices, dists;
	int K = 10;
	for (int x = 0; x < dmap.cols; x++) {
		for (int y = 0; y < dmap.rows; y++) {
			if (dmap.at<float>(y, x) == FLT_MAX) {
				cv::Mat query = (cv::Mat_<float>(1, 2) << x, y);

				flann_index.knnSearch(query, indices, dists, K);

				float ave_d = 0;
				float sum_dist = 0;
				for (int i = 0; i < K; i++) {
					int ind = indices.at<int>(i);
					float dist = sqrt(dists.at<float>(i));
					int sx = valid_pts.at<float>(ind, 0);
					int sy = valid_pts.at<float>(ind, 1);
					ave_d += dmap.at<float>(sy, sx) * (1.f / dist);
					sum_dist += 1.f / dist;
				}

				ave_d /= sum_dist;
				dmap.at<float>(y, x) = ave_d;
			}
		}
	}

//	std::cout<<dmap.row(400)<<std::endl;

//	cv::normalize(dmap, output_img, 0, 255, cv::NORM_MINMAX, CV_8U);
//	cv::cvtColor(output_img, output_img, CV_GRAY2BGR);
//	applyColorMap(output_img, output_img, cv::COLORMAP_JET);
//	imwrite("../data/pandora_liang/set2/1534313570-1534313581_results/oimg_dense.jpg", output_img);

//	cv::Mat output_img(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
//	for (int x = 0; x < dmap.cols; x++) {
//		for (int y = 0; y < dmap.rows; y++) {
//			cv::Scalar c = ColorMapping(dmap.at<float>(y, x));
//			output_img.at < cv::Vec3b > (y, x) = cv::Vec3b(c[0], c[1], c[2]);
//		}
//	}

//	cv::Mat output_img1(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
//	for (int x = 0; x < dmap.cols; x++) {
//		for (int y = 0; y < dmap.rows; y++) {
//			if (dmap.at<float>(y, x) < FLT_MAX) {
//				cv::Scalar c = ColorMapping(dmap.at<float>(y, x));
//				output_img1.at < cv::Vec3b > (y, x) = cv::Vec3b(c[0], c[1],
//						c[2]);
//			}
//		}
//	}
//	imwrite(
//			"../data/pandora_liang/set2/1534313570-1534313581_results/oimg_dense.jpg",
//			output_img1);
//
//	exit(1);

//	output_img1 = output_img1 + img / 3;

	return dmap;
}

/** Main node entry point. */
double min_time = 1534313790, max_time = 1534313828;
char pose_folder[512];
const int NUM_THREADS = 1;
std::vector<bool> is_idle(NUM_THREADS, true);
int main(int argc, char **argv) {

    double start_ts = 0;
    if (argc<5) {
        std::cout<<"exe <start_time> <end_time> <skip_time> <working_folder>"<<std::endl;
        exit(0);
    }
    min_time = std::stoll(argv[1]);
    max_time = std::stoll(argv[2]);
    start_ts = std::stod(argv[3]);

	sprintf(pose_folder, "%s/%llu-%llu", argv[4],
			(long long) (min_time), (long long) (max_time));

	std::map<std::string, loam::Twist> rel_poses = ReadPose(
			std::string(pose_folder) + "/rel_poses.txt");
	std::map<std::string, loam::Twist> glb_poses = ReadPose(
			std::string(pose_folder) + "/global_poses.txt");

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
	std::string img_folder = std::string(argv[4])+"/image/";
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
	std::map<double, cv::Mat> dilated_mask_time_to_cvmat;
	char mask_path[512];
	sprintf(mask_path, "%s/%llu-%llu_mask/", argv[4],
			(long long) (min_time), (long long) (max_time));
	std::string mask_folder(mask_path); //= "../data/pandora_liang/set2/mask/";
	std::vector < std::string > mask_file_list;
	getdir(mask_folder, mask_file_list);
	cv::Mat element = getStructuringElement(cv::MORPH_RECT,
			cv::Size(2 * PATCH_SIZE + 1, 2 * PATCH_SIZE + 1),
			cv::Point(PATCH_SIZE, PATCH_SIZE));

	int img_w, img_h;
	for (auto const &file_name : mask_file_list) {
		int str_len = file_name.length();
		if (str_len > 10 && file_name.substr(str_len - 3, 3) == "png") {
			std::string time_str = file_name.substr(0, 17);
			double ts = std::stod(time_str);
			mask_time_to_cvmat[ts] = imread(mask_folder + file_name,
					cv::IMREAD_GRAYSCALE);
			img_w = mask_time_to_cvmat[ts].cols;
			img_h = mask_time_to_cvmat[ts].rows;
			cv::dilate(mask_time_to_cvmat[ts], dilated_mask_time_to_cvmat[ts],
					element);
		}
	}


//    char flow_path[512];
//    sprintf(flow_path, "../data/pandora_liang/set2/%llu-%llu_flow/",
//            (long long) (min_time), (long long) (max_time));
//    std::string flow_folder(flow_path); //= "../data/pandora_liang/set2/mask/";
//    std::vector < std::string > flow_file_list;
//    getdir(flow_folder, flow_file_list);
//    std::map<double, cv::Mat> forward_flow;
//    std::map<double, cv::Mat> backward_flow;
//    for (auto const &file_name : flow_file_list) {
//        int str_len = file_name.length();
//        if (str_len > 10 && file_name.substr(str_len - 11, 11) == "forward.npy") {
//            std::string time_str = file_name.substr(0, 17);
//            double ts = std::stod(time_str);
//            cnpy::NpyArray arr = cnpy::npy_load(flow_folder + file_name);
//            uchar* loaded_data = arr.data<uchar>();
//            cv::Mat uv_map(img_h, img_w, CV_32FC2, loaded_data);
//            int from_to[] = { 0, 0, 1, 1 };
//            cv::Mat uvc3(uv_map.size(), CV_32FC3);
//            cv::mixChannels(&uv_map, 1, &uvc3, 1, from_to, 2);
//            time2forwardflow[ts] = uvc3;
//        }
//
//        if (str_len > 10 && file_name.substr(str_len - 11, 11) == "ackward.npy") {
//            std::string time_str = file_name.substr(0, 17);
//            double ts = std::stod(time_str);
//            cnpy::NpyArray arr = cnpy::npy_load(flow_folder + file_name);
//            uchar* loaded_data = arr.data<uchar>();
//            cv::Mat uv_map(img_h, img_w, CV_32FC2, loaded_data);
//            int from_to[] = { 0, 0, 1, 1 };
//            cv::Mat uvc3(uv_map.size(), CV_32FC3);
//            cv::mixChannels(&uv_map, 1, &uvc3, 1, from_to, 2);
//
//            time2backwardflow[ts] = uvc3;
//        }
//    }


	std::string lidar_folder = std::string(argv[4])+"/lidar_bg/";
	std::vector < std::string > file_list;
	getdir(lidar_folder, file_list);
	std::sort(file_list.begin(), file_list.end());

	std::map<double, loam::Twist> lidar_endtime_to_pose;
	std::map<double, loam::Twist> lidar_endtime_to_relpose;
	std::map<double, loam::Twist> lidar_starttime_to_pose;
	std::map<double, loam::Twist> lidar_starttime_to_relpose;

	int frm_cnt = 0;
	for (auto const &file_name : file_list) {
		if (file_name.at(0) == 'b') {

			double lidar_end_time = std::stod(file_name.substr(3, 17));
			double lidar_start_time = lidar_end_time - 0.1;

			if (min_time < lidar_end_time && lidar_end_time < max_time) {

                auto closest_img_time = FindClosest(img_time_to_path,
                                                    lidar_start_time);
                if (mask_time_to_cvmat.find(closest_img_time) == mask_time_to_cvmat.end())
                    continue;

				frm_cnt++;
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
						> (lidar_folder + file_name, *tmp) == -1)//* load the file
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
						/*dilated_mask_time_to_cvmat[closest_img_time]*/mask_time_to_cvmat[closest_img_time]);


				auto rel_ptr = rel_poses.find(file_name);
				auto glb_ptr = glb_poses.find(file_name);

				if (rel_ptr != rel_poses.end() && glb_ptr != glb_poses.end()) {
					++frame_cnt;
					std::cout << frame_cnt << std::endl;
					//WriteToPLY(cloud, "../Documents/motion_orig.ply");
					std::vector<PointXYZIS> filtered_cloud = ScanRegistration(
							cloud);


					transformToEnd(filtered_cloud, rel_ptr->second);
					lidar_endtime_to_relpose[lidar_end_time] = rel_ptr->second;
					lidar_starttime_to_relpose[lidar_start_time] =
							rel_ptr->second;

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


	std::map<double, cv::Mat> orig_img;
	for (auto const &m : img_time_to_cvmat) {
		orig_img[m.first] = m.second.clone();
	}

	for (auto const &m : mask_time_to_cvmat) {
		if (img_time_to_cvmat.find(m.first) != img_time_to_cvmat.end()) {
			img_time_to_cvmat[m.first].setTo(0, m.second);
		}
	}

	cv::Mat Rtlocal = cv::Mat::eye(4, 4, CV_64F);
	R.copyTo(Rtlocal(cv::Rect(0, 0, 3, 3)));
	T.copyTo(Rtlocal(cv::Rect(3, 0, 1, 3)));
	cv::Mat axis_switch =
			(cv::Mat_<double>(4, 4) << 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1);
	std::map<double, cv::Mat> tranf_global2img;
	for (auto &img : img_time_to_cvmat) {
		double closest_lidar_start_time = FindClosest(lidar_starttime_to_pose,
				img.first);
		double time_diff = fabs(closest_lidar_start_time - img.first);

		if (time_diff < 0.05) {
			cv::Mat T2local = TransformToLocalFrameMat(
					lidar_starttime_to_pose[closest_lidar_start_time]);
			cv::Mat T2start = transformToStartFromEndMat(
					lidar_starttime_to_relpose[closest_lidar_start_time]);

			cv::Mat T_global2local = T2start * T2local;
			tranf_global2img[img.first] = T_global2local;

//            cv::Mat curRtlocal = Rtlocal.clone();
//
//            std::string newR_file = std::string(pose_folder) + "/"
//                                    + std::to_string(img.first) + "_optR.xml";
//            cv::FileStorage fs(newR_file, cv::FileStorage::READ);
//            if (fs.isOpened()) {
//                cv::Mat optR;
//                fs["optR"] >> optR;
//                fs.release();
//                optR.copyTo(curRtlocal(cv::Rect(0, 0, 3, 3)));
//            }

//			cv::Mat transf_glob2img = curRtlocal * axis_switch * T_global2local;
//			cv::Mat pmat = K * transf_glob2img(cv::Rect(0, 0, 4, 3));

//			std::string depth_file = std::string(pose_folder) + "/"
//					+ std::to_string(img.first) + "_depth.bin";
//			std::ifstream myfile(depth_file, std::ios::binary);
//			cv::Mat dmap;
//			if (myfile.is_open()) {
//				char *buf = new char[img.second.cols * img.second.rows
//						* sizeof(float)];
//				myfile.read(buf,
//						img.second.cols * img.second.rows * sizeof(float));
//				myfile.close();
//				dmap = cv::Mat(img.second.size(), CV_32F, buf);
//				std::cout << img.first << " depth exists." << std::endl;
//
//			} else {
//				dmap = ImageGlobalDepth(stitched_map, pmat, img.second);
//				std::ofstream outfile(depth_file, std::ios::binary);
//				outfile.write((char *) dmap.data,
//						dmap.cols * dmap.rows * sizeof(float));
//				outfile.close();
//			}
//
//			img2depth[&img.second] = dmap;
//			time2depth[img.first] = dmap;
		}
	}

	std::thread myThreads[NUM_THREADS];
	int img_idx = 0;
	for (auto &img : img_time_to_cvmat) {

		//if (fabs(img.first - 1534313591.4320321083)<0.0001) continue; //this frame cause denaulay algorithm stuck
		std::cout<<"skip ts before "<<std::to_string(start_ts)<<std::endl;
		if (img.first < start_ts)			//1534313593.0
			continue;

		std::cout << std::setprecision(20) << img.first << std::endl;

		if (mask_time_to_cvmat[img.first].empty()) {
            std::cout<<"mask not exists..."<<std::endl;
		    continue;
        }

		double closest_lidar_start_time = FindClosest(lidar_starttime_to_pose,
				img.first);
		double time_diff = fabs(closest_lidar_start_time - img.first);


		if (time_diff > 0.05) {
            std::cout << time_diff << std::endl;
            std::cout << "time diff over thresh" << std::endl;
        }
		else
		{

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
            whole_map.clear();

			std::map<double, cv::Mat> img_proj_mats;
			cv::Mat transf_cur = tranf_global2img[img.first].clone();

			for (auto const &T : tranf_global2img) {

			    cv::Mat curRtlocal = Rtlocal.clone();

                std::string newR_file = std::string(pose_folder) + "/"
                                        + std::to_string(T.first) + "_optR.xml";
                cv::FileStorage fs(newR_file, cv::FileStorage::READ);
                if (fs.isOpened()) {
                    cv::Mat optR;
                    fs["optR"] >> optR;
                    fs.release();
//                    std::cout << "optimized R exists for "<< std::setprecision(20)<<T.first << std::endl;
                    optR.copyTo(curRtlocal(cv::Rect(0, 0, 3, 3)));
                }

				cv::Mat transf_cur2img = curRtlocal * axis_switch * T.second
						* transf_cur.inv() * axis_switch.inv();
				cv::Mat pmat = K * transf_cur2img(cv::Rect(0, 0, 4, 3));
				img_proj_mats[T.first] = pmat.clone();
			}


//			cv::Mat space_time_coords;
			cv::Mat &mask = mask_time_to_cvmat[img.first];
			cv::Mat &inp_img = orig_img[img.first];


            double ts = img.first;
            cv::Mat inp_init;
            std::string newR_file = std::string(pose_folder) + "/"
                                    + std::to_string(ts) + "_optR.xml";
            cv::FileStorage fs(newR_file, cv::FileStorage::READ);
            cv::Mat optR;
            if (fs.isOpened()) {
                fs["optR"] >> optR;
                fs.release();
                std::cout << "optimized R exists..." << std::endl;
            } else {
                std::cout<<"compute optR for "<<std::to_string(ts)<<std::endl;
                cv::Mat optR0 = RotationOptimizer(proj_map, R, T, K, inp_img,
                                         mask);
                optR = RotationOptimizer(proj_map, optR0, T, K, inp_img,
                                          mask, 0.0001f);
                cv::FileStorage fsw(newR_file, cv::FileStorage::WRITE);
                fsw << "optR" << optR;
                fsw.release();
            }

            bool doing_inpainting = true;
            cv::Mat pixel_frame_label;
            if (doing_inpainting) {
                auto show_img = InpaintingByDenseMap2(
                        proj_map, optR, T, K, inp_img, mask, inp_init, img_proj_mats, ts,
                        img_time_to_cvmat, mask_time_to_cvmat, pixel_frame_label);
//            auto show_img = InpaintingByDenseMapBP2(
//                    proj_map, optR, T, K, inp_img, mask, inp_init, img_proj_mats, ts,
//                    img_time_to_cvmat, mask_time_to_cvmat, pixel_frame_label);

                char result_folder[512];
                sprintf(result_folder,
                        "%s/%llu-%llu_results", argv[4],
                        (long long) (min_time), (long long) (max_time));
                mkdir(result_folder,0777);

                cv::imwrite(
                        std::string(result_folder) + "/BP_MSG_inpainted_"
                        + std::to_string(ts) + "_init.jpg",
                        inp_img);

                cv::imwrite(
                        std::string(result_folder) + "/BP_MSG_inpainted_"
                        + std::to_string(ts) + ".jpg", show_img);

                cv::imwrite(
                        std::string(result_folder) + "/BP_MSG_inpainted_"
                        + std::to_string(ts) + "_label.png", pixel_frame_label);
            }
		}
	}

//	for (auto &t : myThreads) {
//		t.join();
//	}

	std::cout << "done" << std::endl;

	return 0;
}
