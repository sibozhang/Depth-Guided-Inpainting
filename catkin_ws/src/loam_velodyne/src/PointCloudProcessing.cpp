#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <fstream>


struct Box {
	std::string _type;
	Eigen::Vector3d _pos;
	Eigen::Vector3d _x_dir;
	Eigen::Vector3d _y_dir;
	Eigen::Vector3d _z_dir;
	double confidence;
	double ori;
	double _x_len;
	double _y_len;
	double _z_len;
};

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

bool load_label(const std::string label_file, std::vector<Box> &boxes) {

	std::ifstream infile(label_file.c_str());
	if (!infile.is_open())
		return false;

	std::string stype;
	Eigen::Vector3d point;

	while (infile >> stype) {
		Box box;
		box._type = stype;

		infile >> box.confidence;
		infile >> box.ori;

		std::vector < Eigen::Vector3d > points;
		for (int n = 0; n < 8; ++n) {
			infile >> point(0) >> point(1) >> point(2);
			points.push_back(point);
		}

		Eigen::Vector3d obs_center(0, 0, 0);
		for (int n = 0; n < 8; ++n) {
			obs_center += points.at(n);
		}
		obs_center /= 8.0;
		box._pos = obs_center;

		box._x_dir = (points.at(1) - points.at(2));
		box._y_dir = (points.at(3) - points.at(2));
		box._z_dir = (points.at(4) - points.at(0));
		box._x_dir.normalize();
		box._y_dir.normalize();
		box._z_dir.normalize();

		Eigen::Vector3d x_vec = points.at(1) - points.at(2);
		Eigen::Vector3d y_vec = points.at(3) - points.at(2);
		Eigen::Vector3d z_vec = points.at(4) - points.at(0);
		box._x_len = x_vec.norm() / 2.0;
		box._y_len = y_vec.norm() / 2.0;
		box._z_len = z_vec.norm() / 2.0;

		boxes.push_back(box);
	}

	return true;
}

bool Contain(const Box &box, pcl::PointXYZI const &p) {
	Eigen::Vector3d cur_vec(p.x, p.y, p.z);
	cur_vec -= box._pos;
	double x = cur_vec.dot(box._x_dir);
	double y = cur_vec.dot(box._y_dir);
	double z = cur_vec.dot(box._z_dir);
	if (x >= -box._x_len && x <= box._x_len && y >= -box._y_len
			&& y <= box._y_len && z >= -box._z_len && z <= box._z_len) {
		return true;
	} else {
		return false;
	}

}

/** Main node entry point. */
int main(int argc, char **argv) {

	std::string pcd_folder = "../data/pandora_liang/set2/lidar/";
	std::string label_folder = "../data/pandora_liang/set2/lidar_result_label/";
	std::vector < std::string > pcd_file_list;
	getdir(pcd_folder, pcd_file_list);
	for (auto const &file_name : pcd_file_list) {

	}

	std::vector < std::string > file_list;
	getdir(pcd_folder, file_list);
	std::sort(file_list.begin(), file_list.end());

	for (auto const &file_name : file_list) {
		if (file_name.length() > 10) {
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
					new pcl::PointCloud<pcl::PointXYZI>);

			if (pcl::io::loadPCDFile < pcl::PointXYZI > (pcd_folder + file_name, *cloud)
					== -1) //* load the file
					{
				PCL_ERROR("Couldn't read pcd file\n");
				return (-1);
			}

			std::cout<<file_name<<std::endl;

			pcl::PointCloud<pcl::PointXYZI> bg_cloud;
			std::string label_file = label_folder + file_name + ".txt";
			std::vector<Box> boxes;
			if ( load_label(label_file, boxes)) {
				std::vector<Box> desired_boxes;
				for (auto const &b : boxes){
//					if (b._type=="pedestrian" && b.confidence>0.01){
//						std::cout<<"ped conf: "<<b.confidence<<std::endl;
//						 desired_boxes.push_back(b);
//					}else if (b._type == "smallMot" && b.confidence>0.01){
//						std::cout<<"smallMot conf: "<<b.confidence<<std::endl;
//						desired_boxes.push_back(b);
//					}else if (b._type == "bigMot" && b.confidence>0.01){
//						std::cout<<"bigMot conf: "<<b.confidence<<std::endl;
//						desired_boxes.push_back(b);
//					}

					if (b.confidence>0.3){
						desired_boxes.push_back(b);
					}
				}

				for (auto const &p : cloud->points){
					bool is_dynamic_obj=false;
					for (auto const &b : desired_boxes){
						if (Contain(b, p)){
							is_dynamic_obj=true;
							break;
						}
					}

					if (!is_dynamic_obj){
						bg_cloud.points.push_back(p);
					}
				}
			}

			bg_cloud.width=bg_cloud.points.size();
			bg_cloud.height=1;

			pcl::PLYWriter writer;
			writer.write("../data/pandora_liang/set2/lidar_bg/orig_" + file_name+".ply", *cloud);
			writer.write("../data/pandora_liang/set2/lidar_bg/bg_" + file_name+".ply", bg_cloud);

		}

	}

	return 0;
}
