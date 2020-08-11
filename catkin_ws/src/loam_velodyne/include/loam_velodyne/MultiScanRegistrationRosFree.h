// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#ifndef LOAM_MULTISCANREGISTRATIONROSFREE_H
#define LOAM_MULTISCANREGISTRATIONROSFREE_H

#include "time_utils.h"
#include "CircularBuffer.h"
#include "Vector3.h"
#include "Angle.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <map>

namespace loam_ros_free {

/** \brief A pair describing the start end end index of a range. */
typedef std::pair<size_t, size_t> IndexRange;

/** \brief Class realizing a linear mapping from vertical point angle to the corresponding scan ring.
 *
 */
class MultiScanMapper {
public:

	std::map<int, std::vector<float>> scan_angles_stat;
	/** \brief Construct a new multi scan mapper instance.
	 *
	 * @param lowerBound - the lower vertical bound (degrees)
	 * @param upperBound - the upper vertical bound (degrees)
	 * @param nScanRings - the number of scan rings
	 */
	MultiScanMapper(const float& lowerBound = -15, const float& upperBound = 15,
			const uint16_t& nScanRings = 16);

	const float& getLowerBound() {
		return _lowerBound;
	}
	const float& getUpperBound() {
		return _upperBound;
	}
	const uint16_t& getNumberOfScanRings() {
		return _nScanRings;
	}

	/** \brief Set mapping parameters.
	 *
	 * @param lowerBound - the lower vertical bound (degrees)
	 * @param upperBound - the upper vertical bound (degrees)
	 * @param nScanRings - the number of scan rings
	 */
	void set(const float& lowerBound, const float& upperBound,
			const uint16_t& nScanRings);

	/** \brief Map the specified vertical point angle to its ring ID.
	 *
	 * @param angle the vertical point angle (in rad)
	 * @return the ring ID
	 */
	int getRingForAngle(const float& angle);

	/** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
	static inline MultiScanMapper Velodyne_VLP_16() {
		return MultiScanMapper(-15, 15, 16);
	}
	;

	/** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
	static inline MultiScanMapper Velodyne_HDL_32() {
		return MultiScanMapper(-30.67f, 10.67f, 32);
	}
	;

	/** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
	static inline MultiScanMapper Velodyne_HDL_64E() {
		std::cout<<"Velodyne_HDL_64E"<<std::endl;
		return MultiScanMapper(-24.9f, 2, 64);
	}
	;

	/** Multi scan mapper for Velodyne HDL-64E S3 according to data sheet. */
	static inline MultiScanMapper Velodyne_HDL_64ES3() {
		return MultiScanMapper(-24.33f, 2, 64);
	}
	;

	/** Multi scan mapper for hesai pandora according to data sheet. */
	static inline MultiScanMapper Hesai_Pandora() {
		std::cout<<"Hesai_Pandora"<<std::endl;
		return MultiScanMapper(-16, 7, 40);
	}

private:
	float _lowerBound;      ///< the vertical angle of the first scan ring
	float _upperBound;      ///< the vertical angle of the last scan ring
	uint16_t _nScanRings;   ///< number of scan rings
	float _factor;          ///< linear interpolation factor
	std::vector<float> _hs_scan_vertical_angles;
	std::vector<float> _vl64E_scan_vertical_angles;
};

/** Scan Registration configuration parameters. */
class RegistrationParams {
public:
	RegistrationParams(const float& scanPeriod_ = 0.1,
			const int& imuHistorySize_ = 200, const int& nFeatureRegions_ = 6,
			const int& curvatureRegion_ = 5, const int& maxCornerSharp_ = 2,
			const int& maxSurfaceFlat_ = 4, const float& lessFlatFilterSize_ =
					0.2, const float& surfaceCurvatureThreshold_ = 0.1) :
			scanPeriod(scanPeriod_), imuHistorySize(imuHistorySize_), nFeatureRegions(
					nFeatureRegions_), curvatureRegion(curvatureRegion_), maxCornerSharp(
					maxCornerSharp_), maxCornerLessSharp(10 * maxCornerSharp_), maxSurfaceFlat(
					maxSurfaceFlat_), lessFlatFilterSize(lessFlatFilterSize_), surfaceCurvatureThreshold(
					surfaceCurvatureThreshold_) {
	}

	/** The time per scan. */
	float scanPeriod;

	/** The size of the IMU history state buffer. */
	int imuHistorySize;

	/** The number of (equally sized) regions used to distribute the feature extraction within a scan. */
	int nFeatureRegions;

	/** The number of surrounding points (+/- region around a point) used to calculate a point curvature. */
	int curvatureRegion;

	/** The maximum number of sharp corner points per feature region. */
	int maxCornerSharp;

	/** The maximum number of less sharp corner points per feature region. */
	int maxCornerLessSharp;

	/** The maximum number of flat surface points per feature region. */
	int maxSurfaceFlat;

	/** The voxel size used for down sizing the remaining less flat surface points. */
	float lessFlatFilterSize;

	/** The curvature threshold below / above a point is considered a flat / corner point. */
	float surfaceCurvatureThreshold;
};

/** Point label options. */
enum PointLabel {
	CORNER_SHARP = 2,       ///< sharp corner point
	CORNER_LESS_SHARP = 1,  ///< less sharp corner point
	SURFACE_LESS_FLAT = 0,  ///< less flat surface point
	SURFACE_FLAT = -1       ///< flat surface point
};

/** IMU state data. */
typedef struct IMUState {
	/** The time of the measurement leading to this state (in seconds). */
	loam::Time stamp;

	/** The current roll angle. */
	loam::Angle roll;

	/** The current pitch angle. */
	loam::Angle pitch;

	/** The current yaw angle. */
	loam::Angle yaw;

	/** The accumulated global IMU position in 3D space. */
	loam::Vector3 position;

	/** The accumulated global IMU velocity in 3D space. */
	loam::Vector3 velocity;

	/** The current (local) IMU acceleration in 3D space. */
	loam::Vector3 acceleration;

	/** \brief Interpolate between two IMU states.
	 *
	 * @param start the first IMUState
	 * @param end the second IMUState
	 * @param ratio the interpolation ratio
	 * @param result the target IMUState for storing the interpolation result
	 */
	static void interpolate(const IMUState& start, const IMUState& end,
			const float& ratio, IMUState& result) {
		float invRatio = 1 - ratio;

		result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
		result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
		if (start.yaw.rad() - end.yaw.rad() > M_PI) {
			result.yaw = start.yaw.rad() * invRatio
					+ (end.yaw.rad() + 2 * M_PI) * ratio;
		} else if (start.yaw.rad() - end.yaw.rad() < -M_PI) {
			result.yaw = start.yaw.rad() * invRatio
					+ (end.yaw.rad() - 2 * M_PI) * ratio;
		} else {
			result.yaw = start.yaw.rad() * invRatio + end.yaw.rad() * ratio;
		}

		result.velocity = start.velocity * invRatio + end.velocity * ratio;
		result.position = start.position * invRatio + end.position * ratio;
	}
	;
} IMUState;

/** \brief Class for registering point clouds received from multi-laser lidars.
 *
 */
class MultiScanRegistrationRosFree {
public:
	MultiScanRegistrationRosFree(const MultiScanMapper& scanMapper =
			MultiScanMapper());

	bool setup(std::string const &lidar_type);

	auto getAngleStats(){
		return _scanMapper.scan_angles_stat;
	}

	/** \brief Process a new input cloud.
	 *
	 * @param laserCloudIn the new input cloud to process
	 * @param scanTime the scan (message) timestamp
	 */
	void process(const std::vector<pcl::PointXYZI>& laserCloudIn,
			const loam::Time& scanTime);

	auto const& imuTransform() {
		return _imuTrans;
	}
	auto const& sweepStart() {
		return _sweepStart;
	}
	auto const& laserCloud() {
		return _laserCloud;
	}
	auto const& cornerPointsSharp() {
		return _cornerPointsSharp;
	}
	auto const& cornerPointsLessSharp() {
		return _cornerPointsLessSharp;
	}
	auto const& surfacePointsFlat() {
		return _surfacePointsFlat;
	}
	auto const& surfacePointsLessFlat() {
		return _surfacePointsLessFlat;
	}
	auto const& config() {
		return _config;
	}

private:

	/** \brief Setup component in active mode.
	 *
	 * @param node the ROS node handle
	 * @param privateNode the private ROS node handle
	 */
	bool setupParam(std::string const &lidar_type);

	void reset(const loam::Time& scanTime);

	bool configure(const RegistrationParams& config);

	void setRegionBuffersFor(const size_t& startIdx, const size_t& endIdx);

	void setScanBuffersFor(const size_t& startIdx, const size_t& endIdx);

	void markAsPicked(const size_t& cloudIdx, const size_t& scanIdx);

	void extractFeatures(const uint16_t& beginIdx = 0);

	void processScanlines(const loam::Time& scanTime,
			std::vector<pcl::PointCloud<pcl::PointXYZI>> const& laserCloudScans);

private:
	int _systemDelay = 20;             ///< system startup delay counter
	MultiScanMapper _scanMapper; ///< mapper for mapping vertical point angles to scan ring IDs
	std::vector<pcl::PointCloud<pcl::PointXYZI> > _laserCloudScans;

	RegistrationParams _config;  ///< registration parameter

	pcl::PointCloud<pcl::PointXYZI> _laserCloud; ///< full resolution input cloud
	std::vector<IndexRange> _scanIndices; ///< start and end indices of the individual scans withing the full resolution cloud

	pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp; ///< sharp corner points cloud
	pcl::PointCloud<pcl::PointXYZI> _cornerPointsLessSharp; ///< less sharp corner points cloud
	pcl::PointCloud<pcl::PointXYZI> _surfacePointsFlat; ///< flat surface points cloud
	pcl::PointCloud<pcl::PointXYZI> _surfacePointsLessFlat; ///< less flat surface points cloud

	loam::Time _sweepStart;        ///< time stamp of beginning of current sweep
	loam::Time _scanTime;              ///< time stamp of most recent scan
	IMUState _imuStart; ///< the interpolated IMU state corresponding to the start time of the currently processed laser scan
	IMUState _imuCur; ///< the interpolated IMU state corresponding to the time of the currently processed laser scan point
	loam::Vector3 _imuPositionShift; ///< position shift between accumulated IMU position and interpolated IMU position
	size_t _imuIdx = 0;                ///< the current index in the IMU history
	loam::CircularBuffer<IMUState> _imuHistory; ///< history of IMU states for cloud registration

	pcl::PointCloud<pcl::PointXYZ> _imuTrans = { 4, 1 }; ///< IMU transformation information

	std::vector<float> _regionCurvature;      ///< point curvature buffer
	std::vector<PointLabel> _regionLabel;     ///< point label buffer
	std::vector<size_t> _regionSortIndices; ///< sorted region indices based on point curvature
	std::vector<int> _scanNeighborPicked; ///< flag if neighboring point was already picked

};

} // end namespace loam_ros_free

#endif //LOAM_MULTISCANREGISTRATIONROSFREE_H
