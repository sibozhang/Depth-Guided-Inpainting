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

#include "loam_velodyne/MultiScanRegistrationRosFree.h"
#include "math_utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

namespace loam_ros_free {

MultiScanMapper::MultiScanMapper(const float& lowerBound,
		const float& upperBound, const uint16_t& nScanRings) :
		_lowerBound(lowerBound), _upperBound(upperBound), _nScanRings(
				nScanRings), _factor(
				(nScanRings - 1) / (upperBound - lowerBound)) {

	_hs_scan_vertical_angles.push_back(-15.444);
	_hs_scan_vertical_angles.push_back(-14.543);
	_hs_scan_vertical_angles.push_back(-13.63);
	_hs_scan_vertical_angles.push_back(-12.705);
	_hs_scan_vertical_angles.push_back(-11.772);
	_hs_scan_vertical_angles.push_back(-10.826);
	_hs_scan_vertical_angles.push_back(-9.871);
	_hs_scan_vertical_angles.push_back(-9);
	_hs_scan_vertical_angles.push_back(-8);
	_hs_scan_vertical_angles.push_back(-7);
	_hs_scan_vertical_angles.push_back(-6);
	_hs_scan_vertical_angles.push_back(-5.67);
	_hs_scan_vertical_angles.push_back(-5.33);
	_hs_scan_vertical_angles.push_back(-5);
	_hs_scan_vertical_angles.push_back(-4.67);
	_hs_scan_vertical_angles.push_back(-4.33);
	_hs_scan_vertical_angles.push_back(-4);
	_hs_scan_vertical_angles.push_back(-3.67);
	_hs_scan_vertical_angles.push_back(-3.33);
	_hs_scan_vertical_angles.push_back(-3);
	_hs_scan_vertical_angles.push_back(-2.67);
	_hs_scan_vertical_angles.push_back(-2.33);
	_hs_scan_vertical_angles.push_back(-2);
	_hs_scan_vertical_angles.push_back(-1.67);
	_hs_scan_vertical_angles.push_back(-1.33);
	_hs_scan_vertical_angles.push_back(-1);
	_hs_scan_vertical_angles.push_back(-0.67);
	_hs_scan_vertical_angles.push_back(-0.33);
	_hs_scan_vertical_angles.push_back(0);
	_hs_scan_vertical_angles.push_back(0.33);
	_hs_scan_vertical_angles.push_back(0.67);
	_hs_scan_vertical_angles.push_back(1);
	_hs_scan_vertical_angles.push_back(1.33);
	_hs_scan_vertical_angles.push_back(1.67);
	_hs_scan_vertical_angles.push_back(2);
	_hs_scan_vertical_angles.push_back(3);
	_hs_scan_vertical_angles.push_back(4);
	_hs_scan_vertical_angles.push_back(5);
	_hs_scan_vertical_angles.push_back(6);
	_hs_scan_vertical_angles.push_back(7);




	_vl64E_scan_vertical_angles.push_back(-24.9);
	_vl64E_scan_vertical_angles.push_back(-24.5112);
	_vl64E_scan_vertical_angles.push_back(-24.0143);
	_vl64E_scan_vertical_angles.push_back(-23.6137);
	_vl64E_scan_vertical_angles.push_back(-23.1402);
	_vl64E_scan_vertical_angles.push_back(-22.6306);
	_vl64E_scan_vertical_angles.push_back(-22.4401);
	_vl64E_scan_vertical_angles.push_back(-21.9689);
	_vl64E_scan_vertical_angles.push_back(-21.524);
	_vl64E_scan_vertical_angles.push_back(-21.1277);
	_vl64E_scan_vertical_angles.push_back(-20.6467);
	_vl64E_scan_vertical_angles.push_back(-20.0617);
	_vl64E_scan_vertical_angles.push_back(-19.811);
	_vl64E_scan_vertical_angles.push_back(-19.5229);
	_vl64E_scan_vertical_angles.push_back(-18.995);
	_vl64E_scan_vertical_angles.push_back(-18.5413);
	_vl64E_scan_vertical_angles.push_back(-18.1374);
	_vl64E_scan_vertical_angles.push_back(-17.639);
	_vl64E_scan_vertical_angles.push_back(-17.1873);
	_vl64E_scan_vertical_angles.push_back(-16.6438);
	_vl64E_scan_vertical_angles.push_back(-16.4265);
	_vl64E_scan_vertical_angles.push_back(-16.0727);
	_vl64E_scan_vertical_angles.push_back(-15.4902);
	_vl64E_scan_vertical_angles.push_back(-15.1295);
	_vl64E_scan_vertical_angles.push_back(-14.6345);
	_vl64E_scan_vertical_angles.push_back(-14.2005);
	_vl64E_scan_vertical_angles.push_back(-13.6564);
	_vl64E_scan_vertical_angles.push_back(-13.4791);
	_vl64E_scan_vertical_angles.push_back(-13.0578);
	_vl64E_scan_vertical_angles.push_back(-12.5715);
	_vl64E_scan_vertical_angles.push_back(-12.1458);
	_vl64E_scan_vertical_angles.push_back(-11.6957);
	_vl64E_scan_vertical_angles.push_back(-11.2748);
	_vl64E_scan_vertical_angles.push_back(-10.7881);
	_vl64E_scan_vertical_angles.push_back(-10.2375);
	_vl64E_scan_vertical_angles.push_back(-9.99209);
	_vl64E_scan_vertical_angles.push_back(-9.5181);
	_vl64E_scan_vertical_angles.push_back(-9.06992);
	_vl64E_scan_vertical_angles.push_back(-8.77687);
	_vl64E_scan_vertical_angles.push_back(-8.33208);
	_vl64E_scan_vertical_angles.push_back(-7.78644);
	_vl64E_scan_vertical_angles.push_back(-7.34115);
	_vl64E_scan_vertical_angles.push_back(-6.95957);
	_vl64E_scan_vertical_angles.push_back(-6.52878);
	_vl64E_scan_vertical_angles.push_back(-6.10157);
	_vl64E_scan_vertical_angles.push_back(-5.68974);
	_vl64E_scan_vertical_angles.push_back(-5.28384);
	_vl64E_scan_vertical_angles.push_back(-4.79967);
	_vl64E_scan_vertical_angles.push_back(-4.39941);
	_vl64E_scan_vertical_angles.push_back(-4.01961);
	_vl64E_scan_vertical_angles.push_back(-3.54724);
	_vl64E_scan_vertical_angles.push_back(-3.1087);
	_vl64E_scan_vertical_angles.push_back(-2.74124);
	_vl64E_scan_vertical_angles.push_back(-2.25277);
	_vl64E_scan_vertical_angles.push_back(-1.85808);
	_vl64E_scan_vertical_angles.push_back(-1.43662);
	_vl64E_scan_vertical_angles.push_back(-0.981684);
	_vl64E_scan_vertical_angles.push_back(-0.548023);
	_vl64E_scan_vertical_angles.push_back(-0.158929);
	_vl64E_scan_vertical_angles.push_back(0.311992);
	_vl64E_scan_vertical_angles.push_back(0.75276);
	_vl64E_scan_vertical_angles.push_back(1.12109);
	_vl64E_scan_vertical_angles.push_back(1.60569);
	_vl64E_scan_vertical_angles.push_back(1.9891);

}

void MultiScanMapper::set(const float &lowerBound, const float &upperBound,
		const uint16_t &nScanRings) {
	_lowerBound = lowerBound;
	_upperBound = upperBound;
	_nScanRings = nScanRings;
	_factor = (nScanRings - 1) / (upperBound - lowerBound);
}

int MultiScanMapper::getRingForAngle(const float& angle) {
	if (_lowerBound == -24.33f) {
		float degrees = angle * 180 / M_PI;
		int scan_id = 0;
		if (degrees < -8.58f) {
			scan_id = int((degrees + 24.33) * 2.f + 0.5f);
		} else {
			scan_id = int((degrees + 8.33) * 3.f + 0.5f) + 32;
		}

		//std::cout<<degrees<<" ==> "<<scan_id<<std::endl;
		return scan_id;
	} else if (_lowerBound == -16) {
		float degrees = angle * 180 / M_PI;
		int scan_id = 0;
		auto ptr = std::lower_bound(_hs_scan_vertical_angles.begin(),
				_hs_scan_vertical_angles.end(), degrees);
		if (ptr == _hs_scan_vertical_angles.cend()) {
			scan_id = _nScanRings - 1;
		} else if (ptr == _hs_scan_vertical_angles.cbegin()) {
			scan_id = 0;
		} else {
			auto ptr_prev = ptr;
			ptr_prev--;
			float dist = *ptr - degrees;
			float dist_prev = degrees - *ptr_prev;
			auto closest_ptr = dist < dist_prev ? ptr : ptr_prev;
			scan_id = std::distance(_hs_scan_vertical_angles.begin(),
					closest_ptr);
		}

		//std::cout<<degrees<<" ==> "<<scan_id<<std::endl;
		return scan_id;
	}else if (_lowerBound == -24.9f) {
		float degrees = angle * 180 / M_PI;
		int scan_id = 0;
		auto ptr = std::lower_bound(_vl64E_scan_vertical_angles.begin(),
				_vl64E_scan_vertical_angles.end(), degrees);
		if (ptr == _vl64E_scan_vertical_angles.cend()) {
			scan_id = _nScanRings - 1;
		} else if (ptr == _vl64E_scan_vertical_angles.cbegin()) {
			scan_id = 0;
		} else {
			auto ptr_prev = ptr;
			ptr_prev--;
			float dist = *ptr - degrees;
			float dist_prev = degrees - *ptr_prev;
			auto closest_ptr = dist < dist_prev ? ptr : ptr_prev;
			scan_id = std::distance(_vl64E_scan_vertical_angles.begin(),
					closest_ptr);
		}

		//std::cout<<degrees<<" vl64E "<<scan_id<<std::endl;
		return scan_id;
	}
	else {

//		float degrees = angle * 180 / M_PI;
//		int scan_id=int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
//		std::cout<<degrees<<" ==> "<<scan_id<<std::endl;

		float degrees = angle * 180 / M_PI;
		int scan_id = int((degrees - _lowerBound) * _factor + 0.5);
		scan_angles_stat[scan_id].push_back(degrees);
		return scan_id;

	}
}

MultiScanRegistrationRosFree::MultiScanRegistrationRosFree(
		const MultiScanMapper& scanMapper) :
		_scanMapper(scanMapper) {
}

bool MultiScanRegistrationRosFree::configure(const RegistrationParams& config) {
	_config = config;
	_imuHistory.ensureCapacity(_config.imuHistorySize);
	return true;
}

bool MultiScanRegistrationRosFree::setup(std::string const &lidar_type) {
	if (!setupParam(lidar_type))
		return false;

	RegistrationParams config;
	configure(config);
	return true;
}

bool MultiScanRegistrationRosFree::setupParam(std::string const &lidar_type) {

	if (lidar_type == "VLP-16") {
		_scanMapper = MultiScanMapper::Velodyne_VLP_16();
	} else if (lidar_type == "HDL-32") {
		_scanMapper = MultiScanMapper::Velodyne_HDL_32();
	} else if (lidar_type == "HDL-64E") {
		_scanMapper = MultiScanMapper::Velodyne_HDL_64E();
	} else if (lidar_type == "HDL-64ES3") {
		_scanMapper = MultiScanMapper::Velodyne_HDL_64ES3();
	} else if (lidar_type == "Hesai-Pandora") {
		_scanMapper = MultiScanMapper::Hesai_Pandora();
	} else {
		printf(
				"Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" and \"HDL-64E\" are supported)",
				lidar_type.c_str());
		return false;
	}

	printf("Set  %s  scan mapper.", lidar_type.c_str());

	return true;
}

void MultiScanRegistrationRosFree::reset(const loam::Time& scanTime) {
	_scanTime = scanTime;

	// clear internal cloud buffers at the beginning of a sweep
	_sweepStart = scanTime;

	// clear cloud buffers
	_laserCloud.clear();
	_cornerPointsSharp.clear();
	_cornerPointsLessSharp.clear();
	_surfacePointsFlat.clear();
	_surfacePointsLessFlat.clear();

	// clear scan indices vector
	_scanIndices.clear();

}

void MultiScanRegistrationRosFree::setRegionBuffersFor(const size_t& startIdx,
		const size_t& endIdx) {
	// resize buffers
	size_t regionSize = endIdx - startIdx + 1;
	_regionCurvature.resize(regionSize);
	_regionSortIndices.resize(regionSize);
	_regionLabel.assign(regionSize, SURFACE_LESS_FLAT);

	// calculate point curvatures and reset sort indices
	float pointWeight = -2 * _config.curvatureRegion;

	for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++) {
		float diffX = pointWeight * _laserCloud[i].x;
		float diffY = pointWeight * _laserCloud[i].y;
		float diffZ = pointWeight * _laserCloud[i].z;

		for (int j = 1; j <= _config.curvatureRegion; j++) {
			diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
			diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
			diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
		}

		_regionCurvature[regionIdx] = diffX * diffX + diffY * diffY
				+ diffZ * diffZ;
		_regionSortIndices[regionIdx] = i;
	}

	// sort point curvatures
	for (size_t i = 1; i < regionSize; i++) {
		for (size_t j = i; j >= 1; j--) {
			if (_regionCurvature[_regionSortIndices[j] - startIdx]
					< _regionCurvature[_regionSortIndices[j - 1] - startIdx]) {
				std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
			}
		}
	}
}

void MultiScanRegistrationRosFree::setScanBuffersFor(const size_t& startIdx,
		const size_t& endIdx) {
	// resize buffers
	size_t scanSize = endIdx - startIdx + 1;
	_scanNeighborPicked.assign(scanSize, 0);

	// mark unreliable points as picked
	for (size_t i = startIdx + _config.curvatureRegion;
			i < endIdx - _config.curvatureRegion; i++) {
		const pcl::PointXYZI& previousPoint = (_laserCloud[i - 1]);
		const pcl::PointXYZI& point = (_laserCloud[i]);
		const pcl::PointXYZI& nextPoint = (_laserCloud[i + 1]);

		float diffNext = loam::calcSquaredDiff(nextPoint, point);

		if (diffNext > 0.1) {
			float depth1 = loam::calcPointDistance(point);
			float depth2 = loam::calcPointDistance(nextPoint);

			if (depth1 > depth2) {
				float weighted_distance = std::sqrt(
						loam::calcSquaredDiff(nextPoint, point,
								depth2 / depth1)) / depth2;

				if (weighted_distance < 0.1) {
					std::fill_n(
							&_scanNeighborPicked[i - startIdx
									- _config.curvatureRegion],
							_config.curvatureRegion + 1, 1);

					continue;
				}
			} else {
				float weighted_distance = std::sqrt(
						loam::calcSquaredDiff(point, nextPoint,
								depth1 / depth2)) / depth1;

				if (weighted_distance < 0.1) {
					std::fill_n(&_scanNeighborPicked[i - startIdx + 1],
							_config.curvatureRegion + 1, 1);
				}
			}
		}

		float diffPrevious = loam::calcSquaredDiff(point, previousPoint);
		float dis = loam::calcSquaredPointDistance(point);

		if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) {
			_scanNeighborPicked[i - startIdx] = 1;
		}
	}
}

void MultiScanRegistrationRosFree::markAsPicked(const size_t& cloudIdx,
		const size_t& scanIdx) {
	_scanNeighborPicked[scanIdx] = 1;

	for (int i = 1; i <= _config.curvatureRegion; i++) {
		if (loam::calcSquaredDiff(_laserCloud[cloudIdx + i],
				_laserCloud[cloudIdx + i - 1]) > 0.05) {
			break;
		}

		_scanNeighborPicked[scanIdx + i] = 1;
	}

	for (int i = 1; i <= _config.curvatureRegion; i++) {
		if (loam::calcSquaredDiff(_laserCloud[cloudIdx - i],
				_laserCloud[cloudIdx - i + 1]) > 0.05) {
			break;
		}

		_scanNeighborPicked[scanIdx - i] = 1;
	}
}

void MultiScanRegistrationRosFree::extractFeatures(const uint16_t& beginIdx) {
	// extract features from individual scans
	size_t nScans = _scanIndices.size();
	for (size_t i = beginIdx; i < nScans; i++) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(
				new pcl::PointCloud<pcl::PointXYZI>);
		size_t scanStartIdx = _scanIndices[i].first;
		size_t scanEndIdx = _scanIndices[i].second;

		// skip empty scans
		if (scanEndIdx <= scanStartIdx + 2 * _config.curvatureRegion) {
			continue;
		}

		// Quick&Dirty fix for relative point time calculation without IMU data
		/*float scanSize = scanEndIdx - scanStartIdx + 1;
		 for (int j = scanStartIdx; j <= scanEndIdx; j++) {
		 _laserCloud[j].intensity = i + _scanPeriod * (j - scanStartIdx) / scanSize;
		 }*/

		// reset scan buffers
		setScanBuffersFor(scanStartIdx, scanEndIdx);

		// extract features from equally sized scan regions
		for (int j = 0; j < _config.nFeatureRegions; j++) {
			size_t sp = ((scanStartIdx + _config.curvatureRegion)
					* (_config.nFeatureRegions - j)
					+ (scanEndIdx - _config.curvatureRegion) * j)
					/ _config.nFeatureRegions;
			size_t ep = ((scanStartIdx + _config.curvatureRegion)
					* (_config.nFeatureRegions - 1 - j)
					+ (scanEndIdx - _config.curvatureRegion) * (j + 1))
					/ _config.nFeatureRegions - 1;

			// skip empty regions
			if (ep <= sp) {
				continue;
			}

			size_t regionSize = ep - sp + 1;

			// reset region buffers
			setRegionBuffersFor(sp, ep);

			// extract corner features
			int largestPickedNum = 0;
			for (size_t k = regionSize;
					k > 0 && largestPickedNum < _config.maxCornerLessSharp;) {
				size_t idx = _regionSortIndices[--k];
				size_t scanIdx = idx - scanStartIdx;
				size_t regionIdx = idx - sp;

				if (_scanNeighborPicked[scanIdx] == 0
						&& _regionCurvature[regionIdx]
								> _config.surfaceCurvatureThreshold) {

					largestPickedNum++;
					if (largestPickedNum <= _config.maxCornerSharp) {
						_regionLabel[regionIdx] = CORNER_SHARP;
						_cornerPointsSharp.push_back(_laserCloud[idx]);
					} else {
						_regionLabel[regionIdx] = CORNER_LESS_SHARP;
					}
					_cornerPointsLessSharp.push_back(_laserCloud[idx]);

					markAsPicked(idx, scanIdx);
				}
			}

			// extract flat surface features
			int smallestPickedNum = 0;
			for (int k = 0;
					k < regionSize && smallestPickedNum < _config.maxSurfaceFlat;
					k++) {
				size_t idx = _regionSortIndices[k];
				size_t scanIdx = idx - scanStartIdx;
				size_t regionIdx = idx - sp;

				if (_scanNeighborPicked[scanIdx] == 0
						&& _regionCurvature[regionIdx]
								< _config.surfaceCurvatureThreshold) {

					smallestPickedNum++;
					_regionLabel[regionIdx] = SURFACE_FLAT;
					_surfacePointsFlat.push_back(_laserCloud[idx]);

					markAsPicked(idx, scanIdx);
				}
			}

			// extract less flat surface features
			for (int k = 0; k < regionSize; k++) {
				if (_regionLabel[k] <= SURFACE_LESS_FLAT) {
					surfPointsLessFlatScan->push_back(_laserCloud[sp + k]);
				}
			}
		}

		// down size less flat surface point cloud of current scan
		pcl::PointCloud < pcl::PointXYZI > surfPointsLessFlatScanDS;
		pcl::VoxelGrid < pcl::PointXYZI > downSizeFilter;
		downSizeFilter.setInputCloud(surfPointsLessFlatScan);
		downSizeFilter.setLeafSize(_config.lessFlatFilterSize,
				_config.lessFlatFilterSize, _config.lessFlatFilterSize);
		downSizeFilter.filter(surfPointsLessFlatScanDS);

		_surfacePointsLessFlat += surfPointsLessFlatScanDS;
	}
}

void MultiScanRegistrationRosFree::processScanlines(const loam::Time& scanTime,
		std::vector<pcl::PointCloud<pcl::PointXYZI>> const& laserCloudScans) {
	// reset internal buffers and set IMU start state based on current scan time
	reset(scanTime);

	// construct sorted full resolution cloud
	size_t cloudSize = 0;
	for (int i = 0; i < laserCloudScans.size(); i++) {
		_laserCloud += laserCloudScans[i];

		IndexRange range(cloudSize, 0);
		cloudSize += laserCloudScans[i].size();
		range.second = cloudSize > 0 ? cloudSize - 1 : 0;
		_scanIndices.push_back(range);
	}

	extractFeatures();
}

void MultiScanRegistrationRosFree::process(
		const std::vector<pcl::PointXYZI>& laserCloudIn,
		const loam::Time& scanTime) {
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

	std::cout << "startOri: " << startOri << ", " << "endOri: " << endOri
			<< std::endl;

	bool halfPassed = false;
	pcl::PointXYZI point;
	_laserCloudScans.resize(_scanMapper.getNumberOfScanRings());
	// clear all scanline points
	std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(),
			[](auto&&v) {v.clear();});

	// extract valid points from input cloud
	for (int i = 0; i < cloudSize; i++) {
		point.x = laserCloudIn[i].y;
		point.y = laserCloudIn[i].z;
		point.z = laserCloudIn[i].x;

		// skip NaN and INF valued points
		if (!pcl_isfinite(point.x) || !pcl_isfinite(point.y)
				|| !pcl_isfinite(point.z)) {
			continue;
		}

		// skip zero valued points
		if (point.x * point.x + point.y * point.y + point.z * point.z
				< 0.0001) {
			continue;
		}

		// calculate vertical point angle and scan ID
		float angle = std::atan(
				point.y / std::sqrt(point.x * point.x + point.z * point.z));
		int scanID = _scanMapper.getRingForAngle(angle);
		if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0) {
			continue;
		}

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

		// calculate relative scan time based on point orientation
		float relTime = config().scanPeriod * (ori - startOri)
				/ (endOri - startOri);
		point.intensity = scanID + relTime;

		//projectPointToStartOfSweep(point, relTime);

		_laserCloudScans[scanID].push_back(point);
	}

	processScanlines(scanTime, _laserCloudScans);
}

} // end namespace loam
