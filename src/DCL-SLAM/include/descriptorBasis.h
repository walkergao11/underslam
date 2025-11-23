#ifndef _DESCRIPTOR_H_
#define _DESCRIPTOR_H_

// descriptor
#include <nabo/nabo.h>
#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/grsd.h>
#include <pcl/common/pca.h>
#include <fenv.h>

// file iostream
#include <fstream>
#include <iostream>

using namespace std;

class scan_descriptor
{
public:

	virtual std::vector<float> makeAndSaveDescriptorAndKey(
		const pcl::PointCloud<pcl::PointXYZI>& scan,
		const int8_t robot,
		const int index) = 0;
	
	virtual void saveDescriptorAndKey(
		const float* descriptor_vector,
		const int8_t robot,
		const int index) = 0;
	
	virtual std::pair<int, float> detectIntraLoopClosureID(
		const int current_ptr) = 0;

	virtual std::pair<int, float> detectInterLoopClosureID(
		const int current_ptr) = 0;
	
	virtual std::pair<int8_t, int> getIndex(
		const int key) = 0;
	
	virtual int getSize(
		const int id = -1) = 0;
};

#endif
