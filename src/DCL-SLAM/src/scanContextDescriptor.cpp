#include "scanContextDescriptor.h"

scan_context_descriptor::scan_context_descriptor(
	int ring_num 				= 20,
	int sector_num 				= 60,
	int candidates_num 			= 6,
	double distance_threshold 	= 0.14,
	double lidar_height 		= 1.65,
	double max_radius 			= 80.0,
	int exclude_recent_num 		= 100,
	int robot_num				= 1,
	int id						= 0 ) :
pc_ring_num_(ring_num), // 20 in the original paper (IROS 18)
pc_sector_num_(sector_num), // 60 in the original paper (IROS 18)
candidates_num_(candidates_num),// 10 is enough. (refer the IROS 18 paper)
distance_threshold_(distance_threshold), // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <,
							// DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness).
							// 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
lidar_height_(lidar_height), // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord)
							// if you use robot-coord-transformed lidar scans, just set this as 0.
pc_max_radius_(max_radius), // 80 meter max in the original paper (IROS 18)
exclude_recent_num_(exclude_recent_num), // simply just keyframe gap (related with loopClosureFrequency in yaml), but node position distance-based exclusion is ok. 
robot_num_(robot_num), // number of robot in robotic swarm
id_(id) // this robot id
{
	// allocate memory
	for(int i = 0; i < robot_num_; i++)
	{
		std::vector<Eigen::MatrixXd> base_matrix;
		Eigen::MatrixXf base_vector;
		std::vector<int> base_int;
		polarcontexts.push_back(base_matrix);
		polarcontext_ringkeys.push_back(base_vector);
		indexes_maps.push_back(base_int);
	}
}

scan_context_descriptor::~scan_context_descriptor()
{
	
}

/*** scan context param-independent helper functions ***/
float scan_context_descriptor::xy2theta(
	float & _x,
	float & _y)
{
	// first quadrant
	if((_x >= 0) & (_y >= 0))
	{
		return (180/M_PI) * atan(_y / _x);
	}
	// second quadrant
	if((_x < 0) & (_y >= 0))
	{
		return 180 - ((180/M_PI) * atan(_y / (-_x)));
	}
	// third quadrant
	if((_x < 0) & (_y < 0))
	{
		return 180 + ((180/M_PI) * atan(_y / _x));
	}
	// forth quadrant
	if((_x >= 0) & (_y < 0))
	{
		return 360 - ((180/M_PI) * atan((-_y) / _x));
	}
}

Eigen::MatrixXd scan_context_descriptor::circshift(
	const Eigen::MatrixXd &_mat,
	int _num_shift)
{
	// shift columns to right direction 
	assert(_num_shift >= 0);

	if(_num_shift == 0)
	{
		Eigen::MatrixXd shifted_mat( _mat );
		return shifted_mat; // Early return 
	}

	Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero(_mat.rows(), _mat.cols());
	for(int col_idx = 0; col_idx < _mat.cols(); col_idx++)
	{
		int new_location = (col_idx + _num_shift) % _mat.cols();
		shifted_mat.col(new_location) = _mat.col(col_idx);
	}

	return shifted_mat;
}

std::vector<float> scan_context_descriptor::eig2stdvec(
	Eigen::MatrixXd _eigmat)
{
	std::vector<float> vec(_eigmat.data(), _eigmat.data() + _eigmat.size());
	return vec;
}

/*** scan context functions ***/
Eigen::MatrixXd scan_context_descriptor::makeScancontext(
	const pcl::PointCloud<pcl::PointXYZI>& scan_down,
	std::vector<float>* data)
{
	// TicToc t_making_desc;

	int num_pts_scan_down = scan_down.points.size();

	// main
	const int NO_POINT = -1000;
	Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(pc_ring_num_, pc_sector_num_);

	pcl::PointXYZI pt;
	float azim_angle, azim_range; // wihtin 2d plane
	int ring_idx, sctor_idx;
	for(int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
	{
		pt.x = scan_down.points[pt_idx].x; 
		pt.y = scan_down.points[pt_idx].y;
		pt.z = scan_down.points[pt_idx].z + lidar_height_; // naive adding is ok (all points should be > 0).

		// xyz to ring, sector
		azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
		azim_angle = xy2theta(pt.x, pt.y);

		// if range is out of roi, pass
		if(azim_range > pc_max_radius_)
		{
			continue;
		}

		ring_idx = std::max(std::min(pc_ring_num_, int(ceil((azim_range / pc_max_radius_) * pc_ring_num_))), 1);
		sctor_idx = std::max(std::min(pc_sector_num_, int(ceil((azim_angle / 360.0) * pc_sector_num_))), 1);

		// taking maximum z 
		if(desc(ring_idx-1, sctor_idx-1) < pt.z) // -1 means cpp starts from 0
		{
			desc(ring_idx-1, sctor_idx-1) = pt.z; // update for taking maximum value at that bin
		}
	}

	// reset no points to zero (for cosine dist later)
	for(int row_idx = 0; row_idx < desc.rows(); row_idx++)
	{
		for(int col_idx = 0; col_idx < desc.cols(); col_idx++)
		{
			if(desc(row_idx, col_idx) == NO_POINT)
			{
				desc(row_idx, col_idx) = 0;
			}
			data->push_back(desc(row_idx, col_idx));
		}
	}

	// t_making_desc.toc("PolarContext making");

	return desc;
}

Eigen::MatrixXf scan_context_descriptor::makeRingkeyFromScancontext(
	const Eigen::MatrixXd& desc)
{
	// summary: rowwise mean vector
	Eigen::MatrixXf invariant_key(desc.rows(), 1);
	for(int row_idx = 0; row_idx < desc.rows(); row_idx++)
	{
		Eigen::MatrixXd curr_row = desc.row(row_idx);
		invariant_key(row_idx, 0) = curr_row.mean();
	}

	return invariant_key;
}

Eigen::MatrixXd scan_context_descriptor::makeSectorkeyFromScancontext(
	const Eigen::MatrixXd& desc)
{
	// summary: columnwise mean vector
	Eigen::MatrixXd variant_key(1, desc.cols());
	for(int col_idx = 0; col_idx < desc.cols(); col_idx++)
	{
		Eigen::MatrixXd curr_col = desc.col(col_idx);
		variant_key(0, col_idx) = curr_col.mean();
	}

	return variant_key;
}

int scan_context_descriptor::fastAlignUsingVkey(
	const Eigen::MatrixXd& vkey1,
	const Eigen::MatrixXd& vkey2)
{
	int argmin_vkey_shift = 0;
	double min_veky_diff_norm = 10000000;
	for(int shift_idx = 0; shift_idx < vkey1.cols(); shift_idx++)
	{
		Eigen::MatrixXd vkey2_shifted = circshift(vkey2, shift_idx);

		Eigen::MatrixXd vkey_diff = vkey1 - vkey2_shifted;

		double cur_diff_norm = vkey_diff.norm();
		if(cur_diff_norm < min_veky_diff_norm)
		{
			argmin_vkey_shift = shift_idx;
			min_veky_diff_norm = cur_diff_norm;
		}
	}

	return argmin_vkey_shift;
}

double scan_context_descriptor::distDirectSC(
	const Eigen::MatrixXd& sc1,
	const Eigen::MatrixXd& sc2) // "d" (eq 5) in the original paper (IROS 18)
{
	int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
	double sum_sector_similarity = 0;
	for(int col_idx = 0; col_idx < sc1.cols(); col_idx++)
	{
		Eigen::VectorXd col_sc1 = sc1.col(col_idx);
		Eigen::VectorXd col_sc2 = sc2.col(col_idx);
		
		if((col_sc1.norm() == 0) | (col_sc2.norm() == 0))
		{
			continue; // don't count this sector pair.
		}

		double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

		sum_sector_similarity = sum_sector_similarity + sector_similarity;
		num_eff_cols = num_eff_cols + 1;
	}
	
	double sc_sim = sum_sector_similarity / num_eff_cols;
	return 1.0 - sc_sim;
}

std::pair<double, int> scan_context_descriptor::distanceBtnScanContext(
	const Eigen::MatrixXd& sc1,
	const Eigen::MatrixXd& sc2) // "D" (eq 6) in the original paper (IROS 18)
{
	// 1. fast align using variant key (not in original IROS18)
	Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(sc1);
	Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(sc2);
	int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

	// int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * sc1.cols()); // a half of search range 
	int SEARCH_RADIUS = round(0.5 * 0.1 * sc1.cols()); // a half of search range 
	std::vector<int> shift_idx_search_space { argmin_vkey_shift };
	for(int ii = 1; ii < SEARCH_RADIUS + 1; ii++)
	{
		shift_idx_search_space.push_back((argmin_vkey_shift + ii + sc1.cols()) % sc1.cols());
		shift_idx_search_space.push_back((argmin_vkey_shift - ii + sc1.cols()) % sc1.cols());
	}
	std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

	// 2. fast columnwise diff 
	int argmin_shift = 0;
	double min_sc_dist = 10000000;
	for(int num_shift: shift_idx_search_space)
	{
		Eigen::MatrixXd sc2_shifted = circshift(sc2, num_shift);
		double cur_sc_dist = distDirectSC(sc1, sc2_shifted);
		if(cur_sc_dist < min_sc_dist)
		{
			argmin_shift = num_shift;
			min_sc_dist = cur_sc_dist;
		}
	}

	return std::make_pair(min_sc_dist, argmin_shift);
}

// User-side API
void scan_context_descriptor::saveDescriptorAndKey(
	const float* descriptor_vec,
	const int8_t robot,
	const int index)
{
	// decode scan context
	Eigen::MatrixXd sc = Eigen::MatrixXd::Ones(pc_ring_num_, pc_sector_num_);
	for(int row_idx = 0; row_idx < sc.rows(); row_idx++)
	{
		for(int col_idx = 0; col_idx < sc.cols(); col_idx++)
		{
			sc(row_idx, col_idx) = descriptor_vec[row_idx*sc.cols()+col_idx];
		}
	}

	save(sc, robot, index);
}

void scan_context_descriptor::save(
	const Eigen::MatrixXd sc,
	const int8_t robot,
	const int index)
{
	Eigen::MatrixXf ringkey = makeRingkeyFromScancontext(sc);
	Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc);

	polarcontexts[robot].push_back(sc); // desc
	polarcontext_ringkeys[robot].conservativeResize(pc_ring_num_, polarcontexts[robot].size());
	polarcontext_ringkeys[robot].block(0, polarcontexts[robot].size()-1, pc_ring_num_, 1) = ringkey.block(0, 0, pc_ring_num_, 1);
	indexes_maps[robot].push_back(polarcontext_indexs.size());
	polarcontext_indexs.push_back(std::make_pair(robot, index)); // indexs
}

std::vector<float> scan_context_descriptor::makeAndSaveDescriptorAndKey(
	const pcl::PointCloud<pcl::PointXYZI>& scan,
	const int8_t robot,
	const int index)
{
	std::vector<float> descriptor_data;
	Eigen::MatrixXd sc = makeScancontext(scan, &descriptor_data); // size:(pc_ring_num_, pc_sector_num_)
	save(sc, robot, index);

	return descriptor_data;
}

std::pair<int, float> scan_context_descriptor::detectIntraLoopClosureID(
	const int cur_ptr)
{

}

std::pair<int, float> scan_context_descriptor::detectInterLoopClosureID(
	const int cur_ptr) // int: nearest node index, float: relative yaw  
{
	std::pair<int, float> result {-1, 0.0};
	int robot_id = polarcontext_indexs[cur_ptr].first;
	int frame_id = polarcontext_indexs[cur_ptr].second;
	Eigen::VectorXf ringkey = polarcontext_ringkeys[robot_id].col(frame_id);; // current query ringkey
	auto polarcontext = polarcontexts[robot_id][frame_id]; // current feature

	// step 1: candidates from ringkey tree
	// tree construction
	Eigen::MatrixXf new_polarcontext_ringkeys;
	std::vector<int> new_indexes_maps;
	std::vector<Eigen::MatrixXd> new_polarcontexts;
	if(robot_id == id_)
	{
		for(int i = 0; i < robot_num_; i++)
		{
			if(i != id_ && indexes_maps[i].size() > 0)
			{
				int cur_row = new_polarcontext_ringkeys.cols();
				int add_row = indexes_maps[i].size();
				new_polarcontext_ringkeys.conservativeResize(pc_ring_num_, cur_row + add_row);
				new_polarcontext_ringkeys.block(0, cur_row, pc_ring_num_, add_row) = polarcontext_ringkeys[i].block(0, 0, pc_ring_num_, add_row);
				new_indexes_maps.insert(new_indexes_maps.end(), indexes_maps[i].begin(), indexes_maps[i].end());
				new_polarcontexts.insert(new_polarcontexts.end(), polarcontexts[i].begin(), polarcontexts[i].end());
			}
		}
	}
	else
	{
		if(indexes_maps[id_].size() > 0)
		{
			int size = indexes_maps[id_].size();
			new_polarcontext_ringkeys.conservativeResize(pc_ring_num_, size);
			new_polarcontext_ringkeys.block(0, 0, pc_ring_num_, size) = polarcontext_ringkeys[id_].block(0, 0, pc_ring_num_, size);
			new_indexes_maps.insert(new_indexes_maps.end(), indexes_maps[id_].begin(), indexes_maps[id_].end());
			new_polarcontexts.insert(new_polarcontexts.end(), polarcontexts[id_].begin(), polarcontexts[id_].end());
		}
	}

	if(new_indexes_maps.size() < candidates_num_ + 1)
	{
		return result;
	}

	kdTree = Nabo::NNSearchF::createKDTreeLinearHeap(new_polarcontext_ringkeys, pc_ring_num_);

	// search n nearest neighbors
	Eigen::VectorXi indice(candidates_num_);
	Eigen::VectorXf distance(candidates_num_);
	float min_distance = 10000000.0;
	int min_index = -1;
	int min_bias = 0;

	// knn search
	kdTree->knn(ringkey, indice, distance, candidates_num_);

	// step 2: pairwise distance
	for(int i = 0; i < std::min(candidates_num_, int(indice.size())); i++)
	{
		if(indice[i] >= new_indexes_maps.size())
		{
			continue;
		}

		auto polarcontext_candidate = new_polarcontexts[indice[i]];
		std::pair<double, int> sc_dist_result = distanceBtnScanContext(polarcontext, polarcontext_candidate); 

		double candidateDis = sc_dist_result.first;
		int candidate_align = sc_dist_result.second;

		if(candidateDis < min_distance)
		{
			min_distance = candidateDis;
			min_index = new_indexes_maps[indice[i]];
			min_bias = candidate_align;
		}
	}

	// loop threshold check
	if(min_distance < distance_threshold_)
	{
		result.first = min_index;
		result.second = min_bias;
		ROS_DEBUG("\033[1;33m[SC Loop<%d>] btn %d-%d and %d-%d. Dis: %.2f.\033[0m", id_,
			polarcontext_indexs[cur_ptr].first, polarcontext_indexs[cur_ptr].second,
			polarcontext_indexs[min_index].first, polarcontext_indexs[min_index].second, min_distance);
	}
	else
	{
		ROS_DEBUG("\033[1;33m[SC Not loop<%d>] btn %d-%d and %d-%d. Dis: %.2f.\033[0m", id_,
			polarcontext_indexs[cur_ptr].first, polarcontext_indexs[cur_ptr].second,
			polarcontext_indexs[min_index].first, polarcontext_indexs[min_index].second, min_distance);
	}
	return result;
}

std::pair<int8_t, int> scan_context_descriptor::getIndex(
	const int key)
{
	return polarcontext_indexs[key];
}

int scan_context_descriptor::getSize(
	const int id = -1)
{
	if(id == -1)
	{
		return polarcontext_indexs.size();
	}
	else
	{
		return indexes_maps[id].size();
	}
}
