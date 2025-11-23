#include "descriptorBasis.h"

class scan_context_descriptor : public scan_descriptor
{
	public:
		scan_context_descriptor(
			int ring_num,
			int sector_num,
			int candidates_num,
			double distance_threshold,
			double lidar_height,
			double max_radius,
			int exclude_recent_num,
			int robot_num,
			int id);

		~scan_context_descriptor();

	private:
		/*** scan context param-independent helper functions ***/
		float xy2theta(
			float & _x,
			float & _y);

		Eigen::MatrixXd circshift(
			const Eigen::MatrixXd &_mat,
			int _num_shift);

		std::vector<float> eig2stdvec(
			Eigen::MatrixXd _eigmat);

		/*** scan context functions ***/
		Eigen::MatrixXd makeScancontext(
			const pcl::PointCloud<pcl::PointXYZI>& scan_down,
			std::vector<float>* data);

		Eigen::MatrixXf makeRingkeyFromScancontext(
			const Eigen::MatrixXd& desc);

		Eigen::MatrixXd makeSectorkeyFromScancontext(
			const Eigen::MatrixXd& desc);

		int fastAlignUsingVkey(
			const Eigen::MatrixXd& vkey1,
			const Eigen::MatrixXd& vkey2);

		double distDirectSC(
			const Eigen::MatrixXd& sc1,
			const Eigen::MatrixXd& sc2);

		std::pair<double, int> distanceBtnScanContext(
			const Eigen::MatrixXd& sc1,
			const Eigen::MatrixXd& sc2);

	public:
		// User-side API
		void saveDescriptorAndKey(
			const float* descriptor_vec,
			const int8_t robot,
			const int index);

		void save(
			const Eigen::MatrixXd sc,
			const int8_t robot,
			const int index);

		std::vector<float> makeAndSaveDescriptorAndKey(
			const pcl::PointCloud<pcl::PointXYZI>& scan,
			const int8_t robot,
			const int index);

		std::pair<int, float> detectIntraLoopClosureID(
			const int cur_ptr);

		std::pair<int, float> detectInterLoopClosureID(
			const int cur_ptr);

		std::pair<int8_t, int> getIndex(
			const int key);

		int getSize(
			const int id);

	private:
		int pc_ring_num_;
		int pc_sector_num_;
		double distance_threshold_; 
		double lidar_height_;
		double pc_max_radius_;

		// matching
		int exclude_recent_num_;
		int candidates_num_;

		// data 
		std::vector<std::vector<Eigen::MatrixXd>> polarcontexts;
		std::vector<std::pair<int8_t,int>> polarcontext_indexs;
		std::vector<std::vector<int>> indexes_maps;

		std::vector<Eigen::MatrixXf> polarcontext_ringkeys;
		Nabo::NNSearchF* kdTree = NULL;

		// other
		int robot_num_;
		int id_;
};
