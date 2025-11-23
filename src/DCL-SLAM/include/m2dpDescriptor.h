#include "descriptorBasis.h"

class m2dp_descriptor : public scan_descriptor
{
	public:
		m2dp_descriptor(
			int num_T,
			int num_R,
			int num_P,
			int num_Q,
			int robot_num,
			int id);

		~m2dp_descriptor();

	private:
		Eigen::Matrix<double, 64, 128> GetSignatureMatrix();

		void sph2cart(
			double azm,
			double elv,
			double r,
			Eigen::Matrix<double, 1, 3> &vecN);

		void cart2pol(
			double x,
			double y,
			Eigen::Vector2d &vecN);

		void histogram2d(
			std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points,
			vector<double> thetaList,
			vector<double> rhoList,
			Eigen::MatrixXd &hist);

	public:
		std::vector<float> makeAndSaveDescriptorAndKey(
			const pcl::PointCloud<pcl::PointXYZI>& scan,
			const int8_t robot,
			const int index);

		void save(
			const std::vector<float> m2dp_vec,
			const int8_t robot,
			const int index);
		
		void saveDescriptorAndKey(
			const float* descriptorMat,
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
		// m2dp params
		int num_T_;
		int num_R_;
		int num_P_;
		int num_Q_;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
		Eigen::MatrixXd cloud_pca;
		double max_rho;

		vector<double> azimuth_list;
		vector<double> elevation_list;
		
		Eigen::Matrix<double, 64, 128> A;
		Eigen::Matrix<double,1,192> m2dp;

		// data
		std::vector<std::vector<Eigen::VectorXf>> m2dp_dictionaries;
		std::vector<std::vector<int>> indexes_maps;
		std::vector<std::pair<int8_t,int>> m2dp_index_pairs;

		// other
		int robot_num_;
		int id_;
};
