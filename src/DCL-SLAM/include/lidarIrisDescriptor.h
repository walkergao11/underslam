#include "descriptorBasis.h"

class lidar_iris_descriptor : public scan_descriptor
{
	public:
		struct featureDesc
		{
			cv::Mat1b img;
			cv::Mat1b T;
			cv::Mat1b M;
		};

		lidar_iris_descriptor(
			int rows,
			int columns,
			int n_scan,
			double distance_threshold,
			int exclude_recent_frame_num,
			int match_mode,
			int candidates_num, 
			int nscale,
			int min_wave_length,
			float mult,
			float sigma_on_f,
			int robot_num,
			int id);

		~lidar_iris_descriptor();

	private:
		std::pair<Eigen::VectorXf, cv::Mat1b> getIris(
			const pcl::PointCloud<pcl::PointXYZI> &cloud);

		inline cv::Mat circRowShift(
			const cv::Mat &src,
			int shift_m_rows);

		inline cv::Mat circColShift(
			const cv::Mat &src,
			int shift_n_cols);

		cv::Mat circShift(
			const cv::Mat &src,
			int shift_m_rows,
			int shift_n_cols);

		std::vector<cv::Mat2f> logGaborFilter(
			const cv::Mat1f &src,
			unsigned int nscale,
			int min_wave_length,
			double mult,
			double sigma_on_f);

		void logFeatureEncode(
			const cv::Mat1b &src,
			unsigned int nscale,
			int min_wave_length,
			double mult,
			double sigma_on_f,
			cv::Mat1b &T,
			cv::Mat1b &M);

		featureDesc getFeature(
			const cv::Mat1b &src);

		featureDesc getFeature(
			const cv::Mat1b &src,
			std::vector<float> &vec);

		void recomb(
			cv::Mat &src,
			cv::Mat &dst);

		void forwardFFT(
			cv::Mat &src,
			cv::Mat *f_img,
			bool do_recomb);

		void highpass(cv::Size sz, cv::Mat& dst);

		float logpolar(
			cv::Mat& src,
			cv::Mat& dst);

		cv::RotatedRect logPolarFFTTemplateMatch(
			cv::Mat& im0,
			cv::Mat& im1/*, double canny_threshold1, double canny_threshold2*/);

		cv::RotatedRect fftMatch(
			const cv::Mat& im0,
			const cv::Mat& im1);

		void getHammingDistance(
			const cv::Mat1b &T1,
			const cv::Mat1b &M1,
			const cv::Mat1b &T2,
			const cv::Mat1b &M2,
			int scale,
			float &dis,
			int &bias);

		float compare(
			const featureDesc &img1,
			const featureDesc &img2,
			int *bias);

	public:
		// user-side API
		void saveDescriptorAndKey(
			const float* iris,
			const int8_t robot,
			const int index);

		void save(
			const cv::Mat1b iris,
			Eigen::MatrixXf rowkey,
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
		// descriptor configuration
		int rows_;
		int columns_;
		int n_scan_;
		double distance_threshold_;

		// matching
		int exclude_recent_frame_num_;
		int match_mode_;
		int candidates_num_;

		// kd tree
		Nabo::NNSearchF* kdTree = NULL;

		// descriptor params
		int nscale_;
		int min_wave_length_;
		float mult_;
		float sigma_on_f_;

		// other
		int robot_num_;
		int id_;
		
		// data
		std::vector<unordered_map<int, featureDesc>> iris_features;
		std::vector<unordered_map<int, Eigen::MatrixXf>> iris_rowkeys;
		std::vector<unordered_map<int, int>> indexes_maps;
		std::vector<std::pair<int8_t,int>> iris_feature_index_pairs;
};
