#include "m2dpDescriptor.h"

m2dp_descriptor::m2dp_descriptor(
	int num_T 					= 16,
	int num_R 					= 8,
	int num_P 					= 4,
	int num_Q 					= 16,
	int robot_num				= 1,
	int id						= 0 ) :
	num_T_(num_T), // number of bins in theta, the 't' in paper
	num_R_(num_R), // number of bins in rho, the 'l' in paper
	num_P_(num_P), // number of azimuth angles, the 'p' in paper
	num_Q_(num_Q), // number of elevation angles, the 'q' in paper
	robot_num_(robot_num), // number of robot in robotic swarm
	id_(id) // this robot id
{
	// init
	azimuth_list = *new vector<double>(num_P_);
	for(int i = 0; i < num_P_; i++)
	{
		azimuth_list[i] = -M_PI_2 + i * M_PI / (num_P_ - 1);
	}

	elevation_list = *new vector<double>(num_Q_);
	for(int i = 0; i < num_Q_; i++)
	{
		elevation_list[i] = i * M_PI_2 / (num_Q_ - 1);
	}

	// allocate memory
	for(int i = 0; i < robot_num_; i++)
	{
		std::vector<Eigen::VectorXf> base_vector;
		std::vector<int> base_int;
		m2dp_dictionaries.push_back(base_vector);
		indexes_maps.push_back(base_int);
	}
}

m2dp_descriptor::~m2dp_descriptor()
{
	
}

std::vector<float> m2dp_descriptor::makeAndSaveDescriptorAndKey(
	const pcl::PointCloud<pcl::PointXYZI>& scan,
	const int8_t robot,
	const int index)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
	input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
	for(int i = 0; i < scan.points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = scan.points[i].x;
		p.y = scan.points[i].y;
		p.z = scan.points[i].z;
		input_cloud->push_back(p);
	}
	cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(input_cloud);
	pca.project(*input_cloud, *cloud_filtered);

	max_rho = 0;
	cloud_pca.resize(cloud_filtered->points.size(), 3);
	for(int i = 0; i < cloud_filtered->points.size(); i++)
	{
		cloud_pca(i, 0) = cloud_filtered->points[i].x;
		cloud_pca(i, 1) = cloud_filtered->points[i].y;
		cloud_pca(i, 2) = -cloud_filtered->points[i].z;

		// get the farthest point distance
		double temp_rho = sqrt(
				cloud_filtered->points[i].x * cloud_filtered->points[i].x
			+ cloud_filtered->points[i].x * cloud_filtered->points[i].x
			+ cloud_filtered->points[i].z * cloud_filtered->points[i].z);

		if(temp_rho > max_rho)
		{
			max_rho = temp_rho;
		}
	}

	// get the signature matrix A
	A = GetSignatureMatrix();

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXd u = svd.matrixU();
	Eigen::MatrixXd v = svd.matrixV();
	Eigen::Matrix<double, 1, 64> u_temp;
	Eigen::Matrix<double, 1, 128> v_temp;
	u_temp = u.col(0);
	v_temp = v.col(0);
	m2dp << u_temp, v_temp;

	std::vector<float> m2dp_vec;
	for(int i = 0; i < m2dp.cols()*m2dp.rows(); i++)
	{
		m2dp_vec.push_back(m2dp[i]);
	}
	save(m2dp_vec, robot, index);

	return m2dp_vec;
}

Eigen::Matrix<double, 64, 128> m2dp_descriptor::GetSignatureMatrix()
{
	vector<double> thetaList(num_T_ + 1);
	for(int i = 0; i <= num_T_; i++)
	{
		thetaList[i] = -M_PI + i * 2 * M_PI / (num_T_);
	}

	vector<double> rhoList(num_R_ + 1);
	for(int i = 0; i <= num_R_; i++)
	{
		rhoList[i] = i * sqrt(max_rho) / num_R_;
		rhoList[i] = rhoList[i] * rhoList[i];
	}
	// make sure all points in bins
	rhoList[rhoList.size() - 1] = rhoList[rhoList.size() - 1] + 0.001;

	Eigen::Matrix<double, 64, 128> result_A;
	int index_A = 0;
	// loop on azimuth
	for(int i = 0; i < azimuth_list.size();i++)
	{
		auto azm = azimuth_list[i];
		// loop on elevation
		for(int j = 0; j < elevation_list.size();j++)
		{
			auto elv = elevation_list[j];
			// normal vector vecN of the selected 2D plane
			Eigen::Matrix<double, 1, 3> vecN;
			sph2cart(azm, elv, 1, vecN);
			// distance of vector [1,0,0] to the surface with normal vector vecN
			Eigen::Matrix<double, 1, 3> op(1, 0, 0);
			Eigen::MatrixXd h = op * vecN.transpose();
			// a new vector, c = h*vecN, so that vector [1,0,0]-c is the projection of x-axis onto the plane with normal vector vecN
			Eigen::Matrix<double, 1, 3> c = h(0) * vecN;
			// x-axis - c, the projection
			Eigen::Matrix<double, 1, 3> px = op - c;
			// given the normal vector vecN and the projected x-axis px, the y- axis is cross(vecN,px)
			Eigen::Matrix<double, 1, 3> py = vecN.cross(px);
			// projection of data onto space span{px,py}
			Eigen::MatrixXd pcx = cloud_pca * px.transpose();
			Eigen::MatrixXd pcy = cloud_pca * py.transpose();
			// pdata = np.array([pcx,pcy])
			// represent data in polar coordinates
			// vector<double> rho;
			// vector<double> theta;
			std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points; // x: rho  y: theta
			for(int i = 0; i < pcx.rows(); i++)
			{
				Eigen::Vector2d temp;
				cart2pol(pcx(i), pcy(i), temp);
				points.push_back(temp);
			}
			// main function, count points in bins
			Eigen::MatrixXd hist; //16*8    thetaList 17   rhoList 9
			histogram2d(points, thetaList, rhoList, hist);
			hist = hist / cloud_filtered->points.size();
			int hist_size = hist.cols() * hist.rows();
			for (int i = 0; i < hist_size; i++)
			{
				result_A(index_A, i) = hist(i);
			}
			index_A++;
		}
	}
	return result_A;
}

void m2dp_descriptor::sph2cart(
	double azm,
	double elv,
	double r,
	Eigen::Matrix<double, 1, 3> &vecN)
{
	double x, y, z;
	x = r * cos(elv) * cos(azm);
	y = r * cos(elv) * sin(azm);
	z = r * sin(elv);
	vecN << x, y, z;
}

void m2dp_descriptor::cart2pol(
	double x,
	double y,
	Eigen::Vector2d &vecN)
{
	vecN.x() = sqrt(x * x + y * y); // rho
	vecN.y() = atan2(y, x);         // phi
}

void m2dp_descriptor::histogram2d(
	std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points,
	vector<double> thetaList,
	vector<double> rhoList,
	Eigen::MatrixXd &hist)
{
	int row, col;
	row = thetaList.size() - 1;
	col = rhoList.size() - 1;
	hist = Eigen::MatrixXd::Zero(row, col);
	// Points x: rho  y: theta
	for(auto pt : points)
	{
		int row_index = -1, col_index = -1;
		for(int i = 0; i <= row; i++)
		{
			if(pt.y() < thetaList[i])
			{
				row_index = i - 1;
				break;
			}
		}
		for(int j = 0; j <= col; j++)
		{
			if(pt.x() < rhoList[j])
			{
				col_index = j - 1;
				break;
			}
		}
		if(row_index >= 0 && row_index < row && col_index >= 0 && col_index < col)
		{
			hist(row_index, col_index)++;
		}
	}
}

void m2dp_descriptor::save(
	const std::vector<float> m2dp_vec,
	const int8_t robot,
	const int index)
{
	Eigen::VectorXf m2dp_mat = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(m2dp_vec.data(), 192, 1);
	m2dp_dictionaries[robot].push_back(m2dp_mat);
	indexes_maps[robot].push_back(m2dp_index_pairs.size());
	m2dp_index_pairs.push_back(std::make_pair(robot, index));
}

void m2dp_descriptor::saveDescriptorAndKey(
	const float* descriptorMat,
	const int8_t robot,
	const int index)
{

}

std::pair<int, float> m2dp_descriptor::detectIntraLoopClosureID(
	const int cur_ptr)
{

}

std::pair<int, float> m2dp_descriptor::detectInterLoopClosureID(
	const int cur_ptr)
{
	
}

std::pair<int8_t, int> m2dp_descriptor::getIndex(
	const int key)
{
	return m2dp_index_pairs[key];
}

int m2dp_descriptor::getSize(
	const int id = -1)
{
	return m2dp_index_pairs.size();
}
