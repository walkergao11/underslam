#include "distributedMapping.h"
#include "pcc_project/CompressedPointCloudData.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	class distributedMapping: constructor and destructor
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
distributedMapping::distributedMapping() : paramsServer()
{
	string log_name = name_+"_distributed_mapping";
	google::InitGoogleLogging(log_name.c_str());
	//string log_dir = "/log";
	string log_dir = ".ros/log";
	FLAGS_log_dir = std::getenv("HOME") + log_dir;
	// LOG(INFO) << "distributed mapping class initialization" << endl;

	/*** robot team ***/
	singleRobot robot; // each robot
	// 为每个机器人初始化通信对象
	for(int it = 0; it < number_of_robots_; it++) // 机器人名称格式：/a0, /a1, ...（对应ID 0,1,...）
	{
		/*** robot information ***/
		robot.id_ = it; // robot ID and name
		robot.name_ = "/a";
		robot.name_[1] += it;
		robot.odom_frame_ = robot.name_ + "/" + odom_frame_; // odom frame

		// 为每个机器人加载 pcc_compressor 的 tile_size 和 q_level 参数
		std::string compressor_param_prefix = "/pcc_compressor_";
		// 提取机器人名称（去掉前缀 /，如 /a → a）
		std::string robot_short_name = robot.name_.substr(1); // robot.name_ 是 /a, /b, /c
		int tile_size = 0, q_level = 0;

// 从参数服务器读取（与 launch 文件中 pcc_compressor 配置对应）
if (ros::param::get(compressor_param_prefix + robot_short_name + "/tile_size", tile_size) &&
    ros::param::get(compressor_param_prefix + robot_short_name + "/q_level", q_level)) {
    robot.tile_size = tile_size;
    robot.q_level = q_level;
    ROS_INFO("Robot %s 加载压缩配置成功: tile_size=%d, q_level=%d",
             robot.name_.c_str(), tile_size, q_level);
} else {
    // 若读取失败，设默认值（避免全0）
    robot.tile_size = 4;
    robot.q_level = 3;
    ROS_WARN("Robot %s 未找到压缩参数，使用默认值: tile_size=4, q_level=3",
             robot.name_.c_str());
}

		/*** ros subscriber and publisher ***/
		// 若为当前机器人（自身），初始化发布者
		if(it == id_)
		{
			if(intra_robot_loop_closure_enable_ || inter_robot_loop_closure_enable_)
			{
				// 发布全局描述符（用于回环检测）
				robot.pub_descriptors = nh.advertise<dcl_slam::global_descriptor>(
					robot.name_+"/distributedMapping/globalDescriptors", 5);
				// 发布回环信息
				robot.pub_loop_info = nh.advertise<dcl_slam::loop_info>(
					robot.name_+"/distributedMapping/loopInfo", 5);
			}
			
			// 发布优化状态和位姿估计用于分布式优化
			if(global_optmization_enable_)
			{
				robot.pub_optimization_state = nh.advertise<std_msgs::Int8>(
					robot.name_+"/distributedMapping/optimizationState", 50);
				robot.pub_rotation_estimate_state = nh.advertise<std_msgs::Int8>(
					robot.name_+"/distributedMapping/rotationEstimateState", 50);
				robot.pub_pose_estimate_state = nh.advertise<std_msgs::Int8>(
					robot.name_+"/distributedMapping/poseEstimateState", 50);
				robot.pub_neighbor_rotation_estimates = nh.advertise<dcl_slam::neighbor_estimate>(
					robot.name_+"/distributedMapping/neighborRotationEstimates", 50);
				robot.pub_neighbor_pose_estimates = nh.advertise<dcl_slam::neighbor_estimate>(
					robot.name_+"/distributedMapping/neighborPoseEstimates", 50);
			}
		}

		// 若为其他机器人，初始化订阅者
		else
		{
			if(intra_robot_loop_closure_enable_ || inter_robot_loop_closure_enable_)
			{
				// 订阅其他机器人的全局描述符
				robot.sub_descriptors = nh.subscribe<dcl_slam::global_descriptor>(
					robot.name_+"/distributedMapping/globalDescriptors", 50,
					boost::bind(&distributedMapping::globalDescriptorHandler, this, _1, it));
				// 订阅其他机器人的回环信息
				robot.sub_loop_info = nh.subscribe<dcl_slam::loop_info>(
					robot.name_+"/distributedMapping/loopInfo", 50,
					boost::bind(&distributedMapping::loopInfoHandler, this, _1, it));
			}

			// 订阅其他机器人的优化状态和位姿估计用于分布式优化
			if(global_optmization_enable_)
			{
				robot.sub_optimization_state = nh.subscribe<std_msgs::Int8>(
					robot.name_+"/distributedMapping/optimizationState", 50,
					boost::bind(&distributedMapping::optStateHandler, this, _1, it));
				robot.sub_rotation_estimate_state = nh.subscribe<std_msgs::Int8>(
					robot.name_+"/distributedMapping/rotationEstimateState", 50,
					boost::bind(&distributedMapping::rotationStateHandler, this, _1, it));
				robot.sub_pose_estimate_state = nh.subscribe<std_msgs::Int8>(
					robot.name_+"/distributedMapping/poseEstimateState", 50,
					boost::bind(&distributedMapping::poseStateHandler, this, _1, it));
				robot.sub_neighbor_rotation_estimates = nh.subscribe<dcl_slam::neighbor_estimate>(
					robot.name_+"/distributedMapping/neighborRotationEstimates", 50,
					boost::bind(&distributedMapping::neighborRotationHandler, this, _1, it));
				robot.sub_neighbor_pose_estimates = nh.subscribe<dcl_slam::neighbor_estimate>(
					robot.name_+"/distributedMapping/neighborPoseEstimates", 50,
					boost::bind(&distributedMapping::neighborPoseHandler, this, _1, it));
			}
		}

		/*** other ***/
		robot.time_cloud_input_stamp.init();
		robot.time_cloud_input = 0.0;

		robot.keyframe_cloud.reset(new pcl::PointCloud<PointPose3D>());
		robot.keyframe_cloud_array.clear();

		// 初始化PCC相关的发布者和订阅者
    	if(it == id_) {  // 只给自身机器人创建发布者（避免重复发布）
        std::string compress_input_topic = robot.name_ + "/distributedMapping/point_cloud_to_compress";
        robot.pub_point_cloud_to_compress = nh.advertise<sensor_msgs::PointCloud2>(compress_input_topic, 10);
        ROS_INFO("Robot %s: 初始化待压缩点云发布者，话题: %s", 
                 robot.name_.c_str(), compress_input_topic.c_str());
    	}

    	// 初始化当前机器人的压缩点云订阅者（订阅其他机器人的压缩消息）
    	//std::string compress_output_topic = robot.name_ + "/distributedMapping/compressed_point_cloud";
    	//robot.sub_compressed_point_cloud = nh.subscribe(compress_output_topic, 10,
        //&distributedMapping::compressedPointCloudCallback, this);
    	std::string compress_output_topic = robot.name_ + "/distributedMapping/compressed_point_cloud";
		// 步骤1：用 boost::function 包装 bind 生成的函数对象
		boost::function<void(const pcc_project::CompressedPointCloudData::ConstPtr&)> callback =
    	boost::bind(&distributedMapping::compressedPointCloudCallback,this,_1,it);
		// 为每个机器人订阅对应话题（如it=0→订阅/a的话题，it=1→订阅/b的话题）
		robot.sub_compressed_point_cloud = nh.subscribe<pcc_project::CompressedPointCloudData>(
    		compress_output_topic, 10, callback);
		ROS_INFO("Robot %s: 订阅压缩话题: %s", robot.name_.c_str(), compress_output_topic.c_str());
    	robots.push_back(robot);  // 存入机器人列表
	}


	/*** ros subscriber and publisher ***/
	// loop closure visualization
	pub_loop_closure_constraints = nh.advertise<visualization_msgs::MarkerArray>(
		"distributedMapping/loopClosureConstraints", 1);
	// scan2map cloud
	pub_scan_of_scan2map = nh.advertise<sensor_msgs::PointCloud2>(
		"distributedMapping/scanOfScan2map", 1);
	pub_map_of_scan2map = nh.advertise<sensor_msgs::PointCloud2>(
		"distributedMapping/mapOfScan2map", 1);
	// global map visualization
	pub_global_map = nh.advertise<sensor_msgs::PointCloud2>(
		"distributedMapping/globalMap", 1);
	// path for independent robot
	pub_global_path = nh.advertise<nav_msgs::Path>(
		"distributedMapping/path", 1);
	pub_local_path = nh.advertise<nav_msgs::Path>(
		"distributedMapping/localPath", 1);
	// keypose cloud
	pub_keypose_cloud = nh.advertise<sensor_msgs::PointCloud2>(
		"distributedMapping/keyposeCloud", 1);

	/*** message information ***/
	cloud_for_decript_ds.reset(new pcl::PointCloud<PointPose3D>()); 

	/*** downsample filter ***/
	downsample_filter_for_descriptor.setLeafSize(descript_leaf_size_, descript_leaf_size_, descript_leaf_size_);
	downsample_filter_for_intra_loop.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
	downsample_filter_for_inter_loop.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
	downsample_filter_for_inter_loop2.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
	downsample_filter_for_inter_loop3.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);

	/*** mutex ***/
	// lock_on_call = vector<mutex>(number_of_robots_);
	global_path.poses.clear();
	local_path.poses.clear();

	/*** distributed loopclosure ***/
	inter_robot_loop_ptr = 0;
	intra_robot_loop_ptr = 0;

	intra_robot_loop_close_flag = false;

	if(descriptor_type_num_ == DescriptorType::ScanContext)
	{
		keyframe_descriptor = unique_ptr<scan_descriptor>(new scan_context_descriptor(
			20, 60, knn_candidates_, descriptor_distance_threshold_, 0, 80.0,
			exclude_recent_frame_num_, number_of_robots_, id_));
	}
	else if(descriptor_type_num_ == DescriptorType::LidarIris)
	{
		keyframe_descriptor = unique_ptr<scan_descriptor>(new lidar_iris_descriptor(
			iris_row_, iris_column_, n_scan_, descriptor_distance_threshold_,
			exclude_recent_frame_num_, match_mode_, knn_candidates_, 4, 18, 1.6, 0.75,
			number_of_robots_, id_));
	}
	else if(descriptor_type_num_ == DescriptorType::M2DP)
	{
		keyframe_descriptor = unique_ptr<scan_descriptor>(new m2dp_descriptor(
			16, 8, 4, 16, number_of_robots_, id_));
	}

	loop_closures_candidates.clear();
	loop_indexes.clear();

	// radius search
	copy_keyposes_cloud_3d.reset(new pcl::PointCloud<PointPose3D>());
	copy_keyposes_cloud_6d.reset(new pcl::PointCloud<PointPose6D>());
	kdtree_history_keyposes.reset(new pcl::KdTreeFLANN<PointPose3D>());

	/*** noise model ***/
	odometry_noise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
	prior_noise = noiseModel::Isotropic::Variance(6, 1e-12);

	/*** local pose graph optmazition ***/
	ISAM2Params parameters;
	parameters.relinearizeThreshold = 0.1;
	parameters.relinearizeSkip = 1;
	isam2 = new ISAM2(parameters); // isam2

	keyposes_cloud_3d.reset(new pcl::PointCloud<PointPose3D>());
	keyposes_cloud_6d.reset(new pcl::PointCloud<PointPose6D>());
	
	/*** distributed pose graph optmazition ***/
	optimizer = boost::shared_ptr<distributed_mapper::DistributedMapper>(
		new distributed_mapper::DistributedMapper(id_ + 'a'));

	steps_of_unchange_graph = 0;

	local_pose_graph = boost::make_shared<NonlinearFactorGraph>();
	initial_values = boost::make_shared<Values>();
	graph_values_vec = make_pair(local_pose_graph, initial_values);

	graph_disconnected = true;

	lowest_id_included = id_;
	lowest_id_to_included = lowest_id_included;
	prior_owner = id_;
	prior_added = false;

	adjacency_matrix = gtsam::zeros(number_of_robots_, number_of_robots_);
	optimization_order.clear();
	in_order = false;

	optimizer_state = OptimizerState::Idle;
	optimization_steps = 0;
	sent_start_optimization_flag = false;

	current_rotation_estimate_iteration = 0;
	current_pose_estimate_iteration = 0;

	latest_change = -1;
	steps_without_change = 0;

	rotation_estimate_start = false;
	pose_estimate_start = false;
	rotation_estimate_finished = false;
	pose_estimate_finished = false;
	estimation_done = false;

	neighboring_robots.clear();
	neighbors_within_communication_range.clear();
	neighbors_started_optimization.clear();
	neighbor_state.clear();

	neighbors_rotation_estimate_finished.clear();
	neighbors_pose_estimate_finished.clear();
	neighbors_estimation_done.clear();

	neighbors_lowest_id_included.clear();
	neighbors_anchor_offset.clear();

	local_pose_graph_no_filtering = boost::make_shared<NonlinearFactorGraph>();

	pose_estimates_from_neighbors.clear();
	other_robot_keys_for_optimization.clear();

	accepted_keys.clear();
	rejected_keys.clear();
	measurements_rejected_num = 0;
	measurements_accepted_num = 0;
	
	optimizer->setUseBetweenNoiseFlag(use_between_noise_); // use between noise or not in optimizePoses
	optimizer->setUseLandmarksFlag(use_landmarks_); // use landmarks
	optimizer->loadSubgraphAndCreateSubgraphEdge(graph_values_vec); // load subgraphs
	optimizer->setVerbosity(distributed_mapper::DistributedMapper::ERROR); // verbosity level
	optimizer->setFlaggedInit(use_flagged_init_);
	optimizer->setUpdateType(distributed_mapper::DistributedMapper::incUpdate);
	optimizer->setGamma(gamma_);

	if(global_optmization_enable_)
	{
		distributed_mapping_thread = nh.createTimer(
			ros::Duration(mapping_process_interval_), &distributedMapping::run, this);
	}

	// LOG(INFO) << "distributed mapping class initialization finish" << endl;
}

distributedMapping::~distributedMapping()
{

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	class distributedMapping: other function
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void distributedMapping::lockOnCall()
{
	// lock_on_call.lock();
}

void distributedMapping::unlockOnCall()
{
	// lock_on_call.unlock();
}

pcl::PointCloud<PointPose3D>::Ptr distributedMapping::getLocalKeyposesCloud3D()
{
	return keyposes_cloud_3d;
}

pcl::PointCloud<PointPose6D>::Ptr distributedMapping::getLocalKeyposesCloud6D()
{
	return keyposes_cloud_6d;
}

pcl::PointCloud<PointPose3D> distributedMapping::getLocalKeyframe(const int& index)
{
	return robots[id_].keyframe_cloud_array[index];
}

Pose3 distributedMapping::getLatestEstimate()
{
	return isam2_keypose_estimate;
}

void distributedMapping::poseCovariance2msg(
	const graph_utils::PoseWithCovariance& pose,
	geometry_msgs::PoseWithCovariance& msg)
{
	msg.pose.position.x = pose.pose.x();
	msg.pose.position.y = pose.pose.y();
	msg.pose.position.z = pose.pose.z();

	Vector quaternion = pose.pose.rotation().quaternion();
	msg.pose.orientation.w = quaternion(0);
	msg.pose.orientation.x = quaternion(1);
	msg.pose.orientation.y = quaternion(2);
	msg.pose.orientation.z = quaternion(3);

	for(int i = 0; i < 6; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			msg.covariance[i*6 + j] = pose.covariance_matrix(i, j);
		}
	}
}

void distributedMapping::msg2poseCovariance(
	const geometry_msgs::PoseWithCovariance& msg,
	graph_utils::PoseWithCovariance& pose)
{
	Rot3 rotation(msg.pose.orientation.w, msg.pose.orientation.x,
		msg.pose.orientation.y, msg.pose.orientation.z);
	Point3 translation(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
	
	pose.pose = Pose3(rotation, translation);

	pose.covariance_matrix = gtsam::zeros(6,6);
	for(int i = 0; i < 6; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			pose.covariance_matrix(i, j) = msg.covariance[i*6 + j];
		}
	}
}

