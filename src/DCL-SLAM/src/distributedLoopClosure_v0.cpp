#include "distributedMapping.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	class distributedMapping: handle message callback 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//a1 作为发起者，a0 作为接受者的例子：

//a1 (ID=1) 发现潜在回环：
//   a1 的前端里程计或全局描述符匹配模块发现，自己当前的某个关键帧（索引为 k）可能与
//   a0 的某个关键帧（索引为 m）形成回环。

//a1 发送点云请求 (Situation 1)：
//   a1 需要 a0 的关键帧 m 的点云来进行验证。于是它构建一个 loop_info 消息：
//   msg.robot0 = 1 (因为 a1 是发起者)
//   msg.robot1 = 0 (因为 a0 是目标)
//   msg.index0 = k
//   msg.index1 = m
//   msg.noise = 999 (标志位：请求点云)
//   a1 通过自己的发布者 robots[1].pub_loop_info 将消息发布到 /a1/distributedMapping/loopInfo 话题上。

//a0 (ID=0) 接收并响应请求：
//a0 一直在订阅 a1 的回环话题。它收到了这条 noise=999 的消息。
//它检查 msg.robot1 == id_，即 0 == 0，判断自己就是目标接受者。
//于是，a0 准备好自己的关键帧 m 的点云，构建一个新的 loop_info 消息：
//   new_msg.robot0 = 1 (保持发起者不变)
//   new_msg.robot1 = 0 (保持目标不变)
//   new_msg.scan_cloud = [a0的点云数据] (填充自己的点云)
//   new_msg.noise = 888 (标志位：提供待验证数据)
//   a0 通过自己的发布者 robots[0].pub_loop_info 将消息发布到 /a0/distributedMapping/loopInfo 话题上。

//a1 接收并验证回环 (Situation 2)：
//a1 订阅了 a0 的回环话题，它收到了这条 noise=888 的消息。
//a1 在收到 a0 发来的 888 消息后，会直接进行验证，而不是通过 msg.robot1 来判断。
//msg.robot1 只是指明了点云的来源。实际的验证逻辑可能更直接：“我收到了a0发来的点云，我要用它和我自己的点云做匹配”。
// a1 对自己的点云（索引 k）和收到的 a0 的点云（索引 m）进行 ICP 等匹配算法。

//a1 发布验证结果 (Situation 3)：
//如果验证成功，a1 会计算出一个精确的相对位姿 pose_between。它会构建最后一个 loop_info 消息：
//  final_msg.robot0 = 1
//  final_msg.robot1 = 0
//  final_msg.pose_between = [计算出的相对位姿]
//  final_msg.noise = [一个表示不确定性的数值] (标志位：已验证的结果)
//  a1 将这个最终结果发布出去。

//a0 和 a1 融合回环约束：
//a0 和 a1 都会收到这个最终消息。
//它们都会检查 msg->robot0 == id_ || msg->robot1 == id_。对于 a0，msg->robot1 == 0 为真；对于 a1，msg->robot0 == 1 为真。
//因此，a0 和 a1 都会将这个回环约束添加到自己的本地位姿图中，并触发分布式优化。

//////////////////////////////////////////////////////////////////
void distributedMapping::loopInfoHandler(
	const dcl_slam::loop_infoConstPtr& msg,
	//dcl_slam::loop_info 是一个自定义的消息类型，包含了回环检测所需的所有信息（如涉及的机器人 ID、关键帧索引、位姿、点云等）
	int& id)
	//int& id是一个额外的参数，通过 boost::bind 传入，代表发送这条消息的机器人的 ID。
{
	// Situation 1: need to add pointcloud for loop closure verification
	//当一个机器人（发起者）发现了一个潜在的跨机回环，但需要另一个机器人（目标）的关键帧点云来进行验证时，会发送一个 noise 值为 999 的请求
	if((int)msg->noise == 999)//判断当前收到的消息是一个 “点云请求”
	{
		if(msg->robot0 != id_)// 检查消息的目标接收者是否是自己 (robot0 是发起者，robot1 是目标)
		{
			return;
		}

		// 发送回环验证请求，创建一个新的消息来封装点云数据
		dcl_slam::loop_info loop_msg;
		loop_msg.robot0 = msg->robot0;// 复制发起者机器人ID
		loop_msg.robot1 = msg->robot1;// 复制目标机器人ID
		loop_msg.index0 = msg->index0;// 复制发起者的关键帧索引
		loop_msg.index1 = msg->index1;// 复制目标的关键帧索引
		loop_msg.init_yaw = msg->init_yaw;// 复制初始的角度估计（用于验证的初始值）
		loop_msg.noise = 888.0; // 标记为需要验证,将消息的标志位从 999 (请求) 改为 888 (待验证数据)

		// 安全检查：确保索引有效，防止程序崩溃
		CHECK_LT(loop_msg.index0,keyposes_cloud_6d->size());
		CHECK_LT(loop_msg.index0,robots[id_].keyframe_cloud_array.size());

		// 准备要发送的点云数据
		// filtered pointcloud
		//创建一个临时的点云指针
		pcl::PointCloud<PointPose3D>::Ptr cloudTemp(new pcl::PointCloud<PointPose3D>());
		// 从本地关键帧点云数组中，取出被请求的那个关键帧点云
		*cloudTemp = robots[id_].keyframe_cloud_array[loop_msg.index0];
		// 使用体素滤波对点云进行降采样，减少数据量，加快传输和后续的验证速度
		downsample_filter_for_inter_loop2.setInputCloud(cloudTemp);
		downsample_filter_for_inter_loop2.filter(*cloudTemp);
		// 将 PCL 格式的点云转换为 ROS 消息格式sensor_msgs/PointCloud2，并存储在 loop_msg.scan_cloud 字段中以便通过话题发布
		pcl::toROSMsg(*cloudTemp,loop_msg.scan_cloud);// 疑似1：附加点云数据
		// 打包与该关键帧对应的位姿
		loop_msg.pose0 = gtsamPoseToTransform(pclPointTogtsamPose3(keyposes_cloud_6d->points[loop_msg.index0]));

		//  通过本地的 pub_loop_info 发布者发布到目标机器人
		// 将这个包含了降采样点云和标志位 888 的新消息广播出去
		// 其他机器人（特别是发起回环请求的机器人）订阅了这个话题，就会收到这条消息并进入场景二的处理逻辑
		robots[id_].pub_loop_info.publish(loop_msg);// 疑似1：发布到目标机器人
		//对于 ID 为 0 的机器人 (id_ = 0)：
        //   robot.name_ 的值是 "/a0"。
        //   pub_loop_info 发布者被创建在话题 "/a0/distributedMapping/loopInfo" 上。
        //   当它执行 robots[id_].pub_loop_info.publish(loop_msg) 时，消息就被发布到了 /a0/distributedMapping/loopInfo。

        //对于 ID 为 1 的机器人 (id_ = 1)：
        //   robot.name_ 的值是 "/a1"。
        //   pub_loop_info 发布者被创建在话题 "/a1/distributedMapping/loopInfo" 上。
        //   当它执行 robots[id_].pub_loop_info.publish(loop_msg) 时，消息就被发布到了 /a1/distributedMapping/loopInfo。
	}
	// Situation 2: 处理待验证的回环数据
	// 当一个机器人收到了带有 888 标志位的消息
	// 意味着它收到了另一个机器人的关键帧点云，可以开始进行回环验证
	else if((int)msg->noise == 888)
	{
		// 身份验证：确保自己是这个回环验证任务的执行者
		// 确保只有消息中指定的目标机器人（robot1）才会处理这个验证任务，防止网络中其他无关机器人误处理
		if(msg->robot1 != id_)
		{
			return;
		}

		// LOG(INFO) << "[loopInfoHandler(" << id << ")]" << " check loop "
		// 	<< msg->robot0 << "-" << msg->index0 << " " << msg->robot1 << "-" << msg->index1 << "." << endl;

		// 将收到的待验证消息（包含了对方的点云和自身的索引）存入一个全局队列
		// 将完整的消息（包括对方的点云 msg->scan_cloud）加入到一个名为 loop_closures_candidates 的队列中
		loop_closures_candidates.push_back(*msg);
	}
	// Situation 3: add verified loop closure
	// 如果 noise 不是 999 也不是 888，那么它就是一个已验证的回环结果
	else
	{
		// LOG(INFO) << "[loopInfoHandler(" << id << ")] add loop "
		// 	<< msg->robot0 << "-" << msg->index0 << " " << msg->robot1 << "-" << msg->index1 << "." << endl;

		// extract loop
		// 为这个已验证的回环创建一个噪声模型（不确定性）
		Vector Vector6(6);
		Vector6 << msg->noise, msg->noise, msg->noise, msg->noise, msg->noise, msg->noise;
		noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(Vector6);
		// 将消息中的位姿（transform）转换为 GTSAM 库中的 Pose3 格式
		Pose3 pose_between = transformToGtsamPose(msg->pose_between);
		// 处理其他机器人的回环验证请求，验证通过后发布结果
		// 创建一个 GTSAM 中的 BetweenFactor，这是一个位姿图约束
		// 它表示 robot0 的 index0 关键帧和 robot1 的 index1 关键帧之间存在一个名为 pose_between 的相对位姿关系
		NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
			Symbol('a'+msg->robot0, msg->index0), Symbol('a'+msg->robot1, msg->index1), pose_between, loop_noise));// 构建位姿约束
		
		// 更新邻接矩阵，记录机器人之间的连接关系
		adjacency_matrix(msg->robot0, msg->robot1) += 1;
		adjacency_matrix(msg->robot1, msg->robot0) += 1;
		// 检查这个回环是否与自己有关
		if(msg->robot0 == id_ || msg->robot1 == id_)
		{
			// 如果与自己有关，就将这个回环约束（factor）加入到本地的位姿图中
			local_pose_graph->add(factor);
			local_pose_graph_no_filtering->add(factor);
			// 设置一个标志位，通知系统有新的回环，需要启动或重新进行分布式优化
			sent_start_optimization_flag = true;

			// update pose estimate (for PCM)
			Key key;
			graph_utils::PoseWithCovariance pose;
			pose.covariance_matrix = loop_noise->covariance();
			// 更新本地对 robot1 关键帧的位姿估计
			pose.pose = transformToGtsamPose(msg->pose1);
			key = Symbol('a'+msg->robot1, msg->index1).key();
			updatePoseEstimateFromNeighbor(msg->robot1, key, pose);
			// 更新本地对 robot0 关键帧的位姿估计
			pose.pose = transformToGtsamPose(msg->pose0);
			key = Symbol('a'+msg->robot0, msg->index0).key();
			updatePoseEstimateFromNeighbor(msg->robot0, key, pose);

			// 将这个回环变换添加到 PCM (Pose Consensus Map) 中
			auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
			Matrix covariance_matrix = loop_noise->covariance();
			robot_local_map.addTransform(*new_factor, covariance_matrix);
		}
	}
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	class distributedMapping: loop closure
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void distributedMapping::performRSIntraLoopClosure()
{
	if(copy_keyposes_cloud_3d->size() <= intra_robot_loop_ptr || intra_robot_loop_closure_enable_)
	{
		return;
	}

	// find intra loop closure with radius search
	auto matching_result = detectLoopClosureDistance(intra_robot_loop_ptr);
	int loop_key0 = intra_robot_loop_ptr;
	int loop_key1 = matching_result;
	intra_robot_loop_ptr++;

	if(matching_result < 0) // no loop found
	{
		return;
	}

	// LOG(INFO) << "[IntraLoopRS<" << id_ << ">] [" << loop_key0 << "] and [" << loop_key1 << "]." << endl;

	calculateTransformation(loop_key0, loop_key1);
}

int distributedMapping::detectLoopClosureDistance(
	const int& cur_ptr)
{
	int loop_key0 = cur_ptr;
	int loop_key1 = -1;

	// find the closest history key frame
	vector<int> indices;
	vector<float> distances;
	kdtree_history_keyposes->setInputCloud(copy_keyposes_cloud_3d);
	kdtree_history_keyposes->radiusSearch(copy_keyposes_cloud_3d->points[cur_ptr],
		search_radius_, indices, distances, 0);
	
	for (int i = 0; i < (int)indices.size(); ++i)
	{
		int index = indices[i];
		if(loop_key0 > exclude_recent_frame_num_ + index)
		{
			loop_key1 = index;
			break;
		}
	}

	if(loop_key1 == -1 || loop_key0 == loop_key1)
	{
		return -1;
	}

	return loop_key1;
}

void distributedMapping::performIntraLoopClosure()
{
	if(keyframe_descriptor->getSize(id_) <= intra_robot_loop_ptr || !intra_robot_loop_closure_enable_)
	{
		return;
	}

	// find intra loop closure with global descriptor
	auto matching_result = keyframe_descriptor->detectIntraLoopClosureID(intra_robot_loop_ptr);
	int loop_key0 = intra_robot_loop_ptr;
	int loop_key1 = matching_result.first;
	intra_robot_loop_ptr++;

	if(matching_result.first < 0) // no loop found
	{
		return;
	}

	// LOG(INFO) << "[IntraLoop<" << id_ << ">] [" << loop_key0 << "] and [" << loop_key1 << "]." << endl;

	calculateTransformation(loop_key0, loop_key1);
}

void distributedMapping::calculateTransformation(
	const int& loop_key0,
	const int& loop_key1)
{
	CHECK_LT(loop_key0, copy_keyposes_cloud_6d->size());

	// get initial pose
	Pose3 loop_pose0 = pclPointTogtsamPose3(copy_keyposes_cloud_6d->points[loop_key0]);
	Pose3 loop_pose1 = pclPointTogtsamPose3(copy_keyposes_cloud_6d->points[loop_key1]);

	// extract cloud
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	loopFindNearKeyframes(scan_cloud, loop_key0, 0);
	downsample_filter_for_intra_loop.setInputCloud(scan_cloud);
	downsample_filter_for_intra_loop.filter(*scan_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr map_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr map_cloud_ds(new pcl::PointCloud<PointPose3D>());
	loopFindNearKeyframes(map_cloud, loop_key1, history_keyframe_search_num_);
	downsample_filter_for_intra_loop.setInputCloud(map_cloud);
	downsample_filter_for_intra_loop.filter(*map_cloud_ds);

	// fail safe check for cloud
	if(scan_cloud->size() < 300 || map_cloud->size() < 1000)
	{
		ROS_WARN("keyFrameCloud too little points 1");
		return;
	}

	// publish cloud
	if(pub_scan_of_scan2map.getNumSubscribers() != 0)
	{
		sensor_msgs::PointCloud2 scan_cloud_msg;
		pcl::toROSMsg(*scan_cloud_ds, scan_cloud_msg);
		scan_cloud_msg.header.stamp = ros::Time::now();
		scan_cloud_msg.header.frame_id = world_frame_;
		pub_scan_of_scan2map.publish(scan_cloud_msg);
	}
	if(pub_map_of_scan2map.getNumSubscribers() != 0)
	{
		sensor_msgs::PointCloud2 map_cloud_msg;
		pcl::toROSMsg(*map_cloud_ds, map_cloud_msg);
		map_cloud_msg.header.stamp = ros::Time::now();
		map_cloud_msg.header.frame_id = world_frame_;
		pub_map_of_scan2map.publish(map_cloud_msg);
	}
	
	// icp settings
	static pcl::IterativeClosestPoint<PointPose3D, PointPose3D> icp;
	icp.setMaxCorrespondenceDistance(2*search_radius_);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(0);
	// icp.setRANSACOutlierRejectionThreshold(ransac_outlier_reject_threshold_);

	// align clouds
	icp.setInputSource(scan_cloud_ds);
	icp.setInputTarget(map_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr unused_result(new pcl::PointCloud<PointPose3D>());
	icp.align(*unused_result);

	// check if pass ICP fitness score
	float fitness_score = icp.getFitnessScore();
	if(icp.hasConverged() == false || fitness_score > fitness_score_threshold_)
	{
		ROS_DEBUG("\033[1;34m[IntraLoop<%d>] [%d]-[%d] ICP failed (%.2f > %.2f). Reject.\033[0m",
			id_, loop_key0, loop_key1, fitness_score, fitness_score_threshold_);
		// LOG(INFO) << "[IntraLoop<" << id_ << ">] ICP failed ("
		// 	<< fitness_score << " > " << fitness_score_threshold_ << "). Reject." << endl;
		return;
	}
	ROS_DEBUG("\033[1;34m[IntraLoop<%d>] [%d]-[%d] ICP passed (%.2f < %.2f). Add.\033[0m",
		id_, loop_key0, loop_key1, fitness_score, fitness_score_threshold_);
	// LOG(INFO) << "[IntraLoop<" << id_ << ">] ICP passed ("
	// 	<< fitness_score << " < " << fitness_score_threshold_ << "). Add." << endl;

	// get pose transformation
	float x, y, z, roll, pitch, yaw;
	Eigen::Affine3f icp_final_tf;
	icp_final_tf = icp.getFinalTransformation();
	pcl::getTranslationAndEulerAngles(icp_final_tf, x, y, z, roll, pitch, yaw);
	Eigen::Affine3f origin_tf = gtsamPoseToAffine3f(loop_pose0);
	Eigen::Affine3f correct_tf = icp_final_tf * origin_tf;
	pcl::getTranslationAndEulerAngles(correct_tf, x, y, z, roll, pitch, yaw);
	Pose3 pose_from = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
	Pose3 pose_to = loop_pose1;
	Pose3 pose_between = pose_from.between(pose_to);
	// LOG(INFO) << "[IntraLoop<" << id_ << ">] pose_between: " << pose_between.translation().x() << " "
	// 	<< pose_between.translation().y() << " " << pose_between.translation().z() << "." << endl;
	
	// add loop factor
	Vector vector6(6);
	vector6 << fitness_score, fitness_score, fitness_score, fitness_score, fitness_score, fitness_score;
	noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(vector6);
	NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
		Symbol('a'+id_, loop_key0), Symbol('a'+id_, loop_key1), pose_between, loop_noise));
	isam2_graph.add(factor);
	local_pose_graph->add(factor);
	local_pose_graph_no_filtering->add(factor);
	sent_start_optimization_flag = true; // enable distributed mapping
	intra_robot_loop_close_flag = true;

	// save loop factor in local map (for PCM)
	auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
	Matrix covariance = loop_noise->covariance();
	robot_local_map.addTransform(*new_factor, covariance);

	auto it = loop_indexs.find(loop_key0);
	if(it == loop_indexs.end() || (it != loop_indexs.end() && it->second != loop_key1))
	{
		loop_indexs[loop_key0] = loop_key1;
	}
}

void distributedMapping::loopFindNearKeyframes(
	pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
	const int& key,
	const int& search_num)
{
	// extract near keyframes
	near_keyframes->clear();
	int pose_num = copy_keyposes_cloud_6d->size();
	CHECK_LE(pose_num, robots[id_].keyframe_cloud_array.size());
	for(int i = -search_num; i <= search_num; ++i)
	{
		int key_near = key + i;
		if(key_near < 0 || key_near >= pose_num)
		{
			continue;
		}
		*near_keyframes += *transformPointCloud(
			robots[id_].keyframe_cloud_array[key_near], &copy_keyposes_cloud_6d->points[key_near]);
	}

	if(near_keyframes->empty())
	{
		return;
	}
}

void distributedMapping::performInterLoopClosure()
{
	// early return
	if(keyframe_descriptor->getSize() <= inter_robot_loop_ptr || !inter_robot_loop_closure_enable_)
	{
		return;
	}

	// Place Recognition: find candidates with global descriptor
	auto matching_result = keyframe_descriptor->detectInterLoopClosureID(inter_robot_loop_ptr);
	int loop_robot0 = keyframe_descriptor->getIndex(inter_robot_loop_ptr).first;
	int loop_robot1 = keyframe_descriptor->getIndex(matching_result.first).first;
	int loop_key0 = keyframe_descriptor->getIndex(inter_robot_loop_ptr).second;
	int loop_key1 = keyframe_descriptor->getIndex(matching_result.first).second;
	float init_yaw = matching_result.second;
	inter_robot_loop_ptr++;

	if(matching_result.first < 0) // no loop found
	{
		return;
	}

	// LOG(INFO) << "[InterLoop<" << id_ << ">] found between ["
	// 	<< loop_robot0 << "]-[" << loop_key0 << "][" << inter_robot_loop_ptr-1 << "] and ["
	// 	<< loop_robot1 << "]-[" << loop_key1 << "][" << matching_result.first << "]." << endl;

	dcl_slam::loop_info inter_loop_candidate;
	inter_loop_candidate.robot0 = loop_robot0;
	inter_loop_candidate.robot1 = loop_robot1;
	inter_loop_candidate.index0 = loop_key0;
	inter_loop_candidate.index1 = loop_key1;
	inter_loop_candidate.init_yaw = init_yaw;
	if(loop_robot0 != id_) // send to other for filling pointcloud
	{
		inter_loop_candidate.noise = 999.0;
	}
	else // fill filtered pointcloud
	{
		CHECK_LT(loop_key0, keyposes_cloud_6d->size());
		CHECK_LT(loop_key0, robots[loop_robot0].keyframe_cloud_array.size());
		
		inter_loop_candidate.noise = 888.0;
		pcl::PointCloud<PointPose3D>::Ptr scan_cloud(new pcl::PointCloud<PointPose3D>());
		pcl::PointCloud<PointPose3D>::Ptr scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
		*scan_cloud = robots[loop_robot0].keyframe_cloud_array[loop_key0];
		downsample_filter_for_inter_loop3.setInputCloud(scan_cloud);
		downsample_filter_for_inter_loop3.filter(*scan_cloud_ds);
		pcl::toROSMsg(*scan_cloud_ds, inter_loop_candidate.scan_cloud);
		inter_loop_candidate.pose0 = gtsamPoseToTransform(pclPointTogtsamPose3(keyposes_cloud_6d->points[loop_key0]));
	}
	robots[id_].pub_loop_info.publish(inter_loop_candidate);
}

void distributedMapping::performExternLoopClosure()
{
	// early return
	if(loop_closures_candidates.empty() || !inter_robot_loop_closure_enable_)
	{
		return;
	}

	// extract loop for verification
	dcl_slam::loop_info inter_loop = loop_closures_candidates.front();
	loop_closures_candidates.pop_front();

	auto loop_symbol0 = Symbol('a'+inter_loop.robot0, inter_loop.index0);
	auto loop_symbol1 = Symbol('a'+inter_loop.robot1, inter_loop.index1);
	// check the loop closure if added before
	auto find_key_indexes0 = loop_indexes.find(loop_symbol0);
	auto find_key_indexes1 = loop_indexes.find(loop_symbol1);
	if (find_key_indexes0->second.chr() == loop_symbol1.chr() ||
		find_key_indexes1->second.chr() == loop_symbol0.chr())
	{
		ROS_DEBUG("\033[1;33m[LoopClosure] Loop has added. Skip.\033[0m");
		return;
	}

	// fail safe
	if (initial_values->size() < history_keyframe_search_num_*2 || initial_values->size() <= inter_loop.index1)
	{
		loop_closures_candidates.push_back(inter_loop);
		return;
	}

	// logging
	// LOG(INFO) << "[performExternLoopClosure<" << id_ << ">] Loop: "
	// 	<< inter_loop.robot0 << " " << inter_loop.index0 << " "
	// 	<< inter_loop.robot1 << " " << inter_loop.index1 << endl;
	
	// get initial pose
	CHECK_LT(inter_loop.index1, initial_values->size());
	double initial_yaw_;
	if (descriptor_type_num_ == DescriptorType::LidarIris)
	{
		initial_yaw_ = (inter_loop.init_yaw+1)*2*M_PI/60.0;
	}
	else
	{
		initial_yaw_ = inter_loop.init_yaw*M_PI/180.0;
	}
	if(initial_yaw_ > M_PI)
		initial_yaw_ -= 2*M_PI;
	
	auto initial_loop_pose0 = initial_values->at<Pose3>(loop_symbol1);

	auto loop_pose0 = Pose3(
		Rot3::RzRyRx(
			initial_loop_pose0.rotation().roll(),
			initial_loop_pose0.rotation().pitch(),
			initial_loop_pose0.rotation().yaw() + initial_yaw_),
		Point3(
			initial_loop_pose0.translation().x(),
			initial_loop_pose0.translation().y(),
			initial_loop_pose0.translation().z()));

	auto loop_pose1 = initial_values->at<Pose3>(loop_symbol1);

	// extract cloud
	pcl::PointCloud<PointPose3D>::Ptr scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	pcl::fromROSMsg(inter_loop.scan_cloud, *scan_cloud_ds);
	*scan_cloud_ds = *transformPointCloud(*scan_cloud_ds, loop_pose0);
	pcl::PointCloud<PointPose3D>::Ptr map_cloud(new pcl::PointCloud<PointPose3D>());
	pcl::PointCloud<PointPose3D>::Ptr map_cloud_ds(new pcl::PointCloud<PointPose3D>());
	loopFindGlobalNearKeyframes(map_cloud, inter_loop.index1, history_keyframe_search_num_);
	downsample_filter_for_inter_loop.setInputCloud(map_cloud); // downsample near keyframes
	downsample_filter_for_inter_loop.filter(*map_cloud_ds);

	// safe check for cloud
	if (scan_cloud_ds->size() < 300 || map_cloud_ds->size() < 1000)
	{
		ROS_WARN("keyFrameCloud too little points 2");
		return;
	}
	if (!scan_cloud_ds->is_dense || !map_cloud_ds->is_dense)
	{
		ROS_WARN("keyFrameCloud is not dense");
		return;
	}

	// publish cloud
	if(pub_scan_of_scan2map.getNumSubscribers() != 0)
	{
		sensor_msgs::PointCloud2 scan_cloud_msg;
		pcl::toROSMsg(*scan_cloud_ds, scan_cloud_msg);
		scan_cloud_msg.header.stamp = ros::Time::now();
		scan_cloud_msg.header.frame_id = world_frame_;
		pub_scan_of_scan2map.publish(scan_cloud_msg);
	}
	if(pub_map_of_scan2map.getNumSubscribers() != 0)
	{
		sensor_msgs::PointCloud2 map_cloud_msg;
		pcl::toROSMsg(*map_cloud_ds, map_cloud_msg);
		map_cloud_msg.header.stamp = ros::Time::now();
		map_cloud_msg.header.frame_id = world_frame_;
		pub_map_of_scan2map.publish(map_cloud_msg);
	}

	/*** calculate transform using icp ***/
	// ICP Settings
	static pcl::IterativeClosestPoint<PointPose3D, PointPose3D> icp;
	icp.setMaxCorrespondenceDistance(30);
	icp.setMaximumIterations(100);	
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(ransac_maximum_iteration_);
	icp.setRANSACOutlierRejectionThreshold(ransac_outlier_reject_threshold_);

	// Align clouds
	icp.setInputSource(scan_cloud_ds);
	icp.setInputTarget(map_cloud_ds);
	pcl::PointCloud<PointPose3D>::Ptr correct_scan_cloud_ds(new pcl::PointCloud<PointPose3D>());
	icp.align(*correct_scan_cloud_ds);
	inter_loop.noise = icp.getFitnessScore();

	/*** verification using RANSAC ***/
	// initial matching
	boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<PointPose3D, PointPose3D> correspondence_estimation;
	correspondence_estimation.setInputCloud(correct_scan_cloud_ds);
	correspondence_estimation.setInputTarget(map_cloud_ds);
	correspondence_estimation.determineCorrespondences(*correspondences);

	// RANSAC matching to find inlier
	pcl::Correspondences new_correspondences;
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointPose3D> correspondence_ransac;
	correspondence_ransac.setInputSource(correct_scan_cloud_ds);
	correspondence_ransac.setInputTarget(map_cloud_ds);
	correspondence_ransac.setMaximumIterations(ransac_maximum_iteration_);
	correspondence_ransac.setInlierThreshold(ransac_outlier_reject_threshold_);
	correspondence_ransac.setInputCorrespondences(correspondences);
	correspondence_ransac.getCorrespondences(new_correspondences);

	// check if pass RANSAC outlier threshold
	if(new_correspondences.size() < ransac_threshold_*correspondences->size())
	{
		ROS_DEBUG("\033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] RANSAC failed (%.2f < %.2f). Reject.\033[0m",
			id_, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
			new_correspondences.size()*1.0/correspondences->size()*1.0, ransac_threshold_);
		// LOG(INFO) << "[InterLoop<" << id_ << ">] RANSAC failed ("
		// 	<< new_correspondences.size()*1.0/correspondences->size()*1.0 << " < " 
		// 	<< ransac_threshold_ << "). Reject." << endl;
		return;
	}
	// check if pass ICP fitness score
	if(icp.hasConverged() == false || inter_loop.noise > fitness_score_threshold_*2)
	{
		ROS_DEBUG("\033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] ICP failed (%.2f > %.2f). Reject.\033[0m",
			id_, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
			inter_loop.noise, fitness_score_threshold_*2);
		// LOG(INFO) << "[InterLoop<" << id_ << ">] ICP failed ("
		// 	<< inter_loop.noise << " > " << fitness_score_threshold_*2 << "). Reject." << endl;
		return;
	}
	ROS_DEBUG("\033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] inlier (%.2f < %.2f) fitness (%.2f < %.2f). Add.\033[0m",
		id_, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
		new_correspondences.size()*1.0/correspondences->size()*1.0, ransac_threshold_,
		inter_loop.noise, fitness_score_threshold_*2);
	// LOG(INFO) << "[InterLoop<" << id_ << ">] inlier ("
	// 	<< new_correspondences.size()*1.0/correspondences->size()*1.0
	// 	<< " < " << ransac_threshold_ << ") fitness (" << inter_loop.noise << " < "
	// 	<< fitness_score_threshold_*2 << "). Add." << endl;

	// get pose transformation
	auto icp_final_tf = Pose3(icp.getFinalTransformation().cast<double>());
	auto pose_from = icp_final_tf * loop_pose0;
    auto pose_to = loop_pose1;

	inter_loop.pose1 = gtsamPoseToTransform(pclPointTogtsamPose3(keyposes_cloud_6d->points[inter_loop.index1]));
	if(inter_loop.robot0 > inter_loop.robot1) // the first robot always set to the lower id
	{
		swap(pose_from, pose_to);
		swap(inter_loop.robot0, inter_loop.robot1);
		swap(inter_loop.index0, inter_loop.index1);
		swap(inter_loop.pose0, inter_loop.pose1);
	}
	Pose3 pose_between = pose_from.between(pose_to);
	inter_loop.pose_between = gtsamPoseToTransform(pose_between);
	// LOG(INFO) << "[InterLoop<" << id_ << ">] pose_between: " << pose_between.translation().x()
	// 	<< " " << pose_between.translation().y() << " " << pose_between.translation().z() << endl;

	// get noise model
	Vector Vector6(6);
	Vector6 << inter_loop.noise, inter_loop.noise, inter_loop.noise, inter_loop.noise, 
		inter_loop.noise, inter_loop.noise;
	noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(Vector6);
	// add factor
	NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
		Symbol('a'+inter_loop.robot0, inter_loop.index0),
		Symbol('a'+inter_loop.robot1, inter_loop.index1),
		pose_between, loop_noise));
	adjacency_matrix(inter_loop.robot1, inter_loop.robot0) += 1;
	adjacency_matrix(inter_loop.robot0, inter_loop.robot1) += 1;
	local_pose_graph->add(factor);
	local_pose_graph_no_filtering->add(factor);
	sent_start_optimization_flag = true; // enable distributed mapping

	// update pose estimate (for PCM)
	Key key;
	graph_utils::PoseWithCovariance pose;
	pose.covariance_matrix = loop_noise->covariance();
	pose.pose = transformToGtsamPose(inter_loop.pose1);
	key = loop_symbol1.key();
	updatePoseEstimateFromNeighbor(inter_loop.robot1, key, pose);
	pose.pose = transformToGtsamPose(inter_loop.pose0);
	key = loop_symbol0.key();
	updatePoseEstimateFromNeighbor(inter_loop.robot0, key, pose);

	// add transform to local map (for PCM)
	auto new_factor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor);
	Matrix covariance_matrix = loop_noise->covariance();
	robot_local_map.addTransform(*new_factor, covariance_matrix);

	// publish loop closure
	robots[id_].pub_loop_info.publish(inter_loop);
	loop_indexes.emplace(make_pair(loop_symbol0, loop_symbol1));
	loop_indexes.emplace(make_pair(loop_symbol1, loop_symbol0));
}

void distributedMapping::loopFindGlobalNearKeyframes(
	pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
	const int& key,
	const int& search_num)
{
	// extract near keyframes
	near_keyframes->clear();
	int pose_num = initial_values->size();
	CHECK_LE(pose_num, robots[id_].keyframe_cloud_array.size());
	int add_num = 0;
	for(int i = -search_num; i <= search_num*2; ++i)
	{
		if(add_num >= search_num*2)
		{
			break;
		}

		int key_near = key + i;
		if(key_near < 0 || key_near >= pose_num)
		{
			continue;
		}
		
		*near_keyframes += *transformPointCloud(robots[id_].keyframe_cloud_array[key_near],
			initial_values->at<Pose3>(Symbol('a'+id_, key_near)));
		add_num++;
	}

	if(near_keyframes->empty())
	{
		return;
	}
}

void distributedMapping::updatePoseEstimateFromNeighbor(
	const int& rid,
	const Key& key,
	const graph_utils::PoseWithCovariance& pose)
{
	graph_utils::TrajectoryPose trajectory_pose;
	trajectory_pose.id = key;
	trajectory_pose.pose = pose;
	// find trajectory
	if(pose_estimates_from_neighbors.find(rid) != pose_estimates_from_neighbors.end())
	{
		// update pose
		if(pose_estimates_from_neighbors.at(rid).trajectory_poses.find(key) != 
			pose_estimates_from_neighbors.at(rid).trajectory_poses.end())
		{
			pose_estimates_from_neighbors.at(rid).trajectory_poses.at(key) = trajectory_pose;
		}
		// new pose
		else
		{
			pose_estimates_from_neighbors.at(rid).trajectory_poses.insert(make_pair(key, trajectory_pose));
			if(key < pose_estimates_from_neighbors.at(rid).start_id)
			{
				pose_estimates_from_neighbors.at(rid).start_id = key;
			}
			if(key > pose_estimates_from_neighbors.at(rid).end_id)
			{
				pose_estimates_from_neighbors.at(rid).end_id = key;
			}
		}
	}
	// insert new trajectory
	else
	{
		graph_utils::Trajectory new_trajectory;
		new_trajectory.trajectory_poses.insert(make_pair(key, trajectory_pose));
		new_trajectory.start_id = key;
		new_trajectory.end_id = key;
		pose_estimates_from_neighbors.insert(make_pair(rid, new_trajectory));
	}
}

void distributedMapping::loopClosureThread()
{
	// Terminate the thread if loop closure are not needed
	if(!intra_robot_loop_closure_enable_ && !inter_robot_loop_closure_enable_)
	{
		return;
	}

	ros::Rate rate(1.0/loop_closure_process_interval_);

	while(ros::ok())
	{
		rate.sleep();

		performRSIntraLoopClosure(); // find intra-loop with radius search

		performIntraLoopClosure(); // find intra-loop with descriptor

		performInterLoopClosure(); // find inter-loop with descriptor

		performExternLoopClosure(); // verify all inter-loop here
	}
}