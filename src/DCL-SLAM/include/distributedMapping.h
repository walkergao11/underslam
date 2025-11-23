#ifndef _DISTRIBUTED_MAPPING_H_
#define _DISTRIBUTED_MAPPING_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <flann/flann.hpp>
#include <thread>
#include <deque>
#include <mutex>  // <-- 补充：用于类中mutex成员的声明
// dcl_slam define
#include "paramsServer.h"
#include "scanContextDescriptor.h"
#include "lidarIrisDescriptor.h"
#include "m2dpDescriptor.h"
#include "dcl_slam/loop_info.h"
#include "dcl_slam/global_descriptor.h"
#include "dcl_slam/neighbor_estimate.h"

//新增
#include <pcc_project/CompressedPointCloudData.h>
#include "../../pcc_project/utils/struct.h"   //可能没用//可能又有用了

// pcl
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
// mapping
#include "distributed_mapper/distributed_mapper.h"
#include "distributed_mapper/distributed_mapper_utils.h"
#include <gtsam/nonlinear/ISAM2.h>
// file iostream
#include <fstream>
#include <iostream>
// log
#include <glog/logging.h>
// <-- FIX: 明确使用 boost::shared_ptr
//#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

class DecoderModule;//避免在.h文件中包含decoder_module.h，减少耦合

using namespace gtsam;
using namespace std;

class distributedMapping : public paramsServer
{
	public:
		distributedMapping();

		~distributedMapping();

		pcl::PointCloud<PointPose3D>::Ptr getLocalKeyposesCloud3D();

		pcl::PointCloud<PointPose6D>::Ptr getLocalKeyposesCloud6D();

		pcl::PointCloud<PointPose3D> getLocalKeyframe(const int& index);

		Pose3 getLatestEstimate();

		void lockOnCall();

		void unlockOnCall();

		void performDistributedMapping(
			const Pose3& pose_to,
			const pcl::PointCloud<PointPose3D>::Ptr frame_to,
			const ros::Time& timestamp);

		void compressPointCloud(
        pcl::PointCloud<PointPose3D>::Ptr cloud, 
        int tile_size, 
        int q_level, 
        std::vector<char>& out_data, 
        std::vector<char>& out_meta
    );

		bool saveFrame(
			const Pose3& pose_to);

		void updateLocalPath(
			const PointPose6D& pose);

		bool updatePoses();

		void makeDescriptors();

		void publishPath();

		void publishTransformation(
			const ros::Time& timestamp);

		void loopClosureThread();

		void globalMapThread();
		void performInterLoopClosure();
		
		// 回调函数ROS框架调用
   		void compressedPointCloudCallback(
    	const pcc_project::CompressedPointCloudData::ConstPtr& compressed_msg, 
    	int robot_id
    	);

	private:
		// 回调函数
		bool decompressPointCloud(const pcc_project::CompressedPointCloudData& compressed_data, pcl::PointCloud<PointPose3D>::Ptr& cloud_out);

		void poseCovariance2msg(
			const graph_utils::PoseWithCovariance& pose,
			geometry_msgs::PoseWithCovariance& msg);

		void msg2poseCovariance(
			const geometry_msgs::PoseWithCovariance& msg,
			graph_utils::PoseWithCovariance& pose);

		void globalDescriptorHandler(
			const dcl_slam::global_descriptorConstPtr& msg,
			int& id);

		void loopInfoHandler(
			const dcl_slam::loop_infoConstPtr& msg,
			int& id);

		void optStateHandler(
			const std_msgs::Int8ConstPtr& msg,
			int& id);

		void rotationStateHandler(
			const std_msgs::Int8ConstPtr& msg,
			int& id);

		void poseStateHandler(
			const std_msgs::Int8ConstPtr& msg,
			int& id);

		void neighborRotationHandler(
			const dcl_slam::neighbor_estimateConstPtr& msg,
			int& id);

		void neighborPoseHandler(
			const dcl_slam::neighbor_estimateConstPtr& msg,
			int& id);

		void updatePoseEstimateFromNeighbor(
			const int& rid,
			const Key& key,
			const graph_utils::PoseWithCovariance& pose);

		bool startOptimizationCondition();

		void updateOptimizer();

		void outliersFiltering();

		void computeOptimizationOrder();

		void initializePoseGraphOptimization();

		bool rotationEstimationStoppingBarrier();

		void abortOptimization(
			const bool& log_info);

		void removeInactiveNeighbors();

		void failSafeCheck();

		void initializePoseEstimation();

		bool poseEstimationStoppingBarrier();

		void updateGlobalPath(
			const Pose3& pose_in);

		void incrementalInitialGuessUpdate();

		void endOptimization();

		void changeOptimizerState(
			const OptimizerState& state);

		void run(const ros::TimerEvent&);

		void performRSIntraLoopClosure();

		int detectLoopClosureDistance(
			const int& cur_ptr);
		
		void performIntraLoopClosure();

		void calculateTransformation(
			const int& loop_key_cur,
			const int& loop_key_pre);

		void loopFindNearKeyframes(
			pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
			const int& key, const int& search_num);


		void performExternLoopClosure();

		void loopFindGlobalNearKeyframes(
			pcl::PointCloud<PointPose3D>::Ptr& near_keyframes,
			const int& key, const int& search_num);

		void publishGlobalMap();

		void publishLoopClosureConstraint();

	public:
		mutex lock_on_call; // lock on odometry

	private:
		/*** robot team ***/
		vector<singleRobot> robots;

		/*** ros subscriber and publisher ***/
		ros::Publisher pub_loop_closure_constraints;
		ros::Publisher pub_scan_of_scan2map, pub_map_of_scan2map;
		ros::Publisher pub_global_map;
		ros::Publisher pub_global_path, pub_local_path;
		ros::Publisher pub_keypose_cloud;

		/*** ros service ***/

		/*** message information ***/
		pcl::PointCloud<PointPose3D>::Ptr cloud_for_decript_ds; // input cloud for descriptor
		deque<pair<int, dcl_slam::global_descriptor>> store_descriptors;
		std_msgs::Int8 state_msg; // optimization state msg
		dcl_slam::global_descriptor global_descriptor_msg; // descriptor message
		
		// 临时存储待发送的 loop_info 消息 (使用 map 缓存) ---
		// 用于存储等待点云压缩完成的 loop_info 消息
		// key为dcl_seq，value为loop_info消息
    	std::unordered_map<int, dcl_slam::loop_info> pending_loop_msgs_;
    	// 添加互斥锁，保护多线程访问
    	std::mutex pending_loop_mutex_; 
		// 用于生成唯一的 seq 号，关联原始 loop_info 和压缩后的点云
		int loop_msg_counter_ = 0;
		// ------------------------------------------------

		nav_msgs::Path local_path; // path in local frame
		nav_msgs::Path global_path; // path in global frame

		/*** downsample filter ***/
		pcl::VoxelGrid<PointPose3D> downsample_filter_for_descriptor;
		pcl::VoxelGrid<PointPose3D> downsample_filter_for_intra_loop;
		pcl::VoxelGrid<PointPose3D> downsample_filter_for_inter_loop;
		pcl::VoxelGrid<PointPose3D> downsample_filter_for_inter_loop2;
		pcl::VoxelGrid<PointPose3D> downsample_filter_for_inter_loop3;

		/*** mutex ***/
		// vector<mutex> lock_on_call; // lock on odometry

		/*** distributed loopclosure ***/
		int intra_robot_loop_ptr; // current position pointer for intra-robot loop
		int inter_robot_loop_ptr; // current position pointer for inter-robot loop

		bool intra_robot_loop_close_flag; // intra-robot loop is detected

		unique_ptr<scan_descriptor> keyframe_descriptor; // descriptor for keyframe pointcloud

		deque<dcl_slam::loop_info> loop_closures_candidates; // loop closures need to verify

		// radius search for intra-robot loop closure
		pcl::PointCloud<PointPose3D>::Ptr copy_keyposes_cloud_3d; // copy of local 3-dof keyposes
		pcl::PointCloud<PointPose6D>::Ptr copy_keyposes_cloud_6d; // copy of local 6-dof keyposes

		pcl::KdTreeFLANN<PointPose3D>::Ptr kdtree_history_keyposes; // kdtree for searching history keyposes

		map<int, int> loop_indexs;
		map<Symbol, Symbol> loop_indexes;

		/*** noise model ***/
		noiseModel::Diagonal::shared_ptr odometry_noise; // odometry factor noise
		noiseModel::Diagonal::shared_ptr prior_noise; // prior factor noise

		/*** local pose graph optmazition ***/
		ISAM2 *isam2; // isam2 optimizer

		NonlinearFactorGraph isam2_graph; // local pose graph for isam2
		Values isam2_initial_values; // local initial values for isam2

		Values isam2_current_estimates; // current estimates for isam2
		Pose3 isam2_keypose_estimate; // keypose estimate for isam2

		pcl::PointCloud<PointPose3D>::Ptr keyposes_cloud_3d; // 3-dof keyposes in local frame
		pcl::PointCloud<PointPose6D>::Ptr keyposes_cloud_6d; // 6-dof keyposes in local frame

		/*** distributed pose graph optmazition ***/
		ros::Timer distributed_mapping_thread; // thread for running distributed mapping
		boost::shared_ptr<distributed_mapper::DistributedMapper> optimizer; // distributed mapper (DGS)

		int steps_of_unchange_graph; // stop optimization 

		// measurements
		boost::shared_ptr<NonlinearFactorGraph> local_pose_graph; // pose graph for distributed mapping
		boost::shared_ptr<Values> initial_values; // initial values for distributed mapping
		GraphAndValues graph_values_vec; // vector of pose graph and initial values

		bool graph_disconnected; // pose graph is not connected to others

		int lowest_id_included; // lowest id in this robot
		int lowest_id_to_included; // lowest id to be included in this robot
		int prior_owner; // the robot that own prior factor
		bool prior_added; // this robot have add prior factor

		gtsam::Matrix adjacency_matrix; // adjacency matrix of robot team
		vector<int> optimization_order; // optimization order of robot team
		bool in_order; // this robot in optimization order

		// this robot
		OptimizerState optimizer_state; // current state of optimizer
		int optimization_steps; // steps in optimization
		bool sent_start_optimization_flag; // ready for optimization

		int current_rotation_estimate_iteration; // current iteration time of rotation estimate
		int current_pose_estimate_iteration; // current iteration time of pose estimate

		double latest_change; // latest change of estimate
		int steps_without_change; // setps of estimate without change

		bool rotation_estimate_start; // rotation estimate is start
		bool pose_estimate_start; // pose estimate is start
		bool rotation_estimate_finished; // rotation estimate is finished
		bool pose_estimate_finished; // pose estimate is finished
		bool estimation_done; // estimate is done

		Point3 anchor_offset, anchor_point; // anchor offset

		// neighbors
		set<char> neighboring_robots; // neighbors (name) within communication range
		set<int> neighbors_within_communication_range; // neighbors (id) within communication range
		map<int, bool> neighbors_started_optimization; // neighbors ready for optimization
		map<int, OptimizerState> neighbor_state; // current state of neighbors optimizer

		map<int, bool> neighbors_rotation_estimate_finished; // neighbors rotation estimate is finished
		map<int, bool> neighbors_pose_estimate_finished; // neighbors pose estimate is finished
		map<int, bool> neighbors_estimation_done; // neighbors estimate is done

		map<int, int> neighbors_lowest_id_included; // lowest id in neighbors
		map<int, Point3> neighbors_anchor_offset; // neighbors anchor offset

		// distributed pairwise consistency maximization
		robot_measurements::RobotLocalMap robot_local_map; // local loop closures and transform 
		robot_measurements::RobotLocalMap robot_local_map_backup; // backups in case of abort

		boost::shared_ptr<NonlinearFactorGraph> local_pose_graph_no_filtering; // pose graph without pcm

		map<int, graph_utils::Trajectory> pose_estimates_from_neighbors; // pose estimates of neighbors
		set<Key> other_robot_keys_for_optimization; // keys of neighbors for optimization

		set<pair<Key, Key>> accepted_keys, rejected_keys; // accepted and rejected pairs
		int measurements_accepted_num, measurements_rejected_num;
};

#endif
