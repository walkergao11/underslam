// distributedLoopClosure.cpp
#include "distributedMapping.h"

// RCPCC 解压模块
#include "modules/decoder_module.h"
#include "utils/struct.h"

#include <vector>
#include <exception>
#include <atomic> // compress seq
#include <mutex>


std::atomic<int> compress_seq{0};
std::mutex pending_loop_mutex_;
std::unordered_map<int, dcl_slam::loop_info> pending_loop_msgs_; 
std::unordered_map<int, pcc_project::CompressedPointCloudData> pending_compressed_msgs_;
int last_processed_seq = -1;


void distributedMapping::loopInfoHandler(
    const dcl_slam::loop_infoConstPtr& msg,
    int& id)
{
    ROS_INFO("[loopInfoHandler] LOOP MSG: requester=%d, target=%d, current=%d, noise=%.1f, loop_seq=%d",
             msg->robot1, msg->robot0, id_, msg->noise, msg->header.seq);

    
    if ((int)msg->noise == 999)
    {
        if (msg->robot0 != id_) return; // not target

        int seq_local = compress_seq.fetch_add(1);

        dcl_slam::loop_info loop_msg = *msg;
        loop_msg.noise = 888.0;                 
        loop_msg.header.seq = seq_local;        
        loop_msg.header.stamp = ros::Time::now();
        loop_msg.tile_size = robots[id_].tile_size;
        loop_msg.q_level   = robots[id_].q_level;

        {
            std::lock_guard<std::mutex> lock(pending_loop_mutex_);
            pending_loop_msgs_[seq_local] = loop_msg;
            ROS_INFO("[loopInfoHandler] Cached pending loop request seq=%d (pending_loop_msgs_ size=%lu)",
                     seq_local, pending_loop_msgs_.size());
        }

       
        if (loop_msg.index0 < 0 || loop_msg.index0 >= (int)robots[id_].keyframe_cloud_array.size())
        {
            ROS_ERROR("[loopInfoHandler] index0 out of range idx=%d size=%lu",
                      loop_msg.index0, robots[id_].keyframe_cloud_array.size());
            std::lock_guard<std::mutex> lock(pending_loop_mutex_);
            pending_loop_msgs_.erase(seq_local);
            return;
        }

        
        pcl::PointCloud<PointPose3D>::Ptr cloud(new pcl::PointCloud<PointPose3D>());
        *cloud = robots[id_].keyframe_cloud_array[loop_msg.index0];

        downsample_filter_for_inter_loop2.setInputCloud(cloud);
        downsample_filter_for_inter_loop2.filter(*cloud);

        sensor_msgs::PointCloud2 raw_msg;
        pcl::toROSMsg(*cloud, raw_msg);
        raw_msg.header.seq = seq_local;                 
        raw_msg.header.stamp = ros::Time::now();
        raw_msg.header.frame_id = "point_cloud_compress";

        robots[id_].pub_point_cloud_to_compress.publish(raw_msg);
        ROS_INFO("[loopInfoHandler] Sent raw cloud to compressor: seq=%d", seq_local);
        return;
    }

    
    if ((int)msg->noise == 888)
    {
        int seq = msg->header.seq;
        ROS_INFO("[loopInfoHandler] DETECTED noise==888, start DECOMPRESS, seq=%d", seq);

        pcc_project::CompressedPointCloudData comp;

        {
            std::lock_guard<std::mutex> lock(pending_loop_mutex_);
            auto it = pending_compressed_msgs_.find(seq);

            if (it == pending_compressed_msgs_.end())
            {
                ROS_WARN("[loopInfoHandler] Compressed data for seq=%d not ready yet, skip for now", seq);
                return;
            }

            comp = it->second;
            pending_compressed_msgs_.erase(it);
            ROS_INFO("[loopInfoHandler] Using cached compressed data seq=%d size=%lu",
                     seq, comp.data.size());
        }

        if (comp.data.empty())
        {
            ROS_ERROR("[loopInfoHandler] Cached compressed data empty seq=%d", seq);
            return;
        }

        pcl::PointCloud<PointPose3D>::Ptr out_cloud(new pcl::PointCloud<PointPose3D>());
        if (!decompressPointCloud(comp, out_cloud))
        {
            ROS_ERROR("[loopInfoHandler] decompressPointCloud FAILED seq=%d", seq);
            return;
        }

        ROS_INFO("[loopInfoHandler] decompress OK seq=%d pts=%lu", seq, out_cloud->size());

        dcl_slam::loop_info filled = *msg;
        pcl::toROSMsg(*out_cloud, filled.scan_cloud);
        filled.scan_cloud.header.seq = seq;
        filled.scan_cloud.header.stamp = ros::Time::now();
        if (filled.scan_cloud.header.frame_id.empty())
            filled.scan_cloud.header.frame_id = "point_cloud_decompressed";

        loop_closures_candidates.push_back(filled);
        ROS_INFO("[loopInfoHandler] Added DECOMPRESSED loop info to queue seq=%d", seq);
        return;
    }
    else
    {
    
    Vector Vector6(6);
    Vector6 << msg->noise, msg->noise, msg->noise,
               msg->noise, msg->noise, msg->noise;

    auto loop_noise = noiseModel::Diagonal::Variances(Vector6);
    Pose3 pose_between = transformToGtsamPose(msg->pose_between);

    auto factor = boost::make_shared<BetweenFactor<Pose3>>(
        Symbol('a' + msg->robot0, msg->index0),
        Symbol('a' + msg->robot1, msg->index1),
        pose_between, loop_noise);

    adjacency_matrix(msg->robot0, msg->robot1) += 1;
    adjacency_matrix(msg->robot1, msg->robot0) += 1;

    if (msg->robot0 == id_ || msg->robot1 == id_)
    {
        local_pose_graph->add(factor);
        local_pose_graph_no_filtering->add(factor);
        sent_start_optimization_flag = true;

        Key key;
        graph_utils::PoseWithCovariance pose;

        pose.covariance_matrix = loop_noise->covariance();
        pose.pose = transformToGtsamPose(msg->pose1);
        key = Symbol('a' + msg->robot1, msg->index1).key();
        updatePoseEstimateFromNeighbor(msg->robot1, key, pose);

        pose.pose = transformToGtsamPose(msg->pose0);
        key = Symbol('a' + msg->robot0, msg->index0).key();
        updatePoseEstimateFromNeighbor(msg->robot0, key, pose);

        robot_local_map.addTransform(*factor, loop_noise->covariance());
        ROS_INFO("[loopInfoHandler] Verified loop added.");
    }
    }
}


void distributedMapping::compressedPointCloudCallback(
    const pcc_project::CompressedPointCloudData::ConstPtr& compressed_msg,
    int robot_id)
{
    int seq = compressed_msg->header.seq;
    ROS_INFO("[compressedPointCloudCallback] recv robot=%d seq=%d size=%lu sender_id=%d",
             robot_id, seq, compressed_msg->data.size(), compressed_msg->sender_id);

    {
        std::lock_guard<std::mutex> lock(pending_loop_mutex_);
        pending_compressed_msgs_[seq] = *compressed_msg;
        ROS_INFO("[compressedPointCloudCallback] cached compressed packet seq=%d (cache size=%lu)",
                 seq, pending_compressed_msgs_.size());
    }

    // 对齐 pending_loop_msgs_ 并立即触发解压
    while (true)
    {
        int next = last_processed_seq + 1;
        std::lock_guard<std::mutex> lock(pending_loop_mutex_);

        if (!pending_loop_msgs_.count(next) || !pending_compressed_msgs_.count(next))
            break;

        dcl_slam::loop_info info = pending_loop_msgs_[next];
        pcc_project::CompressedPointCloudData comp = pending_compressed_msgs_[next];

        comp.tile_size = info.tile_size;
        comp.q_level   = info.q_level;
        info.compressed_scan_cloud = comp;
        info.compressed_scan_cloud.sender_id = compressed_msg->sender_id;

        robots[robot_id].pub_loop_info.publish(info);
        ROS_INFO("[compressedPointCloudCallback] Published aligned loop_info seq=%d", next);

        pending_loop_msgs_.erase(next);
        pending_compressed_msgs_.erase(next);
        last_processed_seq = next;
    }
}


bool distributedMapping::decompressPointCloud(
    const pcc_project::CompressedPointCloudData& comp,
    pcl::PointCloud<PointPose3D>::Ptr& cloud_out)
{
    try
    {
        ROS_INFO("[decompressPointCloud] start seq=%d data_size=%lu tile=%d q=%d sender=%d",
                 comp.header.seq, comp.data.size(), comp.tile_size, comp.q_level, comp.sender_id);

        if (comp.data.empty())
        {
            ROS_ERROR("[decompressPointCloud] empty data seq=%d", comp.header.seq);
            return false;
        }

        std::vector<char> buf(comp.data.begin(), comp.data.end());
        if (buf.empty())
        {
            ROS_ERROR("[decompressPointCloud] buffer empty seq=%d", comp.header.seq);
            return false;
        }

        std::vector<char> decoded = decompressData(buf);
        if (decoded.empty())
        {
            ROS_ERROR("[decompressPointCloud] decompressData returned empty seq=%d", comp.header.seq);
            return false;
        }

        DecoderModule dec(decoded, comp.tile_size, false, comp.q_level);

        if (dec.restored_pcloud.empty())
        {
            ROS_ERROR("[decompressPointCloud] restored_pcloud empty seq=%d", comp.header.seq);
            return false;
        }

        cloud_out->resize(dec.restored_pcloud.size());
        for (size_t i = 0; i < dec.restored_pcloud.size(); ++i)
        {
            const auto &p = dec.restored_pcloud[i];
            cloud_out->points[i].x = p.x;
            cloud_out->points[i].y = p.y;
            cloud_out->points[i].z = p.z;
            cloud_out->points[i].intensity = p.intensity;
        }

        ROS_INFO("[decompressPointCloud] OK seq=%d pts=%lu", comp.header.seq, cloud_out->size());
        return true;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[decompressPointCloud] std::exception seq=%d: %s", comp.header.seq, e.what());
        return false;
    }
    catch (...)
    {
        ROS_ERROR("[decompressPointCloud] unknown exception seq=%d", comp.header.seq);
        return false;
    }
}








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
        return;
    }
    ROS_DEBUG("\033[1;34m[IntraLoop<%d>] [%d]-[%d] ICP passed (%.2f < %.2f). Add.\033[0m",
        id_, loop_key0, loop_key1, fitness_score, fitness_score_threshold_);

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

        inter_loop_candidate.compressed_scan_cloud.tile_size = robots[loop_robot0].tile_size;
        inter_loop_candidate.compressed_scan_cloud.q_level   = robots[loop_robot0].q_level;

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
    if(loop_closures_candidates.empty() || !inter_robot_loop_closure_enable_)
    {
        return;
    }

    dcl_slam::loop_info inter_loop = loop_closures_candidates.front();
    loop_closures_candidates.pop_front();

    auto loop_symbol0 = Symbol('a'+inter_loop.robot0, inter_loop.index0);
    auto loop_symbol1 = Symbol('a'+inter_loop.robot1, inter_loop.index1);

    // check the loop closure if added before
    auto find_key_indexes0 = loop_indexes.find(loop_symbol0);
    auto find_key_indexes1 = loop_indexes.find(loop_symbol1);
    if (find_key_indexes0 != loop_indexes.end() && find_key_indexes1 != loop_indexes.end())
    {
        if (find_key_indexes0->second.chr() == loop_symbol1.chr() ||
            find_key_indexes1->second.chr() == loop_symbol0.chr())
        {
            ROS_DEBUG("\033[1;33m[LoopClosure] Loop has added. Skip.\033[0m");
            return;
        }
    }

    // fail safe
    if (initial_values->size() < history_keyframe_search_num_*2 || initial_values->size() <= inter_loop.index1)
    {
        loop_closures_candidates.push_back(inter_loop);
        return;
    }

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
        return;
    }
    // check if pass ICP fitness score
    if(icp.hasConverged() == false || inter_loop.noise > fitness_score_threshold_*2)
    {
        ROS_DEBUG("\033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] ICP failed (%.2f > %.2f). Reject.\033[0m",
            id_, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
            inter_loop.noise, fitness_score_threshold_*2);
        return;
    }
    ROS_DEBUG("\033[1;35m[InterLoop<%d>] [%d][%d]-[%d][%d] inlier (%.2f < %.2f) fitness (%.2f < %.2f). Add.\033[0m",
        id_, inter_loop.robot0, inter_loop.index0, inter_loop.robot1, inter_loop.index1,
        new_correspondences.size()*1.0/correspondences->size()*1.0, ransac_threshold_,
        inter_loop.noise, fitness_score_threshold_*2);

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

    // get noise model
    Vector Vector6(6);
    Vector6 << inter_loop.noise, inter_loop.noise, inter_loop.noise, inter_loop.noise,
        inter_loop.noise, inter_loop.noise;
    noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(Vector6);

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

    // get noise model
    Vector Vector6(6);
    Vector6 << inter_loop.noise, inter_loop.noise, inter_loop.noise, inter_loop.noise,
        inter_loop.noise, inter_loop.noise;
    noiseModel::Diagonal::shared_ptr loop_noise = noiseModel::Diagonal::Variances(Vector6);

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
