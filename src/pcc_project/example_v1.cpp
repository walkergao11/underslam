#include <iostream>
#include <fstream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>  
#include <chrono>  
#include "modules/encoder_module.h"
#include "modules/decoder_module.h"
#include "../utils/struct.h"
#include "../utils/utils.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <deque>
#include <mutex>
#include <thread>

// !!! 包含自定义的压缩消息头文件
#include <pcc_project/CompressedPointCloudData.h>

// !!! Feng: Supporting ROS
class PointCloudCompressor {
public:
    //测试代码
    PointCloudCompressor(ros::NodeHandle& nh) : nh_(nh), start_publishing_(false) {
    // 1. 读取 robot_prefix 并映射 sender_id（原有逻辑不变，正确）
    std::string robot_prefix;
    nh_.param<std::string>("robot_prefix", robot_prefix, "a");
    if (robot_prefix == "a") sender_id_ = 0;
    else if (robot_prefix == "b") sender_id_ = 1;
    else if (robot_prefix == "c") sender_id_ = 2;
    else { sender_id_ = 0; ROS_WARN("Unknown robot_prefix: %s", robot_prefix.c_str()); }

    // 2. input_topic_ 默认值改为「私有话题名」（不含绝对路径）
    nh_.param("input_topic", input_topic_, std::string("input_points")); 
    // 此时 input_topic_ 默认为 "input_points"，结合私有节点句柄（nh_ = ~），实际是 ~input_points

    // 3. output_topic_ 默认值改为「私有话题名」（不含绝对路径）
    nh_.param("output_topic", output_topic_, std::string("output_compressed")); 
    // 此时 output_topic_ 默认为 "output_compressed"，结合私有节点句柄，实际是 ~output_compressed

    // 4. 读取其他参数（压缩等级，原有逻辑不变）
    nh_.param("q_level", q_level_, 2);
    nh_.param("tile_size", tile_size_, 4);
    nh_.param("use_compress", use_compress_, true);

    try {
        encoder_.reset(new EncoderModule(tile_size_, q_level_));
        ROS_INFO("EncoderModule 初始化成功（tile_size=%d, q_level=%d）", tile_size_, q_level_);
    } catch (const std::exception& e) {
        ROS_FATAL("EncoderModule 初始化失败：%s", e.what());
        ros::shutdown();
        return;
    }

    // 5. 订阅和发布初始化（原有逻辑不变，但现在话题名是私有话题，remap 可生效）
    sub_ = nh_.subscribe(input_topic_, 10, &PointCloudCompressor::cloudCallback, this);
    pub_ = nh_.advertise<pcc_project::CompressedPointCloudData>(output_topic_, 10);

    // 6. 启动发布线程和日志（原有逻辑不变，更新日志显示）
    publish_thread_ = std::thread(&PointCloudCompressor::publishLoop, this);
    ROS_INFO("PCC Node Ready. Prefix: %s, Sender ID: %d, Subscribed to: ~%s, Published to: ~%s",
             robot_prefix.c_str(), sender_id_, input_topic_.c_str(), output_topic_.c_str());
}


private:
    // ROS消息回调函数
    // 消息处理回调
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {
  
        // 1. 解析DCL-SLAM的loop_msg seq（从frame_id中恢复）
    int dcl_seq = -1;
    try {
        dcl_seq = std::stoi(input_msg->header.frame_id);
        ROS_INFO("[pcc_compressor_b] Parsed DCL seq: %d from frame_id", dcl_seq);
    } catch (...) {
        ROS_WARN("Failed to parse dcl_seq from frame_id: %s", input_msg->header.frame_id.c_str());
        return;
    }

    // 2. 回调触发日志
    ROS_INFO("[pcc_compressor_b] Callback triggered! Received point cloud: input_seq=%d, dcl_seq=%d, size=%dx%d",
             input_msg->header.seq, dcl_seq, input_msg->width, input_msg->height);

    // 3. 点云格式转换
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *pcl_cloud); // 注意：该函数返回void，不能用于条件判断

    // 4. 过滤无效点云（通过点云是否为空间接判断转换是否有效）
    const int MIN_POINTS = 10;
    if (pcl_cloud->empty() || pcl_cloud->size() < MIN_POINTS) {
        ROS_WARN("Received invalid cloud (size: %lu), skipping compression", pcl_cloud->size());
        return;
    }
    ROS_INFO("[pcc_compressor_b] Filtered cloud size: %lu", pcl_cloud->size());

    // 5. 转换为自定义点云格式
    std::vector<point_cloud> original_data;
    convertPCLToCustom(pcl_cloud, original_data);

    // 6. 执行压缩编码
    std::vector<char> encoded_data = encoder_->encodeToData(original_data, true);
    if (encoded_data.empty()) {
        ROS_ERROR("Compression failed (encoded_data is empty)");
        return;
    }
    ROS_INFO("[pcc_compressor_b] Compressed data size: %lu bytes (original points: %lu)",
             encoded_data.size(), pcl_cloud->size());

    // 7. 创建并填充压缩消息
    pcc_project::CompressedPointCloudData compressed_msg;
    compressed_msg.header.stamp = input_msg->header.stamp;
    compressed_msg.header.seq = dcl_seq;
    compressed_msg.header.frame_id = "";
    compressed_msg.sender_id = sender_id_;
    compressed_msg.q_level = q_level_;
    compressed_msg.original_size = pcl_cloud->size();
    compressed_msg.data.assign(encoded_data.begin(), encoded_data.end());

    ROS_INFO("[pcc_compressor_b] Assigned sender_id=%d, dcl_seq=%d to compressed msg",
             compressed_msg.sender_id, compressed_msg.header.seq);

    // 8. 缓冲逻辑
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        buffer_.push(compressed_msg);
        start_publishing_ = true;
        ROS_INFO("[pcc_compressor_b] Pushed to buffer! Buffer size: %lu", buffer_.size());
    }
}

// 确保转换函数参数类型与点云一致
void convertPCLToCustom(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud, std::vector<point_cloud>& custom_cloud) {
    custom_cloud.resize(pcl_cloud->size());
    for (size_t i = 0; i < pcl_cloud->size(); ++i) {
        custom_cloud[i].x = pcl_cloud->points[i].x;
        custom_cloud[i].y = pcl_cloud->points[i].y;
        custom_cloud[i].z = pcl_cloud->points[i].z;
        // 若需要强度信息且point_cloud结构体支持，取消下面注释
        // custom_cloud[i].intensity = pcl_cloud->points[i].intensity;
    }
}   

    // 发布线程（核心修改：发布压缩消息，而非解压点云）
    void publishLoop() {
        ros::Rate rate(10); // 10Hz频率控制
        while (ros::ok()) {
            if (start_publishing_) {
                pcc_project::CompressedPointCloudData msg; // 改为压缩消息类型
                bool has_msg = false;
                {
                    std::lock_guard<std::mutex> lock(buffer_mutex_);
                    if (!buffer_.empty()) {
                        msg = buffer_.front();
                        buffer_.pop();
                        has_msg = true;
                    }
                }
                if (has_msg) {
                    pub_.publish(msg); // 发布压缩消息
                    ROS_INFO("Published compressed cloud: Seq=%d, Size=%lu bytes", 
                              msg.sender_id, msg.data.size());
                }
            }
            rate.sleep();
        }
    }

    // ROS组件
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    // 算法组件
    std::unique_ptr<EncoderModule> encoder_;
    int q_level_;
    int sender_id_; // 新增：机器人ID
    int tile_size_;
    bool use_compress_;
    std::string input_topic_;
    std::string output_topic_;

    // 缓冲组件（修正缓冲类型为压缩消息）
    std::queue<pcc_project::CompressedPointCloudData> buffer_; // 原sensor_msgs::PointCloud2改为压缩消息
    std::mutex buffer_mutex_;
    bool start_publishing_;
    std::thread publish_thread_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcc_compressor_node"); // 修正节点名，明确是压缩节点
    ros::NodeHandle nh("~");
    
    PointCloudCompressor processor(nh);
    
    ros::spin();
    return 0;
}