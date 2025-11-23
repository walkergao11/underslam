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
#include <string>  
// 包含自定义的压缩消息头文件
#include <pcc_project/CompressedPointCloudData.h>
#include <arpa/inet.h>  // 提供ntohl函数

class PointCloudCompressor {
public:
    PointCloudCompressor(ros::NodeHandle& nh) : nh_(nh), start_publishing_(false) {
        // 1. 强制读取robot_prefix
        if (!nh_.getParam("robot_prefix", robot_prefix_)) {
            ROS_FATAL("[%s] Missing 'robot_prefix' parameter! Set it in launch.", robot_prefix_.c_str());
            ros::shutdown();
            return;
        }
        // 2. 映射sender_id
        if (robot_prefix_ == "a") {
            sender_id_ = 0;
        } else if (robot_prefix_ == "b") {
            sender_id_ = 1;
        } else if (robot_prefix_ == "c") {
            sender_id_ = 2;
        } else {
            ROS_FATAL("[%s] Invalid robot_prefix! Must be 'a', 'b', or 'c'.", robot_prefix_.c_str());
            ros::shutdown();
            return;
        }

        // 3. 读取话题参数（默认使用私有话题，由launch的remap指定实际路径）
        nh_.param("input_topic", input_topic_, std::string("input_points"));
        nh_.param("output_topic", output_topic_, std::string("output_compressed"));

        // 4. 读取压缩参数
        nh_.param("q_level", q_level_, 3);
        nh_.param("tile_size", tile_size_, 4);
        nh_.param("use_compress", use_compress_, true);

        // 5. 初始化编码器
        try {
            encoder_.reset(new EncoderModule(tile_size_, q_level_));
            ROS_INFO("[%s] Encoder initialized (tile_size=%d, q_level=%d)", 
                     robot_prefix_.c_str(), tile_size_, q_level_);
        } catch (const std::exception& e) {
            ROS_FATAL("[%s] Encoder init failed: %s", robot_prefix_.c_str(), e.what());
            ros::shutdown();
            return;
        }

        // 6. 订阅和发布（使用私有话题，实际路径由launch的remap决定）
        sub_ = nh_.subscribe(input_topic_, 10, &PointCloudCompressor::cloudCallback, this);
        pub_ = nh_.advertise<pcc_project::CompressedPointCloudData>(output_topic_, 10);

        // 7. 启动发布线程（打印实际订阅/发布的话题，验证remap是否生效）
        publish_thread_ = std::thread(&PointCloudCompressor::publishLoop, this);
        ROS_INFO("[%s] PCC Node Ready. Sender ID: %d", robot_prefix_.c_str(), sender_id_);
        ROS_INFO("[%s] Subscribed to: %s (remapped path: %s)", 
                 robot_prefix_.c_str(), input_topic_.c_str(), sub_.getTopic().c_str());
        ROS_INFO("[%s] Publishing to: %s (remapped path: %s)", 
                 robot_prefix_.c_str(), output_topic_.c_str(), pub_.getTopic().c_str());
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {

        // 1. 解析DCL-SLAM的seq（从frame_id获取）
        int dcl_seq = input_msg->header.seq; 
        ROS_INFO("[%s] Parsed DCL seq: %d from header.seq", robot_prefix_.c_str(), dcl_seq);


        // 2. 打印回调信息（带robot_prefix标识）
        ROS_INFO("[%s] Callback triggered. Input seq=%d, dcl_seq=%d, size=%dx%d",
                 robot_prefix_.c_str(), input_msg->header.seq, dcl_seq, 
                 input_msg->width, input_msg->height);

        // 3. 点云格式转换
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_msg, *pcl_cloud);

        // 4. 过滤无效点云
        const int MIN_POINTS = 10;
        if (pcl_cloud->empty() || pcl_cloud->size() < MIN_POINTS) {
            ROS_WARN("[%s] Invalid cloud (size: %lu), skipping", 
                     robot_prefix_.c_str(), pcl_cloud->size());
            return;
        }
        ROS_INFO("[%s] Filtered cloud size: %lu", robot_prefix_.c_str(), pcl_cloud->size());

        // 5. 转换为自定义格式
        std::vector<CompressedPoint> original_data;
        convertPCLToCustom(pcl_cloud, original_data);  // Call existing conversion function
        ROS_INFO("[%s] Converted original cloud size: %lu", robot_prefix_.c_str(), original_data.size());// 调用修改后的转换函数


        // 6. Filter invalid points (based on converted original_data)
        std::vector<CompressedPoint> filtered_cloud;
        filtered_cloud.reserve(original_data.size());  // Pre-allocate memory
        for (const auto& pt : original_data) {
        // Filter condition: exclude points with NaN or infinite x/y/z
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            filtered_cloud.push_back(pt);
        } else {
            ROS_WARN("[%s] Filtering invalid point: x=%f, y=%f, z=%f", 
                     robot_prefix_.c_str(), pt.x, pt.y, pt.z);
        }
    }
    ROS_INFO("[%s] Filtered cloud size: %lu", robot_prefix_.c_str(), filtered_cloud.size());


        // 验证前10个点的字段是否合法（使用循环和filtered_cloud）
        size_t print_count = std::min(10ul, filtered_cloud.size());  // 最多打印10个点
        for (size_t i = 0; i < print_count; ++i) {  // 定义i并循环
            const auto& pt = filtered_cloud[i];  // 使用过滤后的点云
            ROS_INFO("[%s] Point %zu: x=%.2f, y=%.2f, z=%.2f, intensity=%.2f, color=[%d,%d,%d]",
                    robot_prefix_.c_str(), i,
                    pt.x, pt.y, pt.z, pt.intensity,
                    pt.r, pt.g, pt.b);
        }

        // 6. 压缩编码
        //std::vector<char> encoded_data = encoder_->encodeToData(original_data, true);
        //if (encoded_data.empty()) {
            //ROS_ERROR("[%s] Compression failed (encoded_data is empty)", robot_prefix_.c_str());
            //return;
        //}
        // 6. 调用EncoderModule压缩，获取完整数据（已包含meta_size+meta_data+point_data）
        
        /*
        std::vector<char> final_compressed_data = encoder_->encodeToData(filtered_cloud, true);
        if (final_compressed_data.empty()) {
            ROS_ERROR("[%s] Compression failed (final data is empty)", robot_prefix_.c_str());
        return;
        }

        // 验证meta_size是否正确
        if (final_compressed_data.size() >= sizeof(int)) {
            int meta_size = *reinterpret_cast<const int*>(final_compressed_data.data());
            ROS_INFO("[%s] Compressed data: total_size=%zu bytes, meta_size=%d bytes (from packData)",
                    robot_prefix_.c_str(), final_compressed_data.size(), meta_size);
        }
        */
    /////////////   
        // 6. 调用encodeToData，获取未压缩数据和压缩后数据
        auto [uncompressed_data, compressed_data] = encoder_->encodeToData(filtered_cloud, use_compress_);
        if (uncompressed_data.empty()) {
            ROS_ERROR("[%s] Compression failed: uncompressed data is empty", robot_prefix_.c_str());
        return;
        }

        // 7. 从“未压缩数据”中读取正确的meta_size
        int meta_size = -1;
        if (uncompressed_data.size() >= sizeof(uint32_t)) {
        // 用ntohl转换，因为uncompressed_data中的meta_size是网络字节序
            uint32_t net_meta_size = *reinterpret_cast<const uint32_t*>(uncompressed_data.data());
            meta_size = static_cast<int>(ntohl(net_meta_size));
            ROS_INFO("[%s] Compressed data: total_size=%zu bytes, meta_size=%d bytes (from uncompressed data)",
                 robot_prefix_.c_str(), 
                 use_compress_ ? compressed_data.size() : uncompressed_data.size(), 
                 meta_size);
        }

        // 8. 选择传输的数据（压缩后或未压缩）
        std::vector<char> final_data;
        if (use_compress_ && !compressed_data.empty()) {
            final_data = compressed_data;
        } else {
            final_data = uncompressed_data;
        }
    /////////////
        // 7. 填充压缩消息（sender_id与robot_prefix严格绑定）
        pcc_project::CompressedPointCloudData compressed_msg;
        compressed_msg.header.stamp = input_msg->header.stamp;
        compressed_msg.header.seq = dcl_seq;
        compressed_msg.header.frame_id = robot_prefix_; // 携带前缀，方便调试
        compressed_msg.sender_id = sender_id_;          // 与robot_prefix绑定
        compressed_msg.q_level = q_level_;
        compressed_msg.tile_size = tile_size_;          // 传递瓦片大小给解压端
        compressed_msg.original_size = filtered_cloud.size();

        compressed_msg.data.assign(final_data.begin(), final_data.end());

        ROS_INFO("[%s] Assigned sender_id=%d, dcl_seq=%d to msg",
                 robot_prefix_.c_str(), compressed_msg.sender_id, compressed_msg.header.seq);

        // 8. 加入缓冲队列
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            buffer_.push(compressed_msg);
            start_publishing_ = true;
            ROS_INFO("[%s] Buffer size: %lu", robot_prefix_.c_str(), buffer_.size());
        }
    }

    // 点云格式转换函数
    void convertPCLToCustom(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud, 
                       std::vector<CompressedPoint>& custom_cloud) {
    custom_cloud.resize(pcl_cloud->size());  // 初始化新结构体向量
        for (size_t i = 0; i < pcl_cloud->size(); ++i) {
            // 1. 赋值坐标和强度（来自PCL点云）
            custom_cloud[i].x = pcl_cloud->points[i].x;
            custom_cloud[i].y = pcl_cloud->points[i].y;
            custom_cloud[i].z = pcl_cloud->points[i].z;
            custom_cloud[i].intensity = pcl_cloud->points[i].intensity;  // 复用PCL强度值
        
            // 2. 赋值颜色（与原逻辑一致，默认白色）
            custom_cloud[i].r = 255;
            custom_cloud[i].g = 255;
            custom_cloud[i].b = 255;
        }
    }

    // 发布线程
    void publishLoop() {
        ros::Rate rate(10);
        while (ros::ok()) {
            if (start_publishing_) {
                pcc_project::CompressedPointCloudData msg;
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
                    pub_.publish(msg);
                    ROS_INFO("[%s] Published compressed msg: seq=%d, size=%lu bytes",
                             robot_prefix_.c_str(), msg.header.seq, msg.data.size());
                }
            }
            rate.sleep();
        }
    }

    // 成员变量
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::unique_ptr<EncoderModule> encoder_;
    
    int q_level_;
    int tile_size_;
    int sender_id_;
    bool use_compress_;
    std::string robot_prefix_;  // 保存当前机器人前缀
    std::string input_topic_;   // 私有输入话题名
    std::string output_topic_;  // 私有输出话题名
    
    std::queue<pcc_project::CompressedPointCloudData> buffer_;
    std::mutex buffer_mutex_;
    volatile bool start_publishing_;
    std::thread publish_thread_;
};

int main(int argc, char**argv) {
    ros::init(argc, argv, "pcc_compressor_node");
    ros::NodeHandle nh("~"); 
    
    PointCloudCompressor processor(nh);
    
    ros::spin();
    return 0;
}
    