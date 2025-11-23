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


// !!! Feng: Supporting ROS
class PointCloudProcessor {
public:
    PointCloudProcessor(ros::NodeHandle& nh) : nh_(nh), start_publishing_(false) {
        // 压缩等级
        nh_.param("q_level", q_level_, 2);
        // nh_.param("input_topic", input_topic_, std::string("/kitti/velo/pointcloud"));
        //nh_.param("input_topic", input_topic_, std::string("/Bob/velodyne_points"));//改成/a/distributedMapping/loopInfo
        nh_.param("input_topic", input_topic_, std::string("/a/distributedMapping/loopInfo"));
        nh_.param("output_topic", output_topic_, std::string("/compressed_cloud"));

        // 初始化ROS组件
        sub_ = nh_.subscribe(input_topic_, 2000, &PointCloudProcessor::cloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 100);
        
        // 初始化编解码器
        encoder_ = std::make_unique<EncoderModule>(4, q_level_);

        // 启动发布线程
        publish_thread_ = std::thread(&PointCloudProcessor::publishLoop, this);
    }

    ~PointCloudProcessor() {
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }
    }

private:
    // ROS消息回调函数
    // 消息处理回调
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {
        // 转换ROS消息为PCL格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_msg, *pcl_cloud);

        // 转换为自定义点云格式
        std::vector<point_cloud> original_data;
        convertPCLToCustom(pcl_cloud, original_data);

        // 执行压缩编码
        std::vector<char> encoded_data = encoder_->encodeToData(original_data, true);

        // 执行解压缩
        DecoderModule decoder(encoded_data, 4, true, q_level_);
        auto restored_data = decoder.restored_pcloud;

        // 转换回PCL格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr restored_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        convertCustomToPCL(restored_data, restored_cloud);

        // 准备待发布的点云消息
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*restored_cloud, output_msg);
        output_msg.header = input_msg->header;  // 保持时间戳和坐标系
        // pub_.publish(output_msg);

        // !!! Feng add: Newly added
        // 将消息加入缓冲队列
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            buffer_.push(output_msg);
            
            // 首次达到100条时激活发布标志
            if (buffer_.size() >= 100 && !start_publishing_) {
                start_publishing_ = true;
                ROS_INFO("Buffer filled. Start publishing at 10Hz");
            }
        }
    }

    // 格式转换函数
    void convertPCLToCustom(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, std::vector<point_cloud>& custom_cloud) {
        custom_cloud.resize(pcl_cloud->size());
        for (size_t i = 0; i < pcl_cloud->size(); ++i) {
            custom_cloud[i].x = pcl_cloud->points[i].x;
            custom_cloud[i].y = pcl_cloud->points[i].y;
            custom_cloud[i].z = pcl_cloud->points[i].z;
        }
    }

    void convertCustomToPCL(const std::vector<point_cloud>& custom_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud) {
        pcl_cloud->width = custom_cloud.size();
        pcl_cloud->height = 1;
        pcl_cloud->is_dense = true;
        pcl_cloud->points.resize(custom_cloud.size());
        for (size_t i = 0; i < custom_cloud.size(); ++i) {
            pcl_cloud->points[i].x = custom_cloud[i].x;
            pcl_cloud->points[i].y = custom_cloud[i].y;
            pcl_cloud->points[i].z = custom_cloud[i].z;
            pcl_cloud->points[i].intensity = 1.0; 
        }
    }

    // 新增发布线程函数
    void publishLoop() {
        ros::Rate rate(10); // 10Hz频率控制
        while (ros::ok()) {
            if (start_publishing_) {
                sensor_msgs::PointCloud2 msg;
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
    std::string input_topic_;
    std::string output_topic_;

    // !!! Feng: Newly added members
    std::queue<sensor_msgs::PointCloud2> buffer_;
    std::mutex buffer_mutex_;
    bool start_publishing_;
    std::thread publish_thread_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "example");
    ros::NodeHandle nh("~");
    
    PointCloudProcessor processor(nh);
    
    ros::spin();
    return 0;
}