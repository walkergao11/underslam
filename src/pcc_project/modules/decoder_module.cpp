#include "decoder_module.h"
#include "dct.h"

#include <arpa/inet.h>  // 提供ntohl函数
#include <ros/ros.h>    // 提供ROS_INFO宏
DecoderModule::DecoderModule(float pitch_precision, float yaw_precision, int tile_size) : pitch_precision(pitch_precision), yaw_precision(yaw_precision), tile_size(tile_size)
{

    row = (VERTICAL_DEGREE / yaw_precision);
    row = ((row + tile_size - 1) / tile_size) * tile_size;
    col = HORIZONTAL_DEGREE / pitch_precision + tile_size;
    col = ((col + tile_size - 1) / tile_size) * tile_size;
    r_mat = cv::Mat(row, col, CV_32FC1, 0.f);
    b_mat = cv::Mat(row / tile_size, col / tile_size, CV_32SC1, 0.f);
    occ_mat = cv::Mat(row / tile_size, col / tile_size, CV_32SC1, 0.f);
    dct_mat = pcc::Matrix<double>(row, std::vector<double>(col, 0.0));
    unfit_mask_mat = pcc::Matrix<bool>(row, std::vector<bool>(col, false));
    unfit_nums = std::vector<float>(row * col, 0.f);
}
/*
DecoderModule::DecoderModule(const std::vector<char> &data, int tile_size, bool use_compress, int ksample) : tile_size(tile_size)
{
    bool flag;
    if (use_compress)
    {
        flag = q_deserializeData(decompressData(data), yaw_precision, pitch_precision, b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    }
    else
    {
        flag = deserializeData(data, b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    }

    if (!flag)
    {
        std::cout << "deserialize data failed" << std::endl;
        return;
    }
    row = idx_sizes[0] * tile_size;
    col = idx_sizes[1] * tile_size;
    r_mat = cv::Mat(row, col, CV_32FC1, 0.f);
    unfit_mask_mat = pcc::Matrix<bool>(row, std::vector<bool>(col, false));
    unfit_nums = std::vector<float>(row * col, 0.f);
    if (ksample >1)//anchor
    {
        decode(b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat, ksample);
        std::cout << "ksample:" << ksample << std::endl;
    }
    else
    {
        decode(b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    }
}
*/

DecoderModule::DecoderModule(const std::vector<char> &data, int tile_size, bool use_compress, int ksample) : tile_size(tile_size) {
    bool flag = false;
    std::vector<char> meta_data;  // 存储元数据
    std::vector<char> point_data; // 存储点云字段数据

    ROS_INFO("[DecoderModule] meta_data size=%lu, point_data size=%lu", meta_data.size(), point_data.size());
    ROS_INFO("[DecoderModule] coefficients size=%lu", coefficients.size());

    int total_size = meta_data.size() + point_data.size();
    ROS_INFO("[DecoderModule] total_size=%d (meta_data + point_data)", total_size);
    // 1. 解析元数据长度（前4字节，对应编码器写入的meta_size）
    if (data.size() < 4) {
        std::cout << "Data too short to parse meta size" << std::endl;
        return;
    }
    //int meta_size;
    //memcpy(&meta_size, data.data(), sizeof(int));  // 读取元数据长度

    uint32_t net_meta_size;  // 用uint32_t接收网络字节序数据
    memcpy(&net_meta_size, data.data(), sizeof(uint32_t));  // 读取4字节（网络字节序）
    int meta_size = static_cast<int>(ntohl(net_meta_size));  // 转回主机字节序
    ROS_INFO("[DecoderModule] Parsed meta_size: %d bytes (after ntohl)", meta_size);  // 新增日志验证

    if (meta_size <= 0 || meta_size + 4 > data.size()) {
        std::cout << "Invalid meta size: " << meta_size << std::endl;
        return;
    }

    // 2. 提取元数据（从第4字节开始，长度为meta_size）
    meta_data.assign(data.begin() + 4, data.begin() + 4 + meta_size);

    // 3. 提取点云字段数据（元数据之后的部分）
    point_data.assign(data.begin() + 4 + meta_size, data.end());

    // 4. 解析元数据（复用原有反序列化函数）
    if (use_compress) {
        flag = q_deserializeData(decompressData(meta_data), yaw_precision, pitch_precision, 
                                b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    } else {
        flag = deserializeData(meta_data, b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    }

    if (!flag) {
        std::cout << "deserialize meta data failed" << std::endl;
        return;
    }

    // 5. 解析点云字段数据（计算点数量：每个CompressedPoint占19字节）
    point_num_ = point_data.size() / 19;
    if (point_data.size() % 19 != 0) {
        std::cout << "Invalid point data size (not multiple of 19): " << point_data.size() << std::endl;
        point_num_ = 0;
    }

    // 6. 初始化矩阵和点云容器
    row = idx_sizes[0] * tile_size;
    col = idx_sizes[1] * tile_size;
    r_mat = cv::Mat(row, col, CV_32FC1, 0.f);
    unfit_mask_mat = pcc::Matrix<bool>(row, std::vector<bool>(col, false));
    unfit_nums = std::vector<float>(row * col, 0.f);
    restored_pcloud.reserve(point_num_);  

    // 7. 解码点云（调用原有解码逻辑，最后补充点云字段赋值）
    if (ksample > 1) {
        decode(b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat, ksample);
        std::cout << "ksample:" << ksample << std::endl;
    } else {
        decode(b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    }

    // 8. 从point_data中解析CompressedPoint字段（补充原有restore_pcloud未处理的字段）
    parseCompressedPointData(point_data);
}

//新增
void DecoderModule::restore_pcloud(const cv::Mat& r_mat, float pitch_precision, float yaw_precision,
                                   std::vector<CompressedPoint>& pcloud) {
    pcloud.clear();
    // 遍历范围矩阵，计算每个有效点的三维坐标（球面→笛卡尔）
    for (int i = 0; i < r_mat.rows; ++i) {
        for (int j = 0; j < r_mat.cols; ++j) {
            float range = r_mat.at<float>(i, j);
            if (range <= 0) continue;  // 过滤无效点（距离≤0）

            // 从矩阵索引计算角度（适配原有投影逻辑）
            float yaw = -VERTICAL_DEGREE / 2 + i * yaw_precision;  // 垂直方向（行→yaw）
            float pitch = -HORIZONTAL_DEGREE / 2 + j * pitch_precision;  // 水平方向（列→pitch）
            float rad_yaw = yaw * M_PI / 180.0f;  // 角度转弧度
            float rad_pitch = pitch * M_PI / 180.0f;

            // 计算x/y/z坐标
            CompressedPoint pt;
            pt.x = range * cos(rad_pitch) * cos(rad_yaw);
            pt.y = range * cos(rad_pitch) * sin(rad_yaw);
            pt.z = range * sin(rad_pitch);

            // 临时初始化其他字段（后续会被parseCompressedPointData覆盖）
            pt.intensity = 0.0f;
            pt.r = 0;
            pt.g = 0;
            pt.b = 0;

            pcloud.push_back(pt);
        }
    }
}

void DecoderModule::restore_extra_pcloud(float pitch_precision, float yaw_precision,
                                         std::vector<CompressedPoint>& pcloud,
                                         const pcc::Matrix<float>& extra_pc) {
    // 假设extra_pc的每个元素存储（yaw索引, pitch索引, range）（适配原有逻辑）
    for (size_t i = 0; i < extra_pc.size(); ++i) {
        int yaw_idx = static_cast<int>(extra_pc[i][0]);
        int pitch_idx = static_cast<int>(extra_pc[i][1]);
        float range = extra_pc[i][2];

        // 计算角度和坐标（逻辑同restore_pcloud）
        float yaw = -VERTICAL_DEGREE / 2 + yaw_idx * yaw_precision;
        float pitch = -HORIZONTAL_DEGREE / 2 + pitch_idx * pitch_precision;
        float rad_yaw = yaw * M_PI / 180.0f;
        float rad_pitch = pitch * M_PI / 180.0f;

        CompressedPoint pt;
        pt.x = range * cos(rad_pitch) * cos(rad_yaw);
        pt.y = range * cos(rad_pitch) * sin(rad_yaw);
        pt.z = range * sin(rad_pitch);
        pt.intensity = 0.0f;
        pt.r = 0;
        pt.g = 0;
        pt.b = 0;

        pcloud.push_back(pt);
    }
}


DecoderModule::~DecoderModule()
{
}
void DecoderModule::decode(cv::Mat &b_mat, const int *idx_sizes,
                           std::vector<cv::Vec4f> &coefficients, cv::Mat &occ_mat,
                           std::vector<int> &tile_fit_lengths,
                           pcc::Matrix<double> &dct_mat)
{ // decode的r_mat是float类型的
    // decoder::single_channel_decode(r_mat, b_mat, idx_sizes,
    //                                coefficients, occ_mat,
    //                                tile_fit_lengths,
    //                                unfit_nums, tile_size);
    decoder::single_channel_decode(r_mat, b_mat, idx_sizes,
                                   coefficients, occ_mat,
                                   tile_fit_lengths,
                                   tile_size, dct_mat);
    std::cout << "tile_fit_lengths size: " << tile_fit_lengths.size() << std::endl;
    std::cout << "coefficients size: " << coefficients.size() << std::endl;
    std::cout << "b_mat size: " << b_mat.size() << std::endl;
    std::cout << "r_mat size: " << r_mat.size() << std::endl;
    std::cout << "occ_mat size: " << occ_mat.size() << std::endl;

    // cv::Mat mask = cv::Mat::zeros(row, col, CV_32FC1);
    // for (int i = 0; i < row; i++)
    // {
    //     for (int j = 0; j < col; j++)
    //     {

    //         mask.at<float>(i, j) = r_mat.at<float>(i, j);
    //     }
    // }
    // cv::imshow("r_mat", mask);
    // cv::waitKey(0);

    restore_pcloud(r_mat, pitch_precision, yaw_precision, restored_pcloud);
    std::cout << "pointcloud size: " << restored_pcloud.size() << std::endl;
}
// ksample version
void DecoderModule::decode(cv::Mat &b_mat, const int *idx_sizes,
                           std::vector<cv::Vec4f> &coefficients, cv::Mat &occ_mat,
                           std::vector<int> &tile_fit_lengths,
                           pcc::Matrix<double> &dct_mat, int ksamlple)
{
    pcc::Matrix<float> extra_pc;
    decoder::single_channel_decode(r_mat, b_mat, idx_sizes,
                                   coefficients, occ_mat,
                                   tile_fit_lengths,
                                   tile_size, dct_mat, ksamlple, extra_pc);
    std::cout << "tile_fit_lengths size: " << tile_fit_lengths.size() << std::endl;
    std::cout << "coefficients size: " << coefficients.size() << std::endl;
    std::cout << "b_mat size: " << b_mat.size() << std::endl;
    std::cout << "r_mat size: " << r_mat.size() << std::endl;
    std::cout << "occ_mat size: " << occ_mat.size() << std::endl;

    // cv::Mat mask = cv::Mat::zeros(row, col, CV_32FC1);
    // for (int i = 0; i < row; i++)
    // {
    //     for (int j = 0; j < col; j++)
    //     {

    //         mask.at<float>(i, j) = r_mat.at<float>(i, j);
    //     }
    // }
    // cv::imshow("r_mat", mask);
    // cv::waitKey(0);

    restore_pcloud(r_mat, pitch_precision, yaw_precision, restored_pcloud);
    restore_extra_pcloud(pitch_precision, yaw_precision, restored_pcloud, extra_pc);
    
    std::cout<<"Extra pointcloud size: "<<extra_pc.size()<<std::endl;
    std::cout << "pointcloud size: " << restored_pcloud.size() << std::endl;
}

void DecoderModule::unpackdata(const std::vector<char> &data)
{
    deserializeData(data, b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
}

void DecoderModule::decodeFromData(const std::vector<char> &data, bool use_compress)
{
    std::vector<char> meta_data;
    std::vector<char> point_data;

    // 1. 拆分元数据和点云数据（同构造函数逻辑）
    if (data.size() >= 4) {
        //int meta_size;
        //memcpy(&meta_size, data.data(), sizeof(int));

        uint32_t net_meta_size;
        memcpy(&net_meta_size, data.data(), sizeof(uint32_t));
        int meta_size = static_cast<int>(ntohl(net_meta_size));
        ROS_INFO("[DecoderModule] decodeFromData meta_size: %d bytes (after ntohl)", meta_size);  // 新增日志验证

        if (meta_size > 0 && meta_size + 4 <= data.size()) {
            meta_data.assign(data.begin() + 4, data.begin() + 4 + meta_size);
            point_data.assign(data.begin() + 4 + meta_size, data.end());
            point_num_ = point_data.size() / 19;
        }
    }
    // 2. 原有元数据解析
    if (use_compress)
    {
        //unpackdata(decompressData(data));
        unpackdata(decompressData(meta_data.empty() ? data : meta_data));
    }
    else
    {
        unpackdata(data);
    }
    // 3. 原有解码逻辑
    decode(b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    // 4. 解析点云字段
    if (!point_data.empty()) {
        parseCompressedPointData(point_data);
    }
}

// 解析点云字段数据，填充restored_pcloud
void DecoderModule::parseCompressedPointData(const std::vector<char>& point_data) {
    if (point_num_ <= 0 || point_data.empty()) {
        std::cout << "parseCompressedPointData: no valid point data" << std::endl;
        return;
    }
// 确保点云容器大小与计算的点数量一致
    if (restored_pcloud.size() != point_num_) {
        restored_pcloud.resize(point_num_);
        std::cout << "parseCompressedPointData: adjusted pcloud size to " << point_num_ << std::endl;
    }
    for (size_t i = 0; i < point_num_; ++i) {
        size_t offset = i * 19;  // 每个点19字节
        CompressedPoint& point = restored_pcloud[i];

        // 解析x（4字节float）
        memcpy(&point.x, point_data.data() + offset, sizeof(float));
        offset += sizeof(float);

        memcpy(&point.y, point_data.data() + offset, sizeof(float));
        offset += sizeof(float);

        memcpy(&point.z, point_data.data() + offset, sizeof(float));
        offset += sizeof(float);

        memcpy(&point.intensity, point_data.data() + offset, sizeof(float));
        offset += sizeof(float);

        point.r = static_cast<uint8_t>(point_data[offset++]);
        point.g = static_cast<uint8_t>(point_data[offset++]);
        point.b = static_cast<uint8_t>(point_data[offset++]);
    }

    std::cout << "parseCompressedPointData: parsed " << point_num_ << " CompressedPoints" << std::endl;
}