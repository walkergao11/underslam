#include "encoder_module.h"
#include "io.h"
#include "pcc_module.h"
#include "serializer.h"
#include "binary_compressor.h"

#include <ros/console.h>  //引入ROS日志宏（ROS_WARN、ROS_INFO等）
#include <arpa/inet.h>  //包含htonl函数（主机字节序→网络字节序）

EncoderModule::EncoderModule(float pitch_precision, float yaw_precision, float threshold,
                             int tile_size, int q_level) : pitch_precision(pitch_precision), yaw_precision(yaw_precision),
                                                           threshold(threshold), tile_size(tile_size), q_level(q_level)
{
    std::cout << "precision: " << pitch_precision << " " << yaw_precision << std::endl;
    row = (VERTICAL_DEGREE / yaw_precision);
    row = ((row + tile_size - 1) / tile_size) * tile_size;
    col = HORIZONTAL_DEGREE / pitch_precision + tile_size;
    col = ((col + tile_size - 1) / tile_size) * tile_size;
    f_mat = cv::Mat(row, col, CV_32FC4, cv::Scalar(0.f, 0.f, 0.f, 0.f));
    b_mat = cv::Mat(row / tile_size, col / tile_size, CV_32SC1, 0.f);
    occ_mat = cv::Mat(row / tile_size, col / tile_size, CV_32SC1, 0.f);
    dct_mat = pcc::Matrix<double>(row, std::vector<double>(col, 0));
    tile_fit_lengths = std::vector<int>();
    coefficients = std::vector<cv::Vec4f>();
    unfit_nums = std::vector<float>();
    idx_sizes[0] = row / tile_size;
    idx_sizes[1] = col / tile_size;
}
EncoderModule::EncoderModule(int tile_size, int q_level):tile_size(tile_size),q_level(q_level)
{   
    yaw_precision=quantization_dict[q_level][0];
    pitch_precision=quantization_dict[q_level][1];
    threshold=quantization_dict[q_level][2];
    row = (VERTICAL_DEGREE / yaw_precision);
    row = ((row + tile_size - 1) / tile_size) * tile_size;
    col = HORIZONTAL_DEGREE / pitch_precision + tile_size;
    col = ((col + tile_size - 1) / tile_size) * tile_size;
    f_mat = cv::Mat(row, col, CV_32FC4, cv::Scalar(0.f, 0.f, 0.f, 0.f));
    b_mat = cv::Mat(row / tile_size, col / tile_size, CV_32SC1, 0.f);
    occ_mat = cv::Mat(row / tile_size, col / tile_size, CV_32SC1, 0.f);
    dct_mat = pcc::Matrix<double>(row, std::vector<double>(col, 0));
    tile_fit_lengths = std::vector<int>();
    coefficients = std::vector<cv::Vec4f>();
    unfit_nums = std::vector<float>();
    idx_sizes[0] = row / tile_size;
    idx_sizes[1] = col / tile_size;
}

EncoderModule::~EncoderModule() {}
void EncoderModule::encode(std::vector<point_cloud> &pcloud_data)
{
    PccResult pcc_res;
    /*******************************************************************/
    // initialization

    double proj_time, fit_time;
    float psnr, total_pcloud_size;

    /*******************************************************************/
    // convert range map
    std::cout << "CURRENT pcloud size: " << pcloud_data.size() << std::endl;
    // Characterize Range Map
    // floating map;

    proj_time = map_projection(f_mat, pcloud_data, pitch_precision, yaw_precision, 'e');

    // cv::Mat value_mat = cv::Mat::zeros(row, col, CV_32FC1);
    // for (int i = 0; i < row; i++)
    // {
    //     for (int j = 0; j < col; j++)
    //     {
    //         value_mat.at<float>(i, j) = f_mat.at<cv::Vec4f>(i, j)[0];
    //     }
    // }
    // cv::imshow("value_mat", value_mat);
    // cv::waitKey(0);
    pcc_res.proj_times->push_back(proj_time);

    // compute compression rate: bit-per-point (bpp)
    pcc_res.compression_rate->push_back(8.0f * f_mat.cols * f_mat.rows / pcloud_data.size());

    // loss error compute;
    // psnr = compute_loss_rate<cv::Vec4f>(*f_mat, pcloud_data, pitch_precision, yaw_precision);

    // update the info;
    pcc_res.loss_rate->push_back(psnr);

    std::cout << "Loss rate [PSNR]: " << psnr << " Compression rate: "
              << pcc_res.compression_rate->back() << " bpp." << std::endl;

    /*******************************************************************/
    // fitting range map
    int mat_div_tile_sizes[] = {row / tile_size, col / tile_size};

    // cv::Mat *b_mat = new cv::Mat(row / tile_size, col / tile_size, CV_32SC1, 0.f);
    // cv::Mat *occ_mat = new cv::Mat(row / tile_size, col / tile_size, CV_32SC1, 0.f);

    // encode the occupatjon map
    encoder::encode_occupation_mat(f_mat, occ_mat, tile_size, mat_div_tile_sizes);

    fit_time = encoder::single_channel_encode(f_mat, b_mat, mat_div_tile_sizes, coefficients,
                                              unfit_nums, tile_fit_lengths,
                                              threshold, tile_size);
    std::cout << "fit_time" << fit_time << std::endl;

    // point cloud mask
    cv::Mat mask = cv::Mat::zeros(row, col, CV_32SC1);
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            if (f_mat.at<cv::Vec4f>(i, j)[0] != 0)
            {
                mask.at<int>(i, j) = 1;
            }
        }
    }
    // clear the fit points tiles
    for (int r_idx = 0; r_idx < mat_div_tile_sizes[0]; r_idx++)
    {
        for (int c_idx = 0; c_idx < mat_div_tile_sizes[1]; c_idx++)
        {
            if (b_mat.at<int>(r_idx, c_idx) == 1)
            {
                for (int i = 0; i < tile_size; i++)
                {
                    for (int j = 0; j < tile_size; j++)
                    {
                        mask.at<int>(r_idx * tile_size + i, c_idx * tile_size + j) = 0;
                    }
                }
            }
        }
    }
    // sadct
    pcc::Matrix<double> value_map(row, std::vector<double>(col, 0));
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            value_map[i][j] = f_mat.at<cv::Vec4f>(i, j)[0];
        }
    }
    pcc::Matrix<bool> mask_map(row, std::vector<bool>(col, false));
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            mask_map[i][j] = mask.at<int>(i, j) == 1 ? true : false;
        }
    }
    // SADCT
    clock_t start = clock();
    #ifdef USE_SADCT
    auto coeff_res = pcc::sadct(value_map, mask_map);
    #else
    auto coeff_res = pcc::delta_sadct(value_map, mask_map);
    #endif


 
    clock_t end = clock();
    std::cout << "SADCT time: " << (double)(end - start) / CLOCKS_PER_SEC << std::endl;
    dct_mat = coeff_res.first;


    pcc_res.fit_times->push_back(fit_time);
}

// 序列化数据
//std::vector<char> EncoderModule::packData()
//{
    //std::vector<char> data;
    //if (q_level != -1)
    //{
        //data = q_serializeData(q_level, b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    //}
    //else
    //{
        //data = serializeData(b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    //}
    //return data;
//}

// 辅助函数：将float转换为字节并添加到数据中（确保字节序一致）
void writeFloat(std::vector<char>& data, float value) {
    const char* bytes = reinterpret_cast<const char*>(&value);
    data.insert(data.end(), bytes, bytes + sizeof(float));
}

// 辅助函数：将uint8_t转换为字节并添加到数据中
void writeByte(std::vector<char>& data, uint8_t value) {
    data.push_back(static_cast<char>(value));
}

/* 1. 在EncoderModule类的辅助函数中新增writeInt（和writeFloat同级）
void writeInt(std::vector<char>& data, int value) {
    const char* bytes = reinterpret_cast<const char*>(&value);
    data.insert(data.end(), bytes, bytes + sizeof(int));
}
*/
// 辅助函数：将int转换为网络字节序（大端）后写入数据
void writeInt(std::vector<char>& data, int value) {
    // 1. 确保value为非负数（长度不可能为负）
    if (value < 0) {
        ROS_ERROR("[writeInt] Invalid meta_size (negative): %d", value);
        value = 0;
    }
    // 2. 将int转换为网络字节序（大端）
    // htonl: host to network long（将主机字节序转换为网络字节序）
    uint32_t net_value = htonl(static_cast<uint32_t>(value));
    // 3. 将转换后的4字节写入数据
    const char* bytes = reinterpret_cast<const char*>(&net_value);
    data.insert(data.end(), bytes, bytes + sizeof(uint32_t));
}

std::vector<char> EncoderModule::packData() {
    std::vector<char> data;

    // 1. 先打包压缩元数据（复用原有序列化函数）
    std::vector<char> meta_data;
    if (q_level != -1) {
        meta_data = q_serializeData(q_level, b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    } else {
        meta_data = serializeData(b_mat, idx_sizes, coefficients, occ_mat, tile_fit_lengths, dct_mat);
    }

    // 确认meta_data是否为空
    if (meta_data.empty()) {
        ROS_WARN("[EncoderModule] meta_data is empty! Check q_serializeData/serializeData.");
    } else {
        ROS_INFO("[EncoderModule] meta_data size: %zu bytes (b_mat=%dx%d, coefficients=%zu)",
                 meta_data.size(), b_mat.rows, b_mat.cols, coefficients.size());
    }

    // 2. 验证meta_data长度（核心校验）
    int meta_size = meta_data.size();
    if (meta_size <= 0) {
        ROS_ERROR("[EncoderModule] packData failed: meta_data is empty (size=%d)", meta_size);
        return {};  // 元数据为空，直接返回失败
    }
    ROS_INFO("[EncoderModule] meta_data size: %d bytes (准备写入)", meta_size);

    // 3. 写入meta_size（已转换为网络字节序）
    writeInt(data, meta_size);
    // 4. 写入元数据内容
    data.insert(data.end(), meta_data.begin(), meta_data.end());
    /* 写入元数据长度（4字节，用于解码器解析）
    int meta_size = meta_data.size();
    writeInt(data, meta_size); // 替换原来的writeFloat
    data.insert(data.end(), meta_data.begin(), meta_data.end());
    */
    // 2. 再打包点云字段（CompressedPoint）
    for (const auto& point : internal_pcloud_) {
        writeFloat(data, point.x);
        writeFloat(data, point.y);
        writeFloat(data, point.z);
        writeFloat(data, point.intensity);
        writeByte(data, point.r);
        writeByte(data, point.g);
        writeByte(data, point.b);
    }

    return data;
}

//std::vector<char> EncoderModule::encodeToData(std::vector<point_cloud> &pcloud_data, bool use_compress)
//{
    //encode(pcloud_data);
    //clock_t start = clock();
    //ANCHOR:use_compress
    //auto temp = use_compress ? compressData(packData()) : packData();
    //clock_t end = clock();
    //std::cout << "compress time: " << (double)(end - start) / CLOCKS_PER_SEC << std::endl;
    //return temp;
//}

//std::vector<char> EncoderModule::encodeToData(const std::vector<CompressedPoint> &pcloud_data, bool use_compress)
std::pair<std::vector<char>, std::vector<char>> EncoderModule::encodeToData(const std::vector<CompressedPoint> &pcloud_data, bool use_compress) 
{
    /* 1. 调整encode函数，使其能处理CompressedPoint（关键修改）
    encode(pcloud_data);  // 后续需修改encode函数，使其遍历CompressedPoint的字段
    
    clock_t start = clock();
    // 2. 保持原有压缩/打包逻辑（无需修改，复用现有流程）
    auto temp = use_compress ? compressData(packData()) : packData();
    */ 

    //定义start变量
    clock_t start = clock();  // 初始化start
    
    // 1. 先获取未压缩的完整数据（含正确meta_size+meta_data+点云）
    std::vector<char> uncompressed_data = packData();
    std::vector<char> compressed_data;

    // 2. 若需要压缩，对未压缩数据整体压缩
    if (use_compress && !uncompressed_data.empty()) {
        compressed_data = compressData(uncompressed_data);
    }

    clock_t end = clock();
    std::cout << "compress time: " << (double)(end - start) / CLOCKS_PER_SEC << std::endl;
    //return temp;

    // 返回未压缩数据（用于获取正确meta_size）和压缩后数据（用于传输）
    return {uncompressed_data, compressed_data};
}

// 修改encode函数处理CompressedPoint（原encode函数处理point_cloud）
void EncoderModule::encode(const std::vector<CompressedPoint> &pcloud_data) {
    // 1. 清空内部缓存
    internal_pcloud_.clear();
    internal_pcloud_ = pcloud_data; // 保存原始点云（供后续打包）

    // 2. 将CompressedPoint转换为原算法所需的point_cloud格式（仅保留必要字段）
    std::vector<point_cloud> legacy_cloud;
    for (const auto& pt : pcloud_data) {
        point_cloud legacy_pt;
        legacy_pt.x = pt.x;
        legacy_pt.y = pt.y;
        legacy_pt.z = pt.z;
        legacy_pt.r = pt.intensity; // 用intensity替代原r字段
        legacy_pt.color[0] = pt.r;
        legacy_pt.color[1] = pt.g;
        legacy_pt.color[2] = pt.b;
        legacy_cloud.push_back(legacy_pt);
    }

    // 3. 复用原有压缩流程（投影→拟合→SADCT）
    PccResult pcc_res;
    double proj_time, fit_time;
    float psnr;

    // 地图投影（填充f_mat）
    proj_time = map_projection(f_mat, legacy_cloud, pitch_precision, yaw_precision, 'e');
    pcc_res.proj_times->push_back(proj_time);

    // 瓦片拟合与占用图编码
    int mat_div_tile_sizes[] = {row / tile_size, col / tile_size};
    encoder::encode_occupation_mat(f_mat, occ_mat, tile_size, mat_div_tile_sizes);
    fit_time = encoder::single_channel_encode(f_mat, b_mat, mat_div_tile_sizes, coefficients,
                                              unfit_nums, tile_fit_lengths,
                                              threshold, tile_size);
    pcc_res.fit_times->push_back(fit_time);

    // SADCT变换（处理未拟合的点）
    cv::Mat mask = cv::Mat::zeros(row, col, CV_32SC1);
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            if (f_mat.at<cv::Vec4f>(i, j)[0] != 0) mask.at<int>(i, j) = 1;
        }
    }
    for (int r_idx = 0; r_idx < mat_div_tile_sizes[0]; r_idx++) {
        for (int c_idx = 0; c_idx < mat_div_tile_sizes[1]; c_idx++) {
            if (b_mat.at<int>(r_idx, c_idx) == 1) {
                for (int i = 0; i < tile_size; i++) {
                    for (int j = 0; j < tile_size; j++) {
                        mask.at<int>(r_idx * tile_size + i, c_idx * tile_size + j) = 0;
                    }
                }
            }
        }
    }
    pcc::Matrix<double> value_map(row, std::vector<double>(col, 0));
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            value_map[i][j] = f_mat.at<cv::Vec4f>(i, j)[0];
        }
    }
    pcc::Matrix<bool> mask_map(row, std::vector<bool>(col, false));
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            mask_map[i][j] = mask.at<int>(i, j) == 1;
        }
    }
    clock_t start = clock();
#ifdef USE_SADCT
    auto coeff_res = pcc::sadct(value_map, mask_map);
#else
    auto coeff_res = pcc::delta_sadct(value_map, mask_map);
#endif
    clock_t end = clock();
    std::cout << "SADCT time: " << (double)(end - start) / CLOCKS_PER_SEC << std::endl;
    dct_mat = coeff_res.first;
}

