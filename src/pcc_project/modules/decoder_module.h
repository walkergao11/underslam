#ifndef DECODER_MODULE_H
#define DECODER_MODULE_H

#include "decoder.h"
#include "config.h"
#include "dct.h"
#include "../utils/struct.h"
#include "serializer.h"
#include "binary_compressor.h"
class DecoderModule
{
public:
  int row;
  int col;
  int tile_size;
  int point_num_;  // 点数量
  float pitch_precision;
  float yaw_precision;
  int idx_sizes[2];
  cv::Mat r_mat;
  cv::Mat b_mat;
  cv::Mat occ_mat;
  pcc::Matrix<double> dct_mat;
  pcc::Matrix<bool> unfit_mask_mat;
  std::vector<cv::Vec4f> coefficients;
  std::vector<int> tile_fit_lengths;
  std::vector<float> unfit_nums;
  std::vector<CompressedPoint> restored_pcloud;

public:
  DecoderModule(float pitch_precision, float yaw_precision, int tile_size);
  DecoderModule(const std::vector<char> &data, int tile_size, bool use_compress, int ksamlple=-1); // decode from data

  ~DecoderModule();

  void decode(cv::Mat &b_mat, const int *idx_sizes,
              std::vector<cv::Vec4f> &coefficients, cv::Mat &occ_mat,
              std::vector<int> &tile_fit_lengths,
              pcc::Matrix<double> &dct_mat);
  /**
   * @brief Decode the bitstream to the point cloud
   *
   * @param b_mat the plane or not plane mask
   * @param idx_sizes size of img divided by tile size
   * @param coefficients plane coefficients
   * @param occ_mat point mask using int to record
   * @param tile_fit_lengths plane length
   * @param dct_mat dct coefficients matrix
   */
  void decode(cv::Mat &b_mat, const int *idx_sizes,
              std::vector<cv::Vec4f> &coefficients, cv::Mat &occ_mat,
              std::vector<int> &tile_fit_lengths,
              pcc::Matrix<double> &dct_mat, int ksamlple);
  void unpackdata(const std::vector<char> &data);
  void decodeFromData(const std::vector<char> &data, bool use_compress = false);
  void decodeFromData(const std::vector<char> &data, bool use_compress, int ksamlple);

    // 从矩阵恢复CompressedPoint点云（坐标）的函数声明
    void restore_pcloud(const cv::Mat& r_mat, float pitch_precision, float yaw_precision,
                       std::vector<CompressedPoint>& pcloud);
    // 解析额外点云（ksample版本用）
    void restore_extra_pcloud(float pitch_precision, float yaw_precision,
                             std::vector<CompressedPoint>& pcloud,
                             const pcc::Matrix<float>& extra_pc);
    //解析点云字段（intensity、r、g、b）
    void parseCompressedPointData(const std::vector<char>& point_data);
    
};

#endif