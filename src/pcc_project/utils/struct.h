#ifndef STRUCT_H
#define STRUCT_H

#include <vector>
#include <string>
#include <cstdint>
#include <pcl/point_types.h>

struct point_cloud
{
  float x;
  float y;
  float z;
  float r;
  u_int8_t color[3];
  point_cloud(float x0, float y0,
              float z0, float r0)
  {
    x = float(x0);
    y = float(y0);
    z = float(z0);
    r = float(r0);
  };
  point_cloud() : x(0), y(0),
                  z(0), r(0), color{255, 255, 255}{};
  std::vector<float> to_vector_xyz()
  {
    return {x, y, z};
  }

  std::string to_string_xyz()
  {
    return "[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "]";
  }
};

//新增
struct CompressedPoint {
    float x;          // 三维坐标x（4字节）
    float y;          // 三维坐标y（4字节）
    float z;          // 三维坐标z（4字节）
    float intensity;  // 强度值（对应PCL的intensity，4字节）
    uint8_t r;        // 颜色R通道（1字节）
    uint8_t g;        // 颜色G通道（1字节）
    uint8_t b;        // 颜色B通道（1字节）
};
POINT_CLOUD_REGISTER_POINT_STRUCT(CompressedPoint,
    (float, x, x)        // 格式：(类型, 成员名, 别名)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint8_t, r, r) // 修正：使用 std::uint8_t
    (std::uint8_t, g, g)
    (std::uint8_t, b, b)
)
///////
struct range3d
{
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
  std::string to_string()
  {
    return "[min]: " + std::to_string(x_min) + "," + std::to_string(y_min) + "," + std::to_string(z_min) + "; [max]: " + std::to_string(x_max) + "," + std::to_string(y_max) + "," + std::to_string(z_max);
  }
};

struct pframe
{
  range3d range;
  std::vector<point_cloud> points;
  pframe(range3d r, std::vector<point_cloud> ps) : range(r), points(ps){};
  pframe(){};
};

struct PccResult
{
  std::vector<float> *compression_rate;
  std::vector<float> *fitting_ratios;
  std::vector<float> *restored_compression_rate;
  std::vector<float> *stream_compression_rates;
  std::vector<float> *loss_rate;
  std::vector<float> *restored_loss_rate;
  std::vector<float> *match_pcts;
  std::vector<float> *unmatch_pcts;
  std::vector<float> *matchs;
  std::vector<float> *unmatchs;
  std::vector<double> *proj_times;
  std::vector<double> *fit_times;
  std::vector<double> *merge_times;
  PccResult()
  {
    compression_rate = new std::vector<float>();
    fitting_ratios = new std::vector<float>();
    restored_compression_rate = new std::vector<float>();
    stream_compression_rates = new std::vector<float>();
    loss_rate = new std::vector<float>();
    restored_loss_rate = new std::vector<float>();
    match_pcts = new std::vector<float>();
    unmatch_pcts = new std::vector<float>();
    matchs = new std::vector<float>();
    unmatchs = new std::vector<float>();
    proj_times = new std::vector<double>();
    fit_times = new std::vector<double>();
    merge_times = new std::vector<double>();
  }
  ~PccResult()
  {
    delete compression_rate;
    delete fitting_ratios;
    delete restored_compression_rate;
    delete stream_compression_rates;
    delete loss_rate;
    delete restored_loss_rate;
    delete match_pcts;
    delete unmatch_pcts;
    delete matchs;
    delete unmatchs;
    delete proj_times;
    delete fit_times;
    delete merge_times;
  }
};

#endif
