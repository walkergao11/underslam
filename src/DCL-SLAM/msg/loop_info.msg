# Loop Info
Header header

int32 robot0
int32 robot1
int32 index0
int32 index1
float32 noise
int32 init_yaw
int32 tile_size   # 点云压缩的瓦片大小
int32 q_level     # 点云压缩的量化等级
geometry_msgs/Transform pose0
geometry_msgs/Transform pose1
sensor_msgs/PointCloud2 scan_cloud
geometry_msgs/Transform pose_between

# --- 新增以下字段 ---
pcc_project/CompressedPointCloudData compressed_scan_cloud
# ------------------------