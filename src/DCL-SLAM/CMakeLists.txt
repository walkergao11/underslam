cmake_minimum_required(VERSION 2.8.3)
project(dcl_slam)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  distributed_mapper_catkin
  # gtsam_catkin  # Feng del
  glog_catkin
  tf
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  pcc_project  # DCL-SLAM 依赖 pcc_project 包
)

# Feng add
set(GTSAM_DIR "/usr/local/lib/cmake/GTSAM")
find_package(GTSAM REQUIRED QUIET)


find_package(Boost REQUIRED COMPONENTS filesystem system thread)
find_package(libnabo)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  loop_info.msg
  global_descriptor.msg
  neighbor_estimate.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  pcc_project  # 添加
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS 

  # Feng add
  GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
  distributed_mapper_catkin
  pcc_project  # 声明 DCL-SLAM 依赖 pcc_project，确保其他包引用时能传递依赖
  LIBRARIES dcl_slam
)
#######################################################################
# include directories
#include_directories(
  #include
  #${GTSAM_INCLUDE_DIR} # Feng add
  #${PCL_INCLUDE_DIRS}
  #${catkin_INCLUDE_DIRS}
  #${OpenCV_INCLUDE_DIRS}
  #${libnabo_INCLUDE_DIRS}# add  libnabo 头文件目录
  #${pcc_project_INCLUDE_DIRS}  # 之前添加的（用于ROS生成的消息头文件）
  #${CMAKE_CURRENT_SOURCE_DIR}/../pcc_project #添加
#)
find_path(PCC_PROJECT_SRC_DIR
    NAMES modules/decoder_module.h
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../pcc_project"
    DOC "Path to the root directory of pcc_project source code"
)

# 检查是否找到了路径，如果没找到则报错并终止
if(NOT PCC_PROJECT_SRC_DIR)
    message(FATAL_ERROR "Failed to find pcc_project source directory containing modules/decoder_module.h. Check the path.")
endif()

# 将找到的 pcc_project 源码根目录添加到头文件搜索路径
include_directories(
  include
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${libnabo_INCLUDE_DIRS}
  ${pcc_project_INCLUDE_DIRS}  # 保留，用于ROS生成的消息头文件
  
  # 使用我们找到的绝对路径
  ${PCC_PROJECT_SRC_DIR}
)
######################################################################
# link directories
link_directories(
  include
  ${GTSAM_LIBRARY_DIRS} # Feng add
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${libnabo_LIBRARY_DIRS}
)

###########
## Build ##
###########
# dcl slam libraay
add_library(dcl_slam
  src/paramsServer.cpp
  src/distributedMappingBasis.cpp 
  src/distributedMapping.cpp
  src/distributedMappingVisualization.cpp
  src/distributedLoopClosure.cpp
  src/scanContextDescriptor.cpp
  src/lidarIrisDescriptor.cpp
  src/m2dpDescriptor.cpp)
target_link_libraries(dcl_slam 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${OpenMP_CXX_FLAGS} 
  ${libnabo_LIBRARIES}
  )

add_executable(${PROJECT_NAME}_loopVisualizationNode src/loopVisualizationNode.cpp)
target_link_libraries(${PROJECT_NAME}_loopVisualizationNode 
dcl_slam
${catkin_LIBRARIES} 
${PCL_LIBRARIES} 
gtsam
${pcc_project_LIBRARIES} )

