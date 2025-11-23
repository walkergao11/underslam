cmake_minimum_required(VERSION 3.16)
project(pcc_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g")
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation # <-- 新增
)
include_directories(${catkin_INCLUDE_DIRS})

# --- 测试代码 ---
# 声明消息文件
add_message_files(
  FILES
  CompressedPointCloudData.msg
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs # 新增，消息依赖于 std_msgs/UInt8MultiArray
  sensor_msgs
)
# --- 测试代码 ---

catkin_package(
    CATKIN_DEPENDS roscpp message_runtime sensor_msgs std_msgs# 新增 message_runtime
    INCLUDE_DIRS ./utils ./modules 
    LIBRARIES pcc_lib
)


# OpenCV
set(OpenCV_DIR "/home/zoratt/opencv-4.4.0/build")
find_package(OpenCV REQUIRED)
#输出cv版本
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
#输出cv库的路径
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
include_directories(${OpenCV_INCLUDE_DIRS})

#FFTW
# 查找 FFTW 库
set(FFTW_INCLUDE_DIR /usr/local/include)
set(FFTW_LIBRARY libfftw3.so)
include_directories(${FFTW_INCLUDE_DIR})
#zstd库
set(ZSTD_LIBRARY libzstd.so)

# Include PCL headers
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(./utils)
include_directories(./modules)

# Source files
set(SOURCES
    utils/utils.cpp
    utils/encoder.cpp
    utils/pcc_module.cpp
    utils/decoder.cpp
    utils/io.cpp
    modules/decoder_module.cpp
    modules/encoder_module.cpp
    modules/dct.cpp
    modules/binary_compressor.cpp
    modules/serializer.cpp
)
# Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem date_time iostreams)
include_directories(${Boost_INCLUDE_DIRS})

#gRPC
#添加库路径




# Compile source files
add_library(pcc_lib ${SOURCES} )
target_link_libraries(
    pcc_lib
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    stdc++fs
    ${FFTW_LIBRARY}
    ${ZSTD_LIBRARY}
)
# 确保库依赖消息生成
add_executable(pcc_compressor_node example.cpp) # 修改节点名和源文件
add_dependencies(pcc_compressor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) # 新增依赖
# --- 测试代码 ---

target_link_libraries(pcc_compressor_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    pthread
    pcc_lib
    ${Boost_INCLUDE_DIRS}
)
