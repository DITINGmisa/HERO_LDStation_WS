cmake_minimum_required(VERSION 2.6)

project(infer)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
find_package(CUDA REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  message_runtime
  OpenCV REQUIRED
)
## Generate messages in the 'msg' folder
add_message_files(FILES
	armor_item.msg
  car_item.msg
  img_result.msg
	)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
	std_msgs
)

if(WIN32)
enable_language(CUDA)
endif(WIN32)

include_directories(${PROJECT_SOURCE_DIR}/src)
# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
set(CUDA_GEN_CODE "-gencode=arch=compute_75,code=sm_75")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}  -std=c++11 -O3 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES yolov5
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
  # DEPENDS system_lib
)


find_package(OpenCV)
include_directories(
  ${catkin_INCLUDE_DIRS};${OpenCV_INCLUDE_DIRS}
)

# 添加cuda静态库
cuda_add_library(car_lib0 SHARED src/car/yololayer_car.cu)
cuda_add_library(armor_lib0 SHARED src/armor/yololayer_armor.cu)

# 原始的检测车的demo，还可以用来转换模型
cuda_add_executable(car_utils src/car/yolov5_car.cpp src/yolov5/preprocess.cu)
target_link_libraries(car_utils nvinfer)
target_link_libraries(car_utils cudart)
target_link_libraries(car_utils car_lib0)
target_link_libraries(car_utils ${OpenCV_LIBS})
# 原始的检测装甲的demo，还可以用来转换模型
cuda_add_executable(armor_utils src/armor/yolov5_armor.cpp src/yolov5/preprocess.cu)
target_link_libraries(armor_utils nvinfer)
target_link_libraries(armor_utils cudart)
target_link_libraries(armor_utils armor_lib0)
target_link_libraries(armor_utils ${OpenCV_LIBS})
# 封装好的检测车的demo，只可以直接用来检测文件夹
cuda_add_executable(car_demo src/car/car_infer.cpp src/yolov5/preprocess.cu)
target_link_libraries(car_demo nvinfer)
target_link_libraries(car_demo cudart)
target_link_libraries(car_demo car_lib0)
target_link_libraries(car_demo ${OpenCV_LIBS})
# 封装好的检测装甲的demo，只可以直接用来检测文件夹
cuda_add_executable(armor_demo src/armor/armor_infer.cpp src/yolov5/preprocess.cu)
target_link_libraries(armor_demo nvinfer)
target_link_libraries(armor_demo cudart)
target_link_libraries(armor_demo armor_lib0)
target_link_libraries(armor_demo ${OpenCV_LIBS})

# 添加动态库
add_library(car_lib1 SHARED src/car/car_infer.cpp)
add_library(armor_lib1 SHARED src/armor/armor_infer.cpp)
# 添加可执行文件infer
cuda_add_executable(infer src/infer.cpp src/yolov5/preprocess.cu)
add_dependencies(infer ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(infer ${catkin_LIBRARIES})
target_link_libraries(infer nvinfer)
target_link_libraries(infer cudart)
target_link_libraries(infer car_lib0 car_lib1)
target_link_libraries(infer armor_lib0 armor_lib1)
target_link_libraries(infer 
${OpenCV_LIBS}
${PLATFORM_LIBS}
${LIBRT_LIBRARIES} -lrt)

catkin_install_python(PROGRAMS
scripts/set_param.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
if(UNIX)
add_definitions(-O2 -pthread)
endif(UNIX)


