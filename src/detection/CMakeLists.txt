cmake_minimum_required(VERSION 3.0.2)
project(detection)

# ****** 自定义选项 ******
option(WITH_MKL        "Compile demo with MKL/OpenBlas support,defaultuseMKL."          ON)
option(WITH_GPU        "Compile demo with GPU/CPU, default use CPU."                    OFF)
SET(PADDLE_DIR "~/Envs/paddle_inference" CACHE PATH "Location of libraries")
SET(PADDLE_LIB_NAME "libpaddle_inference" CACHE STRING "libpaddle_inference")
SET(CUDA_LIB "/usr/local/cuda/lib64" CACHE PATH "Location of libraries")
SET(CUDNN_LIB "/usr/lib/x86_64-linux-gnu" CACHE PATH "Location of libraries")

include(cmake/yaml-cpp.cmake)
include_directories("${CMAKE_SOURCE_DIR}/")
include_directories("${CMAKE_CURRENT_BINARY_DIR}/ext/yaml-cpp/src/ext-yaml-cpp/include")
link_directories("${CMAKE_CURRENT_BINARY_DIR}/ext/yaml-cpp/lib")

# ****** detect用源文件 ******
set(SRCS src/detect/preprocess_op.cc src/detect/object_detector.cc src/detect/picodet_postprocess.cc src/detect/utils.cc)

macro(safe_set_static_flag)
    foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
      if(${flag_var} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
      endif(${flag_var} MATCHES "/MD")
    endforeach(flag_var)
endmacro()

if (WITH_MKL)
    ADD_DEFINITIONS(-DUSE_MKL)
endif()

include_directories("${CMAKE_SOURCE_DIR}/")
include_directories("${PADDLE_DIR}/")
include_directories("${PADDLE_DIR}/third_party/install/protobuf/include")
include_directories("${PADDLE_DIR}/third_party/install/glog/include")
include_directories("${PADDLE_DIR}/third_party/install/gflags/include")
include_directories("${PADDLE_DIR}/third_party/install/xxhash/include")
include_directories("${PADDLE_DIR}/third_party/install/cryptopp/include")
include_directories("${PADDLE_DIR}/third_party/install/onnxruntime/include")
include_directories("${PADDLE_DIR}/third_party/install/paddle2onnx/include")

link_directories("${PADDLE_DIR}/third_party/install/protobuf/lib")
link_directories("${PADDLE_DIR}/third_party/install/glog/lib")
link_directories("${PADDLE_DIR}/third_party/install/gflags/lib")
link_directories("${PADDLE_DIR}/third_party/install/xxhash/lib")
link_directories("${PADDLE_DIR}/third_party/install/cryptopp/lib")
link_directories("${PADDLE_DIR}/third_party/install/onnxruntime/lib")
link_directories("${PADDLE_DIR}/third_party/install/paddle2onnx/lib")
link_directories("${PADDLE_DIR}/paddle/lib/")
link_directories("${CMAKE_CURRENT_BINARY_DIR}")

# ****** opencv ******
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(DEPS ${DEPS} ${OpenCV_LIBS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 -fopenmp -std=c++14")
set(CMAKE_STATIC_LIBRARY_PREFIX "")

# ****** MKLdnn ******
if(WITH_MKL)
  include_directories("${PADDLE_DIR}/third_party/install/mklml/include")
  if (WIN32)
    set(MATH_LIB ${PADDLE_DIR}/third_party/install/mklml/lib/mklml.lib
            ${PADDLE_DIR}/third_party/install/mklml/lib/libiomp5md.lib)
  else ()
    set(MATH_LIB ${PADDLE_DIR}/third_party/install/mklml/lib/libmklml_intel${CMAKE_SHARED_LIBRARY_SUFFIX}
            ${PADDLE_DIR}/third_party/install/mklml/lib/libiomp5${CMAKE_SHARED_LIBRARY_SUFFIX})
    execute_process(COMMAND cp -r ${PADDLE_DIR}/third_party/install/mklml/lib/libmklml_intel${CMAKE_SHARED_LIBRARY_SUFFIX} /usr/lib)
  endif ()
  set(MKLDNN_PATH "${PADDLE_DIR}/third_party/install/mkldnn")
  if(EXISTS ${MKLDNN_PATH})
    include_directories("${MKLDNN_PATH}/include")
    if (WIN32)
      set(MKLDNN_LIB ${MKLDNN_PATH}/lib/mkldnn.lib)
    else ()
      set(MKLDNN_LIB ${MKLDNN_PATH}/lib/libmkldnn.so.0)
    endif ()
  endif()
else()
  set(MATH_LIB ${PADDLE_DIR}/third_party/install/openblas/lib/libopenblas${CMAKE_STATIC_LIBRARY_SUFFIX})
endif()

# ****** DEPS 设置 ******
set(DEPS ${PADDLE_DIR}/paddle/lib/${PADDLE_LIB_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX})
set(DEPS ${DEPS}
    ${MATH_LIB} ${MKLDNN_LIB}
    glog gflags protobuf z xxhash yaml-cpp)
if(WITH_GPU)
  set(DEPS ${DEPS} ${CUDA_LIB}/libcudart${CMAKE_SHARED_LIBRARY_SUFFIX})
  set(DEPS ${DEPS} ${CUDNN_LIB}/libcudnn${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()
if (NOT WIN32)
    set(EXTERNAL_LIB "-ldl -lrt -lgomp -lz -lm -lpthread")
    set(DEPS ${DEPS} ${EXTERNAL_LIB})
endif()

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  roslib
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

# ============================================================================
# ============================================================================

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDR_DIRS}
)

add_executable(detect_node ${SRCS} src/detect_node.cpp)
ADD_DEPENDENCIES(detect_node ext-yaml-cpp)
target_link_libraries(detect_node ${DEPS} ${catkin_LIBRARIES})