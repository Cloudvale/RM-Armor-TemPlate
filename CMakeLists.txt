cmake_minimum_required(VERSION 2.8)

project(SortMake)
set(DEFAULT_CXX_STANDARD 11)
aux_source_directory(src SRC_SUB)
aux_source_directory(src/thread/src SRC_Thread)
aux_source_directory(src/Serial/src SRC_Serial)
aux_source_directory(. SRC_CUR)
include_directories(include)
include_directories(src/thread/include)
include_directories(src/Serial/include)
include_directories(src/Armor/include)
# include_directories(src/Energy/include)
# include_directories(src/Energy/predictor/include)
include_directories(src/Armor/predictor/include)


# list(APPEND CMAKE_PREFIX_PATH D:\\software\\openvino\\w_openvino_toolkit_windows_2024.2.0.15519.5c0f38f83f6_x86_64\\runtime\\cmake)
# list(APPEND CMAKE_PREFIX_PATH D:\\software\\openvino\\w_openvino_toolkit_windows_2024.2.0.15519.5c0f38f83f6_x86_64\\runtime\\3rdparty\\tbb\\lib\\cmake\\TBB)


find_package(OpenCV REQUIRED)
# find_package(OpenVINO REQUIRED)
# find_package(TBB REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS} ${OpenVINO_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS} ${OpenVINO_LIBS} ${TBB_LIBS})
# include_directories("D:\\software\\openvino\\w_openvino_toolkit_windows_2024.2.0.15519.5c0f38f83f6_x86_64\\runtime\\include")

# 判断是否找到 OpenVINO 库
# if(OpenVINO_FOUND)
#     message(STATUS "OpenVINO found: ${OpenVINO_LIBS}")
#     # 链接 OpenVINO 库
#     target_link_libraries(${PROJECT_NAME} ${OpenVINO_LIBS})
# endif()

# 增加一个InferenceEngine引用
include_directories(D:/software/openvino/w_openvino_toolkit_runtime_p_2021.4.582/deployment_tools/inference_engine/include)
# link_directories(D:/software/openvino/w_openvino_toolkit_runtime_p_2021.4.582/deployment_tools/inference_engine/lib)
set(CMAKE_PREFIX_PATH D:/Desktop/w_openvino_toolkit_dev_p_2021.4.582/deployment_tools/inference_engine/share)
# set(CMAKE_PREFIX_PATH D:\\software\\openvino\\w_openvino_toolkit_runtime_p_2021.4.582\\deployment_tools\\inference_engine\\external\\tbb\\cmake)
# find_package(InferenceEngine REQUIRED)
# include_directories(${InferenceEngine_INCLUDE_DIRS})


include_directories(D:/Desktop/lzc-rm/package/eigen-3.4.0) #没法使用类似opencv那种，那种时liunx系统使用的
set(fmt_DIR D:/Desktop/lzc-rm/fmt-master/fmt-master/build) # 该处填写cmake文件的产生路径
find_package(fmt REQUIRED)

# link_directories(D:/Desktop/glog-master/glog-master/lib)
# link_directories(D:/Desktop/lzc-rm/glog-master/glog-master/build2/Release)
# set(glog_DIR D:/Desktop/lzc-rm/package/glog-master/build2) # 该处填写cmake文件的产生路径

# include_directories(${GLOG_INCLUDE_DIRS})
include_directories(D:/Desktop/lzc-rm/package/glog-0.5.0/install/include)
link_directories(D:/Desktop/lzc-rm/package/glog-0.5.0/install/lib)
# find_package(glog REQUIRED)

# include_directories(D:/Desktop/lzc-rm/package/yaml-cpp-master/include/yaml-cpp)
include_directories(D:/Desktop/lzc-rm/package/yaml-cpp-master/include)

include_directories("D:/Desktop/lzc-rm/package/matplotlib-cpp-master")
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})
include_directories("D:/software/Anaconda/envs/lzcpytorch/Lib/site-packages/numpy/core/include")

set(CMAKE_BUILD_TYPE "Debug")
add_definitions(-DARMOR_CFG="${CMAKE_CURRENT_LIST_DIR}/ArmorDetectors.yaml") #
add_definitions(-DSVM_XML="${CMAKE_CURRENT_LIST_DIR}/HOG_SVM.xml")

# 两种方式均可，方法一
add_executable(sort_app test.cpp  
            ${PROJECT_SOURCE_DIR} src/thread/src/thread.cpp
            ${PROJECT_SOURCE_DIR} src/thread/include/thread.h      
            ${PROJECT_SOURCE_DIR} src/Serial/src/serialport.cpp
            ${PROJECT_SOURCE_DIR} src/Serial/include/serialport.h
            ${PROJECT_SOURCE_DIR} src/Armor/src/autoaim.cpp
            ${PROJECT_SOURCE_DIR} src/Armor/include/autoaim.h
            ${PROJECT_SOURCE_DIR} src/Armor/src/inference.cpp
            ${PROJECT_SOURCE_DIR} src/Armor/include/inference.h
            ${PROJECT_SOURCE_DIR} src/Armor/src/armor_tracker.cpp
            ${PROJECT_SOURCE_DIR} src/Armor/include/armor_tracker.h
            # ${PROJECT_SOURCE_DIR} src/Armor/src/coordsolver.cpp
            # ${PROJECT_SOURCE_DIR} src/Armor/include/coordsolver.h
            ${PROJECT_SOURCE_DIR} src/Armor/predictor/src/predictor.cpp
            ${PROJECT_SOURCE_DIR} src/Armor/predictor/include/predictor.h
            ${PROJECT_SOURCE_DIR} src/general/general.cpp
            ${PROJECT_SOURCE_DIR} src/general/general.h
            # ${PROJECT_SOURCE_DIR} src/Energy/src/buff.cpp
            # ${PROJECT_SOURCE_DIR} src/Energy/include/buff.h
            # ${PROJECT_SOURCE_DIR} src/Energy/predictor/src/predictor.cpp
            # ${PROJECT_SOURCE_DIR} src/Energy/predictor/include/predictor.h
            # ${PROJECT_SOURCE_DIR} src/Energy/detector/inference.cpp
            # ${PROJECT_SOURCE_DIR} src/Energy/detector/inference.h
            ${InferenceEngine_INCLUDE_DIRS}
            )

# 两种方式均可，方法二
# add_executable(sort_app ${SRC_SUB} ${SRC_CUR} ${SRC_Thread} ${SRC_Serial})
# target_link_libraries(sort_app  ${OpenCV_LIBS} fmt::fmt glog::glog D:/Desktop/lzc-rm/package/yaml-cpp-master/build/libyaml-cppd.a ${Python3_LIBRARIES} ${InferenceEngine_LIBRARIES})


target_link_libraries(sort_app  ${OpenCV_LIBS} fmt::fmt D:/Desktop/lzc-rm/package/yaml-cpp-master/build/libyaml-cppd.a ${Python3_LIBRARIES} glog)
