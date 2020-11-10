cmake_minimum_required(VERSION 2.8)
project(CROWD-SLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
  #SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

add_compile_options(-std=c++14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

find_package(OpenCV 3.4 REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Pangolin REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Frame.cc
src/Tracking.cc
src/Detector.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${Pangolin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${Boost_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
)


target_link_libraries( ${PROJECT_NAME}
${OpenCV_LIBS}
${Boost_LIBRARIES}
${EIGEN3_INCLUDE_DIR}
${Pangolin_LIBRARIES}

${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

add_executable(rgbd
Examples/RGB-D/rgbd.cc)

target_link_libraries(rgbd ${PROJECT_NAME})




