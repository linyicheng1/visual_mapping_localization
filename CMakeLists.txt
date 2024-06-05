cmake_minimum_required(VERSION 3.1)
project(VISUAL_MAPPING_LOCALIZATION)

set(CMAKE_CXX_STANDARD 14)
set(OpenCV_DIR "/home/vio/libs/opencv/4.8.0/install/lib/cmake/opencv4/")
find_package(OpenCV REQUIRED)
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

find_package(Pangolin REQUIRED)

add_subdirectory(Thirdparty/g2o)

include_directories(
        inc
        ${PROJECT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${G2O_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)

add_library(ReusableMapLib
        src/camera.cpp
        src/matcher.cpp
        src/triangulation.cpp
        src/frame.cpp
        src/bundle_adjustment.cpp
        src/visualization.cpp
        src/detection.cpp
        src/map_point.cpp
        src/mapping.cpp
        src/map_save.cpp)

target_link_libraries(
        ReusableMapLib
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

add_executable(ReusableMap main.cpp)

target_link_libraries(ReusableMap
        ReusableMapLib
)

add_executable(Localization Demo/localization.cpp)

target_link_libraries(Localization
        ReusableMapLib
)

