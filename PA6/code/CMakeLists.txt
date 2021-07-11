cmake_minimum_required(VERSION 3.2)

project(PA6)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(optical_flow
    optical_flow.cpp
)

target_link_libraries(optical_flow
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
)

add_executable(direct_method
    direct_method.cpp
)

target_link_libraries(direct_method
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
    Sophus::Sophus
    pangolin
)
