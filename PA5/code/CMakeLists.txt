cmake_minimum_required(VERSION 3.2)

project(PA5)

find_package(OpenCV 3.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(computeORB
    computeORB.cpp
)

target_link_libraries(computeORB
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
)


add_executable(E2Rt
    E2Rt.cpp
)
target_link_libraries(E2Rt
    Eigen3::Eigen
)

add_executable(GN-BA
    GN-BA.cpp
)
target_link_libraries(GN-BA
    Eigen3::Eigen
    Sophus::Sophus
)

add_executable(icp_align
    icp_align.cpp
)
target_link_libraries(icp_align
    Eigen3::Eigen
    Sophus::Sophus
    pangolin
)