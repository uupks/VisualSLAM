cmake_minimum_required(VERSION 3.2)

find_package(OpenCV 3.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(undistort_image
    undistort_image.cpp
)
target_link_libraries(undistort_image
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
)

add_executable(disparity
    disparity.cpp
)
target_link_libraries(disparity
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
    pangolin
)

add_executable(gaussnewton
    gaussnewton.cpp
)
target_link_libraries(gaussnewton
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
)