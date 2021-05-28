cmake_minimum_required(VERSION 3.1)
project(hello_slam)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build Type : ${CMAKE_BUILD_TYPE}")

add_library(hello SHARED
    src/hello.cpp
)
target_include_directories(hello PUBLIC
    include
)
set_target_properties(hello PROPERTIES
    PUBLIC_HEADER include/hello.h
)


add_executable(sayHello
    useHello.cpp
)
target_link_libraries(sayHello PUBLIC
    hello
)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
INSTALL(TARGETS hello sayHello
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)
