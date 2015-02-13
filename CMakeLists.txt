cmake_minimum_required(VERSION 2.8.3)
project(ros_arcos)

find_package(catkin REQUIRED
             COMPONENTS roscpp roslib tf geometry_msgs nav_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_arcos
)

set(PACKAGE_PATH "${PROJECT_SOURCE_DIR}")
configure_file(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h.in
               ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h)

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE HEADERS include/${PROJECT_NAME}/*.h)
file(GLOB_RECURSE SOURCES src/*.cpp)

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

file(GLOB_RECURSE TESTS ${PROJECT_SOURCE_DIR}/test/*.cpp)

foreach(ITEM ${TESTS})
    get_filename_component(filename ${ITEM} NAME_WE)
    catkin_add_gtest(${filename} ${ITEM})
    target_link_libraries(${filename} ${PROJECT_NAME})
endforeach()
