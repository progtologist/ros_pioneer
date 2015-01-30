cmake_minimum_required(VERSION 2.8.3)
project(ros_arcos)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_arcos
)

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE HEADERS include/${PROJECT_NAME}/*.h)

add_library(arcos_packet src/arcos_packet.cpp ${HEADERS})
target_link_libraries(arcos_packet ${catkin_LIBRARIES})

file(GLOB_RECURSE TESTS ${PROJECT_SOURCE_DIR}/test/*.cpp)

foreach(ITEM ${TESTS})
    get_filename_component(filename ${ITEM} NAME_WE)
    catkin_add_gtest(${filename} ${ITEM})
    target_link_libraries(${filename} arcos_packet)
endforeach()
