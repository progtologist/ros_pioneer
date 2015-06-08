cmake_minimum_required(VERSION 2.8.3)
project(ros_pioneer)

find_package(catkin REQUIRED
             COMPONENTS roscpp roslib tf geometry_msgs nav_msgs)
find_package(Boost REQUIRED 
             COMPONENTS thread)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ros_pioneer
)

set(PACKAGE_PATH "${PROJECT_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

configure_file(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h.in
               ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h)

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE HEADERS include/${PROJECT_NAME}/*.h)
file(GLOB_RECURSE SOURCES src/*.cpp)

##############
## Building ##
##############
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    catkin_add_gtest(test_config test/test_config.cpp)
    target_link_libraries(test_config ${PROJECT_NAME})

    catkin_add_gtest(test_packet test/test_packet.cpp)
    target_link_libraries(test_packet ${PROJECT_NAME})

    add_rostest_gtest(test_driver test/test_driver.test test/test_driver.cpp)
    target_link_libraries(test_driver ${PROJECT_NAME})    
endif(CATKIN_ENABLE_TESTING)
################
## Installing ##
################
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)