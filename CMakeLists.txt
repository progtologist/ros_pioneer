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
configure_file(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h.in
               ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h)

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE HEADERS include/${PROJECT_NAME}/*.h)
file(GLOB_RECURSE SOURCES src/*.cpp)

#################
## Documenting ##
#################
option(BUILD_DOCS "Use Doxygen to create the HTML based API documentation" OFF)
if(BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        configure_file(${PROJECT_SOURCE_DIR}/doc/Doxyfile.in ${PROJECT_BINARY_DIR}/doc/doxygen/Doxyfile @ONLY)
        add_custom_target(docs 
                          COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/doc/doxygen/Doxyfile
                          WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/doc/doxygen/
                          COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
    endif(DOXYGEN_FOUND)
endif(BUILD_DOCS)

option(BUILD_SPHINX "Use sphinx to create the HTML based API documentation" OFF)
if(BUILD_SPHINX)
    find_package(Sphinx)
    # configured documentation tools and intermediate build results
    set(BINARY_BUILD_DIR "${PROJECT_SOURCE_DIR}/doc")
    # Sphinx cache with pickled ReST documents
    set(SPHINX_CACHE_DIR "${PROJECT_BINARY_DIR}/doc/sphinx/_doctrees")
    # HTML output directory
    set(SPHINX_HTML_DIR "${PROJECT_BINARY_DIR}/doc/sphinx/html")
    # Copy the configured files for READTHEDOCS
    configure_file("${PROJECT_SOURCE_DIR}/doc/conf.py.in" "${PROJECT_SOURCE_DIR}/doc/conf.py" @ONLY)
    configure_file("${PROJECT_SOURCE_DIR}/doc/index.rst.in" "${PROJECT_SOURCE_DIR}/doc/index.rst" @ONLY)
    add_custom_target(sphinx 
                      COMMAND ${SPHINX_EXECUTABLE}
                      -q -b html
                      -c "${BINARY_BUILD_DIR}"
                      -d "${SPHINX_CACHE_DIR}"
                      "${BINARY_BUILD_DIR}"
                      "${SPHINX_HTML_DIR}"
                      COMMENT "Building HTML documentation with Sphinx")
endif(BUILD_SPHINX)
##############
## Building ##
##############
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#############
## Testing ##
#############
option(BUILD_TESTS "Use Gtest to create the test cases for the code" OFF)
if(BUILD_TESTS OR ENABLE_COVERAGE)
    enable_testing()
    file(GLOB_RECURSE TESTS ${PROJECT_SOURCE_DIR}/test/*.cpp)
    foreach(ITEM ${TESTS})
        get_filename_component(filename ${ITEM} NAME_WE)
        catkin_add_gtest(${filename} ${ITEM})
        target_link_libraries(${filename} ${PROJECT_NAME})
    endforeach()
endif(BUILD_TESTS)
##############
## Coverage ##
##############
option(ENABLE_COVERAGE "Enable code coverage generation" OFF)
if(${ENABLE_COVERAGE} AND "${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_BUILD_TYPE "Debug")
    add_definitions(-fprofile-arcs -ftest-coverage)
    # link project library to support coverage
    target_link_libraries(${PROJECT_NAME} -fprofile-arcs -ftest-coverage)
    foreach(ITEM ${TESTS})
        get_filename_component(filename ${ITEM} NAME_WE)
        target_link_libraries(${filename} -fprofile-arcs -ftest-coverage)
    endforeach()
endif()
################
## Installing ##
################
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)