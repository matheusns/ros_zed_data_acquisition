cmake_minimum_required(VERSION 2.8.3)

project(zed_data_acquisition)

set(APP_NAME dathomir_basler_node)
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread -w")

SET(Boost_USE_MULTITHREADED ON)


set(MSGS_REQUIRED std_msgs)
set(ZED_CATKIN_DEPENDS roscpp rospy image_transport sensor_msgs cv_bridge dynamic_reconfigure ${MSGS_REQUIRED})

file(GLOB_RECURSE ZED_PARAM_INCLUDE  "include/zed_acquisition_param.hpp")
file(GLOB_RECURSE ZED_PARAM_SOURCE   "src/zed_acquisition_param.cpp")

file(GLOB_RECURSE ZED_FILE_HEADER   "include/zed_file_manager.hpp")
file(GLOB_RECURSE ZED_FILE_SOURCE   "src/zed_file_manager.cpp")

file(GLOB_RECURSE ZED_DEPTH_INCLUDE  "include/zed_depth_acquisition.hpp")
file(GLOB_RECURSE ZED_DEPTH_SOURCE   "src/depth_node/zed_depth_acquisition.cpp")
file(GLOB_RECURSE ZED_DEPTH_MAIN     "src/depth_node/main.cpp")

file(GLOB_RECURSE ZED_LEFT_INCLUDE  "include/zed_left_acquisition.hpp")
file(GLOB_RECURSE ZED_LEFT_SOURCE   "src/left_node/zed_depth_acquisition.cpp")
file(GLOB_RECURSE ZED_LEFT_MAIN     "src/left_node/main.cpp")

file(GLOB_RECURSE ZED_RIGHT_INCLUDE  "include/zed_right_acquisition.hpp")
file(GLOB_RECURSE ZED_RIGHT_SOURCE   "src/right_node/zed_right_acquisition.cpp")
file(GLOB_RECURSE ZED_RIGHT_MAIN     "src/right_node/main.cpp")

set(ZED_FILE_SOURCES ${ZED_FILE_SOURCE} ${ZED_FILE_HEADER})

set(ZED_DEPTH_NODE_SOURCES ${ZED_DEPTH_MAIN} ${ZED_DEPTH_SOURCE} ${ZED_PARAM_SOURCE})
set(ZED_DEPTH_NODE_HEADERS ${ZED_PARAM_INCLUDE} ${ZED_DEPTH_INCLUDE})

set(ZED_LEFT_NODE_SOURCES ${ZED_LEFT_MAIN} ${ZED_LEFT_SOURCE} ${ZED_PARAM_SOURCE})
set(ZED_LEFT_NODE_HEADERS ${ZED_PARAM_INCLUDE} ${ZED_LEFT_INCLUDE})

set(ZED_RIGHT_NODE_SOURCES ${ZED_RIGHT_MAIN} ${ZED_RIGHT_SOURCE} ${ZED_PARAM_SOURCE})
set(ZED_RIGHT_NODE_HEADERS ${ZED_PARAM_INCLUDE} ${ZED_RIGHT_INCLUDE})

find_package(PkgConfig)

find_package(OpenCV 3 REQUIRED)
find_package(Boost  REQUIRED COMPONENTS filesystem system thread)
find_package(catkin  REQUIRED COMPONENTS cv_bridge ${ZED_CATKIN_DEPENDS} cmake_modules message_generation)

generate_dynamic_reconfigure_options(cfg/depth_map.cfg)

catkin_package(
    INCLUDE_DIRS   
    LIBRARIES      ${PROJECT_NAME}
    CATKIN_DEPENDS ${ZED_CATKIN_DEPENDS} 
    DEPENDS        OpenCV Boost
)


include_directories(${CATKIN_DEVEL_PREFIX}/include)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}" "${CMAKE_SOURCE_DIR}")

include_directories(
)

link_directories(
)

set(SHARED_LIBRARIES pthread stdc++ )

add_executable(zed_depth_acqusition_node ${ZED_DEPTH_NODE_SOURCES} ${ZED_DEPTH_NODE_HEADERS} ${ZED_FILE_SOURCES})
target_link_libraries(zed_depth_acqusition_node ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${SHARED_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(zed_depth_acqusition_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(zed_left_acqusition_node ${ZED_RIGHT_NODE_SOURCES} ${ZED_RIGHT_NODE_HEADERS} ${ZED_FILE_SOURCES})
target_link_libraries(zed_left_acqusition_node ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${SHARED_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(zed_right_acqusition_node ${ZED_DEPTH_NODE_SOURCES} ${ZED_DEPTH_NODE_HEADERS} ${ZED_FILE_SOURCES})
target_link_libraries(zed_right_acqusition_node ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${SHARED_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})