cmake_minimum_required(VERSION 2.8.3)

project(zed_data_acquisition)

set(APP_NAME dathomir_basler_node)
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread -w")

set(MSGS_REQUIRED std_msgs)
set(ZED_CATKIN_DEPENDS roscpp rospy image_transport cv_bridge ${MSGS_REQUIRED})

file(GLOB_RECURSE ZED_HEADERS  "include/*.h*")
file(GLOB_RECURSE ZED_SOURCES  "src/*.c*")
file(GLOB_RECURSE ZED_MAIN     "src/main.cpp")

set(ZED_NODE_SOURCES ${ZED_MAIN} ${ZED_SOURCES} )

find_package(PkgConfig)

find_package(OpenCV 3 REQUIRED)
find_package(catkin  REQUIRED COMPONENTS cv_bridge ${ZED_CATKIN_DEPENDS} cmake_modules message_generation)

catkin_package(
    INCLUDE_DIRS   
    LIBRARIES      ${PROJECT_NAME}
    CATKIN_DEPENDS ${ZED_CATKIN_DEPENDS} 
    DEPENDS        OpenCV
)

include_directories(${CATKIN_DEVEL_PREFIX}/include)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}" "${CMAKE_SOURCE_DIR}")

include_directories(
)

link_directories(
)

set(SHARED_LIBRARIES pthread stdc++ )

add_executable(zed_acqusition_node ${ZED_NODE_SOURCES} ${ZED_HEADERS})
target_link_libraries(zed_acqusition_node ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${SHARED_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(zed_acqusition_node ${catkin_EXPORTED_TARGETS})