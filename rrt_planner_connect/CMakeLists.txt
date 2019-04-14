cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_core
    tf
    costmap_2d
    pluginlib
    tf2_geometry_msgs
    tf2_ros
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrt_planner
  CATKIN_DEPENDS roscpp rospy std_msgs nav_core pluginlib costmap_2d 
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(planner
#   src/rrt_planner.cpp
# )

# add_dependencies(planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(planner
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )

link_directories(${catkin_LIBRARY_DIRS})

#add_library(rrt_lib src/rrt_planner.cpp)
add_library(rrt_lib2 src/rrt_planner2.cpp)
target_link_libraries(rrt_lib2 ${catkin_LIBRARIES} base_local_planner)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
