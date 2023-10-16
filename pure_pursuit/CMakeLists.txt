cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

add_compile_options(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################


generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
add_executable(dijkstra_node src/dijkstra_node.cpp)
add_executable(costmap_node src/costmap_node.cpp)

add_dependencies(pure_pursuit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(dijkstra_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(costmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pure_pursuit_node
  ${catkin_LIBRARIES}
)
target_link_libraries(dijkstra_node
  ${catkin_LIBRARIES}
)
target_link_libraries(costmap_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

