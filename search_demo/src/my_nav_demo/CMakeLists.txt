cmake_minimum_required(VERSION 3.0.2)
project(my_drone_nav)

# 1. 查找依赖库 (对应 package.xml 中的 depend)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  eigen_conversions
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs mavros_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 2. 编译你的 C++ 节点
# 假设你把代码保存为 src/cmd_vel_bridge_local.cpp
add_executable(cmd_vel_bridge_node src/cmd_vel_bridge_local.cpp)

# 3. 链接库文件
target_link_libraries(cmd_vel_bridge_node
  ${catkin_LIBRARIES}
)