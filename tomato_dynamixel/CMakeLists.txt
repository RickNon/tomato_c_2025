cmake_minimum_required(VERSION 3.0.2)
project(tomato_dynamixel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_sdk
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mx_pos src/mx_pos.cpp)
target_link_libraries(mx_pos ${catkin_LIBRARIES})

add_executable(xc_pos src/xc_pos.cpp)
target_link_libraries(xc_pos ${catkin_LIBRARIES})

add_executable(ax_pos src/ax_pos.cpp)
target_link_libraries(ax_pos ${catkin_LIBRARIES})

add_executable(ax_double_pos src/ax_double_pos.cpp)
target_link_libraries(ax_double_pos ${catkin_LIBRARIES})

add_executable(ax_joy src/ax_joy.cpp)
target_link_libraries(ax_joy ${catkin_LIBRARIES})


# add_executable(ax_mx_joy src/ax_mx_joy.cpp)
# target_link_libraries(ax_mx_joy ${catkin_LIBRARIES})

add_executable(ax_mx_arm src/ax_mx_arm.cpp)
target_link_libraries(ax_mx_arm ${catkin_LIBRARIES})

add_executable(ax_mx_arm2 src/ax_mx_arm2.cpp)
target_link_libraries(ax_mx_arm2 ${catkin_LIBRARIES})
