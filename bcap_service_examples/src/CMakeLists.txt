cmake_minimum_required(VERSION 3.0)
project(bcap_example)

find_package(bcap_core)
find_package(denso_robot_core)

###########
## Build ##
###########

include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
${denso_robot_core_INCLUDE_DIRS}
)

add_executable(hand_exec hand.cpp)
target_link_libraries(hand_exec
#  ${catkin_LIBRARIES}
  ${bcap_core_LIBRARIES}
  ${denso_robot_core_LIBRARIES}
)


add_executable(motion_prepare_exec hand.cpp)
target_link_libraries(motion_prepare_exec
#  ${catkin_LIBRARIES}
  ${bcap_core_LIBRARIES}
  ${denso_robot_core_LIBRARIES}
)

add_executable(reset_cobotta_exec hand.cpp)
target_link_libraries(reset_cobotta_exec
#  ${catkin_LIBRARIES}
  ${bcap_core_LIBRARIES}
  ${denso_robot_core_LIBRARIES}
)



