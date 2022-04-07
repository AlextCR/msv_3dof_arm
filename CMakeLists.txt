cmake_minimum_required(VERSION 2.8.3)

project(msv_3dof_arm)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_node
  src/teleop_node.cpp
)

catkin_install_python(PROGRAMS src/moteus_com_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_dependencies(teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_node
  ${catkin_LIBRARIES}
)

add_executable(echo_node
  src/echo_node.cpp
)
add_dependencies(echo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(echo_node
  ${catkin_LIBRARIES}
)