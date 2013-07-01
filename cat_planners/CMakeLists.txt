cmake_minimum_required(VERSION 2.8.3)
project(cat_planners)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(catkin COMPONENTS
  moveit_ros_planning
  moveit_core
  roscpp
  rosconsole
  pluginlib
  tf
  dynamic_reconfigure
  REQUIRED
)

generate_dynamic_reconfigure_options(
  "cfg/CVX.cfg"
)

catkin_package(
  LIBRARIES
    cat_planners
    cvx_solvers
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning
    )

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#add_definitions( -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO )

add_library(cvx_solvers
  src/cvxgen_quad/cvx.cpp
  src/cvxgen_constraints/cvx.cpp
  src/cvxgen_extended/cvx.cpp
  src/cvxgen_posture/cvx.cpp
)

add_library(cat_planners
  src/linear_joint_stepper.cpp
  src/convex_constraint_solver.cpp
)
add_dependencies(cat_planners moveit_msgs_gencpp)

target_link_libraries(cat_planners cvx_solvers)


install(TARGETS cat_planners cvx_solvers LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES lib/cat_planners_plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/ DESTINATION include)








