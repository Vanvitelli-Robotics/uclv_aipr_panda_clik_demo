cmake_minimum_required(VERSION 3.8)
project(uclv_aipr_panda_clik_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uclv_aipr_panda_clik_demo_interfaces REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(moveit_ros_planning REQUIRED) # <-- needed to use robot model and robot state in the clik node
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



include_directories(include)

add_executable(joint_traj_action_server src/joint_traj_action_server_node.cpp)
ament_target_dependencies(joint_traj_action_server 
      "rclcpp" "rclcpp_action" "sensor_msgs" "uclv_aipr_panda_clik_demo_interfaces" )

add_executable(cartesian_traj_action_server src/cartesian_traj_action_server_node.cpp)
ament_target_dependencies(cartesian_traj_action_server 
      "rclcpp" "rclcpp_action" "geometry_msgs" "uclv_aipr_panda_clik_demo_interfaces" )

add_executable(clik src/clik_node.cpp)
target_link_libraries(clik Eigen3::Eigen) # Eigen is not an ament package, we have to link it manually
ament_target_dependencies(clik 
      "rclcpp" "geometry_msgs" "sensor_msgs" "std_srvs" "moveit_ros_planning" )

add_executable(task src/task_node.cpp)
ament_target_dependencies(task 
      "rclcpp" "rclcpp_action" "geometry_msgs" "std_srvs" "tf2" "tf2_ros" "uclv_aipr_panda_clik_demo_interfaces")

########## INSTALL nodes #######################
install(TARGETS
  joint_traj_action_server cartesian_traj_action_server clik task
  DESTINATION lib/${PROJECT_NAME}
)
################################################

########## INSTALL the launch files ############
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
################################################


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
