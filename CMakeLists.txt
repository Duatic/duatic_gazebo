cmake_minimum_required(VERSION 3.22)
project(duatic_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(duatic_helper_msgs REQUIRED)

add_executable(sim_scene_node src/sim_scene_node.cpp)
target_link_libraries(sim_scene_node
  rclcpp::rclcpp
  gz-transport::core
  gz-msgs::core
  ${duatic_helper_msgs_TARGETS}
)

install(TARGETS sim_scene_node DESTINATION lib/${PROJECT_NAME})

# Simulated DuaDrive control law, linked into the hardware plugin.
add_library(sim_duadrive_law STATIC src/sim_duadrive_law.cpp)
set_target_properties(sim_duadrive_law PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(sim_duadrive_law PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# rqt plugin that embeds the Gazebo GUI window.
# rqt links Qt5 up to Kilted and Qt6 after, and leaves finding it to its users.
find_package(QT NAMES Qt5 Qt6 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
if(QT_VERSION_MAJOR EQUAL 5)
  # Qt6 has the X11 connection accessor in QtGui.
  find_package(Qt5 REQUIRED COMPONENTS X11Extras)
endif()
find_package(pluginlib REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(XCB REQUIRED IMPORTED_TARGET xcb)

add_library(rqt_gazebo_view SHARED src/rqt_gazebo_view.cpp)
set_target_properties(rqt_gazebo_view PROPERTIES AUTOMOC ON)
target_link_libraries(rqt_gazebo_view
  pluginlib::pluginlib
  rqt_gui_cpp::rqt_gui_cpp
  PkgConfig::XCB
)
if(QT_VERSION_MAJOR EQUAL 5)
  target_link_libraries(rqt_gazebo_view Qt5::X11Extras)
endif()

install(TARGETS rqt_gazebo_view LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(rqt_gui plugin.xml)

# Simulated DuaDrive hardware plugin for gz_ros2_control.
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

add_library(sim_duadrive_system SHARED src/sim_duadrive_system.cpp)
# gz_ros2_control headers only; none of its libraries is linked.
target_include_directories(sim_duadrive_system PRIVATE
  $<TARGET_PROPERTY:gz_ros2_control::gz_ros2_control-system,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(sim_duadrive_system
  sim_duadrive_law
  gz-sim::gz-sim
  gz-transport::core
  gz-msgs::core
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

install(TARGETS sim_duadrive_system LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(gz_ros2_control sim_duadrive_plugin.xml)

# Install the python module "duatic_gazebo"
ament_python_install_package(${PROJECT_NAME})

# Install  directories
install(DIRECTORY
  launch
  worlds
  maps
  models
  docker
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts as executables
install(PROGRAMS
  scripts/start_sim.py
  scripts/unpause_sim.py
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # Enable testing
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/launch_gazebo_warehouse.py)
  add_ros_isolated_launch_test(test/launch_sim_scene_weld.py)
  add_ros_isolated_launch_test(test/launch_sim_duadrive.py TIMEOUT 300)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sim_duadrive_law test/test_sim_duadrive_law.cpp)
  target_link_libraries(test_sim_duadrive_law sim_duadrive_law)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gz_resource_path test/test_gz_resource_path.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_sim_drive test/test_sim_drive.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_sdf_to_map test/test_sdf_to_map.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
endif()

ament_package()
