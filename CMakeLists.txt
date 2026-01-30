cmake_minimum_required(VERSION 3.8)
project(duatic_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install  directories
install(DIRECTORY
  launch
  worlds
  models
  docker
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts as executables
install(PROGRAMS
  scripts/start_sim.py
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
endif()

ament_package()
