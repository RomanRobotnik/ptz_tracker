cmake_minimum_required(VERSION 3.16)
project(ptz_tracker)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

unset( CATKIN_INSTALL_INTO_PREFIX_ROOT )
unset( CATKIN_SYMLINK_INSTALL )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()