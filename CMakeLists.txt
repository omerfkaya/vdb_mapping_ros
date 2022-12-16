cmake_minimum_required(VERSION 3.5)
project(vdb_mapping_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  "rclcpp"
  "vdb_mapping"
  "vdb_mapping_msgs"
  "std_srvs"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "std_msgs"
  "tf2_eigen"
  "tf2_ros"
  "tf2_sensor_msgs"
  "pcl_conversions"
  "pcl_ros"
)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(${PROJECT_NAME}_node src/vdb_mapping_ros_node.cpp)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_node ${THIS_PACKAGE_INCLUDE_DEPENDS})


target_link_libraries(${PROJECT_NAME}_node
  vdb_mapping::vdb_mapping
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
