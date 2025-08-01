cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set C++ standard (cleaned up redundancies)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add root directory definition
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Processor configuration
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number: ${N}")

  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("Cores for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("Cores for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Find packages
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Third-party libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

message(STATUS "Eigen: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")

# Define ROS2 dependencies list
set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

# Generate custom messages
set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

# ============= Common Library =============
add_library(fast_lio_common SHARED
  src/common/GeometryUtils.cpp
  src/common/DataStructures.cpp
)

target_include_directories(fast_lio_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Call ament_target_dependencies FIRST
ament_target_dependencies(fast_lio_common PUBLIC ${dependencies})

# Then call target_link_libraries
target_link_libraries(fast_lio_common PUBLIC
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Handle ROS2 version differences for common library
list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")
if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  rosidl_target_interfaces(fast_lio_common
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fast_lio_common PUBLIC ${cpp_typesupport_target})
endif()

# ============= Main Executable =============
add_executable(fastlio_mapping 
  src/laserMapping.cpp 
  include/ikd-Tree/ikd_Tree.cpp 
  src/preprocess.cpp
)

target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
)

# Call ament_target_dependencies FIRST
ament_target_dependencies(fastlio_mapping PUBLIC ${dependencies})

# Then call target_link_libraries
target_link_libraries(fastlio_mapping PUBLIC
  fast_lio_common  # Link our common library
  ${PCL_LIBRARIES} 
  ${PYTHON_LIBRARIES} 
  Eigen3::Eigen
)

# Handle ROS2 version differences for main executable
if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  rosidl_target_interfaces(fastlio_mapping
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  target_link_libraries(fastlio_mapping PUBLIC ${cpp_typesupport_target})
endif()

# ============= Installation =============
install(TARGETS fast_lio_common
  EXPORT fast_lio_commonTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Export targets
ament_export_targets(fast_lio_commonTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(fast_lio_common)
ament_export_dependencies(${dependencies})

# ============= Testing =============
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Test for config (existing)
  ament_add_gtest(test_config 
    test/test_config.cpp
  )
  
  target_include_directories(test_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/config
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  
  target_link_libraries(test_config
    ${PCL_LIBRARIES}
    Eigen3::Eigen
  )
  
  ament_target_dependencies(test_config rclcpp)
  
  # Tests for common library
  ament_add_gtest(test_common
    test/common/test_constants.cpp
    test/common/test_math_utils.cpp
    test/common/test_geometry_utils.cpp
    test/common/test_time_utils.cpp
    test/common/test_data_structures.cpp
  )
  
  target_include_directories(test_common PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  
  target_link_libraries(test_common
    fast_lio_common
    ${PCL_LIBRARIES}
    Eigen3::Eigen
  )
  
  ament_target_dependencies(test_common ${dependencies})
endif()

ament_package()