cmake_minimum_required(VERSION 3.12)
project(fast_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(ROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(FAST_LOCALIZATION_CPU_COUNT)
  if(FAST_LOCALIZATION_CPU_COUNT GREATER 4)
    add_compile_definitions(MP_EN MP_PROC_NUM=3)
  elseif(FAST_LOCALIZATION_CPU_COUNT GREATER 3)
    add_compile_definitions(MP_EN MP_PROC_NUM=2)
  else()
    add_compile_definitions(MP_PROC_NUM=1)
  endif()
else()
  add_compile_definitions(MP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(OpenCV REQUIRED)
find_package(OpenMP QUIET)

add_executable(fast_localization
  src/laserMapping.cpp
  src/preprocess.cpp
  include/ikd-Tree/ikd_Tree.cpp
  include/Scancontext/Scancontext.cpp)

target_include_directories(fast_localization PRIVATE
  include
  src
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fast_localization
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(fast_localization OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(fast_localization
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions)

install(TARGETS fast_localization
  DESTINATION lib/${PROJECT_NAME})
install(FILES config/robotdog.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
