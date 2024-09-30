cmake_minimum_required(VERSION 3.16)
project(dsr_hardware2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

include(cmake/drfl.cmake)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  dsr_msgs2
  moveit_msgs
  sensor_msgs
  controller_manager
  ament_index_cpp
  yaml-cpp
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

if($ENV{ROS_DISTRO} STREQUAL "humble")                          
else()
    message(FATAL_ERROR "Unknown ROS distro: $ENV{ROS_DISTRO}")
endif()


add_library(
  dsr_hardware2
  SHARED
  src/dsr_hw_interface2.cpp
)

target_compile_features(dsr_hardware2 PUBLIC cxx_std_17)
target_include_directories(dsr_hardware2 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/dsr_hardware2>
)
target_link_libraries(dsr_hardware2 yaml-cpp doosan::drflapi Eigen3::Eigen)
ament_target_dependencies(
  dsr_hardware2 
  ${HW_IF_INCLUDE_DEPENDS}
  # ${CONTROLLER_INCLUDE_DEPENDS}
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DSR_HARDWARE2_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface dsr_hardware2.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS dsr_hardware2
  EXPORT export_dsr_hardware2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_dsr_hardware2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
