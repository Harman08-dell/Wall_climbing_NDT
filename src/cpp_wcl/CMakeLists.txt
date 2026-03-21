cmake_minimum_required(VERSION 3.8)
project(cpp_wcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Build WCL Debug Gazebo Plugin
add_library(wcl_debug_plugin SHARED
  src/wcl_debug_plugin.cpp
)

target_link_libraries(wcl_debug_plugin
  ${GAZEBO_LIBRARIES}
)

# ✅ FIX: removed wcl_debug_plugin from DIRECTORY install
install(
  DIRECTORY launch urdf config model
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS wcl_debug_plugin
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
