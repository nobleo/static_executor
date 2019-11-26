cmake_minimum_required(VERSION 3.5)

project(static_executor)

find_package(rclcpp REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
include_directories(${static_executor_INCLUDE_DIRS})

set(${PROJECT_NAME}_SRCS
  src/executable_list.cpp
  src/static_executor.cpp
  src/executors/static_single_threaded_executor.cpp
)

# "watch" template for changes
configure_file(
  "resource/logging.hpp.em"
  "logging.hpp.em.watch"
  COPYONLY
)
# generate header with logging macros
set(python_code
  "import em"
  "em.invoke(['-o', 'include/rclcpp/logging.hpp', '${CMAKE_CURRENT_SOURCE_DIR}/resource/logging.hpp.em'])")
string(REPLACE ";" "$<SEMICOLON>" python_code "${python_code}")
add_custom_command(OUTPUT include/rclcpp/logging.hpp
  COMMAND ${CMAKE_COMMAND} -E make_directory "include/rclcpp"
  COMMAND ${PYTHON_EXECUTABLE} ARGS -c "${python_code}"
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/logging.hpp.em.watch"
  COMMENT "Expanding logging.hpp.em"
  VERBATIM
)
list(APPEND ${PROJECT_NAME}_SRCS
  include/rclcpp/logging.hpp)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS})
# specific order: dependents before dependencies
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp")

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_BUILDING_LIBRARY")

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(rclcpp)
ament_package()

install(
  DIRECTORY include/ ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION include
)

