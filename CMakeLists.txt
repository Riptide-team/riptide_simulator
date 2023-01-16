cmake_minimum_required(VERSION 3.8)
project(riptide_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package (Eigen3 3.4 REQUIRED)

find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(kdl_parser REQUIRED)

# Protobuf message generation
if (EXISTS "${CMAKE_SOURCE_DIR}/msgs/")
  # Message generation. Only required when using custom Protobuf messages.
  find_package(Protobuf REQUIRED)
  add_subdirectory(msgs)
  set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES GENERATED TRUE)
endif()

# riptide_msgs
set(MSGS_INCLUDE)
foreach(HDR ${PROTO_HEADER})  
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})  
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  list(APPEND MSGS_INCLUDE ${PROJECT_BINARY_DIR}/${DIR})
endforeach()

add_library(riptide_msgs SHARED ${PROTO_HEADER} ${PROTO_SRC})
add_dependencies(riptide_msgs protobuf_compilation)
target_include_directories(riptide_msgs PUBLIC ${MSGS_INCLUDE})


# Simulator library
add_library(simulator_lib src/Simulator.cpp src/RiptideParameters.cpp src/Pool.cpp)
target_include_directories(simulator_lib PUBLIC include)
target_link_libraries(simulator_lib Eigen3::Eigen riptide_msgs)
ament_target_dependencies(simulator_lib rclcpp ignition-transport11 ignition-msgs8 std_msgs visualization_msgs tf2 tf2_ros kdl_parser)

# Simulator executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC simulator_lib)

# Spawner node
add_executable(riptide_spawner src/riptide_spawner.cpp)
target_link_libraries(riptide_spawner riptide_msgs)
ament_target_dependencies(riptide_spawner rclcpp ignition-transport11 ignition-msgs8)

# Install
install(TARGETS ${PROJECT_NAME} riptide_spawner riptide_msgs DESTINATION lib/${PROJECT_NAME})
install(TARGETS riptide_msgs DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

foreach(HDR ${PROTO_HEADER})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})  
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  list(APPEND MSGS_INCLUDE ${PROJECT_BINARY_DIR}/${DIR})
  install(FILES ${HDR} DESTINATION include/${PROJECT_NAME}/${DIR})
endforeach()

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

ament_export_libraries(riptide_msgs)
ament_export_include_directories(include)

ament_package()
