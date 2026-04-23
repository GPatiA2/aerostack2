cmake_minimum_required(VERSION 3.8)
project(as2_behaviors_collision_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_behavior REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(as2_ca REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPS
  rclcpp rclcpp_action rclcpp_components pluginlib
  as2_core as2_behavior as2_msgs as2_ca   # as2_core includes StateInterface (Step 0)
  nav_msgs geometry_msgs std_srvs Eigen3)

# ── Shared plugin library ──────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/path_geometry.cpp
  plugins/pairwise_path_lock_plugin.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${DEPS})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# ── Behavior server library (component) ───────────────────────────────────────
add_library(collision_avoidance_behavior_component SHARED
  src/collision_avoidance_behavior.cpp
)
target_include_directories(collision_avoidance_behavior_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(collision_avoidance_behavior_component ${DEPS})
rclcpp_components_register_nodes(collision_avoidance_behavior_component
  "collision_avoidance_behavior::CollisionAvoidanceBehavior")

# ── Standalone node executable ─────────────────────────────────────────────────
add_executable(collision_avoidance_behavior_node
  src/collision_avoidance_behavior_node.cpp
  src/collision_avoidance_behavior.cpp
  src/path_geometry.cpp
)
target_include_directories(collision_avoidance_behavior_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(collision_avoidance_behavior_node ${DEPS})
target_link_libraries(collision_avoidance_behavior_node ${PROJECT_NAME})

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  ${PROJECT_NAME}
  collision_avoidance_behavior_component
  collision_avoidance_behavior_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_geometry test/test_path_geometry.cpp src/path_geometry.cpp)
  target_include_directories(test_path_geometry PUBLIC include)
  ament_target_dependencies(test_path_geometry as2_msgs nav_msgs geometry_msgs Eigen3)

  ament_add_gtest(test_plugin_logic test/test_plugin_logic.cpp
    plugins/pairwise_path_lock_plugin.cpp src/path_geometry.cpp)
  target_include_directories(test_plugin_logic PUBLIC include)
  ament_target_dependencies(test_plugin_logic ${DEPS})
endif()

ament_package()
