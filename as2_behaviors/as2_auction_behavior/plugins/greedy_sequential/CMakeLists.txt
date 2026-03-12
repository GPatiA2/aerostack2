add_library(greedy_sequential_plugin SHARED
  src/greedy_sequential.cpp
)

target_include_directories(greedy_sequential_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(greedy_sequential_plugin ${PROJECT_DEPENDENCIES})
target_link_libraries(greedy_sequential_plugin ${PROJECT_NAME}_plugin_base)

install(
  TARGETS greedy_sequential_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/plugins/greedy_sequential/config
)
