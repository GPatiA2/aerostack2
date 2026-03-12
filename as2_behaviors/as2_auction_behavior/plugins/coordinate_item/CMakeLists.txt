add_library(coordinate_item_plugin SHARED
  src/coordinate_item.cpp
)

target_include_directories(coordinate_item_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(coordinate_item_plugin ${PROJECT_DEPENDENCIES})
target_link_libraries(coordinate_item_plugin ${PROJECT_NAME}_plugin_base)

install(
  TARGETS coordinate_item_plugin
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
  DESTINATION share/${PROJECT_NAME}/plugins/coordinate_item/config
)
