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

# ── unit tests ────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB GREEDY_TEST_SOURCES "tests/*_gtest.cpp")
  foreach(TEST_SOURCE ${GREEDY_TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    ament_add_gtest(${TEST_NAME} ${TEST_SOURCE})
    ament_target_dependencies(${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${TEST_NAME}
      gtest_main
      ${PROJECT_NAME}_plugin_base
      greedy_sequential_plugin
    )
    target_include_directories(${TEST_NAME} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endforeach()
endif()
