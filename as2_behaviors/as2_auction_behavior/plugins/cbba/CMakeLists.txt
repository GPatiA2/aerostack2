add_library(cbba_plugin SHARED
  src/cbba.cpp
)

target_include_directories(cbba_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cbba_plugin ${PROJECT_DEPENDENCIES})
target_link_libraries(cbba_plugin ${PROJECT_NAME}_plugin_base)

install(
  TARGETS cbba_plugin
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
  DESTINATION share/${PROJECT_NAME}/plugins/cbba/config
)

# ── unit tests ────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB CBBA_TEST_SOURCES "tests/*_gtest.cpp")
  foreach(TEST_SOURCE ${CBBA_TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    ament_add_gtest(${TEST_NAME} ${TEST_SOURCE})
    ament_target_dependencies(${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${TEST_NAME}
      gtest_main
      ${PROJECT_NAME}_plugin_base
      cbba_plugin
    )
    target_include_directories(${TEST_NAME} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endforeach()
endif()
