# GTest
file(GLOB GTEST_SOURCE "*_gtest.cpp")

if(GTEST_SOURCE)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(rclcpp REQUIRED)

  foreach(TEST_SOURCE ${GTEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    ament_add_gtest(${PROJECT_NAME}_${TEST_NAME} ${TEST_SOURCE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME}
      ament_index_cpp::ament_index_cpp
      gtest_main
      ${PROJECT_NAME}_plugin_base
      greedy_sequential_plugin
      coordinate_item_plugin
    )
    target_include_directories(${PROJECT_NAME}_${TEST_NAME} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../plugins/greedy_sequential/include
      ${CMAKE_CURRENT_SOURCE_DIR}/../plugins/coordinate_item/include
    )
  endforeach()
endif()
