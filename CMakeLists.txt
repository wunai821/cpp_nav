cmake_minimum_required(VERSION 3.16)
project(rm_sentinel_nav VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(UNITREE_LIDAR_SDK_ROOT "/home/naiwu/unitree_lidar_sdk" CACHE PATH
    "Path to the Unitree lidar SDK root directory")
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
  set(UNITREE_LIDAR_SDK_ARCH "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(UNITREE_LIDAR_SDK_ARCH "aarch64")
else()
  set(UNITREE_LIDAR_SDK_ARCH "")
endif()

set(UNITREE_LIDAR_SDK_HEADER "${UNITREE_LIDAR_SDK_ROOT}/include/unitree_lidar_sdk.h")
set(UNITREE_LIDAR_SDK_LIBRARY
    "${UNITREE_LIDAR_SDK_ROOT}/lib/${UNITREE_LIDAR_SDK_ARCH}/libunitree_lidar_sdk.a")
set(RM_NAV_HAS_UNITREE_LIDAR_SDK FALSE)
if(UNITREE_LIDAR_SDK_ARCH AND EXISTS "${UNITREE_LIDAR_SDK_HEADER}" AND EXISTS "${UNITREE_LIDAR_SDK_LIBRARY}")
  add_library(unitree_lidar_sdk STATIC IMPORTED)
  set_target_properties(unitree_lidar_sdk PROPERTIES
    IMPORTED_LOCATION "${UNITREE_LIDAR_SDK_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${UNITREE_LIDAR_SDK_ROOT}/include"
  )
  set(RM_NAV_HAS_UNITREE_LIDAR_SDK TRUE)
endif()

add_library(rm_nav_core
  src/app/app.cpp
  src/app/bootstrap.cpp
  src/app/runtime.cpp
  src/common/time.cpp
  src/config/config_loader.cpp
  src/drivers/serial/serial_port.cpp
  src/drivers/stm32/stm32_bridge.cpp
  src/drivers/lidar/l1_packet_parser.cpp
  src/drivers/lidar/l1_decoder.cpp
  src/drivers/lidar/l1_driver.cpp
  src/drivers/imu/imu_decoder.cpp
  src/drivers/imu/imu_driver.cpp
  src/protocol/crc8.cpp
  src/protocol/crc16.cpp
  src/protocol/protocol_codec.cpp
  src/sync/deskew.cpp
  src/sync/imu_preintegrator.cpp
  src/sync/sensor_sync_buffer.cpp
  src/tf/static_tf_registry.cpp
  src/tf/timed_pose_buffer.cpp
  src/tf/transform_query.cpp
  src/tf/tf_tree_lite.cpp
  src/localization/icp_matcher.cpp
  src/localization/initial_pose_provider.cpp
  src/localization/localization_engine.cpp
  src/localization/map_loader.cpp
  src/localization/map_odom_fuser.cpp
  src/localization/ndt_matcher.cpp
  src/localization/pose_quality_estimator.cpp
  src/fsm/nav_fsm.cpp
  src/mapping/mapping_engine.cpp
  src/mapping/map_builder_3d.cpp
  src/mapping/map_projector_2d.cpp
  src/mapping/map_serializer.cpp
  src/mapping/voxel_map.cpp
  src/mapping/waypoint_manager.cpp
  src/perception/preprocess_pipeline.cpp
  src/perception/local_costmap_builder.cpp
  src/perception/mot_manager.cpp
  src/planning/planner_coordinator.cpp
  src/planning/goal_manager.cpp
  src/planning/global_astar.cpp
  src/planning/omni_dwa.cpp
  src/utils/file_io.cpp
  src/utils/logger.cpp
  src/debug/foxglove_server.cpp
)

target_include_directories(rm_nav_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(rm_nav_core PUBLIC cxx_std_17)
target_compile_definitions(rm_nav_core
  PUBLIC
    RM_NAV_VERSION="${PROJECT_VERSION}"
)
if(RM_NAV_HAS_UNITREE_LIDAR_SDK)
  target_link_libraries(rm_nav_core PRIVATE unitree_lidar_sdk)
  target_compile_definitions(rm_nav_core PUBLIC RM_NAV_HAS_UNITREE_LIDAR_SDK=1)
endif()

add_executable(rm_nav_main
  src/app/main.cpp
)

target_link_libraries(rm_nav_main
  PRIVATE
    rm_nav_core
)

set_target_properties(rm_nav_main PROPERTIES
  OUTPUT_NAME rm_nav_main
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(stm32_bridge_test
  tools/stm32_bridge_test.cpp
)
target_link_libraries(stm32_bridge_test PRIVATE rm_nav_core)
set_target_properties(stm32_bridge_test PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(l1_driver_test
  tools/l1_driver_test.cpp
)
target_link_libraries(l1_driver_test PRIVATE rm_nav_core)
set_target_properties(l1_driver_test PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(l1_record_tool
  tools/l1_record_tool.cpp
)
target_link_libraries(l1_record_tool PRIVATE rm_nav_core)
set_target_properties(l1_record_tool PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(imu_driver_test
  tools/imu_driver_test.cpp
)
target_link_libraries(imu_driver_test PRIVATE rm_nav_core)
set_target_properties(imu_driver_test PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(imu_record_tool
  tools/imu_record_tool.cpp
)
target_link_libraries(imu_record_tool PRIVATE rm_nav_core)
set_target_properties(imu_record_tool PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(replay_player
  tools/replay_player.cpp
)
target_link_libraries(replay_player PRIVATE rm_nav_core)
set_target_properties(replay_player PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(deskew_viewer
  tools/deskew_viewer.cpp
)
target_link_libraries(deskew_viewer PRIVATE rm_nav_core)
set_target_properties(deskew_viewer PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(mock_planner_generator
  tools/mock_planner_generator.cpp
)
target_link_libraries(mock_planner_generator PRIVATE rm_nav_core)
set_target_properties(mock_planner_generator PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(warmup_mapping_tool
  tools/warmup_mapping_tool.cpp
)
target_link_libraries(warmup_mapping_tool PRIVATE rm_nav_core)
set_target_properties(warmup_mapping_tool PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

enable_testing()

add_executable(test_ring_queue
  tests/unit/test_ring_queue.cpp
)
target_link_libraries(test_ring_queue PRIVATE rm_nav_core)

add_executable(test_object_pool
  tests/unit/test_object_pool.cpp
)
target_link_libraries(test_object_pool PRIVATE rm_nav_core)

add_executable(test_double_buffer
  tests/unit/test_double_buffer.cpp
)
target_link_libraries(test_double_buffer PRIVATE rm_nav_core)

add_executable(test_tf_tree_lite
  tests/unit/test_tf_tree_lite.cpp
)
target_link_libraries(test_tf_tree_lite PRIVATE rm_nav_core)

add_executable(test_sensor_sync
  tests/unit/test_sensor_sync.cpp
)
target_link_libraries(test_sensor_sync PRIVATE rm_nav_core)

add_executable(test_preprocess_pipeline
  tests/unit/test_preprocess_pipeline.cpp
)
target_link_libraries(test_preprocess_pipeline PRIVATE rm_nav_core)

add_executable(test_map_loader
  tests/unit/test_map_loader.cpp
)
target_link_libraries(test_map_loader PRIVATE rm_nav_core)

add_executable(test_global_astar
  tests/unit/test_global_astar.cpp
)
target_link_libraries(test_global_astar PRIVATE rm_nav_core)

add_executable(test_preintegration
  tests/unit/test_preintegration.cpp
)
target_link_libraries(test_preintegration PRIVATE rm_nav_core)

add_executable(test_localization_loop
  tests/unit/test_localization_loop.cpp
)
target_link_libraries(test_localization_loop PRIVATE rm_nav_core)

add_executable(test_center_hold
  tests/unit/test_center_hold.cpp
)
target_link_libraries(test_center_hold PRIVATE rm_nav_core)

add_executable(test_mot_manager
  tests/unit/test_mot_manager.cpp
)
target_link_libraries(test_mot_manager PRIVATE rm_nav_core)

add_executable(test_planner_approach
  tests/unit/test_planner_approach.cpp
)
target_link_libraries(test_planner_approach PRIVATE rm_nav_core)

add_executable(test_dwa_dynamic_risk
  tests/unit/test_dwa_dynamic_risk.cpp
)
target_link_libraries(test_dwa_dynamic_risk PRIVATE rm_nav_core)

add_executable(test_waypoint_manager
  tests/unit/test_waypoint_manager.cpp
)
target_link_libraries(test_waypoint_manager PRIVATE rm_nav_core)

add_executable(test_nav_fsm
  tests/unit/test_nav_fsm.cpp
)
target_link_libraries(test_nav_fsm PRIVATE rm_nav_core)

add_test(NAME test_ring_queue COMMAND test_ring_queue)
add_test(NAME test_object_pool COMMAND test_object_pool)
add_test(NAME test_double_buffer COMMAND test_double_buffer)
add_test(NAME test_tf_tree_lite COMMAND test_tf_tree_lite)
add_test(NAME test_sensor_sync COMMAND test_sensor_sync)
add_test(NAME test_preprocess_pipeline COMMAND test_preprocess_pipeline)
add_test(NAME test_map_loader COMMAND test_map_loader)
add_test(NAME test_global_astar COMMAND test_global_astar)
add_test(NAME test_preintegration COMMAND test_preintegration)
add_test(NAME test_localization_loop COMMAND test_localization_loop)
add_test(NAME test_center_hold COMMAND test_center_hold)
add_test(NAME test_mot_manager COMMAND test_mot_manager)
add_test(NAME test_planner_approach COMMAND test_planner_approach)
add_test(NAME test_dwa_dynamic_risk COMMAND test_dwa_dynamic_risk)
add_test(NAME test_waypoint_manager COMMAND test_waypoint_manager)
add_test(NAME test_nav_fsm COMMAND test_nav_fsm)

add_executable(test_pipeline_flow
  tests/integration/test_pipeline_flow.cpp
)
target_link_libraries(test_pipeline_flow PRIVATE rm_nav_core)

add_test(NAME test_pipeline_flow COMMAND test_pipeline_flow)

add_executable(test_planner_pipeline
  tests/integration/test_planner_pipeline.cpp
)
target_link_libraries(test_planner_pipeline PRIVATE rm_nav_core)

add_test(NAME test_planner_pipeline COMMAND test_planner_pipeline)

add_executable(test_mapping_pipeline
  tests/integration/test_mapping_pipeline.cpp
)
target_link_libraries(test_mapping_pipeline PRIVATE rm_nav_core)

add_test(NAME test_mapping_pipeline COMMAND test_mapping_pipeline)

add_executable(test_runtime_mapping_mode
  tests/integration/test_runtime_mapping_mode.cpp
)
target_link_libraries(test_runtime_mapping_mode PRIVATE rm_nav_core)

add_test(NAME test_runtime_mapping_mode COMMAND test_runtime_mapping_mode)

add_executable(test_foxglove_server
  tests/integration/test_foxglove_server.cpp
)
target_link_libraries(test_foxglove_server PRIVATE rm_nav_core)
add_test(NAME test_mock_planner_generator
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/mock_planner_generator
          --config config
          --scenario-file config/mock_scenario.yaml
          --no-foxglove
          --duration-s 1
          --hz 10
          --output-dir logs/mock_planner_ctest
)

set_tests_properties(
  test_map_loader
  test_global_astar
  test_localization_loop
  test_center_hold
  test_preprocess_pipeline
  test_mot_manager
  test_planner_approach
  test_dwa_dynamic_risk
  test_waypoint_manager
  test_nav_fsm
  test_pipeline_flow
  test_planner_pipeline
  test_mapping_pipeline
  test_runtime_mapping_mode
  test_mock_planner_generator
  PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
