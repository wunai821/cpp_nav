cmake_minimum_required(VERSION 3.16)
project(rm_sentinel_nav VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =========================
# Build output layout
# All generated artifacts must stay under build/
# =========================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

foreach(OUTPUTCONFIG DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG_UPPER)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_BINARY_DIR}/bin)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_BINARY_DIR}/lib)
endforeach()

# =========================
# Optional Unitree lidar SDK
# =========================
if(DEFINED ENV{UNITREE_LIDAR_SDK_ROOT})
  set(_RM_NAV_UNITREE_LIDAR_SDK_DEFAULT "$ENV{UNITREE_LIDAR_SDK_ROOT}")
else()
  set(_RM_NAV_UNITREE_LIDAR_SDK_DEFAULT "")
endif()

set(UNITREE_LIDAR_SDK_ROOT "${_RM_NAV_UNITREE_LIDAR_SDK_DEFAULT}" CACHE PATH
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
if(UNITREE_LIDAR_SDK_ARCH
   AND EXISTS "${UNITREE_LIDAR_SDK_HEADER}"
   AND EXISTS "${UNITREE_LIDAR_SDK_LIBRARY}")
  add_library(unitree_lidar_sdk STATIC IMPORTED)
  set_target_properties(unitree_lidar_sdk PROPERTIES
    IMPORTED_LOCATION "${UNITREE_LIDAR_SDK_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${UNITREE_LIDAR_SDK_ROOT}/include"
  )
  set(RM_NAV_HAS_UNITREE_LIDAR_SDK TRUE)
endif()

# =========================
# Core library
# =========================
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
  src/localization/relocalization_manager.cpp
  src/fsm/nav_fsm.cpp
  src/fsm/mode_recovery.cpp
  src/mapping/mapping_engine.cpp
  src/mapping/lio_frontend.cpp
  src/mapping/eskf_lio.cpp
  src/mapping/map_builder_3d.cpp
  src/mapping/loop_candidate_detector.cpp
  src/mapping/loop_closure_matcher.cpp
  src/mapping/loop_correction_gate.cpp
  src/mapping/map_projector_2d.cpp
  src/mapping/map_save_manager.cpp
  src/mapping/map_serializer.cpp
  src/mapping/map_validator.cpp
  src/mapping/voxel_map.cpp
  src/mapping/waypoint_manager.cpp
  src/perception/range_crop_filter.cpp
  src/perception/ground_filter.cpp
  src/perception/voxel_filter.cpp
  src/perception/preprocess_pipeline.cpp
  src/perception/obstacle_layer.cpp
  src/perception/inflation_layer.cpp
  src/perception/dynamic_layer.cpp
  src/perception/local_costmap_builder.cpp
  src/perception/euclidean_cluster.cpp
  src/perception/tracker_kf.cpp
  src/perception/mot_manager.cpp
  src/planning/center_hold_controller.cpp
  src/planning/planner_coordinator.cpp
  src/planning/goal_manager.cpp
  src/planning/global_astar.cpp
  src/planning/mission_manager.cpp
  src/planning/omni_dwa.cpp
  src/planning/recovery_planner.cpp
  src/safety/collision_checker.cpp
  src/safety/command_gate.cpp
  src/safety/deadman_checker.cpp
  src/safety/failover_policy.cpp
  src/safety/safety_manager.cpp
  src/safety/watchdog_heartbeat.cpp
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

# =========================
# Helper functions
# =========================
function(rm_nav_add_tool TARGET_NAME)
  add_executable(${TARGET_NAME} ${ARGN})
  target_link_libraries(${TARGET_NAME} PRIVATE rm_nav_core)
endfunction()

function(rm_nav_add_unit_test TARGET_NAME SOURCE_FILE)
  add_executable(${TARGET_NAME} ${SOURCE_FILE})
  target_link_libraries(${TARGET_NAME} PRIVATE rm_nav_core)
  add_test(NAME ${TARGET_NAME} COMMAND $<TARGET_FILE:${TARGET_NAME}>)
  set_tests_properties(${TARGET_NAME} PROPERTIES
    WORKING_DIRECTORY ${RM_NAV_CTEST_WORKING_DIRECTORY}
  )
endfunction()

function(rm_nav_add_integration_test TARGET_NAME SOURCE_FILE)
  add_executable(${TARGET_NAME} ${SOURCE_FILE})
  target_link_libraries(${TARGET_NAME} PRIVATE rm_nav_core)
  add_test(NAME ${TARGET_NAME} COMMAND $<TARGET_FILE:${TARGET_NAME}>)
  set_tests_properties(${TARGET_NAME} PROPERTIES
    WORKING_DIRECTORY ${RM_NAV_CTEST_WORKING_DIRECTORY}
  )
endfunction()

# =========================
# Main executable
# =========================
add_executable(rm_nav_main
  src/app/main.cpp
)

target_link_libraries(rm_nav_main
  PRIVATE
    rm_nav_core
)

set_target_properties(rm_nav_main PROPERTIES
  OUTPUT_NAME rm_nav_main
)

# =========================
# Tools / driver checks
# NOTE:
# These binaries are built into build/bin, never repo root.
# Long-term recommendation:
# - move true test programs into tests/
# - keep developer utilities in tools/
# =========================
rm_nav_add_tool(stm32_bridge_test tools/stm32_bridge_test.cpp)
rm_nav_add_tool(l1_driver_test tools/l1_driver_test.cpp)
rm_nav_add_tool(l1_record_tool tools/l1_record_tool.cpp)
rm_nav_add_tool(imu_driver_test tools/imu_driver_test.cpp)
rm_nav_add_tool(imu_record_tool tools/imu_record_tool.cpp)
rm_nav_add_tool(replay_player tools/replay_player.cpp)
rm_nav_add_tool(deskew_viewer tools/deskew_viewer.cpp)
rm_nav_add_tool(mock_planner_generator tools/mock_planner_generator.cpp)
rm_nav_add_tool(warmup_mapping_tool tools/warmup_mapping_tool.cpp)

# =========================
# Tests
# =========================
enable_testing()

# Keep tests running relative to source root so config/ and test assets resolve.
set(RM_NAV_CTEST_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# Unit tests
rm_nav_add_unit_test(test_ring_queue tests/unit/test_ring_queue.cpp)
rm_nav_add_unit_test(test_object_pool tests/unit/test_object_pool.cpp)
rm_nav_add_unit_test(test_double_buffer tests/unit/test_double_buffer.cpp)
rm_nav_add_unit_test(test_tf_tree_lite tests/unit/test_tf_tree_lite.cpp)
rm_nav_add_unit_test(test_sensor_sync tests/unit/test_sensor_sync.cpp)
rm_nav_add_unit_test(test_preprocess_pipeline tests/unit/test_preprocess_pipeline.cpp)
rm_nav_add_unit_test(test_map_loader tests/unit/test_map_loader.cpp)
rm_nav_add_unit_test(test_global_astar tests/unit/test_global_astar.cpp)
rm_nav_add_unit_test(test_preintegration tests/unit/test_preintegration.cpp)
rm_nav_add_unit_test(test_lio_frontend tests/unit/test_lio_frontend.cpp)
rm_nav_add_unit_test(test_protocol_codec tests/unit/test_protocol_codec.cpp)
rm_nav_add_unit_test(test_localization_loop tests/unit/test_localization_loop.cpp)
rm_nav_add_unit_test(test_ndt_icp tests/unit/test_ndt_icp.cpp)
rm_nav_add_unit_test(test_localization_recovery tests/unit/test_localization_recovery.cpp)
rm_nav_add_unit_test(test_relocalization_manager tests/unit/test_relocalization_manager.cpp)
rm_nav_add_unit_test(test_center_hold tests/unit/test_center_hold.cpp)
rm_nav_add_unit_test(test_collision_checker tests/unit/test_collision_checker.cpp)
rm_nav_add_unit_test(test_command_gate tests/unit/test_command_gate.cpp)
rm_nav_add_unit_test(test_mission_manager tests/unit/test_mission_manager.cpp)
rm_nav_add_unit_test(test_recovery_planner tests/unit/test_recovery_planner.cpp)
rm_nav_add_unit_test(test_local_costmap_builder tests/unit/test_local_costmap_builder.cpp)
rm_nav_add_unit_test(test_mot_manager tests/unit/test_mot_manager.cpp)
rm_nav_add_unit_test(test_planner_approach tests/unit/test_planner_approach.cpp)
rm_nav_add_unit_test(test_dwa_dynamic_risk tests/unit/test_dwa_dynamic_risk.cpp)
rm_nav_add_unit_test(test_omni_dwa tests/unit/test_omni_dwa.cpp)
rm_nav_add_unit_test(test_waypoint_manager tests/unit/test_waypoint_manager.cpp)
rm_nav_add_unit_test(test_mapping_keyframe_cache tests/unit/test_mapping_keyframe_cache.cpp)
rm_nav_add_unit_test(test_map_builder_dynamic_suppression tests/unit/test_map_builder_dynamic_suppression.cpp)
rm_nav_add_unit_test(test_dynamic_suppression_validation tests/unit/test_dynamic_suppression_validation.cpp)
rm_nav_add_unit_test(test_map_projector_probabilistic tests/unit/test_map_projector_probabilistic.cpp)
rm_nav_add_unit_test(test_occupancy_projection tests/unit/test_occupancy_projection.cpp)
rm_nav_add_unit_test(test_loop_candidate_detector tests/unit/test_loop_candidate_detector.cpp)
rm_nav_add_unit_test(test_loop_closure_matcher tests/unit/test_loop_closure_matcher.cpp)
rm_nav_add_unit_test(test_loop_correction_gate tests/unit/test_loop_correction_gate.cpp)
rm_nav_add_unit_test(test_mapping_loop_match tests/unit/test_mapping_loop_match.cpp)
rm_nav_add_unit_test(test_mapping_pose_correction tests/unit/test_mapping_pose_correction.cpp)
rm_nav_add_unit_test(test_mapping_frontend_scan_to_scan tests/unit/test_mapping_frontend_scan_to_scan.cpp)
rm_nav_add_unit_test(test_mapping_frontend_scan_to_map tests/unit/test_mapping_frontend_scan_to_map.cpp)
rm_nav_add_unit_test(test_map_validator_trajectory tests/unit/test_map_validator_trajectory.cpp)
rm_nav_add_unit_test(test_nav_fsm tests/unit/test_nav_fsm.cpp)
rm_nav_add_unit_test(test_safety_manager tests/unit/test_safety_manager.cpp)

# Integration tests
rm_nav_add_integration_test(test_pipeline_flow tests/integration/test_pipeline_flow.cpp)
rm_nav_add_integration_test(test_planner_pipeline tests/integration/test_planner_pipeline.cpp)
rm_nav_add_integration_test(test_mapping_pipeline tests/integration/test_mapping_pipeline.cpp)
rm_nav_add_integration_test(test_runtime_mapping_mode tests/integration/test_runtime_mapping_mode.cpp)
rm_nav_add_integration_test(test_localization_pipeline tests/integration/test_localization_pipeline.cpp)
rm_nav_add_integration_test(test_full_runtime tests/integration/test_full_runtime.cpp)
rm_nav_add_integration_test(test_foxglove_server tests/integration/test_foxglove_server.cpp)

# Tool-driven test
add_test(NAME test_mock_planner_generator
  COMMAND $<TARGET_FILE:mock_planner_generator>
          --config config
          --scenario-file config/mock_scenario.yaml
          --no-foxglove
          --duration-s 1
          --hz 10
          --output-dir logs/mock_planner_ctest
)

# Tests that must run from repo root
set_tests_properties(
  test_map_loader
  test_global_astar
  test_localization_loop
  test_protocol_codec
  test_ndt_icp
  test_localization_recovery
  test_relocalization_manager
  test_center_hold
  test_collision_checker
  test_command_gate
  test_mission_manager
  test_preprocess_pipeline
  test_lio_frontend
  test_local_costmap_builder
  test_mot_manager
  test_planner_approach
  test_dwa_dynamic_risk
  test_omni_dwa
  test_waypoint_manager
  test_mapping_keyframe_cache
  test_loop_candidate_detector
  test_loop_closure_matcher
  test_loop_correction_gate
  test_mapping_loop_match
  test_mapping_pose_correction
  test_mapping_frontend_scan_to_scan
  test_mapping_frontend_scan_to_map
  test_nav_fsm
  test_pipeline_flow
  test_planner_pipeline
  test_mapping_pipeline
  test_runtime_mapping_mode
  test_localization_pipeline
  test_full_runtime
  test_foxglove_server
  test_mock_planner_generator
  PROPERTIES
    WORKING_DIRECTORY ${RM_NAV_CTEST_WORKING_DIRECTORY}
)
