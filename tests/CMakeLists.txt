# ============================================================================
# SushiEngine test pipeline
#
# One binary, se_functional_tests, covering functional/{unit,integration,
# regression}. The engine is header-only and ships no device code of its own, so
# the test binary is a single SYCL translation-unit set: every test that drives
# the ECS schedule or the PGS solver instantiates its kernels here and runs them
# against the real runtime — there are no mocks (see CLAUDE.md).
#
# Tests are grouped into CTest labels (unit / integration / regression) from the
# GTest suite-name prefix convention (Unit_* / Integration_* / Regression_*), so
# `ctest -L unit` (or the umbrella regex the CLI uses) selects a sub-suite and new
# tests are picked up automatically as long as they follow the convention.
# ============================================================================

cmake_minimum_required(VERSION 3.22) # gtest_discover_tests(TEST_FILTER ...)

include(GoogleTest)

set(SE_TEST_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")

# Applies the cross-cutting settings to a test binary: the shared-helpers include
# dir, the engine + GTest links, and the shared SYCL policy (device flags + the
# Windows runtime-DLL copy) from cmake/SyclTarget.cmake — the same helper the
# engine's example apps use, so the two lanes cannot drift apart.
function(sushi_configure_test_target target)
    target_include_directories(${target} PRIVATE ${SE_TEST_COMMON_DIR})
    target_link_libraries(${target} PRIVATE SushiEngine GTest::gtest)
    set_target_properties(${target} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
    sushi_apply_sycl(${target})
endfunction()

add_executable(se_functional_tests
    common/test_main.cpp

    functional/unit/test_world.cpp
    functional/unit/test_command_buffer.cpp
    functional/unit/test_graph_coloring.cpp
    functional/unit/test_floating_origin.cpp
    functional/unit/test_floating_origin_stress.cpp
    functional/unit/test_network_id.cpp

    functional/integration/test_schedule.cpp
    functional/integration/test_pgs_solver.cpp
    functional/integration/test_xpbd_solver.cpp
    functional/integration/test_physics_world.cpp
    functional/integration/test_physics_bridge.cpp
    functional/integration/test_rollback.cpp
    functional/integration/test_deterministic_replay.cpp
    functional/integration/test_net_reconciliation.cpp
    functional/integration/test_net_client_server.cpp
    functional/integration/test_cloth.cpp

    functional/regression/test_regression.cpp
)
sushi_configure_test_target(se_functional_tests)

# One label per group; the "functional" umbrella is expressed by the consumer as
# a label regex (`ctest -L 'unit|integration|regression'`) rather than a second
# baked-in label, which does not survive gtest_discover_tests on current CMake.
gtest_discover_tests(se_functional_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Unit_*"
    PROPERTIES LABELS "unit")
gtest_discover_tests(se_functional_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Integration_*"
    PROPERTIES LABELS "integration")
gtest_discover_tests(se_functional_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Regression_*"
    PROPERTIES LABELS "regression")
