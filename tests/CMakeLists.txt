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
# dir, the engine + GTest links, the SYCL flags (the engine's kernels live in the
# consuming TU), and the Windows runtime-DLL copy.
function(sushi_configure_test_target target)
    target_include_directories(${target} PRIVATE ${SE_TEST_COMMON_DIR})
    target_link_libraries(${target} PRIVATE SushiEngine GTest::gtest)
    set_target_properties(${target} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)

    # Mirror add_engine_executable: add_sycl_to_target is a no-op stub on the
    # intel-llvm path, so add -fsycl to the whole target there ourselves.
    add_sycl_to_target(TARGET ${target})
    if(SR_SYCL_TOOLCHAIN STREQUAL "intel-llvm")
        target_compile_options(${target} PRIVATE -fsycl)
        target_link_options(${target} PRIVATE -fsycl)
    endif()

    if(WIN32)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:sushiruntime>
                $<TARGET_FILE_DIR:${target}>)
    endif()
endfunction()

add_executable(se_functional_tests
    common/test_main.cpp

    functional/unit/test_world.cpp
    functional/unit/test_command_buffer.cpp
    functional/unit/test_graph_coloring.cpp

    functional/integration/test_schedule.cpp
    functional/integration/test_pgs_solver.cpp

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
