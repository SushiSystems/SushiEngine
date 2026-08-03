# native_execution — the native-backend counterpart to sushiengine_functional_tests' Execution
# coverage. Small and separate because it must build with NO SYCL toolchain and NO SushiRuntime
# subproject, which sushiengine_functional_tests (sushiengine_simulation,
# sushiengine_serialization, sushiengine_authoring, sushiengine_gltf) is nowhere close to. It
# runs the exact same vocabulary and DynamicGraph conformance cases as the runtime backend,
# which is the point.

cmake_minimum_required(VERSION 3.22) # gtest_discover_tests(TEST_FILTER ...)

include(GoogleTest)
find_package(GTest CONFIG REQUIRED)

# The three translation units are compiled out of the sibling directories on purpose rather
# than copied: one set of conformance cases, built twice, is the only arrangement in which the
# two execution backends cannot silently drift onto different cases.
add_executable(sushiengine_native_execution_tests
    ../common/test_main.cpp
    ../unit/test_hazard_semantics.cpp
    ../integration/test_execution_dynamic_graph.cpp)
target_include_directories(sushiengine_native_execution_tests
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../common")
target_link_libraries(sushiengine_native_execution_tests PRIVATE SushiEngine GTest::gtest)
# The target carries the repository's prefix so it cannot collide with a SushiRuntime one in the
# single flat CMake namespace the two trees share; the file on disk keeps the short name.
set_target_properties(sushiengine_native_execution_tests PROPERTIES
    OUTPUT_NAME se_native_execution_tests
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

# The same label partition tests/CMakeLists.txt applies, derived the same way from the
# GTest suite-name prefix, so `ctest -L` selects a sub-suite on this lane too and
# `se test --backend native` needs no lane-specific selection rule.
gtest_discover_tests(sushiengine_native_execution_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Unit_*"
    PROPERTIES LABELS "unit")
gtest_discover_tests(sushiengine_native_execution_tests
    DISCOVERY_MODE PRE_TEST
    TEST_FILTER "Integration_*"
    PROPERTIES LABELS "integration")
