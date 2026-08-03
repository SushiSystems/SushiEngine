# ============================================================================
# se_native_execution_tests
#
# The native-backend counterpart to se_functional_tests' Execution coverage —
# small and separate because it must build with NO SYCL toolchain and NO
# SushiRuntime subproject, which se_functional_tests (sushi_sim, the editor
# serializers, sushi_import) is nowhere close to. Runs the exact same
# vocabulary and DynamicGraph conformance cases as the runtime backend, which
# is the point: test_hazard_semantics.cpp and test_execution_dynamic_graph.cpp
# are included verbatim from tests/functional/ rather than copied, so the two
# lanes cannot silently drift onto different cases.
# ============================================================================

cmake_minimum_required(VERSION 3.22) # gtest_discover_tests(TEST_FILTER ...)

include(GoogleTest)
find_package(GTest CONFIG REQUIRED)

add_executable(se_native_execution_tests
    ../common/test_main.cpp
    ../functional/unit/test_hazard_semantics.cpp
    ../functional/integration/test_execution_dynamic_graph.cpp)
target_include_directories(se_native_execution_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../common")
target_link_libraries(se_native_execution_tests PRIVATE SushiEngine GTest::gtest)
set_target_properties(se_native_execution_tests PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

gtest_discover_tests(se_native_execution_tests DISCOVERY_MODE PRE_TEST)
