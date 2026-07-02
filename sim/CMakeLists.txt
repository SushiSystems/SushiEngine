# ============================================================================
# SushiEngine simulation library
#
# sushi_sim is the one place device code lives outside an example: it owns a
# SushiRuntime, an ECS World, and a Schedule, and exposes them behind the plain
# C++ ISimulation seam (include/SushiEngine/sim/simulation.hpp). runtime_simulation.cpp
# is a SYCL translation unit, so the shared SYCL policy applies to it; consumers
# (the editor) link this static library and see only the abstraction.
#
# Because the library carries SYCL device images and links the runtime, its
# consumer's final link must be SYCL-aware — the -fsycl link flag and the include
# path are propagated PUBLIC so a plain add_executable that links sushi_sim links
# correctly without repeating the policy.
# ============================================================================

add_library(sushi_sim STATIC runtime_simulation.cpp)
target_link_libraries(sushi_sim PUBLIC SushiEngine)
target_include_directories(sushi_sim PUBLIC "${CMAKE_SOURCE_DIR}/include")
set_target_properties(sushi_sim PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON)

sushi_apply_sycl(sushi_sim SOURCES runtime_simulation.cpp)

# add_sycl_to_target / sushi_apply_sycl add -fsycl PRIVATE, but a static library's
# device images are finalized at the consuming executable's link, so the flag must
# reach that link too. Propagate it PUBLIC on the intel-llvm path.
if(SR_SYCL_TOOLCHAIN STREQUAL "intel-llvm")
    target_link_options(sushi_sim PUBLIC -fsycl)
endif()
