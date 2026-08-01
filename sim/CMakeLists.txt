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
# sushi_atmosphere is PUBLIC because the global dynamical core is not an
# implementation detail of this library: ProceduralWeather holds one by value in a
# header (sim/weather_provider.hpp), so every consumer of sushi_sim compiles that
# type and must link its definitions.
# sushi_cooking is PUBLIC for the same reason: `physics_simulation.hpp` loads a
# `.sushisoft` and a `.sushinodebeam` blob to build a soft body or a vehicle, and
# `load_soft_body_blob`/`load_node_beam_blob` are compiled there rather than inline.
target_link_libraries(sushi_sim PUBLIC SushiEngine sushi_atmosphere sushi_cooking)
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
