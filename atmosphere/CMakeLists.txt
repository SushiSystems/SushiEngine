# ============================================================================
# sushi_atmosphere — the engine-neutral atmosphere module
#
# T1, the global dynamical core (docs/slop/atmosphere_system.md §5), and the
# Fourier transform its elliptic inversion is built on. Like sushi_geometry it
# links nothing: no Vulkan, no SDL, no SYCL, no sushiruntime. That is not a
# stylistic preference — a tier that answers "what is the weather a thousand
# kilometres away" is consumed by gameplay, by the regional nest's parent
# forcing, and by a headless probe, and none of those three should have to bring
# up a device to ask.
#
# The regional nest (T2) is *not* here: it is a Vulkan compute service and lives
# beside the renderer, in render/atmosphere. The seam between the two tiers is
# plain data, which is what lets them sit on opposite sides of that line.
#
# The headers are the include/SushiEngine/atmosphere/ tree, reached through the
# SushiEngine INTERFACE target.
# ============================================================================

add_library(sushi_atmosphere STATIC
    climatology.cpp
    fourier_transform.cpp
    quasigeostrophic_core.cpp)

target_include_directories(sushi_atmosphere PUBLIC "${CMAKE_SOURCE_DIR}/include")
# The dynamical core seeds its stochastic forcing from core's random number generator.
target_link_libraries(sushi_atmosphere PUBLIC sushiengine_core)
target_compile_features(sushi_atmosphere PUBLIC cxx_std_17)
set_target_properties(sushi_atmosphere PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON)

# The global core's acceptance is a measurement, not a screenshot: baroclinic
# instability either grows out of the mean state at the textbook rate or it does
# not, and that is settled headlessly over simulated weeks. Same shape as the
# regional nest's probe, and for the same reason.
add_executable(atmosphere_global_probe probe/global_core_main.cpp)
target_link_libraries(atmosphere_global_probe PRIVATE sushi_atmosphere)
set_target_properties(atmosphere_global_probe PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
