# The global dynamical core's acceptance is a measurement, not a screenshot: baroclinic
# instability either grows out of the mean state at the textbook rate or it does not, and that
# is settled headlessly over simulated weeks. Same shape as the regional nest's probe, and for
# the same reason.
add_executable(atmosphere_global_probe probe/global_core_main.cpp)
# It reads a baked climatology through the simulation's asset header, which has no module of
# its own yet and is still reached through the central include tree.
target_include_directories(atmosphere_global_probe PRIVATE "${CMAKE_SOURCE_DIR}/include")
target_link_libraries(atmosphere_global_probe PRIVATE sushi_atmosphere)
set_target_properties(atmosphere_global_probe PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
