# probes — headless measuring instruments, run on purpose rather than by ctest. Each brings a
# subsystem up on its own and reports a number or a comparison; none of them asserts, because
# what they measure depends on the GPU and driver underneath and a suite that goes red on the
# hardware it happens to run on is a suite people learn to ignore.

# The global dynamical core's acceptance is a measurement, not a screenshot: baroclinic
# instability either grows out of the mean state at the textbook rate or it does not, and that is
# settled headlessly over simulated weeks.
add_executable(sushiengine_atmosphere_global_probe atmosphere_global/main.cpp)
# It reads a baked climatology through the simulation's asset header. The include root rather
# than the module: that header is neutral C++ and the module itself is a SYCL library the probe
# has no reason to link, and no reason to require a configure to have built.
target_include_directories(sushiengine_atmosphere_global_probe
    PRIVATE "${CMAKE_SOURCE_DIR}/engine/world/simulation/include")
target_link_libraries(sushiengine_atmosphere_global_probe PRIVATE sushiengine_atmosphere)
# Every probe target carries the repository's prefix so none can collide with a SushiRuntime one
# in the single flat CMake namespace the two trees share; each file on disk keeps the short name
# the probe is documented and invoked by.
set_target_properties(sushiengine_atmosphere_global_probe PROPERTIES
    OUTPUT_NAME atmosphere_global_probe
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

if(SUSHIENGINE_BUILD_RENDER)
    # Where the renderer's own headers live. All three probes below construct the Vulkan device
    # themselves and reach past the RHI seam into the caches, the asset library and the regional
    # nest behind it: a probe measures the implementation, not the abstraction over it.
    set(SUSHIENGINE_RENDER_SOURCE_DIR "${CMAKE_SOURCE_DIR}/engine/presentation/render/source")

    # Brings the device up, renders the triangle offscreen and reads the pixels back to prove the
    # pipeline path — the render analogue of the ECS sandbox, and it runs in CI without a display.
    add_executable(sushiengine_render_probe render/main.cpp)
    target_include_directories(sushiengine_render_probe PRIVATE "${SUSHIENGINE_RENDER_SOURCE_DIR}")
    target_link_libraries(sushiengine_render_probe PRIVATE sushiengine_render)
    set_target_properties(sushiengine_render_probe PROPERTIES
        OUTPUT_NAME render_probe
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)

    # RHI0's golden-image harness: renders a fixed scene headlessly and compares the frame, whole
    # and per pass, against a recorded reference. Run before and after a render refactor, which is
    # the only time it can answer the question it exists for; `--goldens` overrides where it reads
    # the references from.
    add_executable(sushiengine_render_golden render_golden/main.cpp)
    target_include_directories(sushiengine_render_golden PRIVATE "${SUSHIENGINE_RENDER_SOURCE_DIR}")
    target_link_libraries(sushiengine_render_golden PRIVATE sushiengine_render)
    target_compile_definitions(sushiengine_render_golden PRIVATE
        SE_GOLDEN_DIR="${CMAKE_SOURCE_DIR}/tests/goldens/render")
    set_target_properties(sushiengine_render_golden PROPERTIES
        OUTPUT_NAME render_golden
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)

    # Steps the regional nest through hours of simulated weather in seconds of wall clock and
    # prints the observer column's vertical profile, taking the editor out of the loop for the
    # questions docs/design/atmosphere_system.md §11's Phase B2c leaves open.
    add_executable(sushiengine_atmosphere_probe atmosphere/main.cpp)
    # The nest is configured from the simulation's settings header, taken by include root for the
    # same reason the global probe takes the climatology one.
    target_include_directories(sushiengine_atmosphere_probe PRIVATE
        "${SUSHIENGINE_RENDER_SOURCE_DIR}"
        "${CMAKE_SOURCE_DIR}/engine/world/simulation/include")
    target_link_libraries(sushiengine_atmosphere_probe PRIVATE sushiengine_render)
    set_target_properties(sushiengine_atmosphere_probe PROPERTIES
        OUTPUT_NAME atmosphere_probe
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
endif()
