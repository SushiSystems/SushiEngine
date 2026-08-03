# simulation — the live world behind the ISimulation seam: it owns a SushiRuntime, an ECS World
# and a Schedule, and hands all three out only as plain C++. source/runtime_simulation.cpp is the
# one device translation unit in the engine outside an example, which is why the SYCL policy is
# applied here and a consumer sees nothing of it but a SYCL-aware final link.

sushiengine_add_module(NAME simulation LAYER world
    SOURCES source/runtime_simulation.cpp
    # Public because each is spelled in a header this target publishes: components.hpp carries the
    # ECS and the core value types, physics_services.hpp the bodies and joints, weather_types.hpp
    # the atmospheric column, audio_extract.hpp the acoustic scene, and simulation.hpp the
    # animation, astronomy, environment and particle vocabulary a frame is described in.
    # ui is here for the same reason: simulation.hpp hands out a Render::SceneView, whose header
    # carries a UI draw list, so a consumer of this target resolves that header through it.
    PUBLIC_DEPENDS animation astro atmosphere audio core ecs environment execution physics ui vfx
    # The crowd skeletons and clips runtime_simulation.cpp imports from glTF, and the clock it
    # steps on; no header here names either.
    PRIVATE_DEPENDS gltf loop
    # sushi_cooking is public because physics_simulation.hpp builds a soft body or a vehicle
    # straight from a `.sushisoft`/`.sushinodebeam` blob and the loaders for both are compiled
    # there. SushiEngine is private: runtime_simulation.cpp includes the umbrella header to
    # assemble a whole world, but nothing this target publishes names it.
    EXTERNAL_PUBLIC sushi_cooking
    EXTERNAL_PRIVATE SushiEngine)

# simulation.hpp hands the renderer a Render::SceneView, so every consumer resolves the renderer's
# published headers through this target. The include root rather than the module: nothing here
# calls into the renderer, and a configure with SE_BUILD_RENDER off has no library to link.
target_include_directories(sushiengine_simulation
    PUBLIC "${CMAKE_SOURCE_DIR}/engine/presentation/render/include")

# Position-independent because the shells that link it are, and SYCL-compiled because its one
# source is device code; sushiengine_add_module() states neither.
set_target_properties(sushiengine_simulation PROPERTIES POSITION_INDEPENDENT_CODE ON)
sushi_apply_sycl(sushiengine_simulation SOURCES source/runtime_simulation.cpp)

# sushi_apply_sycl adds -fsycl PRIVATE, but a static library's device images are finalized at the
# consuming executable's link, so the flag has to reach that link too. Propagating it PUBLIC is
# what lets a plain add_executable link this target without repeating the policy.
if(SR_SYCL_TOOLCHAIN STREQUAL "intel-llvm")
    target_link_options(sushiengine_simulation PUBLIC -fsycl)
endif()

# The name the editor, the player, the sample-scene generator and the test suite link.
add_library(sushi_sim ALIAS sushiengine_simulation)
