# editor — a Unity-style authoring shell: a Vulkan-backed SDL2 window hosting Dear ImGui
# (docking branch, vendored at third_party/imgui). It presents through the engine's Vulkan
# renderer (sushiengine_render) rather than OpenGL, so the same device that draws the viewport also
# draws the UI.
#
# OS-agnostic and CMake-clean: SDL2 comes from the platform's package manager (vcpkg on
# Windows, the system on Linux/macOS) via find_package; ImGui is vendored as a submodule and
# compiled here, the idiomatic way to ship it. The editor is NOT a SYCL translation unit, so it
# does not link the runtime directly — the live world arrives behind the plain-C++ simulation
# seam. It requires SUSHIENGINE_BUILD_RENDER for the Vulkan backend (the top-level CMake turns render on
# whenever the editor is on).

if(NOT TARGET sushiengine_render)
    message(FATAL_ERROR
        "The editor needs the Vulkan renderer. Configure with -DSUSHIENGINE_BUILD_RENDER=ON "
        "(the top-level CMake enables it automatically when SUSHIENGINE_BUILD_EDITOR is on).")
endif()

find_package(SDL2 CONFIG REQUIRED)
# The imported Vulkan targets are directory-scoped; the renderer's own find_package does not
# export them here, so the editor resolves them itself.
find_package(VulkanHeaders CONFIG REQUIRED)
find_package(VulkanLoader CONFIG REQUIRED)
# Header-only JSON for the editor's persisted Preferences and the .sushiscene format.
find_package(nlohmann_json CONFIG REQUIRED)
# stb_image for the Project panel's thumbnail decode path. The symbols themselves are already
# compiled once into sushiengine_render (source/material/stb_impl.cpp) and this target already
# links that library, so this only adds the header search path — sushiengine_render keeps its
# own Stb_INCLUDE_DIR private, so the editor must resolve it again itself.
find_package(Stb REQUIRED)

set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/third_party/imgui")
if(NOT EXISTS "${IMGUI_DIR}/imgui.cpp")
    message(FATAL_ERROR
        "Dear ImGui sources not found at ${IMGUI_DIR}. "
        "Initialize submodules: git submodule update --init --recursive")
endif()

# ImGui core plus the SDL2 platform backend and the Vulkan renderer backend. The
# Vulkan backend links the loader for its prototype calls; the color format is fed
# in at init time (dynamic rendering, no render-pass object).
add_library(sushiengine_imgui STATIC
    "${IMGUI_DIR}/imgui.cpp"
    "${IMGUI_DIR}/imgui_draw.cpp"
    "${IMGUI_DIR}/imgui_tables.cpp"
    "${IMGUI_DIR}/imgui_widgets.cpp"
    "${IMGUI_DIR}/imgui_demo.cpp"
    "${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp")
target_include_directories(sushiengine_imgui PUBLIC
    "${IMGUI_DIR}"
    "${IMGUI_DIR}/backends")
target_link_libraries(sushiengine_imgui PUBLIC SDL2::SDL2 Vulkan::Headers Vulkan::Loader)
target_compile_features(sushiengine_imgui PUBLIC cxx_std_17)

add_executable(sushiengine_editor
    source/main.cpp
    source/ui/editor_panels.cpp
    source/ui/profiler_panel.cpp
    source/ui/panel_widgets.cpp
    source/atmosphere/meteorology_panel.cpp
    source/environment/weather_panel.cpp
    source/vfx/particle_panel.cpp
    source/render/lighting_panel.cpp
    source/render/render_settings_panels.cpp
    source/project/project_panel.cpp
    source/project/project_picker.cpp
    source/project/thumbnail_cache.cpp
    source/project/model_thumbnail_cache.cpp
    source/project/prefab_thumbnail_cache.cpp
    source/scene/scene_commands.cpp
    source/scene/hierarchy_panel.cpp
    source/scene/inspector_panel.cpp
    source/scripting/script_panel.cpp
    source/ui/modals.cpp
    source/core/preferences_window.cpp
    source/input/input_manager_window.cpp
    source/ui/material_inspector.cpp
    source/ui/imgui_backend.cpp
    source/ui/viewport_panel.cpp
    source/animation/skeleton_debug_draw.cpp
    source/animation/animation_panel.cpp
    source/animation/animator_graph_panel.cpp
    source/animation/animated_mesh_preview.cpp
    source/animation/animator_preview_panel.cpp
    source/vfx/effect_preview.cpp
    source/audio/audio_editor_system.cpp
    source/audio/audio_panels.cpp
    source/audio/audio_authoring_panel.cpp
    source/physics/physics_statistics_panel.cpp
    source/physics/assembly_panel.cpp
    source/physics/physics_overlay.cpp
    source/physics/joint_widgets.cpp
    source/physics/vehicle_drive.cpp
    source/physics/vehicle_panel.cpp
    # The bake surface (§14): the widgets over the cook/bake model, and the collider overlay.
    source/physics/cook_bake_panel.cpp
    source/physics/collision_overlay.cpp
    source/physics/soft_body_overlay.cpp
    # The Terrain window: the layer stack a body's ground is composed from, and what the
    # last frame's node selection made of it.
    source/terrain/terrain_panel.cpp
    source/gizmo/gizmo_controller.cpp)
target_link_libraries(sushiengine_editor PRIVATE
    sushiengine_imgui
    sushiengine_render
    sushiengine_simulation
    sushiengine_serialization
    sushiengine_authoring
    sushiengine_model
    sushiengine_model_import
    sushiengine_platform
    sushiengine_input_backend
    sushiengine_audio_backend
    sushiengine_imaging
    sushiengine_profiling
    nlohmann_json::nlohmann_json)
# Stb_INCLUDE_DIR is PRIVATE on sushiengine_render, so it does not arrive transitively; the
# thumbnail cache's stb_image.h include needs it resolved here directly.
target_include_directories(sushiengine_editor PRIVATE "${Stb_INCLUDE_DIR}")
# The bake surface cooks, and cooking needs the pipeline plus the neutral glTF mesh importer
# it is wired to through the `MeshLoader` seam — the editor is the consumer that owns that
# seam, which is why `sushiengine_physics_cooking` itself links neither cgltf nor this target.
target_link_libraries(sushiengine_editor PRIVATE sushiengine_physics_cooking sushiengine_gltf)
# ShellExecuteW (Project panel's "open with default app") needs Shell32.
if(WIN32)
    target_link_libraries(sushiengine_editor PRIVATE Shell32)
endif()
# SDL2 defines SDL_main on some platforms (Windows); link the entry shim when the
# target exists so WinMain resolves without forcing it on Linux/macOS.
if(TARGET SDL2::SDL2main)
    target_link_libraries(sushiengine_editor PRIVATE SDL2::SDL2main)
endif()
# The target carries the repository's prefix so it cannot collide with a SushiRuntime one in the
# single flat CMake namespace the two trees share; the file on disk keeps the short name, which
# is what `se editor` looks for and what a user types.
set_target_properties(sushiengine_editor PROPERTIES
    OUTPUT_NAME se_editor
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

# On Windows the SDL2 runtime DLL must sit next to the executable.
if(WIN32 AND TARGET SDL2::SDL2)
    add_custom_command(TARGET sushiengine_editor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>
            $<TARGET_FILE_DIR:sushiengine_editor>)
endif()

# The runtime DLL must sit next to the editor too, since sushiengine_simulation links it. The
# SYCL policy copies it next to sushiengine_simulation (a static lib dir), not the exe, so copy
# it here where sushiengine_editor actually runs.
if(WIN32 AND TARGET sushiruntime)
    add_custom_command(TARGET sushiengine_editor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:sushiruntime>
            $<TARGET_FILE_DIR:sushiengine_editor>)
endif()
