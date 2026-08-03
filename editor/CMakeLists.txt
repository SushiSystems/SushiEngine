# ============================================================================
# SushiEngine editor
#
# A Unity-style editor shell: a Vulkan-backed SDL2 window hosting Dear ImGui
# (docking branch, vendored at third_party/imgui). It presents through the engine's
# Vulkan renderer (sushi_render) rather than OpenGL, so the same device that draws
# the viewport also draws the UI. Later steps wire in a live World and viewport.
#
# OS-agnostic and CMake-clean: SDL2 comes from the platform's package manager
# (vcpkg on Windows, the system on Linux/macOS) via find_package; ImGui is vendored
# as a submodule and compiled here, the idiomatic way to ship it. The editor is NOT
# a SYCL translation unit, so it does not link the runtime directly — the live world
# arrives behind the plain-C++ simulation seam. It requires SE_BUILD_RENDER for the
# Vulkan backend (the top-level CMake turns render on whenever the editor is on).
# ============================================================================

if(NOT TARGET sushi_render)
    message(FATAL_ERROR
        "The editor needs the Vulkan renderer. Configure with -DSE_BUILD_RENDER=ON "
        "(the top-level CMake enables it automatically when SE_BUILD_EDITOR is on).")
endif()

find_package(SDL2 CONFIG REQUIRED)
# The imported Vulkan targets are directory-scoped; the renderer's own find_package does not
# export them here, so the editor resolves them itself.
find_package(VulkanHeaders CONFIG REQUIRED)
find_package(VulkanLoader CONFIG REQUIRED)
# Header-only JSON for the editor's persisted Preferences and the .sushiscene format.
find_package(nlohmann_json CONFIG REQUIRED)

set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/third_party/imgui")
if(NOT EXISTS "${IMGUI_DIR}/imgui.cpp")
    message(FATAL_ERROR
        "Dear ImGui sources not found at ${IMGUI_DIR}. "
        "Initialize submodules: git submodule update --init --recursive")
endif()

# ImGui core plus the SDL2 platform backend and the Vulkan renderer backend. The
# Vulkan backend links the loader for its prototype calls; the color format is fed
# in at init time (dynamic rendering, no render-pass object).
add_library(sushi_imgui STATIC
    "${IMGUI_DIR}/imgui.cpp"
    "${IMGUI_DIR}/imgui_draw.cpp"
    "${IMGUI_DIR}/imgui_tables.cpp"
    "${IMGUI_DIR}/imgui_widgets.cpp"
    "${IMGUI_DIR}/imgui_demo.cpp"
    "${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp")
target_include_directories(sushi_imgui PUBLIC
    "${IMGUI_DIR}"
    "${IMGUI_DIR}/backends")
target_link_libraries(sushi_imgui PUBLIC SDL2::SDL2 Vulkan::Headers Vulkan::Loader)
target_compile_features(sushi_imgui PUBLIC cxx_std_17)

add_executable(se_editor
    main.cpp
    ui/editor_panels.cpp
    ui/panel_widgets.cpp
    atmosphere/meteorology_panel.cpp
    environment/weather_panel.cpp
    vfx/particle_panel.cpp
    render/lighting_panel.cpp
    render/render_settings_panels.cpp
    project/project_panel.cpp
    scene/scene_commands.cpp
    scene/hierarchy_panel.cpp
    scene/inspector_panel.cpp
    scripting/script_panel.cpp
    ui/modals.cpp
    core/preferences_window.cpp
    input/input_manager_window.cpp
    ui/material_inspector.cpp
    ui/imgui_backend.cpp
    ui/viewport_panel.cpp
    animation/skeleton_debug_draw.cpp
    animation/animation_panel.cpp
    animation/animator_graph_panel.cpp
    animation/animated_mesh_preview.cpp
    animation/animator_preview_panel.cpp
    vfx/effect_preview.cpp
    audio/audio_editor_system.cpp
    audio/audio_panels.cpp
    audio/audio_authoring_panel.cpp
    physics/physics_statistics_panel.cpp
    physics/assembly_panel.cpp
    physics/physics_overlay.cpp
    scene/physics_sample_scene.cpp
    physics/joint_widgets.cpp
    physics/vehicle_drive.cpp
    physics/vehicle_panel.cpp
    # The bake surface (§14): the model, the widgets over it, and the collider overlay. The
    # model is a UI-free translation unit on purpose — it holds every decision the panel makes
    # and is tested without ImGui, the way the scene serializer already is.
    physics/cook_bake_state.cpp
    physics/cook_bake_panel.cpp
    physics/collision_overlay.cpp
    physics/soft_body_overlay.cpp
    gizmo/gizmo_controller.cpp
    core/preferences.cpp
    core/command_history.cpp)
target_link_libraries(se_editor PRIVATE
    sushi_imgui sushi_render sushi_sim sushi_scene sushi_platform sushi_input
    sushi_audio nlohmann_json::nlohmann_json)
# The bake surface cooks, and cooking needs the pipeline plus the neutral glTF mesh importer
# it is wired to through the `MeshLoader` seam — the editor is the consumer that owns that
# seam, which is why `sushi_cooking` itself links neither cgltf nor this target.
target_link_libraries(se_editor PRIVATE sushi_cooking sushi_import)
# ShellExecuteW (Project panel's "open with default app") needs Shell32.
if(WIN32)
    target_link_libraries(se_editor PRIVATE Shell32)
endif()
# SDL2 defines SDL_main on some platforms (Windows); link the entry shim when the
# target exists so WinMain resolves without forcing it on Linux/macOS.
if(TARGET SDL2::SDL2main)
    target_link_libraries(se_editor PRIVATE SDL2::SDL2main)
endif()
set_target_properties(se_editor PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

# On Windows the SDL2 runtime DLL must sit next to the executable.
if(WIN32 AND TARGET SDL2::SDL2)
    add_custom_command(TARGET se_editor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>
            $<TARGET_FILE_DIR:se_editor>)
endif()

# The runtime DLL must sit next to the editor too, since sushi_sim links it. The
# SYCL policy copies it next to sushi_sim (a static lib dir), not the exe, so copy
# it here where se_editor actually runs.
if(WIN32 AND TARGET sushiruntime)
    add_custom_command(TARGET se_editor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:sushiruntime>
            $<TARGET_FILE_DIR:se_editor>)
endif()
