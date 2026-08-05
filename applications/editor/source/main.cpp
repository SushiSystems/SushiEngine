/**************************************************************************/
/* main.cpp                                                               */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Editor shell: a Vulkan-backed window hosting Dear ImGui with a full-window
// dockspace and a Unity-style panel set — Hierarchy, Inspector, Project, and a
// text editor. It edits an editor-side scene and the on-disk project; a live
// viewport and world follow in later increments. The window (SDLWindow), the
// graphics device and swapchain (Render::IWindowRenderer), and the ImGui/Vulkan
// glue (ImGuiBackend) sit behind narrow seams, so this loop names no windowing or
// graphics API directly and a different backend could replace either.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <filesystem>

#include <imgui.h>
#include <imgui_internal.h>

#include <vector>

#include <SushiEngine/astro/ephemeris.hpp>
#include <SushiEngine/authoring/autosave.hpp>
#include <SushiEngine/authoring/preferences.hpp>
#include <SushiEngine/input/bindings_json.hpp>
#include <SushiEngine/input/input_manager.hpp>
#include <SushiEngine/render/window_renderer.hpp>
#include <SushiEngine/simulation/simulation.hpp>
#include <SushiEngine/simulation/simulation_settings.hpp>

#include "input/editor_contexts.hpp"
#include "sdl/sdl_input_translator.hpp"

#include <memory>

#include "core/editor_context.hpp"
#include "core/game_view_render_policy.hpp"
#include "atmosphere/meteorology_panel.hpp"
#include "environment/weather_panel.hpp"
#include "render/lighting_panel.hpp"
#include "project/project_panel.hpp"
#include "project/project_picker.hpp"
#include "render/render_settings_panels.hpp"
#include "core/preferences_window.hpp"
#include "input/input_manager_window.hpp"
#include "scene/hierarchy_panel.hpp"
#include "scene/inspector_panel.hpp"
#include "scene/scene_commands.hpp"
#include "ui/editor_panels.hpp"
#include "ui/modals.hpp"
#include "ui/imgui_backend.hpp"
#include "effect_serializer.hpp"
#include "scene_serializer.hpp"
#include "sdl_window.hpp"
#include "ui/viewport_panel.hpp"
#include "animation/animation_panel.hpp"
#include "animation/animator_graph_panel.hpp"
#include "animation/animator_preview_panel.hpp"
#include <SushiEngine/audio/authoring.hpp>

#include "audio/audio_authoring_panel.hpp"
#include "audio/audio_editor_system.hpp"
#include "audio/audio_panels.hpp"
#include <SushiEngine/gltf/mesh_import.hpp>

#include "physics/cook_bake_panel.hpp"
#include "physics/physics_statistics_panel.hpp"
#include "physics/assembly_panel.hpp"
#include "physics/vehicle_drive.hpp"
#include "physics/vehicle_panel.hpp"
#include "terrain/terrain_panel.hpp"

namespace
{
    // Where user-authored projects live by default: never inside the engine's own
    // source tree (writing project code next to the engine's is exactly the mixing
    // the Project panel exists to avoid). Falls back to the current directory only if
    // the per-user profile directory cannot be resolved.
    std::string default_projects_root()
    {
        std::filesystem::path home;
#ifdef _WIN32
        char* value = nullptr;
        std::size_t length = 0;
        if (_dupenv_s(&value, &length, "USERPROFILE") == 0 && value != nullptr)
        {
            home = value;
            std::free(value);
        }
#else
        if (const char* value = std::getenv("HOME"))
            home = value;
#endif
        std::filesystem::path root =
            !home.empty() ? home / "sushiengine" / "project" : std::filesystem::current_path();
        std::error_code ec;
        std::filesystem::create_directories(root, ec);
        return root.string();
    }
} // namespace

namespace
{
    // A single dockspace covering the main viewport, so the panels can be dragged,
    // tabbed, and split Unity-style. Rebuilt each frame; ImGui persists the layout
    // to layout.ini in the per-user config directory (io.IniFilename is pinned there
    // at startup, so the launch directory does not matter). On the first run (no
    // persisted node yet), or when @p force_default_layout is set (Window ▸ Reset
    // Layout), the default layout is applied via build_default_layout. Must be
    // submitted after the menu/toolbar/status side bars so the work area it fills
    // already excludes them.
    //
    // @return true on the frame the default layout was (re)built.
    bool draw_dockspace(bool force_default_layout)
    {
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);

        ImGuiWindowFlags flags =
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
            ImGuiWindowFlags_NoDocking;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::Begin("SushiEngineDockHost", nullptr, flags);
        ImGui::PopStyleVar(3);

        const ImGuiID dockspace_id = ImGui::GetID("SushiEngineDockSpace");
        const bool needs_layout =
            force_default_layout || ImGui::DockBuilderGetNode(dockspace_id) == nullptr;
        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f),
                         ImGuiDockNodeFlags_PassthruCentralNode);
        if (needs_layout)
            SushiEngine::Editor::build_default_layout(dockspace_id);
        ImGui::End();
        return needs_layout;
    }
}

int main(int argc, char** argv)
{
    try
    {
        SushiEngine::Platform::SDLWindow window("SushiEngine Editor", 1600, 900);

        std::uint32_t width = 0;
        std::uint32_t height = 0;
        window.drawable_size(width, height);

        SushiEngine::Render::WindowRendererDescription description;
        description.required_instance_extensions = window.vulkan_instance_extensions();
        description.surface_factory = [&window](std::uint64_t instance)
        {
            return window.create_vulkan_surface(instance);
        };
        // Validation is opt-in per run: the layers cost real frame time, but without them a
        // synchronisation mistake surfaces as a device loss with no message.
        for (int i = 1; i < argc; ++i)
            if (std::string(argv[i]) == "--validation")
                description.enable_validation = true;
        description.width = width != 0 ? width : 1600;
        description.height = height != 0 ? height : 900;
        std::unique_ptr<SushiEngine::Render::IWindowRenderer> renderer =
            SushiEngine::Render::create_window_renderer(description);

        SushiEngine::Editor::ImGuiBackend imgui(window, *renderer);

        // Pin the dock-layout ini beside preferences.json in the per-user config
        // directory. ImGui's default is a CWD-relative imgui.ini, which silently
        // forked the layout per launch directory. The path string must outlive the
        // ImGui context, so it lives in main's scope; the directory is created now
        // because ImGui writes the file itself and creates nothing.
        const std::string layout_ini_path =
            (std::filesystem::path(SushiEngine::Authoring::user_config_directory()) /
             "layout.ini")
                .string();
        {
            std::error_code layout_directory_error;
            std::filesystem::create_directories(
                std::filesystem::path(layout_ini_path).parent_path(), layout_directory_error);
        }
        ImGui::GetIO().IniFilename = layout_ini_path.c_str();

        // The device-abstracted input manager. The SDL translator is registered after ImGui
        // on the window's handler list (so ImGui still sees events first); the editor consumes
        // the resolved snapshot for its shortcuts and tool keys (see the EditorGlobal/
        // EditorViewport contexts pushed below).
        SushiEngine::Input::InputManager input;
        SushiEngine::Input::SDLInputTranslator input_translator(input);
        window.add_event_handler([&input_translator](const void* event)
        {
            input_translator.handle_native_event(event);
        });

        // The editor's shortcut and tool keys as rebindable contexts. Built with their
        // compiled-in defaults here; any saved overrides are applied once preferences load
        // (below), and the Preferences page rebinds against these live objects.
        SushiEngine::Input::InputContext editor_global{"EditorGlobal"};
        SushiEngine::Input::InputContext editor_viewport{"EditorViewport"};
        SushiEngine::Editor::build_editor_global_context(editor_global);
        SushiEngine::Editor::build_editor_viewport_context(editor_viewport);
        SushiEngine::Input::InputContext editor_drive{"EditorDrive"};
        SushiEngine::Editor::build_editor_drive_context(editor_drive);

        // Two Unity viewports, each a ViewportPanel over the same world but a
        // different injected camera: the Scene view flies freely, the Game view
        // follows the world's camera. The cameras are declared before the panels so
        // they outlive the references the panels hold.
        SushiEngine::Editor::FlyCameraSource scene_camera;
        SushiEngine::Editor::WorldCameraSource game_camera;
        SushiEngine::Editor::ViewportPanel scene_view(*renderer, imgui, "Scene", scene_camera);
        SushiEngine::Editor::ViewportPanel game_view(*renderer, imgui, "Game", game_camera);
        // The one place that decides whether the Game view has anything to play the
        // scene through and onto, kept separate from both the frame loop and the panel.
        SushiEngine::Editor::GameViewRenderPolicy game_view_render_policy;
        // A third surface for the previewed effect. It shows no world geometry on purpose: an
        // effect being authored is not in the scene yet, and mixing it into the Scene view would
        // make the preview read as a stray entity nobody could select or delete.
        SushiEngine::Editor::FlyCameraSource preview_camera;
        SushiEngine::Editor::ViewportPanel preview_view(*renderer, imgui, "Preview",
                                                        preview_camera);

        // The world is the single source of truth for entities; the panels read and
        // edit it through the injected simulation. There is no editor-side scene model.
        SushiEngine::Editor::EditorContext context;

        // Load persisted preferences first, so the editor opens in the user's theme
        // and camera speed. The store is injected into the context for the
        // Preferences window to display its path.
        std::unique_ptr<SushiEngine::Authoring::IPreferencesStore> preferences_store =
            SushiEngine::Authoring::create_preferences_store();
        context.preferences_store = preferences_store.get();
        context.preferences = preferences_store->load();
        SushiEngine::Editor::apply_theme(context.preferences.theme);
        context.render_settings = context.preferences.render_settings;
        context.simulation_settings = context.preferences.simulation;
        // Session state the preferences persist: the open window set, the Game view
        // toolbar, and the transform tool — so the editor reopens as it was left.
        context.panels = context.preferences.panels;
        context.game_view_settings = context.preferences.game_view;
        context.gizmo_mode = context.preferences.gizmo_mode;
        context.gizmo_space = context.preferences.gizmo_space;

        // Folds the live session state back into the preferences aggregate; called
        // before every save so the file and the running editor never disagree about
        // what the next launch restores.
        const auto sync_session_preferences = [&context]()
        {
            context.preferences.render_settings = context.render_settings;
            context.preferences.simulation = context.simulation_settings;
            context.preferences.panels = context.panels;
            context.preferences.game_view = context.game_view_settings;
            context.preferences.gizmo_mode = context.gizmo_mode;
            context.preferences.gizmo_space = context.gizmo_space;
        };

        // Apply any saved binding overrides onto the compiled-in defaults, then push the
        // contexts and expose them (and the manager) to the panels. A stale or partial blob
        // degrades to defaults inside bindings_from_json, so a corrupt file never blocks input.
        if (!context.preferences.input_bindings.empty())
        {
            const nlohmann::json saved_bindings =
                nlohmann::json::parse(context.preferences.input_bindings, nullptr, false);
            SushiEngine::Input::bindings_from_json(editor_global, saved_bindings);
            SushiEngine::Input::bindings_from_json(editor_viewport, saved_bindings);
        }
        input.push_context(editor_global);
        input.push_context(editor_viewport);
        input.push_context(editor_drive);
        context.input_manager = &input;
        context.editor_global_context = &editor_global;
        context.editor_viewport_context = &editor_viewport;

        // The live world, ticked on SushiRuntime behind the plain-C++ ISimulation
        // seam. The editor sees only the abstraction and the extracted RenderScene;
        // the runtime, SYCL, and ECS stay inside sushiengine_simulation.
        std::unique_ptr<SushiEngine::Simulation::ISimulation> simulation =
            SushiEngine::Simulation::create_simulation();
        std::vector<SushiEngine::Render::MeshInstance> instances;
        context.simulation = simulation.get();
        context.assets = &renderer->assets();
        // The one thing that flows renderer -> simulation: the GPU atmosphere's asynchronous
        // readback (docs/design/atmosphere_system.md §3.2). Bound once here rather than ferried
        // every frame, because the host is the only party that owns both. A host that skips this
        // is not broken -- every weather query then answers from the base state, a clear sky with
        // the synoptic wind, instead of from a sky that has silently stopped evolving.
        simulation->set_atmosphere_mirror(&renderer->assets());

        // The live VFX preview: authored in the Particle Editor panel, simulated and
        // billboarded by the renderer, gizmo'd in the Scene viewport.
        SushiEngine::Editor::EffectPreview particle_preview;
        context.particle_preview = &particle_preview;
        context.world_entity_count = simulation->entity_count();

        // The live editor audio system (S9): owns an AudioEngine + SDL device and each
        // frame projects the world's audio emitters / reverb zones into the voice pool with
        // the Scene camera as the listener, so authored sound is heard in-editor.
        SushiEngine::Editor::AudioEditorSystem audio_system;
        context.audio = &audio_system;

        // The sound-designer's authoring project (audio phase S9's DAW panel): owned
        // here like the particle preview — the panel is a view over it, and it bakes
        // to a runtime bank. Session-scoped until project-file persistence lands.
        SushiEngine::Audio::AudioAuthoringProject audio_authoring_project;

        // The Project panel's root: the last one the user browsed to, or a
        // %USERPROFILE%/SushiProjects default — never the engine's own source tree,
        // so authored project code never mixes with the engine's. Resolved here, ahead of
        // the Bake surface below, because that surface's cooking profile is project-scoped
        // storage and needs to know where the project lives before it can load anything.
        context.project_root = context.preferences.last_project_root.empty()
                                    ? default_projects_root()
                                    : context.preferences.last_project_root;
        context.current_directory = context.project_root;
        if (context.preferences.last_project_root != context.project_root)
        {
            context.preferences.last_project_root = context.project_root;
            preferences_store->save(context.preferences);
        }

        // The bake surface's model, and the one place the `MeshLoader` seam is wired: the
        // editor is the consumer, so the editor is what decides that "a path" means a glTF
        // file. The cooking module itself links no importer, which is what lets it be built
        // and tested on a machine with neither cgltf nor a device.
        SushiEngine::Authoring::CookBakeState cook_bake_state(
            [](const std::string& path, SushiEngine::Geometry::TriangleMesh& out)
            { return SushiEngine::Geometry::import_gltf_mesh(path.c_str(), out); },
            "cooked");
        // §16.45.3: the project-default fidelity dial, its pin overrides, and every per-asset
        // override are persisted here. Set before the Bake or Cooking Override UI can touch
        // `profiles()`, so the very first frame shows what was saved rather than the hard-coded
        // defaults for one frame before an overwrite.
        cook_bake_state.set_profile_storage_path(
            (std::filesystem::path(context.project_root) / "cooking_profile.json").string());
        if (!cook_bake_state.load_profiles())
        {
            SushiEngine::Editor::editor_log(
                context, "Could not parse the project's cooking_profile.json; using defaults.",
                SushiEngine::Editor::LogLevel::Warning);
        }
        // Injected so a panel that brings a mesh into the project can queue it for
        // cooking automatically (see project_panel.cpp's glTF open handler) instead of
        // an artist having to find and press the Bake panel's button for every asset.
        context.cook_bake_state = &cook_bake_state;

        // The vehicle under authoring (§11). A document rather than a selected
        // entity's component, because there is no `Vehicle` component in the ECS
        // yet -- a vehicle reaches a scene through `VehicleInstanceT` against a
        // solver, and the authoring record has no owner in the entity world to
        // hang from. When that component exists the panel becomes an inspector
        // over it and this line goes away.
        SushiEngine::Editor::VehicleAuthoringState vehicle_authoring;

        // The assembly under authoring (§10.2), and the one place P3's joint library
        // can be reached from. Instancing it produces ordinary entities rather than a
        // scene-graph node of its own, so the parts stay editable afterwards -- see
        // assembly_panel.hpp for why that is the decision and what it costs.
        SushiEngine::Editor::AssemblyAuthoringState assembly_authoring;

        // The live GPU-skinned character preview: the A1 "load a rigged, animated glTF and see
        // it looping, skinned on the GPU" surface (design `slop/animation_system.md` §12.1) —
        // the one place that builds a live SkinnedInstance for the viewport. Loading a demo
        // asset fails silently (no rig at that path just leaves the preview empty) rather than
        // blocking editor startup; a proper Animator-driven scene entity is future work.
        SushiEngine::Editor::AnimatedMeshPreview animated_mesh_preview;
        animated_mesh_preview.load_gltf("assets/models/rigged_arm_anim.gltf", *context.assets);
        context.animated_mesh_preview = &animated_mesh_preview;

        // The two animation-authoring documents, owned here for the same reason the previews
        // are: they outlive a frame, and exactly one of each should exist.
        SushiEngine::Editor::AnimationState animation_state;
        SushiEngine::Editor::GraphState animator_graph;

        // The Terrain window's draft layer, owned here for the same reason: it is the one
        // piece of that panel not already held by the body's own layer stack.
        SushiEngine::Editor::TerrainPanelState terrain_panel_state;

        // The editor opens with no scene, which is the "new scene" state — so the world
        // starts from the user's default environment. A scene opened later brings its
        // own: the environment is scene content, loaded and saved with the file.
        simulation->world().set_environment(context.preferences.default_environment);

        scene_camera.set_move_speed(context.preferences.camera_move_speed);

        SushiEngine::Editor::editor_log(context, "Editor ready (Vulkan).");
        SushiEngine::Editor::editor_log(context, "No scene open. Use File > New Scene or open a "
                                            ".sushiscene from the Project panel.");

        bool running = true;
        bool gizmo_was_dragging = false;
        bool ui_was_dragging = false;
        // Autosave: the timer accumulates only while a save would be meaningful
        // (enabled, scene has a file, scene is dirty) — see autosave.hpp for the policy.
        SushiEngine::Authoring::AutosaveTimer autosave_timer;
        // Multi-select drag: every co-selected entity's pose captured at the grab, so a
        // translate drag carries the whole selection rigidly with the primary.
        SushiEngine::Simulation::EntityTransform multi_primary_start;
        std::vector<std::pair<SushiEngine::Simulation::EntityId,
                              SushiEngine::Simulation::EntityTransform>>
            multi_drag_start;
        // The one wall-clock read in the editor loop: real elapsed time since the
        // last frame, fed into ISimulation::tick() so its FixedTimestepClock can
        // turn it into whole fixed steps. The sim itself never reads the clock.
        std::chrono::steady_clock::time_point last_frame_time =
            std::chrono::steady_clock::now();
        while (running)
        {
            const std::chrono::steady_clock::time_point frame_time =
                std::chrono::steady_clock::now();
            const SushiEngine::Scalar real_delta_seconds =
                std::chrono::duration<SushiEngine::Scalar>(frame_time - last_frame_time).count();
            context.console.uptime_seconds += static_cast<double>(real_delta_seconds);
            last_frame_time = frame_time;

            // Advance the VFX preview and rebuild this frame's emitter views, clamped so a
            // long stall (a resize, a breakpoint) does not spawn a burst of catch-up particles.
            particle_preview.update(
                static_cast<float>(real_delta_seconds > 0.1 ? 0.1 : real_delta_seconds));
            animated_mesh_preview.update(
                static_cast<float>(real_delta_seconds > 0.1 ? 0.1 : real_delta_seconds));

            // A close request (the window's X, or File > Exit) is not obeyed directly;
            // it only sets close_requested, so draw_exit_confirm_modal below gets a
            // chance to hold the window open while unsaved changes are pending.
            if (!window.pump_events())
                context.close_requested = true;

            // Fold this frame's input after the pump (so the translator has received the
            // native events) and before the world ticks. The gate mirrors ImGui's capture
            // flags so key/mouse actions stand down while a widget owns the device.
            {
                const ImGuiIO& io = ImGui::GetIO();
                SushiEngine::Input::InputGate gate;
                gate.want_capture_keyboard = io.WantCaptureKeyboard;
                gate.want_capture_mouse = io.WantCaptureMouse;
                gate.want_text_input = io.WantTextInput;
                input.set_gate(gate);
                input.begin_frame();
                context.input_snapshot = &input.snapshot();
            }

            // The Physics panel's profiling request rides the panel's visibility: the
            // per-stage timings cost timestamps on the dispatch hot path, so they are
            // collected only while someone is looking (the flag is consumed when the
            // solve graph is next built — see IPhysicsStepper::set_profiling_requested).
            simulation->set_physics_profiling(context.panels.physics);

            // Tick the world on the runtime only while playing, so the toolbar's
            // Play/Pause gates motion; then take the fresh snapshot to draw. Step
            // advances exactly one fixed step (via a zero-length real delta plus a
            // full accumulated one) regardless of play_state (normally pressed while
            // Paused) and is a one-shot request the toolbar sets.
            if (context.play_state == SushiEngine::Editor::PlayState::Playing)
                simulation->tick(real_delta_seconds);
            else if (context.step_requested)
                simulation->tick(simulation->fixed_dt_seconds());
            context.step_requested = false;

            const SushiEngine::Simulation::RenderScene& scene = simulation->render_scene();
            instances.clear();
            instances.reserve(scene.instances.size());
            for (const SushiEngine::Simulation::RenderInstance& source : scene.instances)
            {
                SushiEngine::Render::MeshInstance instance;
                instance.model = source.model;
                instance.color = source.color;
                instance.id = static_cast<std::uint32_t>(source.id);
                // Simulation::PrimitiveKind and Render::MeshKind share Box/Sphere/Cylinder
                // ordinal order by construction; Plane never reaches a RenderInstance
                // (Terrain's visual Shape is always a Box).
                instance.kind =
                    static_cast<SushiEngine::Render::MeshKind>(source.shape_kind);
                instance.shape_parameters = source.shape_parameters;
                instance.material = source.material;
                instances.push_back(instance);
            }
            context.world_entity_count = simulation->entity_count();
            context.physics_statistics = simulation->physics_statistics();
            simulation->set_park_sleeping_joints(context.park_sleeping_joints);

            // Soft-body surfaces: one view per simulated mesh, pointing directly into the
            // snapshot's concatenated vertex and index buffers for this frame's lifetime.
            // Both offsets are applied here and nowhere else — the sim numbers each
            // surface's triangles from its own first vertex, so the pointer arithmetic is
            // all it takes to hand the renderer a self-contained mesh.
            std::vector<SushiEngine::Render::DeformableMeshView> deformable;
            deformable.reserve(scene.deformable_instances.size());
            for (const SushiEngine::Simulation::DeformableInstance& surface :
                 scene.deformable_instances)
            {
                SushiEngine::Render::DeformableMeshView view;
                view.vertices = scene.deformable_vertices.data() + surface.first_vertex;
                view.vertex_count = surface.vertex_count;
                view.indices = scene.deformable_indices.data() + surface.first_index;
                view.index_count = surface.index_count;
                view.topology_revision = surface.topology_revision;
                view.color = surface.color;
                view.id = static_cast<std::uint32_t>(surface.id);
                deformable.push_back(view);
            }

            // Deterministic particles: one billboard per live particle in every emitter
            // entity's pool, already world-space from the sim's fixed-tick integration.
            std::vector<SushiEngine::Render::ParticleBillboard> particle_billboards;
            particle_billboards.reserve(scene.particle_billboards.size());
            for (const SushiEngine::Simulation::ParticleBillboard& particle :
                 scene.particle_billboards)
            {
                SushiEngine::Render::ParticleBillboard billboard;
                billboard.position = particle.position;
                billboard.color = particle.color;
                billboard.size = particle.size;
                billboard.alpha = particle.alpha;
                billboard.rotation = particle.rotation;
                particle_billboards.push_back(billboard);
            }

            // Cosmetic emitters come across already in render shape — the sim only placed them —
            // so this channel is passed straight through to the viewport.
            const SushiEngine::Render::ParticleEmitterView* scene_emitters =
                scene.particle_emitters.empty() ? nullptr : scene.particle_emitters.data();
            const std::size_t scene_emitter_count = scene.particle_emitters.size();

            // Resolve which display the Game view shows: the selected display's camera
            // if present, else the default. Also gather the display options for the
            // Game panel's selector so two cameras on different displays never conflict.
            std::vector<std::uint32_t> displays;
            displays.reserve(scene.display_cameras.size());
            const SushiEngine::Simulation::CameraState* game = &scene.camera;
            bool selected_display_present = false;
            for (const SushiEngine::Simulation::DisplayCamera& display_camera :
                 scene.display_cameras)
            {
                displays.push_back(display_camera.display);
                if (display_camera.display == context.game_display)
                {
                    game = &display_camera.state;
                    selected_display_present = true;
                }
            }
            // If the chosen display vanished (its camera was deleted), fall back to the
            // first available so the Game view keeps rendering.
            if (!selected_display_present && !scene.display_cameras.empty())
            {
                context.game_display = scene.display_cameras.front().display;
                game = &scene.display_cameras.front().state;
            }
            game_camera.set_pose(game->position, game->target, game->up,
                                 game->vertical_fov_radians, game->near_plane, game->far_plane);

            imgui.new_frame();

            // Global undo/redo/save shortcuts, resolved through the EditorGlobal input
            // context (rebindable, persisted). The mapper's capture gate already suppresses
            // these while a text field owns the keyboard, so Ctrl+Z in a rename field is not
            // hijacked — one `!WantTextInput` guard, centralized.
            if (simulation != nullptr)
            {
                using EditorContextType = SushiEngine::Editor::EditorContext;
                const SushiEngine::Input::ActionSnapshot& actions = input.snapshot();
                SushiEngine::Simulation::IWorldEditor& editor_world = simulation->world();
                if (actions.pressed("Undo") && context.history.undo(editor_world))
                    SushiEngine::Editor::select_only(context, SushiEngine::Simulation::NULL_ENTITY);
                else if (actions.pressed("Redo") && context.history.redo(editor_world))
                    SushiEngine::Editor::select_only(context, SushiEngine::Simulation::NULL_ENTITY);
                else if (actions.pressed("Save"))
                    SushiEngine::Editor::save_current_scene(context);
                else if (actions.pressed("Copy"))
                    context.pending_entity_command = EditorContextType::EntityCommand::Copy;
                else if (actions.pressed("Cut"))
                    context.pending_entity_command = EditorContextType::EntityCommand::Cut;
                else if (actions.pressed("Paste"))
                    context.pending_entity_command = EditorContextType::EntityCommand::Paste;
                else if (actions.pressed("Duplicate"))
                    context.pending_entity_command = EditorContextType::EntityCommand::Duplicate;
                else if (actions.pressed("Delete"))
                    context.pending_entity_command = EditorContextType::EntityCommand::Delete;
                else if (actions.pressed("NewScene"))
                    SushiEngine::Editor::request_new_scene(context);
                if (actions.pressed("SceneFullscreen"))
                    context.scene_view_fullscreen = !context.scene_view_fullscreen;

                // The selected car, driven. A no-op unless the selection carries a Vehicle,
                // which is what lets the arrow keys stay ordinary keys the rest of the time.
                // Clamped like the other wall-clock consumers above: a frame that took half a
                // second is a stall, not half a second of somebody holding the throttle.
                SushiEngine::Editor::drive_selected_vehicle(
                    context, editor_world, actions,
                    static_cast<float>(real_delta_seconds > 0.1 ? 0.1 : real_delta_seconds));
            }

            // The menu bar, toolbar strip, and status bar are viewport side bars, so
            // they are submitted before the dockspace: each one shrinks the main
            // viewport's work area and the dockspace fills what remains. Submitting
            // them after would bake the first frame's default layout against a work
            // area that still included them.
            SushiEngine::Editor::draw_menu_bar(context);
            SushiEngine::Editor::draw_toolbar(context);
            SushiEngine::Editor::draw_status_bar(context);

            // Window ▸ Reset Layout (raised by the menu just drawn): rebuild the
            // default dock arrangement this same frame, with the open set back at
            // its defaults — the escape hatch for any wedged drag state.
            const bool reset_layout = context.layout_reset_requested;
            context.layout_reset_requested = false;
            if (reset_layout)
                context.panels = SushiEngine::Authoring::PanelVisibility{};
            draw_dockspace(reset_layout);

            // Selection is shared between the viewports and the panels. The scene
            // renderer speaks 32-bit ids; entity ids stay small, so the round-trip is
            // lossless. A left-click in either viewport picks the entity under it.
            std::uint32_t selected = static_cast<std::uint32_t>(context.selected_entity);

            // The Scene view gets the transform gizmo at the selection; a drag edits a
            // local copy of the transform written back to the world afterwards.
            SushiEngine::Simulation::IWorldEditor& world = simulation->world();
            const bool has_selection = world.exists(context.selected_entity);
            SushiEngine::Simulation::EntityTransform selected_transform;
            SushiEngine::Simulation::EntityTransform* gizmo_target = nullptr;
            if (has_selection)
            {
                selected_transform = world.world_transform(context.selected_entity);
                gizmo_target = &selected_transform;
            }

            // Gizmo snapping comes from the Scene preferences; off unless enabled there.
            SushiEngine::Editor::GizmoSnap snap;
            snap.enabled = context.preferences.snap_enabled;
            snap.translate = context.preferences.snap_translate;
            snap.rotate_degrees = context.preferences.snap_rotate_degrees;
            snap.scale = context.preferences.snap_scale;

            // UI overlay: every entity carrying a UI element, flattened with its UI
            // parent resolved to an index, so the viewport can lay it out against the
            // panel rect and paint canvases/panels/images/text/buttons over the 3D view.
            std::vector<SushiEngine::Editor::UIOverlayElement> ui_overlay;
            std::vector<SushiEngine::Simulation::EntityId> ui_ids;
            for (const SushiEngine::Simulation::EntityId id : world.entities())
            {
                if (!world.has_ui(id) || !world.visible(id))
                    continue;
                SushiEngine::Editor::UIOverlayElement element;
                element.id = static_cast<std::uint32_t>(id);
                element.parameters = world.ui_parameters(id);
                element.selected = id == context.selected_entity;
                ui_overlay.push_back(element);
                ui_ids.push_back(id);
            }
            for (std::size_t i = 0; i < ui_overlay.size(); ++i)
            {
                const SushiEngine::Simulation::EntityId parent_id = world.parent(ui_ids[i]);
                for (std::size_t j = 0; j < ui_ids.size(); ++j)
                    if (ui_ids[j] == parent_id)
                    {
                        ui_overlay[i].parent = static_cast<int>(j);
                        break;
                    }
            }

            // The Scene view edits the UI (translucent, interactive); the Game view shows
            // it solid. Both draw the same (possibly just-dragged) elements.
            SushiEngine::Editor::UIOverlay scene_ui;
            scene_ui.elements = ui_overlay.data();
            scene_ui.count = ui_overlay.size();
            scene_ui.edit_mode = true;
            scene_ui.selected_id = static_cast<std::uint32_t>(context.selected_entity);
            SushiEngine::Editor::UIOverlay game_ui;
            game_ui.elements = ui_overlay.data();
            game_ui.count = ui_overlay.size();
            game_ui.edit_mode = false;

            // Solar-system sky: repopulate the far-field bodies and stars from the authored
            // epoch and observer each frame. A local copy of the world's environment is
            // mutated and handed to the viewports, so driving the ephemeris never
            // re-extracts the world (unlike set_environment, which does).
            SushiEngine::Render::Environment environment = scene.environment;
            // The nest grid is a per-user simulation budget, resolved from the atmosphere
            // tier here and carried to the renderer on the environment — never on the
            // world's authored environment and never in a scene file.
            environment.atmosphere_nest_size =
                SushiEngine::Simulation::resolve_atmosphere_quality(
                    context.simulation_settings.atmosphere.quality);
            if (context.sky_enabled)
            {
                // The simulation owns the master epoch: drive its flow from the panel's
                // animate/rate, seek it when the authored start date changes, and read it
                // back so the sky and the orbital dynamics share one clock. The epoch
                // advances with the world's fixed steps, so it moves under Play/Step.
                simulation->set_time_scale_days_per_second(
                    context.sky_animate ? context.sky_days_per_second : 0.0);
                const double authored_start =
                    SushiEngine::Astro::julian_date_from_calendar(context.sky_date);
                if (authored_start != context.sky_authored_start_cache)
                {
                    simulation->set_julian_date(authored_start);
                    context.sky_authored_start_cache = authored_start;
                }
                const double epoch = simulation->julian_date();
                environment.observer.julian_date = epoch;
                context.sky_accumulated_days = epoch - authored_start;
                environment.observer.latitude_radians =
                    context.sky_latitude_degrees * SushiEngine::Astro::DEGREES_TO_RADIANS;
                environment.observer.longitude_radians =
                    context.sky_longitude_degrees * SushiEngine::Astro::DEGREES_TO_RADIANS;
                environment.observer.astronomical_sun = context.sky_astronomical_sun;

                // One continuous scene: every body is placed camera-relative in metres,
                // so flying up from the ground and on toward the Moon or a planet just
                // works — the ephemeris hands the analytic ground off to a far-field
                // Earth past the hand-off altitude, with no mode switch.
                SushiEngine::Editor::FlyCamera& fly = scene_camera.camera();
                const SushiEngine::WorldVector3 camera_position{fly.position.x, fly.position.y,
                                                                fly.position.z};
                SushiEngine::Astro::fill_environment_sky(environment, camera_position);

                // Quick travel from the environment panel: jump the camera to the
                // requested body's sunlit side (Earth means home, the scene origin),
                // snap the local vertical instead of slewing, and refill the sky so
                // this frame already renders from the destination.
                double up_retarget_seconds = real_delta_seconds;
                if (context.sky_travel_target >= 0)
                {
                    const int target = context.sky_travel_target;
                    context.sky_travel_target = -1;
                    bool moved = false;
                    // The scene origin is the *observer* body's surface point, so "go to the
                    // body you are already anchored to" means return to that origin — for
                    // whichever body is currently the observer, not Earth specifically.
                    // (Selecting Earth while standing on another planet must instead travel to
                    // Earth's real position, which the body loop below handles.)
                    if (target == environment.observer.observer_body &&
                        environment.planet_surface_visible)
                    {
                        fly.position = SushiEngine::Vector3{0.0, 2.0, 0.0};
                        moved = true;
                    }
                    else
                    {
                        for (int i = 0; i < environment.body_count; ++i)
                        {
                            const SushiEngine::Render::CelestialBody& body =
                                environment.bodies[i];
                            if (static_cast<int>(body.body_id) != target)
                                continue;
                            const SushiEngine::Vector3 body_center =
                                fly.position +
                                body.direction *
                                    static_cast<double>(body.distance_metres);
                            const double stand_off =
                                static_cast<double>(body.mean_radius_metres) * 3.0;
                            fly.position = body_center + body.sun_direction * stand_off;
                            moved = true;
                            break;
                        }
                    }
                    if (moved)
                    {
                        SushiEngine::Astro::fill_environment_sky(
                            environment,
                            SushiEngine::WorldVector3{fly.position.x, fly.position.y,
                                                      fly.position.z});
                        up_retarget_seconds = 1.0e9;
                    }
                }

                // Anchor the scene to whichever body the camera is on: the dominant
                // (near-field) body becomes the observer body, so its own spin and orbit
                // drive the sky and the camera keeps full precision near it — the same on
                // every planet, not just Earth. Once a body is the observer, a camera at rest
                // already tracks it because the scene origin is its surface point. When the
                // dominant body changes, the camera is rebased so it stays put relative to the
                // new ground rather than jumping as the scene re-anchors.
                const int dominant_body = environment.dominant_body_id;
                if (dominant_body >= 0 &&
                    dominant_body != environment.observer.observer_body)
                {
                    const SushiEngine::Vector3 relative_to_body{
                        fly.position.x - environment.dominant_center_metres.x,
                        fly.position.y - environment.dominant_center_metres.y,
                        fly.position.z - environment.dominant_center_metres.z};
                    environment.observer.observer_body = dominant_body;
                    SushiEngine::Astro::fill_environment_sky(
                        environment,
                        SushiEngine::WorldVector3{fly.position.x, fly.position.y,
                                                  fly.position.z});
                    fly.position = SushiEngine::Vector3{
                        environment.dominant_center_metres.x + relative_to_body.x,
                        environment.dominant_center_metres.y + relative_to_body.y,
                        environment.dominant_center_metres.z + relative_to_body.z};
                    SushiEngine::Astro::fill_environment_sky(
                        environment,
                        SushiEngine::WorldVector3{fly.position.x, fly.position.y,
                                                  fly.position.z});
                }
                context.sky_ride_body = dominant_body;
                context.sky_ride_center = environment.dominant_center_metres;

                // Hand the sim the observer it must place free astro bodies against, so a
                // body and the planet it orbits share the same scene frame. Cheap (no world
                // re-extract); the epoch it carries is ignored, the sim owning that clock.
                simulation->set_sky_observer(environment.observer);

                // Proximity-scaled navigation and the local vertical: both come from the
                // nearest body's surface. Motion slows to a controllable crawl on
                // approach, and the camera's up reference is swung toward the radial
                // vertical of that body — so the horizon stays level standing on any
                // hemisphere of any planet, and entering from any side rolls the view
                // smoothly upright instead of arriving "hanging off" the globe.
                double nearest = 0.0;
                SushiEngine::Vector3 nearest_center{0.0, 0.0, 0.0};
                bool have_nearest = false;
                if (environment.planet_surface_visible)
                {
                    nearest_center =
                        SushiEngine::Vector3{environment.planet_center.x,
                                             environment.planet_center.y,
                                             environment.planet_center.z};
                    nearest = SushiEngine::length(fly.position - nearest_center) -
                              environment.planet.mean_radius();
                    have_nearest = true;
                }
                for (int i = 0; i < environment.body_count; ++i)
                {
                    const double body_distance =
                        static_cast<double>(environment.bodies[i].distance_metres);
                    const double surface_distance =
                        body_distance -
                        static_cast<double>(environment.bodies[i].mean_radius_metres);
                    if (!have_nearest || surface_distance < nearest)
                    {
                        nearest = surface_distance;
                        nearest_center =
                            fly.position + environment.bodies[i].direction * body_distance;
                        have_nearest = true;
                    }
                }
                if (nearest < 0.0)
                    nearest = 0.0;
                scene_camera.controller().proximity_distance =
                    static_cast<SushiEngine::Scalar>(nearest);
                if (have_nearest)
                {
                    scene_camera.controller().retarget_up(
                        fly, fly.position - nearest_center,
                        static_cast<SushiEngine::Scalar>(up_retarget_seconds));
                }
            }
            else
            {
                scene_camera.controller().proximity_distance = SushiEngine::Scalar(-1);
                scene_camera.controller().retarget_up(
                    scene_camera.camera(),
                    SushiEngine::Vector3{SushiEngine::Scalar(0), SushiEngine::Scalar(1),
                                         SushiEngine::Scalar(0)},
                    static_cast<SushiEngine::Scalar>(real_delta_seconds));
            }

            bool gizmo_edited = false;
            if (context.panels.scene_view)
            {
                // A surface-anchored entity's world axes are meaningless on a curved planet
                // (world +Y is not "up" except at one point), so its gizmo resolves against
                // the local East-North-Up ground frame — which is exactly the entity's own
                // orientation, since it is stored ground-local and composed onto the tangent
                // frame. Forcing Local there gives the author east/north/up handles; other
                // entities keep the chosen World/Local space.
                const SushiEngine::Authoring::GizmoSpace gizmo_space =
                    (has_selection && world.surface_anchored(context.selected_entity))
                        ? SushiEngine::Authoring::GizmoSpace::Local
                        : context.gizmo_space;
                // Pushed before the draw, so a change made in the Environment panel this
                // frame is the setting the frame is actually rendered with.
                scene_view.set_render_settings(context.render_settings);
                // Unity's Shift+Space maximize, through the same fullscreen state
                // machine the Game view's checkbox drives.
                scene_view.set_fullscreen(context.scene_view_fullscreen);
                SushiEngine::Editor::ViewportFrameInputs scene_inputs;
                scene_inputs.instances = instances.data();
                scene_inputs.instance_count = instances.size();
                scene_inputs.pickable = true;
                scene_inputs.gizmo_target = gizmo_target;
                scene_inputs.gizmo_mode = context.gizmo_mode;
                scene_inputs.gizmo_space = gizmo_space;
                scene_inputs.gizmo_snap = &snap;
                scene_inputs.deformable = deformable.data();
                scene_inputs.deformable_count = deformable.size();
                // The selected cooked collider, drawn over the mesh it came from (§14). Scene
                // view only: the Game view is what the player sees, and a debug overlay there
                // would be showing them the workings.
                scene_inputs.collision_wireframe = &cook_bake_state.collision_wireframe();
                // §14's physics debug draw and the joint gizmo. Scene view only, for the same
                // reason as the collider above, and the world is handed over live rather than
                // snapshotted: a contact list, an island partition and a sleep flag are all
                // this tick's, and a copy would be a second thing to keep in step.
                scene_inputs.physics_world = &world;
                scene_inputs.physics_overlay = context.physics_overlay;
                scene_inputs.selected_entity = context.selected_entity;
                // The selected soft body's interior (§9.3/§9.4, P6-G5), read live off the
                // simulated body. Only when a view is actually on: the read copies every
                // particle and every element out of the physics, which is a per-frame cost
                // worth paying for a view somebody is looking at and not otherwise. Scene
                // view only, for the same reason as the collider above.
                std::vector<SushiEngine::Vector3> soft_body_positions;
                std::vector<SushiEngine::Simulation::SoftBodyElementSample> soft_body_elements;
                if (context.soft_body_debug_view !=
                        SushiEngine::Authoring::SoftBodyDebugView::Off &&
                    simulation != nullptr &&
                    context.selected_entity != SushiEngine::Simulation::NULL_ENTITY)
                {
                    SushiEngine::Simulation::IWorldEditor& soft_world = simulation->world();
                    std::vector<std::uint32_t> unused_indices;
                    if (soft_world.has_soft_body(context.selected_entity) &&
                        soft_world.soft_body_surface(context.selected_entity, soft_body_positions,
                                                     unused_indices) &&
                        soft_world.soft_body_elements(context.selected_entity, soft_body_elements))
                    {
                        scene_inputs.soft_body_positions = &soft_body_positions;
                        scene_inputs.soft_body_elements = &soft_body_elements;
                        scene_inputs.soft_body_material =
                            soft_world.soft_body_parameters(context.selected_entity).material;
                        scene_inputs.soft_body_view = context.soft_body_debug_view;
                    }
                }
                scene_inputs.lights = scene.lights.data();
                scene_inputs.light_count = scene.lights.size();
                scene_inputs.decals = scene.decals.data();
                scene_inputs.decal_count = scene.decals.size();
                scene_inputs.ui_overlay = &scene_ui;
                scene_inputs.show_grid = context.preferences.grid_visible;
                scene_inputs.skeleton_names = false;
                // The Scene view shows the world's own emitters, which are entities. The
                // previewed effect is drawn here only when the author asks for it, since it
                // belongs to nothing and cannot be selected.
                scene_inputs.particle_preview =
                    particle_preview.scene_preview() ? &particle_preview : nullptr;
                scene_inputs.billboards = particle_billboards.data();
                scene_inputs.billboard_count = particle_billboards.size();
                scene_inputs.animated_mesh = &animated_mesh_preview;
                scene_inputs.emitters = scene_emitters;
                scene_inputs.emitter_count = scene_emitter_count;
                scene_inputs.ik_gizmo = true;
                scene_inputs.scene_skinned = scene.skinned_instances.data();
                scene_inputs.scene_skinned_count = scene.skinned_instances.size();
                gizmo_edited = scene_view.draw(context.panels.scene_view, environment, selected,
                                               scene_inputs);
                // The Scene view is the surface the UI is authored against, so its size
                // drives every Canvas's layout — the per-frame equivalent of a window
                // resize event for a full-viewport UI root.
                world.set_ui_target_size(scene_view.target_width(), scene_view.target_height());
                scene_view.render_resolution(context.scene_render_width,
                                             context.scene_render_height);
                scene_view.cull_statistics(context.scene_cull_drawn,
                                           context.scene_cull_tested);
            }
            SushiEngine::Editor::GameViewRenderInputs game_view_render_inputs;
            game_view_render_inputs.has_active_camera = scene.has_camera;
            game_view_render_inputs.has_display = !displays.empty();
            if (context.panels.game_view)
            {
                if (game_view_render_policy.should_render(game_view_render_inputs))
                {
                    // The Game view is played, not authored: no picking, no gizmo. It offers
                    // a display selector so multiple cameras can target different displays.
                    SushiEngine::Editor::DisplaySelector selector;
                    selector.displays = displays.data();
                    selector.count = displays.size();
                    selector.selected = &context.game_display;
                    // Pass no selection so the Scene view's pick never highlights in the
                    // Game view; it is not pickable here so nothing writes this back.
                    std::uint32_t no_selection = 0;
                    game_view.set_render_settings(context.render_settings);
                    SushiEngine::Editor::ViewportFrameInputs game_inputs;
                    game_inputs.instances = instances.data();
                    game_inputs.instance_count = instances.size();
                    game_inputs.pickable = false;
                    game_inputs.display = &selector;
                    game_inputs.deformable = deformable.data();
                    game_inputs.deformable_count = deformable.size();
                    game_inputs.lights = scene.lights.data();
                    game_inputs.light_count = scene.lights.size();
                    game_inputs.decals = scene.decals.data();
                    game_inputs.decal_count = scene.decals.size();
                    game_inputs.ui_overlay = &game_ui;
                    game_inputs.billboards = particle_billboards.data();
                    game_inputs.billboard_count = particle_billboards.size();
                    game_inputs.animated_mesh = &animated_mesh_preview;
                    game_inputs.emitters = scene_emitters;
                    game_inputs.emitter_count = scene_emitter_count;
                    game_inputs.game_view = &context.game_view_settings;
                    game_inputs.scene_skinned = scene.skinned_instances.data();
                    game_inputs.scene_skinned_count = scene.skinned_instances.size();
                    game_view.draw(context.panels.game_view, environment, no_selection,
                                   game_inputs);
                }
                else
                {
                    // No camera to play through (or nothing for it to target): keep the
                    // Game window open with its toolbar, same as Unity's Game view showing
                    // "Display 1 No cameras rendering" instead of just disappearing. The
                    // same panel object draws it, so the fullscreen state machine is shared
                    // with the rendering path instead of duplicated in function statics.
                    game_view.draw_no_camera(context.panels.game_view,
                                             context.game_view_settings);
                }
            }

            if (context.panels.preview)
            {
                // The one preview surface: whatever is being authored, in isolation. No world
                // instances, no world lights, no world emitters — the previewed effect and the
                // previewed character both show here, because "the thing being authored" is a
                // property of this surface rather than a separate window per kind of thing.
                std::uint32_t no_selection = 0;
                preview_view.set_render_settings(context.render_settings);
                SushiEngine::Editor::ViewportFrameInputs preview_inputs;
                preview_inputs.pickable = false;
                preview_inputs.show_grid = context.preferences.grid_visible;
                preview_inputs.skeleton_names = false;
                preview_inputs.particle_preview = &particle_preview;
                preview_inputs.animated_mesh = &animated_mesh_preview;
                preview_inputs.ik_gizmo = true;
                preview_inputs.preview_controls = true;
                preview_view.draw(context.panels.preview, environment, no_selection,
                                  preview_inputs);
            }

            // Copy each visible viewport's per-pass GPU times out for the Statistics
            // panel. Copied because the scene view owns the timing storage only until
            // its next render; gated on visibility so a closed viewport's stale times
            // are not presented as live.
            context.gpu_statistics.clear();
            const struct
            {
                SushiEngine::Editor::ViewportPanel* panel;
                bool visible;
            } profiled_viewports[] = {{&scene_view, context.panels.scene_view},
                                      {&game_view, context.panels.game_view}};
            for (const auto& entry : profiled_viewports)
            {
                if (!entry.visible)
                    continue;
                const std::size_t timing_count = entry.panel->pass_timing_count();
                if (timing_count == 0)
                    continue;
                SushiEngine::Editor::ViewportGPUStatistics statistics;
                statistics.viewport = entry.panel->title();
                statistics.passes.reserve(timing_count);
                for (std::size_t i = 0; i < timing_count; ++i)
                {
                    const SushiEngine::Render::ScenePassTiming timing =
                        entry.panel->pass_timing(i);
                    statistics.passes.push_back({timing.name, timing.milliseconds});
                }
                context.gpu_statistics.push_back(std::move(statistics));
            }

            // Fold the UI overlay's interaction into the shared selection/edit flow: a UI
            // pick in the Scene view replaces the 3D pick this frame, and a UI drag writes
            // the element's new rect back to the world.
            if (scene_ui.picked_id != 0)
                selected = scene_ui.picked_id;
            const bool ui_is_dragging = scene_view.ui_dragging();
            if (ui_is_dragging && !ui_was_dragging)
                context.history.begin_change(world);
            else if (!ui_is_dragging && ui_was_dragging)
                context.history.end_change();
            ui_was_dragging = ui_is_dragging;
            if (scene_ui.edited_index >= 0 &&
                static_cast<std::size_t>(scene_ui.edited_index) < ui_ids.size())
                world.set_ui_parameters(
                    ui_ids[static_cast<std::size_t>(scene_ui.edited_index)],
                    ui_overlay[static_cast<std::size_t>(scene_ui.edited_index)].parameters);

            // One undo step per whole drag, not one per frame: snapshot on the frame
            // the handle is grabbed, commit on the frame it is released.
            const bool gizmo_is_dragging = scene_view.gizmo_dragging();
            if (gizmo_is_dragging && !gizmo_was_dragging)
            {
                context.history.begin_change(world);
                // Snapshot the co-selected entities at the grab, before this frame's
                // write-back, so every snapshot is the pre-drag pose the delta is measured
                // from.
                multi_primary_start = world.world_transform(context.selected_entity);
                multi_drag_start.clear();
                for (const SushiEngine::Simulation::EntityId entity : context.selected_entities)
                    if (entity != context.selected_entity && world.exists(entity))
                        multi_drag_start.push_back({entity, world.world_transform(entity)});
            }
            else if (!gizmo_is_dragging && gizmo_was_dragging)
            {
                context.history.end_change();
                multi_drag_start.clear();
            }
            gizmo_was_dragging = gizmo_is_dragging;

            // Write a gizmo edit back only when the selection did not change this frame
            // (a pick and a drag are mutually exclusive).
            if (has_selection && gizmo_edited &&
                selected == static_cast<std::uint32_t>(context.selected_entity))
            {
                world.set_world_transform(context.selected_entity, selected_transform);
                // Carry the primary's translation delta onto the rest of the selection.
                // Only position is shared here: rotating or scaling a multi-selection about
                // a shared pivot is a separate feature, so co-selected entities keep their
                // own orientation and size.
                const SushiEngine::Vector3 delta =
                    selected_transform.position - multi_primary_start.position;
                for (const auto& entry : multi_drag_start)
                {
                    SushiEngine::Simulation::EntityTransform moved = entry.second;
                    moved.position = entry.second.position + delta;
                    world.set_world_transform(entry.first, moved);
                }
            }
            // A viewport click always replaces the whole selection (no multi-select
            // there yet), but only when it actually changed the pick this frame — this
            // runs every frame regardless, and re-collapsing to one entity every frame
            // would fight the Hierarchy's Ctrl/Shift multi-select.
            if (selected != static_cast<std::uint32_t>(context.selected_entity))
                SushiEngine::Editor::select_only(
                    context, static_cast<SushiEngine::Simulation::EntityId>(selected));
            else
                context.selected_entity = selected;

            // Service the Scene-camera / framing requests raised by the Hierarchy
            // (double-click) and the Entity menu, now that the selection is settled.
            if (world.exists(context.selected_entity) &&
                (context.frame_selected_requested || context.align_with_view_requested ||
                 context.move_to_view_requested))
            {
                const SushiEngine::Simulation::EntityTransform target =
                    world.world_transform(context.selected_entity);
                SushiEngine::Editor::FlyCamera& fly = scene_camera.camera();
                if (context.frame_selected_requested)
                {
                    // Teleport the camera beside the entity, keeping its facing.
                    fly.position =
                        target.position - fly.forward() * static_cast<SushiEngine::Scalar>(6);
                }
                if (context.align_with_view_requested)
                {
                    SushiEngine::Simulation::EntityTransform aligned = target;
                    aligned.position = fly.position;
                    aligned.rotation = fly.orientation();
                    world.set_world_transform(context.selected_entity, aligned);
                }
                if (context.move_to_view_requested)
                {
                    SushiEngine::Simulation::EntityTransform moved = target;
                    moved.position =
                        fly.position + fly.forward() * static_cast<SushiEngine::Scalar>(6);
                    world.set_world_transform(context.selected_entity, moved);
                }
            }
            context.frame_selected_requested = false;
            context.align_with_view_requested = false;
            context.move_to_view_requested = false;
            // Drive the live audio from the world with the Scene camera as the listener,
            // then refresh the profiler telemetry the audio panels display.
            {
                SushiEngine::Editor::FlyCamera& listener_cam = scene_camera.camera();
                audio_system.update(simulation->world(), listener_cam.position,
                                    listener_cam.forward(), listener_cam.up());
                audio_system.poll_profile();
            }
            SushiEngine::Editor::draw_hierarchy_panel(context);
            SushiEngine::Editor::draw_inspector_panel(context);
            SushiEngine::Editor::draw_environment_panel(context);
            SushiEngine::Editor::draw_rendering_panel(context);
            SushiEngine::Editor::draw_lighting_panel(context);
            SushiEngine::Editor::draw_post_process_panel(context);
            SushiEngine::Editor::draw_meteorology_panel(context);
            // The Scene view's terrain, not the Game view's: the two views select and
            // compile independently, and the ground is authored where it is being looked at.
            SushiEngine::Editor::draw_terrain_panel(context, terrain_panel_state,
                                                    scene_view.terrain_authoring());
            SushiEngine::Editor::draw_gpu_culling_panel(context);
            SushiEngine::Editor::draw_project_panel(context);
            SushiEngine::Editor::draw_cooking_override_modal(context);
            SushiEngine::Editor::draw_text_editor_panel(context);
            SushiEngine::Editor::draw_console_panel(context);
            SushiEngine::Editor::draw_statistics_panel(context);
            SushiEngine::Editor::draw_animation_panel(context, animation_state);
            SushiEngine::Editor::draw_animator_graph_panel(context, animator_graph);
            SushiEngine::Editor::draw_animator_preview_panel(context);
            SushiEngine::Editor::draw_audio_mixer_panel(context, audio_system);
            SushiEngine::Editor::draw_audio_profiler_panel(context, audio_system);
            SushiEngine::Editor::draw_audio_authoring_panel(audio_authoring_project, audio_system,
                                                            &context.panels.audio_authoring);
            SushiEngine::Editor::draw_physics_statistics_panel(context);
            SushiEngine::Editor::draw_cook_bake_panel(context, cook_bake_state);
            SushiEngine::Editor::draw_project_picker(context);
            SushiEngine::Editor::draw_vehicle_panel(context, vehicle_authoring);
            SushiEngine::Editor::draw_assembly_panel(context, assembly_authoring);
            SushiEngine::Editor::draw_preferences_window(context);
            SushiEngine::Editor::draw_input_manager_window(context);
            SushiEngine::Editor::draw_save_scene_as_modal(context, running);
            SushiEngine::Editor::draw_exit_confirm_modal(context, running);
            SushiEngine::Editor::draw_scene_action_confirm_modal(context);

            // Copy/Cut/Paste/Duplicate/Delete run here, after every panel has drawn: each
            // of them creates or destroys entities, and the Hierarchy offers them from
            // inside a walk over its own copy of the entity list. Acting where the gesture
            // happens would leave the rest of that walk holding stale ids.
            SushiEngine::Editor::run_pending_entity_command(context);

            // Persist preferences once per frame after any edit, and apply the fields
            // that take effect live (the camera speed; theme is applied on change).
            // Autosave, after the panels ran (so this frame's edits are already in the
            // history revision the dirty check reads). Eligibility rather than a bare
            // timer: an unsaved scene never pops Save-As, a clean scene never rewrites.
            if (autosave_timer.tick(context.preferences.autosave &&
                                        !context.scene_path.empty() &&
                                        SushiEngine::Editor::scene_is_dirty(context),
                                    real_delta_seconds,
                                    context.preferences.autosave_interval_seconds))
            {
                if (SushiEngine::Editor::save_current_scene(context))
                    SushiEngine::Editor::editor_log(context, "Autosaved.");
            }

            if (context.preferences_dirty)
            {
                scene_camera.set_move_speed(context.preferences.camera_move_speed);
                sync_session_preferences();
                preferences_store->save(context.preferences);
                context.preferences_dirty = false;
            }
            if (context.show_imgui_demo)
                ImGui::ShowDemoWindow(&context.show_imgui_demo);

            window.drawable_size(width, height);
            // Present pacing belongs to the window, not to a scene view, so it is applied
            // here rather than through set_settings(); the call is a no-op unless the mode
            // actually changed.
            renderer->set_present_mode(context.render_settings.delivery.present_mode);
            if (void* command_buffer = renderer->begin_frame(width, height))
            {
                imgui.render(command_buffer);
                renderer->end_frame();
            }
            else
            {
                ImGui::EndFrame(); // no frame presented (minimized/resize); close the UI frame
            }
        }

        sync_session_preferences();
        preferences_store->save(context.preferences);
        renderer->wait_idle();
        return 0;
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "SushiEngine editor: %s\n", error.what());
        return 1;
    }
}
