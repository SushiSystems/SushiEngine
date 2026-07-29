/**************************************************************************/
/* scene_commands.cpp                                                    */
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

#include "scene_commands.hpp"

#include "../ui/panel_widgets.hpp"

#include <algorithm>
#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include <imgui.h>

#include <SushiEngine/render/environment.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::EntityId;
        using SushiEngine::Simulation::IWorldEditor;
        using SushiEngine::Simulation::NULL_ENTITY;


        // Shared "Create Empty Entity" / "Camera" / "Objects > Box/Sphere/Cylinder/
        // Terrain" menu items, reused by the Entity menu, the hierarchy's row context
        // menu, its empty-space context menu, and the filtered search view, so all
        // entry points create identically — the Entity menu can never drift out of
        // sync with the Hierarchy's right-click menu. The new entity is selected the
        // same way every other creation path in this file already does.
        void draw_create_object_menu_items(EditorContext& context, IWorldEditor* world)
        {
            if (ImGui::MenuItem("Create Empty Entity", nullptr, false, world != nullptr))
            {
                context.history.record(*world);
                select_only(context, world->create("Entity"));
                editor_log(context, "Created entity 'Entity'.");
            }
            if (ImGui::MenuItem("Camera", nullptr, false, world != nullptr))
            {
                context.history.record(*world);
                select_only(context, world->create_camera("Camera"));
                editor_log(context, "Created camera 'Camera'.");
            }
            if (ImGui::BeginMenu("Objects", world != nullptr))
            {
                if (ImGui::MenuItem("Box"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_box("Box"));
                    editor_log(context, "Created object 'Box'.");
                }
                if (ImGui::MenuItem("Sphere"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_sphere("Sphere"));
                    editor_log(context, "Created object 'Sphere'.");
                }
                if (ImGui::MenuItem("Cylinder"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_cylinder("Cylinder"));
                    editor_log(context, "Created object 'Cylinder'.");
                }
                if (ImGui::MenuItem("Terrain"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_terrain("Terrain"));
                    editor_log(context, "Created object 'Terrain'.");
                }
                if (ImGui::MenuItem("Cloth"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_cloth("Cloth"));
                    editor_log(context, "Created object 'Cloth'.");
                }
                if (ImGui::MenuItem("Particle System"))
                {
                    context.history.record(*world);
                    select_only(context,
                                world->create_particle_emitter("Particle System"));
                    editor_log(context, "Created object 'Particle System'.");
                }
                if (ImGui::MenuItem("Light"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_light("Light"));
                    editor_log(context, "Created object 'Light'.");
                }
                if (ImGui::MenuItem("Decal"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_decal("Decal"));
                    editor_log(context, "Created object 'Decal'.");
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("UI", world != nullptr))
            {
                // Image/Text/Button parent to the selected UI entity (typically a
                // Canvas) so they lay out inside it; with no UI selected they anchor
                // straight to the viewport, still visible and re-parentable later.
                const EntityId ui_parent =
                    world != nullptr && world->has_ui(context.selected_entity)
                        ? context.selected_entity
                        : NULL_ENTITY;
                if (ImGui::MenuItem("Canvas"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_canvas("Canvas"));
                    editor_log(context, "Created UI 'Canvas'.");
                }
                if (ImGui::MenuItem("Panel"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_ui_element(
                                             "Panel", SushiEngine::Simulation::UIElementKind::Panel,
                                             ui_parent));
                    editor_log(context, "Created UI 'Panel'.");
                }
                if (ImGui::MenuItem("Image"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_ui_element(
                                             "Image", SushiEngine::Simulation::UIElementKind::Image,
                                             ui_parent));
                    editor_log(context, "Created UI 'Image'.");
                }
                if (ImGui::MenuItem("Text"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_ui_element(
                                             "Text", SushiEngine::Simulation::UIElementKind::Text,
                                             ui_parent));
                    editor_log(context, "Created UI 'Text'.");
                }
                if (ImGui::MenuItem("Button"))
                {
                    context.history.record(*world);
                    select_only(context, world->create_ui_element(
                                             "Button", SushiEngine::Simulation::UIElementKind::Button,
                                             ui_parent));
                    editor_log(context, "Created UI 'Button'.");
                }
                ImGui::EndMenu();
            }
        }

        // Shared Copy/Cut/Paste menu items, reused everywhere
        // `draw_create_object_menu_items` is (see its comment) so clipboard actions
        // never drift out of sync between the Edit menu and the Hierarchy either.
        void draw_clipboard_menu_items(EditorContext& context, IWorldEditor* world)
        {
            const bool has_selection = world != nullptr && !context.selected_entities.empty();
            using Command = EditorContext::EntityCommand;
            if (menu_item_for_action(context, "Copy", "Copy", has_selection))
                context.pending_entity_command = Command::Copy;
            if (menu_item_for_action(context, "Cut", "Cut", has_selection))
                context.pending_entity_command = Command::Cut;
            if (menu_item_for_action(context, "Paste", "Paste",
                                     world != nullptr && !context.clipboard.empty()))
                context.pending_entity_command = Command::Paste;
            if (menu_item_for_action(context, "Duplicate", "Duplicate", has_selection))
                context.pending_entity_command = Command::Duplicate;
            if (menu_item_for_action(context, "Delete", "Delete", has_selection))
                context.pending_entity_command = Command::Delete;
        }

        void run_pending_entity_command(EditorContext& context)
        {
            using Command = EditorContext::EntityCommand;
            const Command command = context.pending_entity_command;
            context.pending_entity_command = Command::None;
            switch (command)
            {
                case Command::Copy:
                    copy_selection(context);
                    break;
                case Command::Cut:
                    cut_selection(context);
                    break;
                case Command::Paste:
                    paste_clipboard(context);
                    break;
                case Command::Duplicate:
                    duplicate_selection(context);
                    break;
                case Command::Delete:
                    delete_selection(context);
                    break;
                case Command::None:
                    break;
            }
        }

        std::vector<ClipboardEntity> snapshot_selection(EditorContext& context)
        {
            std::vector<ClipboardEntity> out;
            IWorldEditor* world = world_of(context);
            if (world == nullptr || context.selected_entities.empty())
                return out;

            out.reserve(context.selected_entities.size());
            for (const EntityId id : context.selected_entities)
            {
                ClipboardEntity entry;
                entry.original = id;
                entry.original_parent = world->parent(id);
                entry.name = world->name(id);
                entry.transform = world->transform(id);
                entry.color = world->color(id);
                entry.visible = world->visible(id);
                entry.has_renderer = world->has_renderer(id);
                entry.material = world->material(id);
                entry.material_texture_paths = world->material_texture_paths(id);
                entry.is_camera = world->is_camera(id);
                entry.camera_params = world->camera_params(id);
                entry.has_physics_body = world->has_physics_body(id);
                entry.physics_body_params = world->physics_body_params(id);
                entry.has_cloth = world->has_cloth(id);
                entry.cloth_params = world->cloth_params(id);
                entry.has_light = world->has_light(id);
                entry.light_params = world->light_params(id);
                entry.has_decal = world->has_decal(id);
                entry.decal_params = world->decal_params(id);
                entry.has_shape = world->has_shape(id);
                entry.shape_params = world->shape_params(id);
                entry.has_collider = world->has_collider(id);
                entry.collider_params = world->collider_params(id);
                entry.has_particle_emitter = world->has_particle_emitter(id);
                if (entry.has_particle_emitter)
                {
                    entry.particle_emitter_params = world->particle_emitter_params(id);
                    entry.particle_effect = world->particle_effect_source(id);
                }
                entry.has_audio_emitter = world->has_audio_emitter(id);
                entry.audio_emitter_params = world->audio_emitter_params(id);
                entry.has_reverb_zone = world->has_reverb_zone(id);
                entry.reverb_zone_params = world->reverb_zone_params(id);
                entry.has_audio_listener = world->has_audio_listener(id);
                entry.audio_listener_params = world->audio_listener_params(id);
                entry.surface_anchored = world->surface_anchored(id);
                entry.surface_local_orientation = world->surface_local_orientation(id);
                entry.entity_frame = world->entity_frame(id);
                entry.has_ui = world->has_ui(id);
                entry.ui_params = world->ui_params(id);
                for (const std::string& type_name : world->script_components(id))
                    entry.scripts.push_back(world->script_component(id, type_name));
                out.push_back(entry);
            }
            return out;
        }

        void copy_selection(EditorContext& context)
        {
            std::vector<ClipboardEntity> snapshot = snapshot_selection(context);
            if (snapshot.empty())
                return;
            context.clipboard = std::move(snapshot);
            editor_log(context, "Copied " + std::to_string(context.clipboard.size()) + " entit" +
                                     (context.clipboard.size() == 1 ? "y" : "ies") + ".");
        }

        void cut_selection(EditorContext& context)
        {
            IWorldEditor* world = world_of(context);
            if (world == nullptr || context.selected_entities.empty())
                return;

            copy_selection(context);
            context.history.record(*world);
            for (const EntityId id : context.selected_entities)
                world->destroy(id);
            select_only(context, NULL_ENTITY);
            editor_log(context, "Cut " + std::to_string(context.clipboard.size()) + " entit" +
                                     (context.clipboard.size() == 1 ? "y" : "ies") + ".");
        }

        std::vector<EntityId> instantiate_entities(EditorContext& context,
                                                   const std::vector<ClipboardEntity>& source)
        {
            std::vector<EntityId> pasted;
            IWorldEditor* world = world_of(context);
            if (world == nullptr || source.empty())
                return pasted;

            std::unordered_map<EntityId, EntityId> original_to_new;
            pasted.reserve(source.size());

            for (const ClipboardEntity& entry : source)
            {
                const EntityId id = world->create(entry.name);
                world->set_transform(id, entry.transform);
                world->set_color(id, entry.color);
                world->set_visible(id, entry.visible);
                world->set_has_renderer(id, entry.has_renderer);
                world->set_material(id, entry.material);
                world->set_material_texture_paths(id, entry.material_texture_paths);
                world->set_is_camera(id, entry.is_camera);
                if (entry.is_camera)
                    world->set_camera_params(id, entry.camera_params);
                world->set_has_physics_body(id, entry.has_physics_body);
                if (entry.has_physics_body)
                    world->set_physics_body_params(id, entry.physics_body_params);
                world->set_has_cloth(id, entry.has_cloth);
                if (entry.has_cloth)
                    world->set_cloth_params(id, entry.cloth_params);
                world->set_has_light(id, entry.has_light);
                if (entry.has_light)
                    world->set_light_params(id, entry.light_params);
                world->set_has_decal(id, entry.has_decal);
                if (entry.has_decal)
                    world->set_decal_params(id, entry.decal_params);
                world->set_has_shape(id, entry.has_shape);
                if (entry.has_shape)
                    world->set_shape_params(id, entry.shape_params);
                world->set_has_collider(id, entry.has_collider);
                if (entry.has_collider)
                    world->set_collider_params(id, entry.collider_params);
                world->set_has_particle_emitter(id, entry.has_particle_emitter);
                if (entry.has_particle_emitter)
                {
                    world->set_particle_emitter_params(id, entry.particle_emitter_params);
                    world->set_particle_effect_source(id, entry.particle_effect);
                }
                world->set_has_audio_emitter(id, entry.has_audio_emitter);
                if (entry.has_audio_emitter)
                    world->set_audio_emitter_params(id, entry.audio_emitter_params);
                world->set_has_reverb_zone(id, entry.has_reverb_zone);
                if (entry.has_reverb_zone)
                    world->set_reverb_zone_params(id, entry.reverb_zone_params);
                world->set_has_audio_listener(id, entry.has_audio_listener);
                if (entry.has_audio_listener)
                    world->set_audio_listener_params(id, entry.audio_listener_params);
                world->set_entity_frame(id, entry.entity_frame);
                world->set_surface_anchored(id, entry.surface_anchored);
                if (entry.surface_anchored)
                    world->set_surface_local_orientation(id, entry.surface_local_orientation);
                world->set_has_ui(id, entry.has_ui);
                if (entry.has_ui)
                    world->set_ui_params(id, entry.ui_params);
                for (const SushiEngine::Simulation::ScriptComponent& script : entry.scripts)
                    world->add_script_component(id, script);

                original_to_new[entry.original] = id;
                pasted.push_back(id);
            }

            // Second pass: internal parent/child links between copied entities take
            // priority; anything else falls back to the original's external parent
            // (still alive) so a paste-in-place keeps its old spot in the hierarchy.
            for (std::size_t i = 0; i < source.size(); ++i)
            {
                const ClipboardEntity& entry = source[i];
                const EntityId new_id = pasted[i];
                const auto mapped = original_to_new.find(entry.original_parent);
                if (mapped != original_to_new.end())
                    world->set_parent(new_id, mapped->second);
                else if (world->exists(entry.original_parent))
                    world->set_parent(new_id, entry.original_parent);
            }

            context.selected_entity = pasted.empty() ? NULL_ENTITY : pasted.back();
            context.selection_anchor = context.selected_entity;
            context.selected_entities = pasted;
            return pasted;
        }

        void paste_clipboard(EditorContext& context)
        {
            IWorldEditor* world = world_of(context);
            if (world == nullptr || context.clipboard.empty())
                return;
            context.history.record(*world);
            const std::vector<EntityId> pasted = instantiate_entities(context, context.clipboard);
            editor_log(context, "Pasted " + std::to_string(pasted.size()) + " entit" +
                                     (pasted.size() == 1 ? "y" : "ies") + ".");
        }

        void duplicate_selection(EditorContext& context)
        {
            IWorldEditor* world = world_of(context);
            if (world == nullptr || context.selected_entities.empty())
                return;

            // Deliberately not routed through the clipboard: Unity's Ctrl+D does not
            // clobber what the user has copied, and a duplicate that silently replaced it
            // would be a data loss the gesture never advertised.
            const std::vector<ClipboardEntity> snapshot = snapshot_selection(context);
            context.history.record(*world);
            const std::vector<EntityId> made = instantiate_entities(context, snapshot);
            editor_log(context, "Duplicated " + std::to_string(made.size()) + " entit" +
                                     (made.size() == 1 ? "y" : "ies") + ".");
        }

        void delete_selection(EditorContext& context)
        {
            IWorldEditor* world = world_of(context);
            if (world == nullptr || context.selected_entities.empty())
                return;

            const std::size_t removed = context.selected_entities.size();
            context.history.record(*world);
            for (const EntityId id : context.selected_entities)
                world->destroy(id);
            select_only(context, NULL_ENTITY);
            editor_log(context, "Deleted " + std::to_string(removed) + " entit" +
                                     (removed == 1 ? "y" : "ies") + ".");
        }

        SceneSkyState capture_sky_state(const EditorContext& context)
        {
            SceneSkyState sky;
            sky.enabled = context.sky_enabled;
            sky.date = context.sky_date;
            sky.latitude_degrees = context.sky_latitude_degrees;
            sky.longitude_degrees = context.sky_longitude_degrees;
            sky.astronomical_sun = context.sky_astronomical_sun;
            sky.animate = context.sky_animate;
            sky.days_per_second = context.sky_days_per_second;
            sky.accumulated_days = context.sky_accumulated_days;
            return sky;
        }

        namespace
        {
            // The inverse of `capture_sky_state`, applied as a scene is loaded.
            void apply_sky_state(EditorContext& context, const SceneSkyState& sky)
            {
                context.sky_enabled = sky.enabled;
                context.sky_date = sky.date;
                context.sky_latitude_degrees = sky.latitude_degrees;
                context.sky_longitude_degrees = sky.longitude_degrees;
                context.sky_astronomical_sun = sky.astronomical_sun;
                context.sky_animate = sky.animate;
                context.sky_days_per_second = sky.days_per_second;
                context.sky_accumulated_days = sky.accumulated_days;
                // Forces the main loop's date-change detection to re-seek the epoch from
                // the freshly-loaded date rather than keeping whatever scene came before.
                context.sky_authored_start_cache = -1.0;
            }

        } // namespace

        void note_recent_scene(EditorContext& context, const std::string& path)
        {
            // Most-recent-first, deduplicated, capped: the File ▸ Open Recent list.
            std::vector<std::string>& recent = context.preferences.recent_scenes;
            recent.erase(std::remove(recent.begin(), recent.end(), path), recent.end());
            recent.insert(recent.begin(), path);
            constexpr std::size_t RECENT_SCENE_CAP = 10;
            if (recent.size() > RECENT_SCENE_CAP)
                recent.resize(RECENT_SCENE_CAP);
            context.preferences_dirty = true;
        }

        bool save_current_scene(EditorContext& context)
        {
            IWorldEditor* world = world_of(context);
            if (world == nullptr)
                return false;
            if (context.scene_path.empty())
            {
                context.save_scene_as_name = "Scene.sushiscene";
                context.show_save_scene_as = true;
                return false;
            }
            const SceneSkyState sky = capture_sky_state(context);
            if (save_scene(*world, context.scene_path, &sky))
            {
                context.saved_scene_revision = context.history.revision();
                note_recent_scene(context, context.scene_path);
                editor_log(context, "Saved scene '" + context.scene_path + "'.");
                return true;
            }
            editor_log(context, "Failed to save scene '" + context.scene_path + "'.",
                       LogLevel::Error);
            return false;
        }

        // Applies @p environment onto the live world while preserving the channels
        // the runtime owns and re-publishes (the atmosphere forcing pointers and the
        // weather field): an authored environment never carries those, and applying
        namespace
        {
            // one verbatim would sever the running weather from its own simulation.
            void set_environment_preserving_runtime(
                IWorldEditor& world, SushiEngine::Render::Environment environment)
            {
                const SushiEngine::Render::Environment& live = world.environment();
                environment.atmosphere_forcing = live.atmosphere_forcing;
                environment.weather_field = live.weather_field;
                world.set_environment(environment);
            }

            // Wipes the live world back to empty and starts it from the user's default
            // environment. Shared by the immediate path (scene already clean) and
            // `perform_pending_scene_action` (scene was dirty and the unsaved-changes
            // prompt just resolved it).
            void new_scene(EditorContext& context)
            {
                IWorldEditor* world = world_of(context);
                if (world == nullptr)
                    return;
                context.history.record(*world);
                for (const EntityId id : world->entities())
                    world->destroy(id);
                // The one place the preferences' default environment applies: a fresh
                // scene starts from it. A *loaded* scene keeps its own — the environment
                // is scene content, and the old always-override is what made the scene
                // file's environment block write-only.
                set_environment_preserving_runtime(*world,
                                                   context.preferences.default_environment);
                context.scene_path.clear();
                context.saved_scene_revision = context.history.revision();
                select_only(context, NULL_ENTITY);
                editor_log(context, "New scene.");
            }

            // Loads @p path over the live world. Shared by the immediate path and
            // `perform_pending_scene_action`.
            void open_scene(EditorContext& context, const std::string& path)
            {
                IWorldEditor* world = world_of(context);
                if (world == nullptr)
                    return;
                SceneSkyState sky = capture_sky_state(context);
                if (load_scene(*world, path, &sky, context.assets))
                {
                    apply_sky_state(context, sky);
                    context.scene_path = path;
                    context.saved_scene_revision = context.history.revision();
                    note_recent_scene(context, path);
                    select_only(context, NULL_ENTITY);
                    editor_log(context, "Loaded scene '" + path + "'.");
                }
                else
                {
                    editor_log(context, "Failed to load scene '" + path + "'.", LogLevel::Error);
                }
            }

        } // namespace
        // Runs whichever scene replacement was parked in `pending_scene_action`, then
        // clears it. Called once the unsaved-changes prompt (or its absence, when the
        // scene was already clean) has cleared the way.
        void perform_pending_scene_action(EditorContext& context)
        {
            switch (context.pending_scene_action)
            {
                case EditorContext::PendingSceneAction::New:
                    new_scene(context);
                    break;
                case EditorContext::PendingSceneAction::Open:
                    open_scene(context, context.pending_scene_open_path);
                    break;
                case EditorContext::PendingSceneAction::None:
                    break;
            }
            context.pending_scene_action = EditorContext::PendingSceneAction::None;
            context.pending_scene_open_path.clear();
        }

        // Requests a scene replacement (New or Open), deferring to the unsaved-changes
        // prompt when the current scene is dirty rather than discarding it silently.
        void request_new_scene(EditorContext& context)
        {
            if (scene_is_dirty(context))
                context.pending_scene_action = EditorContext::PendingSceneAction::New;
            else
                new_scene(context);
        }

        void request_open_scene(EditorContext& context, const std::string& path)
        {
            if (scene_is_dirty(context))
            {
                context.pending_scene_action = EditorContext::PendingSceneAction::Open;
                context.pending_scene_open_path = path;
            }
            else
            {
                open_scene(context, path);
            }
        }
    } // namespace Editor
} // namespace SushiEngine
