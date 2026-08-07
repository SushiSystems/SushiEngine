/**************************************************************************/
/* inspector_panel.cpp                                                    */
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

#include "inspector_panel.hpp"

#include <SushiEngine/authoring/cook_bake_state.hpp>

#include "../audio/audio_panels.hpp"
#include "../physics/joint_widgets.hpp"
#include "../render/lighting_panel.hpp"
#include "../scripting/script_panel.hpp"
#include "../ui/component_editor.hpp"
#include "../ui/material_inspector.hpp"
#include "../ui/panel_widgets.hpp"
#include "../vfx/particle_panel.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/render/imported_mesh.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::EntityFrame;
        using SushiEngine::Simulation::EntityId;
        using SushiEngine::Simulation::EntityTransform;
        using SushiEngine::Simulation::FrameMode;
        using SushiEngine::Simulation::IWorldEditor;
        using SushiEngine::Simulation::NULL_ENTITY;

        namespace
        {
            /**
             * @brief Every entity an Inspector edit applies to, the primary first.
             *
             * The Hierarchy's multi-selection when there is one, otherwise the primary alone.
             * Primary first because it is the entity whose values are displayed, and a field
             * that shows one entity's number and writes to a different one first would make
             * the order observable.
             *
             * @param context Editor state holding the selection.
             * @param world The world, for discarding ids that no longer exist.
             * @param primary The Inspector's displayed entity.
             * @return The target list, never empty.
             */
            std::vector<EntityId> edit_targets(const EditorContext& context, IWorldEditor& world,
                                               EntityId primary)
            {
                std::vector<EntityId> targets;
                targets.push_back(primary);
                for (const EntityId id : context.selected_entities)
                {
                    if (id == primary || !world.exists(id))
                        continue;
                    targets.push_back(id);
                }
                return targets;
            }

            /**
             * @brief Whether the selection disagrees about a plain per-entity value.
             *
             * For the handful of things that are not a component parameter struct — an
             * entity's visibility, its name colour, its reference frame — where
             * @ref ComponentEditor's pointer-to-member route does not apply.
             *
             * @param targets The entities to compare, primary first.
             * @param read Reads the value from one entity.
             * @param reference The primary's value, which the others are compared against.
             * @return True when at least one other target differs.
             */
            template <typename Read, typename Value>
            bool selection_mixed(const std::vector<EntityId>& targets, Read read,
                                 const Value& reference)
            {
                for (std::size_t i = 1; i < targets.size(); ++i)
                    if (!field_equal(read(targets[i]), reference))
                        return true;
                return false;
            }

            /**
             * @brief What the Renderer header's value actions carry.
             *
             * The Renderer has no single parameter aggregate — its colour is an entity
             * property and its mesh is a separate component — so Copy Values needs a shape of
             * its own. The material is deliberately absent: its texture ids are references
             * the asset library counts, and handing one entity's ids to another would let the
             * first release free a texture the second is still drawing with.
             */
            struct RendererValues
            {
                SushiEngine::Vector3 color{SushiEngine::Vector3{1, 1, 1}};
                bool has_shape = true;
                SushiEngine::Simulation::ShapeParameters shape;
            };

            /** @brief The amber the editor says "this will not do what you meant" in. */
            ImVec4 warning_color() { return ImVec4(1.0f, 0.75f, 0.3f, 1.0f); }

            /**
             * @brief Imports a crowd's skinned mesh and the material that came with it.
             *
             * The character file is where a crowd's geometry *and* its maps are named, so a
             * bind takes both: keeping the previous material would leave texture ids that
             * belong to a different character. The mesh and material are cleared first, so a
             * file that does not import leaves the component visibly unbound and reports why,
             * rather than drawing on with whatever was there before.
             *
             * @param context Editor state; supplies the asset library and the console.
             * @param values  The crowd being authored; its mesh and material are written from
             *                @c mesh_path.
             */
            void bind_crowd_mesh(EditorContext& context,
                                 SushiEngine::Simulation::CrowdParameters& values)
            {
                values.mesh = SushiEngine::Render::INVALID_MESH;
                values.material = SushiEngine::Render::Material{};
                if (context.assets == nullptr || values.mesh_path.empty())
                    return;
                // Skin 0, the one register_crowd_skeleton cooks its rig from: geometry taken
                // from a different skin would be posed by the wrong joints.
                SushiEngine::Render::MeshId meshes[1] = {SushiEngine::Render::INVALID_MESH};
                SushiEngine::Render::Material materials[1]{};
                if (context.assets->load_gltf_skinned_mesh(values.mesh_path.c_str(), 0, meshes,
                                                           materials, 1) == 0)
                {
                    editor_log(context,
                               "No skinned mesh imported from '" + values.mesh_path + "'.",
                               LogLevel::Warning);
                    return;
                }
                values.mesh = meshes[0];
                values.material = materials[0];
            }

            /**
             * @brief Imports a Renderer's mesh from its authored path.
             *
             * Unlike @ref bind_crowd_mesh, the imported material is not adopted onto the entity's
             * own Material: a Shape always has its own authored, serialized Material already, and
             * silently overwriting it on every re-import would make it a value the file cannot
             * actually hold still.
             *
             * @param context Editor state; supplies the asset library and the console.
             * @param values  The Shape being authored; its mesh is written from @c mesh_path.
             */
            void bind_shape_mesh(EditorContext& context,
                                 SushiEngine::Simulation::ShapeParameters& values)
            {
                values.mesh = SushiEngine::Render::INVALID_MESH;
                if (context.assets == nullptr || values.mesh_path.empty())
                    return;
                // The same resolve the scene load uses, joined on the file's own node and
                // primitive indices. A Shape picked by hand names node 0, primitive 0, which
                // is the first thing in the file — the mesh the picker used to hand back.
                values.mesh = SushiEngine::Render::resolve_imported_mesh(
                    *context.assets, values.mesh_path.c_str(), values.source_node,
                    values.primitive);
                if (values.mesh == SushiEngine::Render::INVALID_MESH)
                    editor_log(context, "No mesh imported from '" + values.mesh_path + "'.",
                              LogLevel::Warning);
            }

            /**
             * @brief Draws the partner picker: which body this entity is jointed to.
             *
             * Only entities that carry a Rigid Body are offered, because only they can be an
             * endpoint — an immovable endpoint is a body of zero inverse mass, not a missing
             * one. An entity the author already picked that has since lost its body is still
             * shown as the current value rather than silently reset to None, since resetting
             * it would destroy authoring in response to an edit made somewhere else.
             *
             * @param context   Shared editor state; a combo commits in one frame, so the undo
             *                  step is recorded on the pick rather than through the drag
             *                  bracket the numeric rows use.
             * @param world     The world, for the entity list and their names.
             * @param owner     The entity that owns the joint; never offered as its own partner.
             * @param connected The partner id, edited in place.
             * @return Whether the partner changed this frame.
             */
            bool draw_joint_partner(EditorContext& context, IWorldEditor& world, EntityId owner,
                                    EntityId& connected)
            {
                const std::string current =
                    connected == NULL_ENTITY ? std::string("None") : world.name(connected);
                bool changed = false;
                if (ImGui::BeginCombo("Connected Body", current.c_str()))
                {
                    if (ImGui::Selectable("None", connected == NULL_ENTITY))
                    {
                        connected = NULL_ENTITY;
                        changed = true;
                    }
                    for (const EntityId candidate : world.entities())
                    {
                        if (candidate == owner || !world.has_physics_body(candidate))
                            continue;
                        if (!ImGui::Selectable(world.name(candidate).c_str(),
                                               candidate == connected))
                            continue;
                        connected = candidate;
                        changed = true;
                    }
                    ImGui::EndCombo();
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("The other body this one is attached to. Both ends must "
                                      "have a Rigid Body; pin one by giving it an inverse mass "
                                      "of zero rather than by leaving it out.");
                if (changed)
                    context.history.record(world);
                return changed;
            }
        } // namespace

        void draw_inspector_panel(EditorContext& context)
        {
            draw_new_script_modal(context);
            if (!context.panels.inspector)
                return;
            if (!ImGui::Begin("Inspector", &context.panels.inspector))
            {
                ImGui::End();
                return;
            }

            IWorldEditor* world = world_of(context);
            if (world == nullptr || !world->exists(context.selected_entity))
            {
                ImGui::TextDisabled("Nothing selected.");
                ImGui::End();
                return;
            }

            const EntityId id = context.selected_entity;
            const std::vector<EntityId> targets = edit_targets(context, *world, id);
            const bool multi = targets.size() > 1;

            // Presence is a bool per component, so attaching and detaching across the
            // selection is one shape for all of them — and one undo step, not one per entity.
            const auto set_presence = [&](void (IWorldEditor::*setter)(EntityId, bool),
                                          bool value)
            {
                context.history.record(*world);
                for (const EntityId target : targets)
                    (world->*setter)(target, value);
            };

            bool enabled = world->enabled(id);
            const bool enabled_mixed = selection_mixed(
                targets, [&](EntityId e) { return world->enabled(e); }, enabled);
            if (enabled_mixed)
                ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
            const bool enabled_changed = ImGui::Checkbox("##enabled", &enabled);
            if (enabled_mixed)
                ImGui::PopItemFlag();
            if (enabled_changed)
            {
                context.history.record(*world);
                for (const EntityId target : targets)
                    world->set_enabled(target, enabled);
            }
            ImGui::SameLine();

            bool visible = world->visible(id);
            const bool visible_mixed = selection_mixed(
                targets, [&](EntityId e) { return world->visible(e); }, visible);
            if (visible_mixed)
                ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
            const bool visible_changed = ImGui::Checkbox("##visible", &visible);
            if (visible_mixed)
                ImGui::PopItemFlag();
            if (visible_changed)
            {
                context.history.record(*world);
                for (const EntityId target : targets)
                    world->set_visible(target, visible);
            }
            ImGui::SameLine();
            std::string name = world->name(id);
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::InputText("##name", &name))
                for (const EntityId target : targets)
                    world->set_name(target, name);
            track_item_undo(context, *world);
            if (multi)
                ImGui::TextDisabled("Editing %zu entities: a field you change is written to "
                                    "every selected entity that has it.",
                                    targets.size());
            else
                ImGui::Text("Id: %llu", static_cast<unsigned long long>(id));
            ImGui::Separator();

            // Above Transform, because it says what this entity *is* rather than where it
            // stands, and because an artist who does not know a subtree is a prefab instance
            // will not understand why it changed shape when they opened the scene.
            if (!multi && world->has_prefab_instance(id) &&
                ImGui::CollapsingHeader("Prefab", ImGuiTreeNodeFlags_DefaultOpen))
            {
                const Simulation::PrefabInstanceParameters link = world->prefab_instance(id);
                ImGui::TextDisabled("Source");
                ImGui::SameLine();
                ImGui::TextUnformatted(
                    std::filesystem::path(link.path).filename().string().c_str());
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", link.path.c_str());

                // The revision is shown because it is the only thing that explains why a
                // subtree changed on open: without it an artist cannot tell a refresh from
                // someone else's edit.
                ImGui::TextDisabled("Revision");
                ImGui::SameLine();
                ImGui::TextUnformatted(link.revision.c_str());

                std::error_code prefab_error;
                if (!std::filesystem::exists(link.path, prefab_error))
                    // Unlinked, not broken: the entities are all still here and the link is
                    // still recorded, so restoring the file restores the connection.
                    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f),
                                       "Unlinked: this prefab was not found.");
                ImGui::Separator();
            }

            if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen))
            {
                // Reference row: a celestial body + interpretation mode. When a body is
                // picked the transform below is authored *frame-local* (small metres from the
                // body, ground-local rotation in Surface), the Unity-parent analogue with the
                // body as the parent; "Scene" (-1) is the plain scene transform, unchanged.
                EntityFrame frame = world->entity_frame(id);
                const auto write_frame = [&](const EntityFrame& value)
                {
                    context.history.record(*world);
                    for (const EntityId target : targets)
                        world->set_entity_frame(target, value);
                };
                if (ImGui::BeginTable("reference", 2, ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                    ImGui::TableSetupColumn("value");
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextUnformatted("Reference");
                    ImGui::TableSetColumnIndex(1);

                    const bool body_mixed = selection_mixed(
                        targets,
                        [&](EntityId e) { return world->entity_frame(e).reference_body; },
                        frame.reference_body);
                    const char* current_body =
                        body_mixed
                            ? "-"
                            : (frame.reference_body < 0
                                   ? "Scene"
                                   : SushiEngine::Astro::body_properties(
                                         static_cast<SushiEngine::Astro::BodyId>(
                                             frame.reference_body))
                                         .name);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    if (ImGui::BeginCombo("##reference_body", current_body))
                    {
                        if (ImGui::Selectable("Scene", frame.reference_body < 0))
                        {
                            frame.reference_body = -1;
                            write_frame(frame);
                        }
                        for (int b = 0; b < SushiEngine::Astro::BODY_COUNT; ++b)
                        {
                            const char* body_name =
                                SushiEngine::Astro::body_properties(
                                    static_cast<SushiEngine::Astro::BodyId>(b)).name;
                            if (ImGui::Selectable(body_name, frame.reference_body == b))
                            {
                                frame.reference_body = b;
                                write_frame(frame);
                            }
                        }
                        ImGui::EndCombo();
                    }

                    // Mode is only meaningful once a body is the reference.
                    if (frame.reference_body >= 0)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::AlignTextToFramePadding();
                        ImGui::TextUnformatted("Mode");
                        ImGui::TableSetColumnIndex(1);
                        static const char* const MODES[] = {"Auto", "Free", "Surface"};
                        int mode_index = static_cast<int>(frame.mode);
                        ImGui::SetNextItemWidth(-FLT_MIN);
                        if (ImGui::Combo("##frame_mode", &mode_index, MODES, 3))
                        {
                            frame.mode = static_cast<FrameMode>(mode_index);
                            write_frame(frame);
                        }
                    }
                    ImGui::EndTable();
                }

                // With a body picked, edit the frame-local transform; otherwise the plain
                // scene transform. The widget/write path is identical either way.
                const bool frame_local = frame.reference_body >= 0;
                const auto read_transform = [&](EntityId e)
                {
                    return frame_local ? world->frame_local_transform(e) : world->transform(e);
                };
                EntityTransform transform = read_transform(id);
                // ImGui edits at float precision; the components are engine Scalar, so
                // narrow explicitly into the widget buffers and widen back on write.
                float position[3] = {to_float(transform.position.x),
                                     to_float(transform.position.y),
                                     to_float(transform.position.z)};
                float rotation[3];
                quaternion_to_euler_degrees(transform.rotation, rotation);
                float scale[3] = {to_float(transform.scale.x), to_float(transform.scale.y),
                                  to_float(transform.scale.z)};

                const bool position_mixed = selection_mixed(
                    targets, [&](EntityId e) { return read_transform(e).position; },
                    transform.position);
                const bool scale_mixed = selection_mixed(
                    targets, [&](EntityId e) { return read_transform(e).scale; },
                    transform.scale);
                // Rotation is compared as the Euler triple the row actually shows, not as the
                // quaternion: two quaternions that differ only in sign are the same rotation
                // and must not read as a disagreement.
                const bool rotation_mixed = [&]
                {
                    for (std::size_t i = 1; i < targets.size(); ++i)
                    {
                        float other[3];
                        quaternion_to_euler_degrees(read_transform(targets[i]).rotation, other);
                        if (other[0] != rotation[0] || other[1] != rotation[1] ||
                            other[2] != rotation[2])
                            return true;
                    }
                    return false;
                }();

                // In Surface mode the position is geodetic (lat deg, lon deg, altitude m) —
                // the place-on-a-planet coordinate — so it is shown as three labelled fields
                // rather than an opaque X/Y/Z. Free/Scene keep the Cartesian Position vector.
                const bool surface = frame_local && world->is_surface_frame(id);

                // Which rows changed, so only those are written to the rest of the
                // selection: a shared Position edit must not flatten everyone's Scale.
                bool position_changed = false;
                bool rotation_changed = false;
                bool scale_changed = false;
                if (ImGui::BeginTable("transform", 2, ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                    ImGui::TableSetupColumn("value");
                    if (surface)
                    {
                        position_changed |= scalar_field(
                            context, *world, "Latitude", &position[0], 0.01f, -90.0f, 90.0f,
                            "%.5f deg", position_mixed,
                            "Geodetic latitude on the reference body, north positive.");
                        position_changed |= scalar_field(
                            context, *world, "Longitude", &position[1], 0.01f, -180.0f, 180.0f,
                            "%.5f deg", position_mixed,
                            "Geodetic longitude on the reference body, east positive.");
                        position_changed |= scalar_field(
                            context, *world, "Altitude", &position[2], 0.1f, -1.0e7f, 1.0e9f,
                            "%.2f m", position_mixed,
                            "Height above the reference body's ellipsoid, in metres.");
                    }
                    else
                    {
                        position_changed |=
                            vector3_field(context, *world, "Position", position, 0.05f,
                                          position_mixed, "%.3f m",
                                          frame_local
                                              ? "Offset from the reference body, in metres."
                                              : "Position in scene space, in metres.");
                    }
                    rotation_changed |=
                        vector3_field(context, *world, "Rotation", rotation, 0.5f, rotation_mixed,
                                      "%.2f deg", "Roll, pitch and yaw in degrees.");
                    scale_changed |=
                        vector3_field(context, *world, "Scale", scale, 0.05f, scale_mixed, "%.3f",
                                      "Multiplier on the mesh's own size along each local axis.");
                    ImGui::EndTable();
                }

                if (position_changed || rotation_changed || scale_changed)
                {
                    const SushiEngine::Vector3 new_position{
                        to_scalar(position[0]), to_scalar(position[1]), to_scalar(position[2])};
                    const Quaternion new_rotation = euler_degrees_to_quat(rotation);
                    const SushiEngine::Vector3 new_scale{
                        to_scalar(scale[0]), to_scalar(scale[1]), to_scalar(scale[2])};
                    for (const EntityId target : targets)
                    {
                        EntityTransform edited = read_transform(target);
                        if (position_changed)
                            edited.position = new_position;
                        if (rotation_changed)
                            edited.rotation = new_rotation;
                        if (scale_changed)
                            edited.scale = new_scale;
                        if (frame_local)
                            world->set_frame_local_transform(target, edited);
                        else
                            world->set_transform(target, edited);
                    }
                }
            }

            // Unity-style modular components: Transform above is mandatory on every
            // entity; the rest are independently attached and detached below, each behind
            // the same header — an "x" to remove it and a right-click menu for Reset, Copy
            // Values, Paste Values and Remove — plus an Add Component menu for whichever
            // are currently missing.
            if (world->is_camera(id))
            {
                const ComponentSection section = component_header(context, "Camera");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_is_camera, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::CameraParameters> access{
                        &IWorldEditor::is_camera, &IWorldEditor::camera_parameters,
                        &IWorldEditor::set_camera_parameters};
                    ComponentEditor<SushiEngine::Simulation::CameraParameters> editor(
                        context, *world, access, id);
                    apply_component_section(context, section, "Camera", editor);
                    if (section.open)
                    {
                        // Stored in radians, authored in degrees, like every camera in every
                        // editor; 57.29578 is the conversion the renderer uses too.
                        editor.number("Field of View", &decltype(editor)::Values::vertical_fov_radians,
                                      0.5f, 10.0f, 170.0f, "%.1f deg",
                                      "Vertical angle the frame spans. Wider sees more and "
                                      "distorts more.",
                                      57.29578f);
                        editor.number("Near Clip", &decltype(editor)::Values::near_plane, 0.01f,
                                      0.001f, 10.0f, "%.3f m",
                                      "Nothing closer than this is drawn. Raising it buys "
                                      "depth precision.");
                        editor.number("Far Clip", &decltype(editor)::Values::far_plane, 1.0f, 1.0f,
                                      10000.0f, "%.1f m",
                                      "Nothing beyond this is drawn.");
                        editor.integer("Display", &decltype(editor)::Values::display_index, 0.1f, 0,
                                       15, "Which display this camera drives.");
                        editor.integer("Priority", &decltype(editor)::Values::priority, 0.1f, -999,
                                       999,
                                       "Among active cameras on the same display, the highest "
                                       "priority renders it.");
                        editor.toggle("Active", &decltype(editor)::Values::active,
                                      "An inactive camera is ignored entirely.");
                    }
                }
            }

            if (world->has_renderer(id))
            {
                const ComponentSection section = component_header(context, "Renderer");

                // Reset and Copy/Paste Values carry the colour and the mesh, which is what a
                // Renderer's own authored state amounts to; see @ref RendererValues for why
                // the material stays out of it.
                if (section.copy)
                {
                    RendererValues values;
                    values.color = world->color(id);
                    values.has_shape = world->has_shape(id);
                    if (values.has_shape)
                        values.shape = world->shape_parameters(id);
                    copy_component_values(context, "Renderer", values);
                }
                RendererValues incoming;
                if (section.reset ||
                    (section.paste && paste_component_values(context, "Renderer", incoming)))
                {
                    context.history.record(*world);
                    for (const EntityId target : targets)
                    {
                        world->set_color(target, incoming.color);
                        world->set_has_shape(target, incoming.has_shape);
                        if (incoming.has_shape)
                            world->set_shape_parameters(target, incoming.shape);
                    }
                }

                if (section.remove)
                {
                    // The mesh (Shape) is a feature of the Renderer, so removing the
                    // Renderer takes its mesh with it — a Renderer never lingers as an
                    // invisible component and a mesh never survives without one to draw it.
                    context.history.record(*world);
                    for (const EntityId target : targets)
                    {
                        world->set_has_shape(target, false);
                        world->set_has_renderer(target, false);
                    }
                }
                else if (section.open)
                {
                    const SushiEngine::Vector3 current = world->color(id);
                    float color[3] = {to_float(current.x), to_float(current.y),
                                      to_float(current.z)};
                    const bool color_mixed = selection_mixed(
                        targets, [&](EntityId e) { return world->color(e); }, current);
                    if (ImGui::ColorEdit3("Color", color))
                    {
                        const SushiEngine::Vector3 edited{
                            to_scalar(color[0]), to_scalar(color[1]), to_scalar(color[2])};
                        for (const EntityId target : targets)
                            world->set_color(target, edited);
                    }
                    track_item_undo(context, *world);
                    if (color_mixed)
                    {
                        ImGui::SameLine();
                        ImGui::TextDisabled("mixed");
                    }

                    // The full PBR surface: maps, scalars, and rendering state, laid out
                    // like Unity's Standard shader. Albedo's tint is the Color row above,
                    // which the material editor leaves alone. The texture paths ride with
                    // the ids so the sources survive undo, save, and reload. Authored on the
                    // primary entity only: a material's texture ids are references the asset
                    // library counts, and handing one entity's ids to three more would let
                    // the first release free a texture the others are still drawing with.
                    SushiEngine::Render::Material material = world->material(id);
                    SushiEngine::Simulation::MaterialTexturePaths material_paths =
                        world->material_texture_paths(id);
                    if (context.assets != nullptr &&
                        draw_material_editor(material, material_paths, *context.assets))
                    {
                        context.history.begin_change(*world);
                        world->set_material(id, material);
                        world->set_material_texture_paths(id, material_paths);
                        context.history.end_change();
                    }
                    if (multi)
                        ImGui::TextDisabled("Material is authored on the primary selection only.");

                    // The mesh is the Renderer's own property (Unity's MeshFilter folded
                    // into the MeshRenderer here): its primitive kind and per-kind
                    // dimensions are edited inline, and a Renderer with no mesh yet can be
                    // given one so it starts drawing.
                    if (world->has_shape(id))
                    {
                        const ComponentAccess<SushiEngine::Simulation::ShapeParameters> access{
                            &IWorldEditor::has_shape, &IWorldEditor::shape_parameters,
                            &IWorldEditor::set_shape_parameters};
                        ComponentEditor<SushiEngine::Simulation::ShapeParameters> editor(
                            context, *world, access, id);

                        SushiEngine::Simulation::ShapeParameters& values = editor.mutable_values();
                        bool changed = false;

                        // Reset transient UI state if we've switched to a different entity.
                        if (context.shape_picker_pending_entity != id)
                        {
                            context.shape_picker_pending_entity = id;
                            context.shape_picker_wants_imported = false;
                            context.shape_picker_source_path = values.mesh_path;
                        }

                        // Derive whether we should show the Imported path-entry UI: either the
                        // entity has a loaded mesh, or the user just picked Imported mode.
                        bool imported = values.mesh != SushiEngine::Render::INVALID_MESH ||
                                        context.shape_picker_wants_imported;

                        // Every other selected entity that also carries a Shape: the set a
                        // primitive-kind pick below fans out to, mirroring what
                        // ComponentEditor::choice does for an ordinary field. Gathered here
                        // rather than read off `editor` because leaving Imported mode has to
                        // carry `mesh` and `mesh_path` along with `kind`, which is more than
                        // one field method can address as a single undo step.
                        std::vector<EntityId> shape_targets;
                        for (const EntityId target : targets)
                            if (world->has_shape(target))
                                shape_targets.push_back(target);

                        // Plane is not a drawable mesh (Terrain uses a thin Box), so only the
                        // three solid primitives are offered as the Renderer's mesh. Imported is
                        // a fourth option, but only for a single selection: it hinges on the
                        // transient, entity-scoped "pending import" state above, which has no
                        // sound multi-entity meaning -- each entity's imported mesh is its own
                        // asset choice, not a value a multi-edit should force identical, the
                        // same reasoning the Crowd section's own asset binding follows below. A
                        // primary entity that is already Imported still shows and edits its
                        // Source Mesh row further down even while multi-selected, exactly as
                        // Crowd and Soft Body do for their own assets.
                        static const char* const MESH_NAMES[] = {"Box", "Sphere", "Cylinder",
                                                                 "Imported"};
                        const int option_count = multi ? 3 : 4;

                        // An entity counts as Imported for comparison purposes even when `kind`
                        // still reads as a primitive: importing never touches `kind`, only
                        // `mesh` says whether it is actually in effect.
                        const auto effective_choice =
                            [](const SushiEngine::Simulation::ShapeParameters& v) -> int
                        {
                            return v.mesh != SushiEngine::Render::INVALID_MESH
                                       ? 3
                                       : static_cast<int>(v.kind);
                        };
                        bool kind_mixed = false;
                        for (const EntityId target : shape_targets)
                        {
                            if (target == id)
                                continue;
                            if (effective_choice(world->shape_parameters(target)) !=
                                effective_choice(values))
                            {
                                kind_mixed = true;
                                break;
                            }
                        }

                        const int display_choice = imported ? 3 : static_cast<int>(values.kind);
                        const int clamped_choice =
                            display_choice >= 0 && display_choice < 4 ? display_choice : 0;
                        const char* const preview = kind_mixed ? "-" : MESH_NAMES[clamped_choice];
                        if (ImGui::BeginCombo("Mesh", preview))
                        {
                            for (int option = 0; option < option_count; ++option)
                            {
                                if (!ImGui::Selectable(
                                        MESH_NAMES[option],
                                        !kind_mixed && option == display_choice))
                                    continue;
                                if (option == 3)
                                {
                                    context.shape_picker_wants_imported = true;
                                }
                                else
                                {
                                    // Leaving Imported mode (or just picking a different
                                    // primitive) always clears `mesh` and `mesh_path` together
                                    // with `kind`, on every selected entity with a Shape, as one
                                    // undo step -- `resolve_scene_assets` re-imports from any
                                    // non-empty `mesh_path` unconditionally on load, so a
                                    // leftover path here would silently resurrect Imported mode
                                    // after Save then Load.
                                    context.history.record(*world);
                                    context.shape_picker_wants_imported = false;
                                    context.shape_picker_source_path.clear();
                                    const auto new_kind =
                                        static_cast<SushiEngine::Simulation::PrimitiveKind>(
                                            option);
                                    values.kind = new_kind;
                                    values.mesh = SushiEngine::Render::INVALID_MESH;
                                    values.mesh_path.clear();
                                    imported = false;
                                    changed = true;
                                    for (const EntityId target : shape_targets)
                                    {
                                        if (target == id)
                                            continue;
                                        SushiEngine::Simulation::ShapeParameters target_values =
                                            world->shape_parameters(target);
                                        target_values.kind = new_kind;
                                        target_values.mesh =
                                            SushiEngine::Render::INVALID_MESH;
                                        target_values.mesh_path.clear();
                                        world->set_shape_parameters(target, target_values);
                                    }
                                }
                            }
                            ImGui::EndCombo();
                        }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Which primitive this renderer draws, or "
                                              "Imported for a glTF mesh loaded below.");

                        if (imported)
                        {
                            ImGui::SetNextItemWidth(-80.0f);
                            ImGui::InputText("Source Mesh", &context.shape_picker_source_path);
                            ImGui::SameLine();
                            if (ImGui::Button("Load"))
                            {
                                context.history.record(*world);
                                values.mesh_path = context.shape_picker_source_path;
                                bind_shape_mesh(context, values);
                                changed = true;
                            }
                            if (values.mesh == SushiEngine::Render::INVALID_MESH)
                                ImGui::TextColored(warning_color(),
                                                   "No mesh imported -- this renderer draws "
                                                   "nothing yet.");
                            else
                                ImGui::TextDisabled("Mesh imported.");
                            if (multi)
                                ImGui::TextDisabled("Source Mesh is authored on the primary "
                                                    "selection only.");
                        }
                        else
                        {
                            switch (values.kind)
                            {
                                case SushiEngine::Simulation::PrimitiveKind::Sphere:
                                    if (editor.vector_component(
                                            "Radius##Mesh",
                                            &decltype(editor)::Values::parameters, 0, 0.01f,
                                            0.01f, 1000.0f, "%.3f m",
                                            "Sphere radius before Scale, in metres."))
                                        changed = true;
                                    break;
                                case SushiEngine::Simulation::PrimitiveKind::Cylinder:
                                    if (editor.vector("Radius / Half Height##Mesh",
                                                      &decltype(editor)::Values::parameters, 0.01f,
                                                      0.01f, 1000.0f, "%.3f m",
                                                      "X is the radius, Y the half height, in metres; "
                                                      "Z is unused."))
                                        changed = true;
                                    break;
                                default:
                                    if (editor.vector("Half Extents##Mesh",
                                                      &decltype(editor)::Values::parameters, 0.01f,
                                                      0.01f, 1000.0f, "%.3f m",
                                                      "Half the box's size along each local axis, in "
                                                      "metres, before Scale."))
                                        changed = true;
                                    break;
                            }
                        }

                        if (changed)
                            editor.write_primary();
                    }
                    else if (ImGui::SmallButton("Add Mesh"))
                    {
                        set_presence(&IWorldEditor::set_has_shape, true);
                    }
                }
            }

            if (world->has_physics_body(id))
            {
                const ComponentSection section = component_header(context, "Rigid Body");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_physics_body, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::PhysicsBodyParameters> access{
                        &IWorldEditor::has_physics_body, &IWorldEditor::physics_body_parameters,
                        &IWorldEditor::set_physics_body_parameters};
                    ComponentEditor<SushiEngine::Simulation::PhysicsBodyParameters> editor(
                        context, *world, access, id);
                    apply_component_section(context, section, "Rigid Body", editor);
                    if (section.open)
                    {
                        // First, because ticking it makes every number below it moot: a
                        // kinematic body's mass is not a small quantity or a large one,
                        // it is not a quantity. Reading them greyed out under the reason
                        // is how the panel says so without an error.
                        editor.toggle("Kinematic", &decltype(editor)::Values::kinematic,
                                      "Moved by the game, never by the simulation. It pushes "
                                      "what it meets and nothing pushes back: a lift, a "
                                      "platform, a door on an animation. Drive it by writing "
                                      "this entity's transform each tick.");
                        const bool kinematic = editor.values().kinematic;
                        ImGui::BeginDisabled(kinematic);
                        editor.number("Density", &decltype(editor)::Values::density, 1.0f, 0.0f,
                                      25000.0f, "%.0f kg/m3",
                                      "Above zero, the mass and inertia below are derived from "
                                      "the scaled collider instead of authored. Steel is about "
                                      "7800, oak 700, water 1000.");
                        const bool derived = editor.values().density > SushiEngine::Scalar(0);
                        ImGui::BeginDisabled(derived);
                        editor.number("Inverse Mass", &decltype(editor)::Values::inv_mass, 0.01f,
                                      0.0f, 100.0f, "%.3f 1/kg",
                                      "One over the mass; zero pins the body in place.");
                        editor.vector("Inverse Inertia", &decltype(editor)::Values::inv_inertia,
                                      0.01f, 0.0f, 100.0f, "%.3f",
                                      "Diagonal of the body-local inverse inertia tensor; zero "
                                      "on an axis means no rotation about it.");
                        ImGui::EndDisabled();
                        // Not while kinematic: the density line explains where the two
                        // numbers above came from, and under a kinematic body they did
                        // not come from anywhere — they are not read at all.
                        if (derived && !kinematic)
                            ImGui::TextDisabled("Mass and inertia come from Density and the "
                                                "collider.");
                        editor.number("Drag Coefficient",
                                      &decltype(editor)::Values::drag_coefficient, 0.001f, 0.0f,
                                      100.0f, "%.4f 1/m",
                                      "Quadratic drag k: acceleration -k|v|v per metre. Zero "
                                      "disables it; higher means a lower terminal speed.");
                        ImGui::EndDisabled();
                        if (kinematic)
                            ImGui::TextDisabled("A kinematic body has no mass and takes no "
                                                "gravity or drag.");
                    }
                }
            }

            if (world->has_character(id))
            {
                const ComponentSection section = component_header(context, "Character Controller");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_character, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::CharacterParameters> access{
                        &IWorldEditor::has_character, &IWorldEditor::character_parameters,
                        &IWorldEditor::set_character_parameters};
                    ComponentEditor<SushiEngine::Simulation::CharacterParameters> editor(
                        context, *world, access, id);
                    apply_component_section(context, section, "Character Controller", editor);
                    if (section.open)
                    {
                        // Shape first, limits second, and the separator between them is
                        // the point: the top two say how big the character is and the
                        // rest say what it can get over. Authors reach for them at
                        // different times — one while building it, the other while
                        // finding out where it catches.
                        editor.number("Radius", &decltype(editor)::Values::radius, 0.01f, 0.05f,
                                      5.0f, "%.2f m",
                                      "How wide a gap the character needs to pass through. Not "
                                      "the collider it is hit as — that is authored separately, "
                                      "to match the art.");
                        editor.number("Height", &decltype(editor)::Values::height, 0.01f, 0.1f,
                                      10.0f, "%.2f m", "Total standing height, caps included.");

                        ImGui::SeparatorText("Movement limits");
                        editor.number("Step Height", &decltype(editor)::Values::step_height,
                                      0.01f, 0.0f, 5.0f, "%.2f m",
                                      "The tallest lip it climbs without jumping. Zero means "
                                      "every kerb is a wall.");
                        editor.number("Max Slope",
                                      &decltype(editor)::Values::max_slope_degrees, 0.5f, 0.0f,
                                      89.0f, "%.0f deg",
                                      "Steeper than this is a wall, not a hill — and a "
                                      "character standing on it is not grounded, so it cannot "
                                      "jump off one.");
                        editor.number("Skin Width", &decltype(editor)::Values::skin_width, 0.001f,
                                      0.0f, 0.5f, "%.3f m",
                                      "Clearance kept from every surface. The first dial to "
                                      "reach for when a character catches on geometry: too "
                                      "small and it sticks, too large and it floats.");
                        editor.number("Ground Snap", &decltype(editor)::Values::ground_snap,
                                      0.01f, 0.0f, 2.0f, "%.2f m",
                                      "How far below to look for ground. What keeps it on a "
                                      "downward ramp instead of leaving the floor at every lip.");

                        // Stated rather than enforced. A character with no body is a
                        // half-authored entity and not an error — the two are attached
                        // separately on purpose — but `move_character` refuses a body
                        // that is not kinematic, and an author who is never told that
                        // finds out by watching the character do nothing.
                        if (!world->has_physics_body(id))
                            ImGui::TextDisabled("Needs a Rigid Body with Kinematic ticked.");
                        else if (!world->physics_body_parameters(id).kinematic)
                            ImGui::TextDisabled("The Rigid Body above is not Kinematic, so this "
                                                "character will not move.");
                    }
                }
            }

            if (world->has_impact_response(id))
            {
                const ComponentSection section = component_header(context, "Impact Response");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_impact_response, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::ImpactResponse> access{
                        &IWorldEditor::has_impact_response, &IWorldEditor::impact_response,
                        &IWorldEditor::set_impact_response};
                    ComponentEditor<SushiEngine::Simulation::ImpactResponse> editor(
                        context, *world, access, id);
                    apply_component_section(context, section, "Impact Response", editor);
                    if (section.open)
                    {
                        // The two thresholds first and adjacent, because they are one
                        // idea: where a hit starts counting and where it stops getting
                        // louder.
                        editor.number("Minimum Impulse",
                                      &decltype(editor)::Values::minimum_impulse, 0.05f, 0.0f,
                                      500.0f, "%.2f N*s",
                                      "Below this, nothing happens. Impulse is what separates "
                                      "a scrape from a crash.");
                        editor.number("Full Impulse", &decltype(editor)::Values::full_impulse,
                                      0.5f, 0.0f, 5000.0f, "%.1f N*s",
                                      "At or above this, the response is at full strength. "
                                      "Between the two it ramps linearly.");
                        editor.number("Cooldown", &decltype(editor)::Values::cooldown_seconds,
                                      0.01f, 0.0f, 5.0f, "%.2f s",
                                      "How long before this entity may respond again. Without "
                                      "it a crate that lands and bounces a millimetre sounds "
                                      "twice.");

                        ImGui::SeparatorText("What it does");
                        editor.toggle("Plays Audio", &decltype(editor)::Values::plays_audio,
                                      "Restarts this entity's Audio Emitter, with its gain "
                                      "scaled by the ramp above.");
                        editor.toggle("Emits Particles",
                                      &decltype(editor)::Values::emits_particles,
                                      "Runs this entity's Particle System for a moment.");
                        const bool particles = editor.values().emits_particles;
                        ImGui::BeginDisabled(!particles);
                        editor.number("Particle Time",
                                      &decltype(editor)::Values::particle_seconds, 0.01f, 0.0f,
                                      5.0f, "%.2f s",
                                      "How long the emitter runs after an impact.");
                        ImGui::EndDisabled();

                        // Named rather than silent. Neither is created by this component,
                        // and an author who ticks a box and hears nothing should be told
                        // which half is missing rather than going looking in the audio
                        // settings for a sound that was never asked to play.
                        if (editor.values().plays_audio && !world->has_audio_emitter(id))
                            ImGui::TextDisabled("No Audio Emitter on this entity.");
                        if (particles && !world->has_particle_emitter(id))
                            ImGui::TextDisabled("No Particle System on this entity.");
                    }
                }
            }

            if (world->has_collider(id))
            {
                const ComponentSection section = component_header(context, "Collider");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_collider, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::ColliderParameters> access{
                        &IWorldEditor::has_collider, &IWorldEditor::collider_parameters,
                        &IWorldEditor::set_collider_parameters};
                    ComponentEditor<SushiEngine::Simulation::ColliderParameters> editor(
                        context, *world, access, id);
                    apply_component_section(context, section, "Collider", editor);
                    if (section.open)
                    {
                        static const char* const KIND_NAMES[] = {"Box", "Sphere", "Cylinder",
                                                                 "Plane"};
                        editor.choice("Kind", &decltype(editor)::Values::kind, KIND_NAMES, 4,
                                      "The collision volume's shape, independent of the "
                                      "Renderer's mesh.");
                        if (editor.values().kind ==
                            SushiEngine::Simulation::PrimitiveKind::Plane)
                        {
                            editor.vector("Normal", &decltype(editor)::Values::parameters, 0.01f,
                                          -1.0f, 1.0f, "%.3f",
                                          "The plane's local-space normal; the solid half-space "
                                          "is behind it.");
                        }
                        else
                        {
                            const bool box = editor.values().kind ==
                                             SushiEngine::Simulation::PrimitiveKind::Box;
                            editor.vector(box ? "Half Extents##Collider"
                                              : "Radius / Half Height##Collider",
                                          &decltype(editor)::Values::parameters, 0.01f, 0.01f,
                                          1000.0f, "%.3f m",
                                          box ? "Half the volume's size along each local axis, "
                                                "in metres."
                                              : "X is the radius, Y the half height, in metres.");
                        }

                        // §5.3's surface. Authored on the collider because that is where a
                        // surface is: two crates of the same mesh can be ice and rubber, and
                        // nothing about them is shared but the shape.
                        ImGui::SeparatorText("Surface");
                        editor.number("Static Friction",
                                      &decltype(editor)::Values::static_friction, 0.01f, 0.0f,
                                      2.0f, "%.2f",
                                      "Resistance to a contact that is not yet sliding. Ice is "
                                      "about 0.05, wood on wood 0.5, rubber on asphalt 1.0.");
                        editor.number("Dynamic Friction",
                                      &decltype(editor)::Values::dynamic_friction, 0.01f, 0.0f,
                                      2.0f, "%.2f",
                                      "Resistance once it is already sliding; at or a little "
                                      "below the static value for a real surface.");
                        editor.number("Restitution", &decltype(editor)::Values::restitution, 0.01f,
                                      0.0f, 1.0f, "%.2f",
                                      "How much closing speed comes back. Zero is a sandbag, "
                                      "0.8 a superball. Below the anti-jitter threshold a "
                                      "resting body does not bounce whatever this says.");
                        static const char* const COMBINE_NAMES[] = {"Average", "Minimum",
                                                                     "Multiply", "Maximum"};
                        editor.choice("Friction Combine",
                                      &decltype(editor)::Values::friction_combine, COMBINE_NAMES,
                                      4,
                                      "Both bodies have an opinion and the pair needs one "
                                      "number. The stricter of the two modes wins, so a "
                                      "surface that insists on Minimum cannot be overruled "
                                      "into grip by whatever it touches.");
                        editor.choice("Restitution Combine",
                                      &decltype(editor)::Values::restitution_combine,
                                      COMBINE_NAMES, 4,
                                      "Maximum by default, deliberately: the mean of a "
                                      "superball and concrete is neither, and an author who "
                                      "made one object bouncy expects it to bounce.");

                        // §14 asks for a Physics Material preview — "a ball dropped on a
                        // ramp at the authored friction and restitution". These are that
                        // scene's *answers*, derived rather than simulated, and the reason
                        // is that they are exact: the angle at which an object begins to
                        // slide is `atan(static friction)` and the height it returns to is
                        // `restitution squared`, so a simulated ramp could only reproduce
                        // these two numbers with noise on them. It would also need its own
                        // render target and its own physics world, and neither buys an
                        // author anything the two rows below do not already say.
                        //
                        // The same dimmed derived-column convention the Vehicle window
                        // established: a coefficient nobody can picture, next to the
                        // consequence everybody can.
                        const double slide_degrees =
                            std::atan(double(editor.values().static_friction)) * 57.2957795;
                        const double bounce = double(editor.values().restitution) *
                                              double(editor.values().restitution);
                        ImGui::TextDisabled("Slides on a ramp past %.1f deg", slide_degrees);
                        if (bounce > 0.0)
                            ImGui::TextDisabled("Returns to %.0f%% of the height it fell from",
                                                bounce * 100.0);
                        else
                            ImGui::TextDisabled("Does not bounce");

                        // Trigger and continuous collision: both bits the solver reads off
                        // `Collider::flags`.
                        ImGui::SeparatorText("Behaviour");
                        editor.toggle("Trigger", &decltype(editor)::Values::trigger,
                                      "Reports overlaps as ContactEvent::trigger instead of "
                                      "resolving them. The body has no collision response at "
                                      "all — it passes through everything and everything "
                                      "passes through it.");
                        editor.toggle("Continuous Collision",
                                      &decltype(editor)::Values::continuous_collision,
                                      "Opts in to conservative-advancement sweeps so a fast, "
                                      "thin body cannot tunnel through a thin static collider "
                                      "in one substep. Costs extra broadphase work; leave off "
                                      "unless this body is small and fast.");

                        // §7.7's filter. Two bodies interact only when *each* one's layer is
                        // in the other's mask, which is what makes the relation symmetric by
                        // construction — and what makes a one-sided exclusion do nothing.
                        ImGui::SeparatorText("Collision Filter");
                        editor.integer("Layer", &decltype(editor)::Values::layer, 0.1f, 0, 31,
                                       "Which layer this body is in. One layer per body: "
                                       "layer 0 is where everything unauthored lands and "
                                       "layer 1 is the engine's own cloth layer.");
                        std::uint32_t mask = editor.values().collides_with;
                        bool mask_changed = false;
                        if (ImGui::TreeNode("Collides With"))
                        {
                            for (int bit = 0; bit < 32; ++bit)
                            {
                                if (bit % 8 != 0)
                                    ImGui::SameLine();
                                ImGui::PushID(bit);
                                bool on = (mask & (std::uint32_t(1) << bit)) != 0;
                                if (ImGui::Checkbox("##layer", &on))
                                {
                                    mask = on ? (mask | (std::uint32_t(1) << bit))
                                              : (mask & ~(std::uint32_t(1) << bit));
                                    mask_changed = true;
                                }
                                if (ImGui::IsItemHovered())
                                    ImGui::SetTooltip("Layer %d", bit);
                                ImGui::PopID();
                            }
                            ImGui::TreePop();
                        }
                        if (mask_changed)
                        {
                            context.history.record(*world);
                            SushiEngine::Simulation::ColliderParameters updated = editor.values();
                            updated.collides_with = mask;
                            editor.write_all(updated);
                        }
                    }
                }
            }

            if (world->has_joint(id))
            {
                const ComponentSection section = component_header(context, "Physics Joint");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_joint, false);
                }
                else
                {
                    using SushiEngine::Simulation::JointState;
                    using SushiEngine::Simulation::PhysicsJointParameters;

                    const ComponentAccess<PhysicsJointParameters> access{
                        &IWorldEditor::has_joint, &IWorldEditor::joint_parameters,
                        &IWorldEditor::set_joint_parameters};
                    ComponentEditor<PhysicsJointParameters> editor(context, *world, access, id);
                    apply_component_section(context, section, "Physics Joint", editor);
                    if (section.open)
                    {
                        // Edited through `mutable_values`/`write_primary` — the escape hatch
                        // ComponentEditor documents — rather than through its pointer-to-
                        // member field methods, which address one level. That is not merely
                        // a workaround here: fanning a joint out across a selection would
                        // attach every selected entity to the *same* partner at the *same*
                        // anchor, which is never what an author means. A joint is authored
                        // per entity because both of its endpoints are.
                        PhysicsJointParameters& values = editor.mutable_values();
                        bool changed =
                            draw_joint_partner(context, *world, id, values.connected_body);

                        changed |= draw_joint_parameters(context, *world, values.joint);

                        if (changed)
                            editor.write_primary();

                        // The live half. Three states an author has to be able to tell apart,
                        // because all three look like "no load" from a number alone.
                        ImGui::Separator();
                        JointState load;
                        if (world->joint_broken(id))
                        {
                            ImGui::TextColored(warning_color(),
                                               "Broken. Edit any field above to put it back.");
                        }
                        else if (world->joint_load(id, load))
                        {
                            ImGui::Text("Load %.1f N, %.1f N.m", double(length(load.force)),
                                        double(length(load.torque)));
                            ImGui::TextDisabled("Peak this tick %.1f N, %.1f N.m",
                                                double(load.peak_force),
                                                double(load.peak_torque));
                            if (values.joint.break_force > SushiEngine::Scalar(0))
                                ImGui::TextDisabled(
                                    "%.0f%% of its break force",
                                    100.0 * double(load.peak_force / values.joint.break_force));
                        }
                        else if (values.connected_body == NULL_ENTITY)
                        {
                            ImGui::TextDisabled("Not connected: pick a body above.");
                        }
                        else if (!world->has_physics_body(id) ||
                                 !world->has_physics_body(values.connected_body))
                        {
                            ImGui::TextColored(warning_color(),
                                               "Both ends need a Rigid Body before this holds "
                                               "anything.");
                        }
                        else
                        {
                            ImGui::TextDisabled("Live on the next step.");
                        }
                    }
                }
            }

            if (world->has_vehicle(id))
            {
                // No `ComponentEditor` here: a vehicle's authoring is a path plus a large
                // nested setup the Vehicle window owns, and fanning one across a selection
                // would build every selected entity into the same car. This section says
                // what is on the entity and sends the author to the window that edits it.
                const ComponentSection section = component_header(context, "Vehicle", false);
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_vehicle, false);
                }
                else if (section.open)
                {
                    const SushiEngine::Simulation::VehicleInstanceParameters parameters =
                        world->vehicle_parameters(id);
                    if (parameters.asset_path.empty())
                        ImGui::TextDisabled("No structure named yet.");
                    else
                        ImGui::TextWrapped("%s", parameters.asset_path.c_str());

                    SushiEngine::Simulation::VehicleReport report;
                    if (world->vehicle_report(id, report))
                        ImGui::Text("%.0f rpm, %d parts off",
                                    double(report.engine_rate) * 9.5493,
                                    int(report.parts_detached));
                    else if (!parameters.asset_path.empty())
                        ImGui::TextColored(warning_color(),
                                           "The structure did not load as a node-beam asset.");

                    if (ImGui::Button("Open Vehicle Window"))
                        context.panels.vehicle = true;
                }
            }

            if (world->surface_anchored(id))
            {
                // No authored fields of its own, so the header offers no value actions:
                // attaching the component *is* the whole setting.
                const ComponentSection section =
                    component_header(context, "Surface Anchor", false);
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_surface_anchored, false);
                }
                else if (section.open)
                {
                    ImGui::TextWrapped(
                        "Orientation is ground-local: the East-North-Up frame on the "
                        "dominant body is composed onto it each step, so the entity stays "
                        "upright anywhere on the planet. Rotate it to face along the "
                        "local horizon.");
                }
            }

            if (world->has_cloth(id))
            {
                const ComponentSection section = component_header(context, "Cloth");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_cloth, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::ClothParameters> access{
                        &IWorldEditor::has_cloth, &IWorldEditor::cloth_parameters,
                        &IWorldEditor::set_cloth_parameters};
                    ComponentEditor<SushiEngine::Simulation::ClothParameters> editor(
                        context, *world, access, id);
                    apply_component_section(context, section, "Cloth", editor);
                    if (section.open)
                    {
                        editor.integer("Rows", &decltype(editor)::Values::rows, 0.1f, 1, 64,
                                       "Grid rows; row 0 is pinned.");
                        editor.integer("Columns", &decltype(editor)::Values::cols, 0.1f, 1, 64,
                                       "Grid columns.");
                        editor.number("Spacing", &decltype(editor)::Values::spacing, 0.01f, 0.01f,
                                      10.0f, "%.3f m",
                                      "Rest distance between neighbouring grid points, in "
                                      "metres.");
                        editor.number("Compliance", &decltype(editor)::Values::compliance,
                                      0.0001f, 0.0f, 1.0f, "%.5f m/N",
                                      "Inverse stiffness of every constraint; zero is "
                                      "inextensible, higher stretches.");
                    }
                }
            }

            if (world->has_soft_body(id))
            {
                // §16.45.2: the Inspector section over `SoftBodyParameters`, which the solver
                // reads as thoroughly as `ClothParameters`. Without it the only way to put one
                // on an entity is `IWorldEditor::create_soft_body`.
                const ComponentSection section = component_header(context, "Soft Body");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_soft_body, false);
                }
                else
                {
                    using SushiEngine::Simulation::SoftBodyParameters;

                    const ComponentAccess<SoftBodyParameters> access{
                        &IWorldEditor::has_soft_body, &IWorldEditor::soft_body_parameters,
                        &IWorldEditor::set_soft_body_parameters};
                    ComponentEditor<SoftBodyParameters> editor(context, *world, access, id);
                    apply_component_section(context, section, "Soft Body", editor);
                    if (section.open)
                    {
                        // Edited through `mutable_values`/`write_primary`, the same escape
                        // hatch the Physics Joint section uses and for the same reason: the
                        // cooked asset bytes are this entity's own body, not a setting a
                        // multi-selection should be fanned the same copy of.
                        SoftBodyParameters& values = editor.mutable_values();
                        bool changed = false;

                        if (values.asset.empty())
                            ImGui::TextColored(warning_color(),
                                               "No cooked asset -- this body simulates nothing "
                                               "yet.");
                        else
                            ImGui::TextDisabled("Cooked asset loaded (%zu bytes).",
                                                values.asset.size());

                        // A live readout rather than an authored field: the largest von
                        // Mises stress any element in this body carried at the end of the
                        // last completed tick (§9.3). It sits beside the cook status because
                        // both answer "what is this body doing now" rather than "what was it
                        // set to", and it is read against the Yield stress authored further
                        // down — which is the comparison the number exists for.
                        const Scalar peak_stress = world->soft_body_maximum_stress(id);
                        ImGui::TextDisabled("Measured peak stress %.3e Pa (last tick)",
                                            double(peak_stress));
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Read off the simulated body every frame, not "
                                              "authored. Zero until the body has been built "
                                              "and stepped.");
                        if (values.material.yield_stress > Scalar(0) &&
                            peak_stress > values.material.yield_stress)
                            ImGui::TextColored(warning_color(),
                                               "Past yield: elements are taking permanent "
                                               "strain.");

                        ImGui::SetNextItemWidth(-80.0f);
                        ImGui::InputText("Source Mesh", &context.soft_body_source_path);
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("The path a mesh was cooked from with 'Cook soft "
                                              "body' on in the Bake panel -- the same path the "
                                              "Project panel shows.");
                        ImGui::SameLine();
                        if (ImGui::Button("Load") && context.cook_bake_state != nullptr)
                        {
                            const Authoring::BakedAssetEntry* entry =
                                context.cook_bake_state->entry(context.soft_body_source_path);
                            if (entry != nullptr && entry->has_soft_body())
                            {
                                values.asset = entry->soft_body_bytes;
                                changed = true;
                            }
                            else
                            {
                                editor_log(context,
                                          "No cooked soft-body asset at '" +
                                              context.soft_body_source_path +
                                              "' -- cook it with 'Cook soft body' on in the "
                                              "Bake panel first.",
                                          LogLevel::Warning);
                            }
                        }

                        ImGui::SetNextItemWidth(180.0f);
                        int level = int(values.level);
                        if (ImGui::DragInt("Level", &level, 0.05f, 0, 8))
                        {
                            values.level = std::uint32_t(level < 0 ? 0 : level);
                            changed = true;
                        }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Which cooked simulation level to build; 0 is "
                                              "finest.");

                        ImGui::SeparatorText("Material");
                        static const char* const PRESETS[] = {"Apply preset...", "Rubber",
                                                               "Foam",           "Soft tissue",
                                                               "Sheet steel",    "Aluminium"};
                        int preset = 0;
                        if (ImGui::Combo("Preset", &preset, PRESETS, IM_ARRAYSIZE(PRESETS)))
                        {
                            switch (preset)
                            {
                                case 1:
                                    values.material = Physics::rubber_material<Scalar>();
                                    break;
                                case 2:
                                    values.material = Physics::foam_material<Scalar>();
                                    break;
                                case 3:
                                    values.material = Physics::soft_tissue_material<Scalar>();
                                    break;
                                case 4:
                                    values.material = Physics::sheet_steel_material<Scalar>();
                                    break;
                                case 5:
                                    values.material = Physics::aluminium_material<Scalar>();
                                    break;
                                default: break;
                            }
                            changed = changed || preset != 0;
                        }
                        float young_modulus = float(values.material.young_modulus);
                        if (ImGui::InputFloat("Young's modulus", &young_modulus, 0.0f, 0.0f,
                                              "%.3e Pa"))
                        {
                            values.material.young_modulus = Scalar(young_modulus);
                            changed = true;
                        }
                        float poisson_ratio = float(values.material.poisson_ratio);
                        if (ImGui::DragFloat("Poisson ratio", &poisson_ratio, 0.005f, -0.999f,
                                             0.499f, "%.3f"))
                        {
                            values.material.poisson_ratio = Scalar(poisson_ratio);
                            changed = true;
                        }
                        float density = float(values.material.density);
                        if (ImGui::DragFloat("Density", &density, 5.0f, 1.0f, 20000.0f,
                                             "%.0f kg/m^3"))
                        {
                            values.material.density = Scalar(density);
                            changed = true;
                        }
                        float damping = float(values.material.damping);
                        if (ImGui::DragFloat("Damping", &damping, 0.01f, 0.0f, 5.0f, "%.3f"))
                        {
                            values.material.damping = Scalar(damping);
                            changed = true;
                        }
                        float yield_stress = float(values.material.yield_stress);
                        if (ImGui::InputFloat("Yield stress", &yield_stress, 0.0f, 0.0f,
                                              "%.3e Pa"))
                        {
                            values.material.yield_stress = Scalar(yield_stress);
                            changed = true;
                        }
                        float plastic_creep = float(values.material.plastic_creep);
                        if (ImGui::DragFloat("Plastic creep", &plastic_creep, 0.005f, 0.0f, 1.0f,
                                             "%.3f"))
                        {
                            values.material.plastic_creep = Scalar(plastic_creep);
                            changed = true;
                        }
                        float maximum_plastic_strain = float(values.material.maximum_plastic_strain);
                        if (ImGui::DragFloat("Maximum plastic strain", &maximum_plastic_strain,
                                             0.005f, 0.0f, 2.0f, "%.3f"))
                        {
                            values.material.maximum_plastic_strain = Scalar(maximum_plastic_strain);
                            changed = true;
                        }
                        float fracture_stress = float(values.material.fracture_stress);
                        if (ImGui::InputFloat("Fracture stress", &fracture_stress, 0.0f, 0.0f,
                                              "%.3e Pa"))
                        {
                            values.material.fracture_stress = Scalar(fracture_stress);
                            changed = true;
                        }

                        ImGui::SeparatorText("Surface");
                        float thickness = float(values.thickness);
                        if (ImGui::DragFloat("Thickness", &thickness, 0.001f, 0.0f, 1.0f,
                                             "%.3f m"))
                        {
                            values.thickness = Scalar(thickness);
                            changed = true;
                        }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Contact half-width of the surface.");
                        if (ImGui::Checkbox("Self Collision", &values.self_collision))
                            changed = true;
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Whether the surface is tested against itself.");
                        if (ImGui::Checkbox("Cosmetic", &values.cosmetic))
                            changed = true;
                        if (ImGui::IsItemHovered())
                        {
                            ImGui::SetTooltip("Simulated in float instead of double when nothing "
                                              "replays this body deterministically. A body "
                                              "inside the deterministic island is simulated in "
                                              "double however loudly this asks -- see §6.5.");
                        }

                        if (changed)
                            editor.write_primary();
                    }
                }
            }

            if (world->has_crowd(id))
            {
                // §12.3/§12.4's device-batched skinned character. Its three assets are bound
                // here and nowhere else: the extract skips any crowd whose skeleton or mesh
                // is unset, so without this section the component can only ever draw nothing.
                const ComponentSection section = component_header(context, "Crowd");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_crowd, false);
                }
                else
                {
                    using SushiEngine::Simulation::CrowdParameters;

                    const ComponentAccess<CrowdParameters> access{
                        &IWorldEditor::has_crowd, &IWorldEditor::crowd_parameters,
                        &IWorldEditor::set_crowd_parameters};
                    ComponentEditor<CrowdParameters> editor(context, *world, access, id);

                    // The header's value actions carry the playback state but never the bound
                    // assets, on the Decal's reasoning: a mesh id and a material's maps are
                    // references the asset library counts, and handing one entity's ids to
                    // another would let the first release free what the second is still
                    // drawing with. The files stay each crowd's own.
                    if (section.reset || section.paste)
                    {
                        CrowdParameters source;
                        const bool have =
                            section.reset || paste_component_values(context, "Crowd", source);
                        if (have)
                        {
                            CrowdParameters merged = editor.values();
                            merged.time_seconds = source.time_seconds;
                            merged.loop = source.loop;
                            merged.playing = source.playing;
                            editor.write_all(merged);
                        }
                    }
                    if (section.copy)
                        copy_component_values(context, "Crowd", editor.values());

                    if (section.open)
                    {
                        // The surface rows below carry a material's field labels, which the
                        // Renderer's own material editor already uses in this same window —
                        // and an ImGui id is the label, not the header it sits under.
                        ImGui::PushID("Crowd");
                        CrowdParameters& values = editor.mutable_values();
                        bool changed = false;

                        if (values.skeleton == 0)
                            ImGui::TextColored(warning_color(),
                                               "No skeleton bound -- this crowd draws "
                                               "nothing.");
                        else if (values.clip == 0)
                            ImGui::TextColored(warning_color(),
                                               "No clip bound -- this crowd draws nothing.");
                        else if (values.mesh == SushiEngine::Render::INVALID_MESH)
                            ImGui::TextColored(warning_color(),
                                               "No skinned mesh bound -- this crowd draws "
                                               "nothing.");
                        else
                            ImGui::TextDisabled("Bound: skeleton %u, clip %u, mesh %u.",
                                                unsigned(values.skeleton),
                                                unsigned(values.clip), unsigned(values.mesh));

                        // One asset row: type a path and press Enter, press Bind, or drop a
                        // file from the Project browser. The typed path commits on one of
                        // those three rather than per keystroke, because a component that
                        // took every prefix of a path would ask the registry to open each one
                        // and would unbind a working rig in the middle of a rename.
                        const auto asset_row = [&](const char* label, std::string& stored,
                                                   const char* tooltip)
                        {
                            ImGui::PushID(label);
                            char buffer[512] = {};
                            stored.copy(buffer, sizeof(buffer) - 1);
                            ImGui::SetNextItemWidth(-140.0f);
                            const bool enter =
                                ImGui::InputText(label, buffer, sizeof(buffer),
                                                 ImGuiInputTextFlags_EnterReturnsTrue);
                            // Read now, shown at the end of the row: a tooltip opens a window
                            // of its own, and drawing one between the field and its drop
                            // target would leave the target attached to the tooltip's text.
                            const bool hovered = ImGui::IsItemHovered();
                            std::string dropped;
                            const bool drop = accept_asset_drop(dropped);
                            ImGui::SameLine();
                            const bool bind = ImGui::SmallButton("Bind");
                            const bool commit = enter || bind || drop;
                            if (commit)
                            {
                                context.history.record(*world);
                                stored = drop ? dropped : std::string(buffer);
                                changed = true;
                            }
                            if (hovered)
                                ImGui::SetTooltip("%s", tooltip);
                            ImGui::PopID();
                            return commit;
                        };

                        asset_row("Skeleton", values.skeleton_path,
                                  "glTF naming the rig. Every crowd drawn in one frame shares "
                                  "one skeleton; the rest are skipped that frame. Enter or "
                                  "Bind applies the path.");
                        asset_row("Clip", values.clip_path,
                                  "glTF naming the animation this character plays. Its joint "
                                  "order must match the skeleton above.");
                        if (asset_row("Skinned Mesh", values.mesh_path,
                                      "glTF naming the skinned geometry. Binding it also takes "
                                      "that file's material, replacing the surface below."))
                            bind_crowd_mesh(context, values);

                        editor.number("Time", &decltype(editor)::Values::time_seconds, 0.01f,
                                      0.0f, 3600.0f, "%.3f s",
                                      "Playback position in the clip; the fixed tick advances "
                                      "it while Playing.");
                        editor.toggle("Loop", &decltype(editor)::Values::loop,
                                      "Whether playback wraps at the clip's end instead of "
                                      "holding its last pose.");
                        editor.toggle("Playing", &decltype(editor)::Values::playing,
                                      "Whether the fixed tick advances Time.");

                        ImGui::SeparatorText("Surface");
                        ImGui::TextDisabled("The maps come from the character file; these tune "
                                            "the shading over them.");
                        float albedo[3] = {float(values.material.albedo.x),
                                           float(values.material.albedo.y),
                                           float(values.material.albedo.z)};
                        if (ImGui::ColorEdit3("Albedo", albedo))
                        {
                            values.material.albedo = Vector3{
                                Scalar(albedo[0]), Scalar(albedo[1]), Scalar(albedo[2])};
                            changed = true;
                        }
                        if (ImGui::SliderFloat("Metallic", &values.material.metallic, 0.0f,
                                               1.0f, "%.2f"))
                            changed = true;
                        if (ImGui::SliderFloat("Roughness", &values.material.roughness, 0.0f,
                                               1.0f, "%.2f"))
                            changed = true;
                        if (ImGui::Checkbox("Cast Shadows", &values.material.cast_shadows))
                            changed = true;

                        if (changed)
                        {
                            // Primary only, like the Decal's maps: a rig, a clip and a mesh
                            // are what this entity *is*, not a setting to fan across a
                            // selection. The world re-registers the skeleton and clip from
                            // their paths as it takes the write, so the handles shown above
                            // are read back from it rather than guessed here.
                            editor.write_primary();
                            values = world->crowd_parameters(id);
                        }
                        if (multi)
                            ImGui::TextDisabled("Assets and surface are authored on the "
                                                "primary selection only.");
                        ImGui::PopID();
                    }
                }
            }

            if (world->has_particle_emitter(id))
            {
                const ComponentSection section = component_header(context, "Particle System");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_particle_emitter, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::ParticleEmitterParameters>
                        access{&IWorldEditor::has_particle_emitter,
                               &IWorldEditor::particle_emitter_parameters,
                               &IWorldEditor::set_particle_emitter_parameters};
                    ComponentEditor<SushiEngine::Simulation::ParticleEmitterParameters> editor(
                        context, *world, access, id);
                    apply_component_section(context, section, "Particle System", editor);
                    if (section.open)
                    {
                        // No effect picker: the effect belongs to this entity and is authored
                        // below.
                        editor.integer("Seed", &decltype(editor)::Values::seed, 1.0f, 0, 1000000,
                                       "Seeds the deterministic random stream, so two emitters "
                                       "with the same seed play identically.");
                        editor.toggle("Playing", &decltype(editor)::Values::playing,
                                      "Whether the emitter is spawning particles.");

                        // The whole authoring surface lives in the component that owns it: a
                        // particle system is not a mode the editor is in, it is something an
                        // entity has. Authored on the primary entity only — the effect is a
                        // per-entity asset, not a field.
                        draw_particle_system_component(context, *world, id);
                        if (multi)
                            ImGui::TextDisabled(
                                "The effect is authored on the primary selection only.");
                    }
                }
            }

            if (world->has_light(id))
            {
                const ComponentSection section = component_header(context, "Light");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_light, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::LightParameters> access{
                        &IWorldEditor::has_light, &IWorldEditor::light_parameters,
                        &IWorldEditor::set_light_parameters};
                    ComponentEditor<SushiEngine::Simulation::LightParameters> editor(
                        context, *world, access, id);
                    apply_component_section(context, section, "Light", editor);
                    // The field list lives with the lights (see lighting_panel.hpp), because
                    // the Lighting panel offers the same fields and a second copy here is how
                    // the two views drift apart.
                    if (section.open)
                        draw_light_fields(editor);
                }
            }

            if (world->has_decal(id))
            {
                const ComponentSection section = component_header(context, "Decal");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_decal, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::DecalParameters> access{
                        &IWorldEditor::has_decal, &IWorldEditor::decal_parameters,
                        &IWorldEditor::set_decal_parameters};
                    ComponentEditor<SushiEngine::Simulation::DecalParameters> editor(
                        context, *world, access, id);

                    // The header's value actions cover the tint, box and opacity but never the
                    // maps: a TextureId is a reference the asset library counts, so copying
                    // one onto three decals would let the first release free a texture the
                    // other two are still projecting. The paths stay each decal's own.
                    if (section.reset || section.paste)
                    {
                        SushiEngine::Simulation::DecalParameters source;
                        const bool have = section.reset ||
                                          paste_component_values(context, "Decal", source);
                        if (have)
                        {
                            SushiEngine::Simulation::DecalParameters merged = editor.values();
                            merged.color = source.color;
                            merged.half_extents = source.half_extents;
                            merged.opacity = source.opacity;
                            editor.write_all(merged);
                        }
                    }
                    if (section.copy)
                        copy_component_values(context, "Decal", editor.values());

                    if (section.open)
                    {
                        editor.color("Tint", &decltype(editor)::Values::color,
                                     "Linear colour blended onto the surfaces the box covers.");
                        editor.vector("Half Extents", &decltype(editor)::Values::half_extents,
                                      0.05f, 0.05f, 100.0f, "%.2f m",
                                      "Half the projector box's size along right, up and "
                                      "forward, in metres.");
                        editor.fraction("Opacity", &decltype(editor)::Values::opacity, 0.0f, 1.0f,
                                        "%.2f", "How strongly the decal covers the surface.");

                        // Optional projected maps, loaded through the asset library exactly as
                        // a material's are (path field + Load/Clear). The typed path lives on
                        // the decal itself, so it survives selection changes, undo, and the
                        // scene file, where a load from disk re-resolves the id from it.
                        if (context.assets != nullptr)
                        {
                            SushiEngine::Simulation::DecalParameters& parameters =
                                editor.mutable_values();
                            bool maps_changed = false;
                            const auto map_field =
                                [&](const char* label, SushiEngine::Render::TextureId& tex,
                                    std::string& stored,
                                    SushiEngine::Render::TextureColorSpace cs)
                            {
                                ImGui::PushID(label);
                                char buffer[512] = {};
                                stored.copy(buffer, sizeof(buffer) - 1);
                                ImGui::SetNextItemWidth(-140.0f);
                                const bool enter =
                                    ImGui::InputText(label, buffer, sizeof(buffer),
                                                     ImGuiInputTextFlags_EnterReturnsTrue);
                                if (stored != buffer)
                                {
                                    stored = buffer;
                                    maps_changed = true;
                                }
                                // A file dropped from the Project browser is the same edit as
                                // typing its path and pressing Enter.
                                std::string dropped;
                                const bool drop = accept_asset_drop(dropped);
                                if (drop)
                                    stored = dropped;
                                ImGui::SameLine();
                                const bool load = ImGui::SmallButton("Load");
                                if (enter || load || drop)
                                {
                                    context.history.record(*world);
                                    const SushiEngine::Render::TextureId loaded =
                                        stored.empty()
                                            ? SushiEngine::Render::INVALID_TEXTURE
                                            : context.assets->load_texture(stored.c_str(), cs);
                                    if (tex != SushiEngine::Render::INVALID_TEXTURE)
                                        context.assets->release_texture(tex);
                                    tex = loaded;
                                    maps_changed = true;
                                }
                                ImGui::SameLine();
                                if (ImGui::SmallButton("Clear") &&
                                    tex != SushiEngine::Render::INVALID_TEXTURE)
                                {
                                    context.history.record(*world);
                                    context.assets->release_texture(tex);
                                    tex = SushiEngine::Render::INVALID_TEXTURE;
                                    stored.clear();
                                    maps_changed = true;
                                }
                                ImGui::SameLine();
                                ImGui::TextDisabled("%s",
                                                    tex != SushiEngine::Render::INVALID_TEXTURE
                                                        ? "set"
                                                        : "none");
                                ImGui::PopID();
                            };
                            map_field("Albedo Map", parameters.albedo_map,
                                      parameters.albedo_map_path,
                                      SushiEngine::Render::TextureColorSpace::SRGB);
                            map_field("ORM Map", parameters.orm_map, parameters.orm_map_path,
                                      SushiEngine::Render::TextureColorSpace::Linear);
                            if (maps_changed)
                                editor.write_primary();
                            if (multi)
                                ImGui::TextDisabled(
                                    "Maps are authored on the primary selection only.");
                        }
                    }
                }
            }

            if (world->has_ui(id))
            {
                const ComponentSection section = component_header(context, "UI Element");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_ui, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::UIElementParameters> access{
                        &IWorldEditor::has_ui, &IWorldEditor::ui_parameters,
                        &IWorldEditor::set_ui_parameters};
                    ComponentEditor<SushiEngine::Simulation::UIElementParameters> editor(
                        context, *world, access, id);
                    apply_component_section(context, section, "UI Element", editor);
                    if (section.open)
                    {
                        static const char* const UI_KINDS[] = {"Canvas", "Panel", "Image", "Text",
                                                               "Button"};
                        editor.choice("Kind##UI", &decltype(editor)::Values::kind, UI_KINDS, 5,
                                      "Canvas is the full-viewport root every other element "
                                      "lays out inside.");

                        // A mixed Kind means the rows below apply to some of the selection and
                        // not the rest, so the section stops rather than guess.
                        if (editor.mixed(&decltype(editor)::Values::kind))
                        {
                            ImGui::TextDisabled("Fields hidden: the selection mixes UI kinds.");
                        }
                        else
                        {
                            const bool is_canvas =
                                editor.values().kind ==
                                SushiEngine::Simulation::UIElementKind::Canvas;
                            const bool has_text =
                                editor.values().kind ==
                                    SushiEngine::Simulation::UIElementKind::Text ||
                                editor.values().kind ==
                                    SushiEngine::Simulation::UIElementKind::Button;

                            if (!is_canvas)
                            {
                                editor.fraction("Anchor Min X",
                                                &decltype(editor)::Values::anchor_min_x, 0.0f,
                                                1.0f, "%.2f",
                                                "Left edge as a fraction of the parent's width.");
                                editor.fraction("Anchor Min Y",
                                                &decltype(editor)::Values::anchor_min_y, 0.0f,
                                                1.0f, "%.2f",
                                                "Bottom edge as a fraction of the parent's "
                                                "height.");
                                editor.fraction("Anchor Max X",
                                                &decltype(editor)::Values::anchor_max_x, 0.0f,
                                                1.0f, "%.2f",
                                                "Right edge as a fraction of the parent's "
                                                "width.");
                                editor.fraction("Anchor Max Y",
                                                &decltype(editor)::Values::anchor_max_y, 0.0f,
                                                1.0f, "%.2f",
                                                "Top edge as a fraction of the parent's "
                                                "height.");
                                editor.fraction("Pivot X", &decltype(editor)::Values::pivot_x,
                                                0.0f, 1.0f, "%.2f",
                                                "The element's own handle, which Position and "
                                                "Rotation act about.");
                                editor.fraction("Pivot Y", &decltype(editor)::Values::pivot_y,
                                                0.0f, 1.0f, "%.2f",
                                                "The element's own handle, which Position and "
                                                "Rotation act about.");
                                editor.number("Position X",
                                              &decltype(editor)::Values::position_x, 1.0f,
                                              -8192.0f, 8192.0f, "%.0f px",
                                              "Offset from the anchored span, in pixels.");
                                editor.number("Position Y",
                                              &decltype(editor)::Values::position_y, 1.0f,
                                              -8192.0f, 8192.0f, "%.0f px",
                                              "Offset from the anchored span, in pixels.");
                            }

                            editor.number(is_canvas ? "Reference Width" : "Width",
                                          &decltype(editor)::Values::size_x, 1.0f, 0.0f, 8192.0f,
                                          "%.0f px",
                                          is_canvas ? "Design resolution the canvas scales from."
                                                    : "Width added to the anchored span.");
                            editor.number(is_canvas ? "Reference Height" : "Height",
                                          &decltype(editor)::Values::size_y, 1.0f, 0.0f, 8192.0f,
                                          "%.0f px",
                                          is_canvas ? "Design resolution the canvas scales from."
                                                    : "Height added to the anchored span.");

                            if (!is_canvas)
                            {
                                editor.color("Color##UI", &decltype(editor)::Values::color,
                                             "Fill colour, or text colour for a label.");
                                editor.fraction("Opacity", &decltype(editor)::Values::alpha, 0.0f,
                                                1.0f, "%.2f", "Element opacity.");
                            }

                            if (has_text)
                            {
                                editor.number("Font Size",
                                              &decltype(editor)::Values::font_size, 0.5f, 4.0f,
                                              128.0f, "%.0f px", "Label point size.");
                                editor.text("Text", &decltype(editor)::Values::text,
                                            "The label this element draws.");
                            }
                        }
                    }
                }
            }

            // Script (custom) components. Any type discovered here that is not yet in
            // the Add Component catalog is registered, so a scene loaded from disk
            // repopulates the menu without a separate load hook. Authored on the primary
            // entity only: a script's fields are named and typed per definition, so there
            // is no field identity to fan out across a selection whose scripts may not even
            // share a schema.
            for (const std::string& type_name : world->script_components(id))
            {
                register_script_definition(context, world->script_component(id, type_name));

                bool keep_script = true;
                ImGui::PushID(type_name.c_str());
                const bool script_open = ImGui::CollapsingHeader(
                    type_name.c_str(), &keep_script, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_script)
                {
                    context.history.record(*world);
                    for (const EntityId target : targets)
                        if (world->has_script_component(target, type_name))
                            world->remove_script_component(target, type_name);
                }
                else if (script_open)
                {
                    SushiEngine::Simulation::ScriptComponent script =
                        world->script_component(id, type_name);
                    if (draw_script_fields(context, *world, script))
                        world->set_script_component(id, script);
                }
                ImGui::PopID();
            }

            // Audio authoring sections (S9): emitter, reverb zone, and listener, each drawn
            // only when the entity carries that component. The emitter section auditions
            // through the live editor audio system.
            if (context.audio != nullptr)
                draw_audio_emitter_inspector(context, *world, id, *context.audio);
            draw_reverb_zone_inspector(context, *world, id);
            draw_audio_listener_inspector(context, *world, id);

            ImGui::Separator();
            if (ImGui::Button("Add Component"))
                ImGui::OpenPopup("AddComponentPopup");
            if (ImGui::BeginPopup("AddComponentPopup"))
            {
                // Offered when the primary lacks the component, and then attached to every
                // selected entity that lacks it — `set_presence` is idempotent, so the ones
                // that already have it are unaffected.
                if (!world->has_renderer(id) && ImGui::MenuItem("Renderer"))
                {
                    // A Renderer comes with a default Box mesh, since the mesh is the
                    // Renderer's own property here (see the Renderer header above).
                    context.history.record(*world);
                    for (const EntityId target : targets)
                    {
                        world->set_has_renderer(target, true);
                        world->set_has_shape(target, true);
                    }
                }
                if (!world->is_camera(id) && ImGui::MenuItem("Camera"))
                    set_presence(&IWorldEditor::set_is_camera, true);
                if (!world->has_physics_body(id) && ImGui::MenuItem("Rigid Body"))
                    set_presence(&IWorldEditor::set_has_physics_body, true);
                if (!world->has_character(id) && ImGui::MenuItem("Character Controller"))
                    set_presence(&IWorldEditor::set_has_character, true);
                if (!world->has_impact_response(id) && ImGui::MenuItem("Impact Response"))
                    set_presence(&IWorldEditor::set_has_impact_response, true);
                if (!world->has_cloth(id) && ImGui::MenuItem("Cloth"))
                    set_presence(&IWorldEditor::set_has_cloth, true);
                if (!world->has_soft_body(id) && ImGui::MenuItem("Soft Body"))
                    set_presence(&IWorldEditor::set_has_soft_body, true);
                if (!world->has_crowd(id) && ImGui::MenuItem("Crowd"))
                    set_presence(&IWorldEditor::set_has_crowd, true);
                if (!world->has_particle_emitter(id) && ImGui::MenuItem("Particle System"))
                    set_presence(&IWorldEditor::set_has_particle_emitter, true);
                if (!world->has_light(id) && ImGui::MenuItem("Light"))
                    set_presence(&IWorldEditor::set_has_light, true);
                if (!world->has_audio_emitter(id) && ImGui::MenuItem("Audio Emitter"))
                    set_presence(&IWorldEditor::set_has_audio_emitter, true);
                if (!world->has_reverb_zone(id) && ImGui::MenuItem("Reverb Zone"))
                    set_presence(&IWorldEditor::set_has_reverb_zone, true);
                if (!world->has_audio_listener(id) && ImGui::MenuItem("Audio Listener"))
                    set_presence(&IWorldEditor::set_has_audio_listener, true);
                if (!world->has_decal(id) && ImGui::MenuItem("Decal"))
                    set_presence(&IWorldEditor::set_has_decal, true);
                if (!world->has_collider(id) && ImGui::MenuItem("Collider"))
                    set_presence(&IWorldEditor::set_has_collider, true);
                if (!world->has_joint(id) && ImGui::MenuItem("Physics Joint"))
                    set_presence(&IWorldEditor::set_has_joint, true);
                if (!world->has_vehicle(id) && ImGui::MenuItem("Vehicle"))
                    set_presence(&IWorldEditor::set_has_vehicle, true);
                if (!world->surface_anchored(id) && ImGui::MenuItem("Surface Anchor"))
                    set_presence(&IWorldEditor::set_surface_anchored, true);
                if (!world->has_ui(id) && ImGui::MenuItem("UI Element"))
                    set_presence(&IWorldEditor::set_has_ui, true);

                // User-defined script components: every catalog entry not already on
                // the entity, plus the New Script scaffold. This is the editor's
                // MonoBehaviour-style "attach a custom component" surface.
                if (ImGui::BeginMenu("Scripts"))
                {
                    for (const SushiEngine::Simulation::ScriptComponent& definition :
                         context.script_catalog)
                    {
                        if (world->has_script_component(id, definition.type_name))
                            continue;
                        if (ImGui::MenuItem(definition.type_name.c_str()))
                        {
                            context.history.record(*world);
                            for (const EntityId target : targets)
                                if (!world->has_script_component(target, definition.type_name))
                                    world->add_script_component(target, definition);
                        }
                    }
                    ImGui::Separator();
                    if (ImGui::MenuItem("New Script..."))
                    {
                        context.show_new_script = true;
                        context.new_script_name.clear();
                        context.new_script_target = id;
                    }
                    ImGui::EndMenu();
                }
                ImGui::EndPopup();
            }

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
