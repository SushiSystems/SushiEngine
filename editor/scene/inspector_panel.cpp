/**************************************************************************/
/* inspector_panel.cpp                                                   */
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
#include <string>
#include <vector>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

#include <SushiEngine/astro/celestial_bodies.hpp>

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
                SushiEngine::Simulation::ShapeParams shape;
            };

            /** @brief The amber the editor says "this will not do what you meant" in. */
            ImVec4 warning_color() { return ImVec4(1.0f, 0.75f, 0.3f, 1.0f); }

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
                    const ComponentAccess<SushiEngine::Simulation::CameraParams> access{
                        &IWorldEditor::is_camera, &IWorldEditor::camera_params,
                        &IWorldEditor::set_camera_params};
                    ComponentEditor<SushiEngine::Simulation::CameraParams> editor(
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
                        values.shape = world->shape_params(id);
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
                            world->set_shape_params(target, incoming.shape);
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
                        const ComponentAccess<SushiEngine::Simulation::ShapeParams> access{
                            &IWorldEditor::has_shape, &IWorldEditor::shape_params,
                            &IWorldEditor::set_shape_params};
                        ComponentEditor<SushiEngine::Simulation::ShapeParams> editor(
                            context, *world, access, id);

                        // Plane is not a drawable mesh (Terrain uses a thin Box), so only
                        // the three solid primitives are offered as the Renderer's mesh.
                        static const char* const MESH_NAMES[] = {"Box", "Sphere", "Cylinder"};
                        editor.choice("Mesh", &decltype(editor)::Values::kind, MESH_NAMES, 3,
                                      "Which primitive this renderer draws.");

                        switch (editor.values().kind)
                        {
                            case SushiEngine::Simulation::PrimitiveKind::Sphere:
                                editor.vector_component(
                                    "Radius##Mesh", &decltype(editor)::Values::params, 0, 0.01f,
                                    0.01f, 1000.0f, "%.3f m",
                                    "Sphere radius before Scale, in metres.");
                                break;
                            case SushiEngine::Simulation::PrimitiveKind::Cylinder:
                                editor.vector("Radius / Half Height##Mesh",
                                              &decltype(editor)::Values::params, 0.01f, 0.01f,
                                              1000.0f, "%.3f m",
                                              "X is the radius, Y the half height, in metres; "
                                              "Z is unused.");
                                break;
                            default:
                                editor.vector("Half Extents##Mesh",
                                              &decltype(editor)::Values::params, 0.01f, 0.01f,
                                              1000.0f, "%.3f m",
                                              "Half the box's size along each local axis, in "
                                              "metres, before Scale.");
                                break;
                        }
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
                    const ComponentAccess<SushiEngine::Simulation::PhysicsBodyParams> access{
                        &IWorldEditor::has_physics_body, &IWorldEditor::physics_body_params,
                        &IWorldEditor::set_physics_body_params};
                    ComponentEditor<SushiEngine::Simulation::PhysicsBodyParams> editor(
                        context, *world, access, id);
                    apply_component_section(context, section, "Rigid Body", editor);
                    if (section.open)
                    {
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
                        if (derived)
                            ImGui::TextDisabled("Mass and inertia come from Density and the "
                                                "collider.");
                        editor.number("Drag Coefficient",
                                      &decltype(editor)::Values::drag_coefficient, 0.001f, 0.0f,
                                      100.0f, "%.4f 1/m",
                                      "Quadratic drag k: acceleration -k|v|v per metre. Zero "
                                      "disables it; higher means a lower terminal speed.");
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
                    const ComponentAccess<SushiEngine::Simulation::ColliderParams> access{
                        &IWorldEditor::has_collider, &IWorldEditor::collider_params,
                        &IWorldEditor::set_collider_params};
                    ComponentEditor<SushiEngine::Simulation::ColliderParams> editor(
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
                            editor.vector("Normal", &decltype(editor)::Values::params, 0.01f,
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
                                          &decltype(editor)::Values::params, 0.01f, 0.01f,
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
                            SushiEngine::Simulation::ColliderParams updated = editor.values();
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
                    using SushiEngine::Simulation::PhysicsJointParams;

                    const ComponentAccess<PhysicsJointParams> access{
                        &IWorldEditor::has_joint, &IWorldEditor::joint_params,
                        &IWorldEditor::set_joint_params};
                    ComponentEditor<PhysicsJointParams> editor(context, *world, access, id);
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
                        PhysicsJointParams& values = editor.mutable_values();
                        bool changed =
                            draw_joint_partner(context, *world, id, values.connected_body);

                        changed |= draw_joint_params(context, *world, values.joint);

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
                    const SushiEngine::Simulation::VehicleInstanceParams params =
                        world->vehicle_params(id);
                    if (params.asset_path.empty())
                        ImGui::TextDisabled("No structure named yet.");
                    else
                        ImGui::TextWrapped("%s", params.asset_path.c_str());

                    SushiEngine::Simulation::VehicleReport report;
                    if (world->vehicle_report(id, report))
                        ImGui::Text("%.0f rpm, %d parts off",
                                    double(report.engine_rate) * 9.5493,
                                    int(report.parts_detached));
                    else if (!params.asset_path.empty())
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
                    const ComponentAccess<SushiEngine::Simulation::ClothParams> access{
                        &IWorldEditor::has_cloth, &IWorldEditor::cloth_params,
                        &IWorldEditor::set_cloth_params};
                    ComponentEditor<SushiEngine::Simulation::ClothParams> editor(context, *world,
                                                                                access, id);
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

            if (world->has_particle_emitter(id))
            {
                const ComponentSection section = component_header(context, "Particle System");
                if (section.remove)
                {
                    set_presence(&IWorldEditor::set_has_particle_emitter, false);
                }
                else
                {
                    const ComponentAccess<SushiEngine::Simulation::ParticleEmitterParams> access{
                        &IWorldEditor::has_particle_emitter,
                        &IWorldEditor::particle_emitter_params,
                        &IWorldEditor::set_particle_emitter_params};
                    ComponentEditor<SushiEngine::Simulation::ParticleEmitterParams> editor(
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
                    const ComponentAccess<SushiEngine::Simulation::LightParams> access{
                        &IWorldEditor::has_light, &IWorldEditor::light_params,
                        &IWorldEditor::set_light_params};
                    ComponentEditor<SushiEngine::Simulation::LightParams> editor(context, *world,
                                                                                access, id);
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
                    const ComponentAccess<SushiEngine::Simulation::DecalParams> access{
                        &IWorldEditor::has_decal, &IWorldEditor::decal_params,
                        &IWorldEditor::set_decal_params};
                    ComponentEditor<SushiEngine::Simulation::DecalParams> editor(context, *world,
                                                                                access, id);

                    // The header's value actions cover the tint, box and opacity but never the
                    // maps: a TextureId is a reference the asset library counts, so copying
                    // one onto three decals would let the first release free a texture the
                    // other two are still projecting. The paths stay each decal's own.
                    if (section.reset || section.paste)
                    {
                        SushiEngine::Simulation::DecalParams source;
                        const bool have = section.reset ||
                                          paste_component_values(context, "Decal", source);
                        if (have)
                        {
                            SushiEngine::Simulation::DecalParams merged = editor.values();
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
                            SushiEngine::Simulation::DecalParams& params =
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
                            map_field("Albedo Map", params.albedo_map, params.albedo_map_path,
                                      SushiEngine::Render::TextureColorSpace::Srgb);
                            map_field("ORM Map", params.orm_map, params.orm_map_path,
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
                    const ComponentAccess<SushiEngine::Simulation::UIElementParams> access{
                        &IWorldEditor::has_ui, &IWorldEditor::ui_params,
                        &IWorldEditor::set_ui_params};
                    ComponentEditor<SushiEngine::Simulation::UIElementParams> editor(
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
                if (!world->has_cloth(id) && ImGui::MenuItem("Cloth"))
                    set_presence(&IWorldEditor::set_has_cloth, true);
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
