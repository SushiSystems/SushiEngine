/**************************************************************************/
/* hierarchy_panel.cpp                                                   */
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

#include "hierarchy_panel.hpp"

#include "scene_commands.hpp"
#include "../ui/panel_widgets.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <imgui.h>
#include <imgui_internal.h>
#include <misc/cpp/imgui_stdlib.h>

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
            // Depth-first pre-order (roots in world order, each followed by its children)
            // — the same order draw_entity_node recurses in, so a Shift-range over it
            // matches what is visually listed whenever every node is expanded.
            void collect_display_order(IWorldEditor* world, EntityId parent,
                                       std::vector<EntityId>& out)
            {
                for (const EntityId id : world->entities())
                    if (world->parent(id) == parent)
                    {
                        out.push_back(id);
                        collect_display_order(world, id, out);
                    }
            }

            // Shift+click: select every entity between the anchor (the last plain or
            // Ctrl click) and @p id in @p order, inclusive. The anchor itself does not
            // move, so repeated Shift-clicks re-extend the same range rather than
            // chaining from the previous Shift target.
            void select_range(EditorContext& context, const std::vector<EntityId>& order,
                              EntityId id)
            {
                const auto anchor_it =
                    std::find(order.begin(), order.end(), context.selection_anchor);
                const auto to_it = std::find(order.begin(), order.end(), id);
                if (anchor_it == order.end() || to_it == order.end())
                {
                    select_only(context, id);
                    return;
                }
                auto a = static_cast<std::size_t>(std::distance(order.begin(), anchor_it));
                auto b = static_cast<std::size_t>(std::distance(order.begin(), to_it));
                if (a > b)
                    std::swap(a, b);
                context.selected_entities.clear();
                for (std::size_t i = a; i <= b; ++i)
                    context.selected_entities.push_back(order[i]);
                context.selected_entity = id;
            }

            // One Hierarchy row: rename field or selectable label, drag-reparent source
            // and target, context menu, and (when not renaming) a recursive draw of its
            // children so parenting nests visually the way Unity's hierarchy does.
            // @p order is the full display-order flattening, used to resolve Shift-range
            // selection.
            void draw_entity_node(EditorContext& context, IWorldEditor* world, EntityId id,
                                   const std::vector<EntityId>& order)
            {
                const std::string entity_name = world->name(id);
                ImGui::PushID(static_cast<int>(id));

                if (context.renaming_entity == id)
                {
                    std::string renamed;
                    if (inline_rename_field(context, std::to_string(id), entity_name, -FLT_MIN,
                                            renamed))
                    {
                        context.history.record(*world);
                        world->set_name(id, renamed);
                        context.renaming_entity = NULL_ENTITY;
                    }
                    ImGui::PopID();
                    return;
                }

                std::vector<EntityId> children;
                for (const EntityId candidate : world->entities())
                    if (world->parent(candidate) == id)
                        children.push_back(candidate);

                ImGuiTreeNodeFlags flags =
                    ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
                if (children.empty())
                    flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
                if (is_selected(context, id))
                    flags |= ImGuiTreeNodeFlags_Selected;

                const bool open = ImGui::TreeNodeEx(entity_name.c_str(), flags);
                if (ImGui::IsItemClicked(ImGuiMouseButton_Left) && !ImGui::IsItemToggledOpen())
                {
                    const ImGuiIO& io = ImGui::GetIO();
                    if (io.KeyShift)
                        select_range(context, order, id);
                    else if (io.KeyCtrl)
                        toggle_selected(context, id);
                    else
                        select_only(context, id);
                    if (!io.KeyShift && !io.KeyCtrl &&
                        ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                        context.frame_selected_requested = true;
                }

                if (ImGui::BeginDragDropSource())
                {
                    ImGui::SetDragDropPayload("HIERARCHY_ENTITY", &id, sizeof(EntityId));
                    ImGui::TextUnformatted(entity_name.c_str());
                    ImGui::EndDragDropSource();
                }
                if (ImGui::BeginDragDropTarget())
                {
                    if (const ImGuiPayload* payload =
                            ImGui::AcceptDragDropPayload("HIERARCHY_ENTITY", ImGuiDragDropFlags_AcceptBeforeDelivery | ImGuiDragDropFlags_AcceptNoDrawDefaultRect))
                    {
                        const EntityId dragged = *static_cast<const EntityId*>(payload->Data);
                        
                        ImVec2 mouse_pos = ImGui::GetMousePos();
                        ImVec2 item_pos = ImGui::GetItemRectMin();
                        ImVec2 item_size = ImGui::GetItemRectSize();
                        
                        bool drop_before = mouse_pos.y < item_pos.y + item_size.y * 0.25f;
                        bool drop_after = mouse_pos.y > item_pos.y + item_size.y * 0.75f;
                        
                        if (payload->IsDelivery())
                        {
                            context.history.record(*world);
                            if (drop_before)
                            {
                                world->set_parent(dragged, world->parent(id));
                                world->move_entity(dragged, id, false);
                            }
                            else if (drop_after)
                            {
                                world->set_parent(dragged, world->parent(id));
                                world->move_entity(dragged, id, true);
                            }
                            else
                            {
                                world->set_parent(dragged, id);
                            }
                        }
                        else
                        {
                            if (drop_before)
                            {
                                ImGui::GetWindowDrawList()->AddLine(
                                    item_pos, ImVec2(item_pos.x + item_size.x, item_pos.y),
                                    IM_COL32(255, 255, 0, 255), 2.0f);
                            }
                            else if (drop_after)
                            {
                                ImGui::GetWindowDrawList()->AddLine(
                                    ImVec2(item_pos.x, item_pos.y + item_size.y),
                                    ImVec2(item_pos.x + item_size.x, item_pos.y + item_size.y),
                                    IM_COL32(255, 255, 0, 255), 2.0f);
                            }
                            else
                            {
                                ImGui::GetWindowDrawList()->AddRect(
                                    item_pos, ImVec2(item_pos.x + item_size.x, item_pos.y + item_size.y),
                                    IM_COL32(255, 255, 0, 255), 0.0f, 0, 2.0f);
                            }
                        }
                    }
                    ImGui::EndDragDropTarget();
                }

                if (ImGui::BeginPopupContextItem())
                {
                    // Right-clicking an entity outside the current selection replaces it;
                    // right-clicking one already selected preserves the multi-selection so
                    // Delete can act on the whole group.
                    if (!is_selected(context, id))
                        select_only(context, id);
                    if (ImGui::MenuItem("Rename"))
                        context.renaming_entity = id;
                    if (ImGui::MenuItem("Unparent", nullptr, false,
                                        world->parent(id) != NULL_ENTITY))
                    {
                        context.history.record(*world);
                        world->set_parent(id, NULL_ENTITY);
                    }
                    ImGui::Separator();
                    draw_clipboard_menu_items(context, world);
                    ImGui::Separator();
                    draw_create_object_menu_items(context, world);
                    ImGui::EndPopup();
                }

                if (open && !children.empty())
                {
                    for (const EntityId child : children)
                        draw_entity_node(context, world, child, order);
                    ImGui::TreePop();
                }

                ImGui::PopID();
            }
        } // namespace

        void draw_hierarchy_panel(EditorContext& context)
        {
            if (!context.panels.hierarchy)
                return;
            if (!ImGui::Begin("Hierarchy", &context.panels.hierarchy))
            {
                ImGui::End();
                return;
            }

            IWorldEditor* world = world_of(context);
            if (world == nullptr)
            {
                ImGui::TextDisabled("No world.");
                ImGui::End();
                return;
            }

            if (ImGui::Button("Add Entity"))
            {
                context.history.record(*world);
                select_only(context, world->create("Entity"));
                editor_log(context, "Created entity 'Entity'.");
            }
            ImGui::SameLine();
            ImGui::BeginDisabled(context.selected_entities.empty());
            if (ImGui::Button("Delete"))
                context.pending_entity_command = EditorContext::EntityCommand::Delete;
            ImGui::EndDisabled();

            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputTextWithHint("##filter", "Search...", &context.hierarchy_filter);
            ImGui::Separator();

            const std::string lower_filter = to_lower(context.hierarchy_filter);

            if (ImGui::BeginChild("entities"))
            {
                if (!lower_filter.empty())
                {
                    // A search filter flattens the tree: nesting is meaningless once most
                    // of the hierarchy is hidden, so matches are listed directly. Ctrl/Shift
                    // still work, ranging over the filtered order shown here.
                    std::vector<EntityId> filtered_order;
                    for (const EntityId id : world->entities())
                        if (to_lower(world->name(id)).find(lower_filter) != std::string::npos)
                            filtered_order.push_back(id);

                    for (const EntityId id : filtered_order)
                    {
                        const std::string entity_name = world->name(id);
                        ImGui::PushID(static_cast<int>(id));

                        if (context.renaming_entity == id)
                        {
                            std::string renamed;
                            if (inline_rename_field(context, std::to_string(id), entity_name,
                                                    -FLT_MIN, renamed))
                            {
                                context.history.record(*world);
                                world->set_name(id, renamed);
                                context.renaming_entity = NULL_ENTITY;
                            }
                        }
                        else
                        {
                            const bool selected = is_selected(context, id);
                            if (ImGui::Selectable(entity_name.c_str(), selected,
                                                  ImGuiSelectableFlags_AllowDoubleClick))
                            {
                                const ImGuiIO& io = ImGui::GetIO();
                                if (io.KeyShift)
                                    select_range(context, filtered_order, id);
                                else if (io.KeyCtrl)
                                    toggle_selected(context, id);
                                else
                                    select_only(context, id);
                                if (!io.KeyShift && !io.KeyCtrl &&
                                    ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                                    context.frame_selected_requested = true;
                            }
                            if (ImGui::BeginPopupContextItem())
                            {
                                if (!is_selected(context, id))
                                    select_only(context, id);
                                if (ImGui::MenuItem("Rename"))
                                    context.renaming_entity = id;
                                if (ImGui::MenuItem("Unparent", nullptr, false,
                                                    world->parent(id) != NULL_ENTITY))
                                {
                                    context.history.record(*world);
                                    world->set_parent(id, NULL_ENTITY);
                                }
                                ImGui::Separator();
                                draw_clipboard_menu_items(context, world);
                                ImGui::Separator();
                                draw_create_object_menu_items(context, world);
                                ImGui::EndPopup();
                            }
                        }
                        ImGui::PopID();
                    }
                }
                else
                {
                    std::vector<EntityId> order;
                    collect_display_order(world, NULL_ENTITY, order);

                    // The root canvas itself accepts drops too, so dragging an entity onto
                    // empty space unparents it back to the top level.
                    if (ImGui::BeginDragDropTargetCustom(ImGui::GetCurrentWindow()->InnerRect,
                                                         ImGui::GetID("hierarchy_root")))
                    {
                        if (const ImGuiPayload* payload =
                                ImGui::AcceptDragDropPayload("HIERARCHY_ENTITY"))
                        {
                            const EntityId dragged = *static_cast<const EntityId*>(payload->Data);
                            context.history.record(*world);
                            world->set_parent(dragged, NULL_ENTITY);
                        }
                        ImGui::EndDragDropTarget();
                    }

                    for (const EntityId id : world->entities())
                        if (world->parent(id) == NULL_ENTITY)
                            draw_entity_node(context, world, id, order);

                    // Right-clicking blank space below the tree, not any row, so this
                    // never fires on top of a row's own BeginPopupContextItem above.
                    if (ImGui::BeginPopupContextWindow("hierarchy_empty_space",
                                                        ImGuiPopupFlags_MouseButtonRight |
                                                            ImGuiPopupFlags_NoOpenOverItems))
                    {
                        draw_clipboard_menu_items(context, world);
                        ImGui::Separator();
                        draw_create_object_menu_items(context, world);
                        ImGui::EndPopup();
                    }
                }

                if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::IsAnyItemHovered())
                {
                    select_only(context, NULL_ENTITY);
                }
            }
            ImGui::EndChild();

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
