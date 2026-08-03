/**************************************************************************/
/* component_editor.hpp                                                   */
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

#pragma once

/**
 * @file component_editor.hpp
 * @brief Multi-object component editing: one field, every selected entity.
 *
 * The Inspector edits the *selection*, not one entity. Doing that per widget by hand
 * would mean, at every one of the Inspector's ~50 fields, a loop over the selection, a
 * comparison to decide whether the entities even agree on the value, and a fan-out
 * write — which is why editors that grow this feature late usually ship it for the
 * transform only. Instead the mechanism lives here once: a field is named by a
 * pointer-to-member, so this header can read it from every selected entity to decide
 * whether the value is shared, and write it back to every one of them when it changes,
 * without knowing which component or which field it is.
 *
 * That pointer-to-member is also what makes the fan-out *sound*. A tempting shortcut is
 * to diff the component's bytes before and after the widgets run and copy the changed
 * bytes to the other entities — exhaustive for free, and wrong: nudging a float from
 * 1.0 to 1.0000001 changes only its low bytes, and copying those into another entity's
 * 5.0 leaves 5.000-something instead of 1.0000001. A field is the unit of an edit, so a
 * field is what this header addresses.
 *
 * Mixed values are shown, not hidden. Where a widget can express indeterminacy it does
 * (a checkbox through ImGui's own mixed-value flag, a numeric field by displaying a dash
 * instead of a number, a combo by previewing one); where it cannot — a colour swatch has
 * no "no colour" — the primary entity's value is drawn and the row is labelled `mixed`,
 * because silently showing one entity's colour as if it were all of theirs is the kind
 * of lie §2.4 exists to prevent.
 */

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <imgui.h>
#include <imgui_internal.h>

#include "../core/editor_context.hpp"
#include "panel_widgets.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief How to reach one component kind through the world editor.
         *
         * The three operations every component section already performs — ask whether the
         * entity carries the component, read its parameters, write them back — named as
         * data so @ref ComponentEditor can perform them for an entity it was never told
         * about. Member-function pointers rather than callables so the parameter type is
         * checked at the binding site: passing `set_light_parameters` alongside
         * `collider_parameters` does not compile.
         *
         * @tparam Parameters The component's authoring parameter aggregate.
         */
        template <typename Parameters>
        struct ComponentAccess
        {
            /** @brief Whether the entity carries this component. */
            bool (Simulation::IWorldEditor::*present)(Simulation::EntityId) const;
            /** @brief The entity's current parameters. */
            Parameters (Simulation::IWorldEditor::*read)(Simulation::EntityId) const;
            /** @brief Installs parameters on the entity. */
            void (Simulation::IWorldEditor::*write)(Simulation::EntityId, const Parameters&);
        };

        /**
         * @brief Equality for a component field, for deciding whether the selection agrees.
         *
         * A free overload set rather than `operator==` on the engine's vector type: whether
         * two positions should compare equal is a question for the engine's math seam, and
         * the editor only needs "would these two draw the same value in this widget".
         *
         * @param a First value.
         * @param b Second value.
         * @return Whether the two would render identically.
         */
        template <typename Field>
        inline bool field_equal(const Field& a, const Field& b)
        {
            return a == b;
        }

        /** @brief Component-wise equality for vector fields; see @ref field_equal. */
        inline bool field_equal(const Vector3& a, const Vector3& b)
        {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }

        /**
         * @brief Records a copy of one component's parameters for Paste Values.
         *
         * The copy is type-erased behind the component's display name, which is also what
         * gates the cast back: a paste only ever reads bytes it wrote under the same name,
         * so the clipboard can hold any component's parameters — including the ones that
         * are not trivially copyable, like the Decal's map paths — without a variant per
         * component or a serialization format.
         *
         * @param context    Editor state holding the clipboard.
         * @param component  The component's display name, e.g. "Light".
         * @param parameters The values to remember.
         */
        template <typename Parameters>
        void copy_component_values(EditorContext& context, const char* component,
                                   const Parameters& parameters)
        {
            context.component_clipboard.component = component;
            context.component_clipboard.values = std::make_shared<Parameters>(parameters);
        }

        /**
         * @brief Reads back values stored by @ref copy_component_values, if they match.
         *
         * @param context Editor state holding the clipboard.
         * @param component The component asking; a different name means no paste.
         * @param out Receives the stored values only when the names match.
         * @return Whether @p out was filled.
         */
        template <typename Parameters>
        bool paste_component_values(const EditorContext& context, const char* component,
                                    Parameters& out)
        {
            if (context.component_clipboard.values == nullptr ||
                context.component_clipboard.component != component)
                return false;
            out = *static_cast<const Parameters*>(context.component_clipboard.values.get());
            return true;
        }

        template <typename Parameters>
        class ComponentEditor;

        /**
         * @brief Tag selecting @ref ComponentEditor's single-entity constructor.
         *
         * A list view addresses a *row* — the Lighting panel's light list edits the light on
         * that line, not the one the Hierarchy happens to have selected — so it needs an
         * editor that deliberately does not follow the selection. A tag rather than a bool
         * because `ComponentEditor(context, world, access, id, false)` at the call site would
         * say nothing about what the false means.
         */
        struct OneEntity
        {
        };

        /**
         * @brief Runs a component header's whole-component actions.
         *
         * Reset, Copy Values and Paste Values mean the same thing for every component, so
         * they are spelled once here rather than in each of the Inspector's sections — where
         * the eleventh copy would have been the one that forgot to record an undo step.
         *
         * @param context Editor state; owns the value clipboard.
         * @param section What the header asked for.
         * @param component The component's display name, the clipboard's key.
         * @param editor The section's editor, which performs the writes.
         */
        template <typename Parameters>
        void apply_component_section(EditorContext& context, const ComponentSection& section,
                                     const char* component, ComponentEditor<Parameters>& editor)
        {
            if (section.reset)
                editor.write_all(Parameters{});
            if (section.copy)
                copy_component_values(context, component, editor.values());
            if (!section.paste)
                return;
            Parameters pasted;
            if (paste_component_values(context, component, pasted))
                editor.write_all(pasted);
        }

        /**
         * @brief Edits one component across the whole selection, one field at a time.
         *
         * Constructed per component section per frame: it resolves which of the selected
         * entities actually carry the component, reads the primary entity's values as the
         * ones to display, and from then on every field method draws that value, reports
         * whether the selection agrees on it, and on an edit writes the new value to every
         * resolved entity as one undo step.
         *
         * The field methods also absorb what would otherwise be spelled at each call site —
         * the `Scalar`/float narrowing, the undo bracket, the tooltip — so a section reads as
         * a list of the component's fields rather than a list of ImGui calls.
         *
         * @tparam Parameters The component's authoring parameter aggregate.
         */
        template <typename Parameters>
        class ComponentEditor
        {
        public:
            /**
             * @brief The parameter aggregate, so a section can name its fields off the editor.
             *
             * Lets a call site write `&decltype(editor)::Values::intensity` instead of
             * repeating the fully qualified component type at every one of its fields, which
             * is what keeps a section readable as a list of fields.
             */
            using Values = Parameters;

            /**
             * @brief Resolves the selection this editor will write to.
             *
             * @param context Editor state; supplies the selection and the undo history.
             * @param world The world being edited.
             * @param access How to reach the component (see @ref ComponentAccess).
             * @param primary The entity whose values are displayed — the Inspector's
             *                primary selection, which is always one of the targets.
             */
            ComponentEditor(EditorContext& context, Simulation::IWorldEditor& world,
                            const ComponentAccess<Parameters>& access, Simulation::EntityId primary)
                : context_(context), world_(world), access_(access), primary_(primary),
                  values_((world.*access.read)(primary))
            {
                targets_.push_back(primary);
                for (const Simulation::EntityId id : context.selected_entities)
                {
                    if (id == primary || !world.exists(id) || !(world.*access.present)(id))
                        continue;
                    targets_.push_back(id);
                }
            }

            /**
             * @brief Edits @p only, ignoring the selection entirely.
             *
             * @param context Editor state; supplies the undo history.
             * @param world The world being edited.
             * @param access How to reach the component (see @ref ComponentAccess).
             * @param only The single entity to display and write.
             */
            ComponentEditor(EditorContext& context, Simulation::IWorldEditor& world,
                            const ComponentAccess<Parameters>& access, Simulation::EntityId only,
                            OneEntity)
                : context_(context), world_(world), access_(access), primary_(only),
                  values_((world.*access.read)(only))
            {
                targets_.push_back(only);
            }

            /** @brief The primary entity's values, as displayed; edits are folded in here. */
            const Parameters& values() const noexcept { return values_; }

            /** @brief How many selected entities this editor writes to (at least one). */
            std::size_t target_count() const noexcept { return targets_.size(); }

            /**
             * @brief Whether the selection disagrees about @p member.
             *
             * Public because a section sometimes branches on a field the widgets do not
             * draw — the Light's `is_spot` decides whether the cone rows appear at all,
             * and with a mixed selection there is no honest answer, so the section says so
             * instead of showing one entity's cone.
             *
             * @param member The field to compare across the selection.
             * @return True when at least two targets hold different values.
             */
            template <typename Field>
            bool mixed(Field Parameters::*member) const
            {
                const Field& reference = values_.*member;
                for (std::size_t i = 1; i < targets_.size(); ++i)
                {
                    const Parameters other = (world_.*access_.read)(targets_[i]);
                    if (!field_equal(other.*member, reference))
                        return true;
                }
                return false;
            }

            /**
             * @brief A numeric field, drawn as a drag with optional clamps.
             *
             * The drag is the editor's default for a physical quantity: a range like a far
             * plane's 1..10000 metres is *bounded* but a slider across it cannot resolve a
             * metre, so §2.5's "bounded means slider" rule is honoured for normalized
             * fractions (see @ref fraction) and everything else clamps a drag instead.
             *
             * @param label Field label, also the ImGui id.
             * @param member The field to edit.
             * @param speed Drag sensitivity in units per pixel.
             * @param low Lower clamp; pass equal to @p high to leave it unbounded.
             * @param high Upper clamp.
             * @param format printf-style display format, carrying the unit suffix.
             * @param tooltip Explanation shown on hover, or nullptr for none.
             * @param scale Display units per stored unit, for the fields the engine keeps in
             *              one unit and an author thinks in another — a field of view is
             *              radians in the struct and degrees in every editor there has ever
             *              been. @p speed, @p low and @p high are in display units too.
             * @return Whether the field changed this frame.
             */
            template <typename Field>
            bool number(const char* label, Field Parameters::*member, float speed, float low,
                        float high, const char* format, const char* tooltip = nullptr,
                        float scale = 1.0f)
            {
                return edit(member, tooltip, scale,
                            [&](float& value, bool is_mixed)
                            {
                                return ImGui::DragFloat(label, &value, speed, low, high,
                                                        is_mixed ? "-" : format);
                            });
            }

            /**
             * @brief A normalized fraction, drawn as a slider over [@p low, @p high].
             *
             * The one case §2.5's slider rule fits without reservation: the whole range is
             * visible, so the handle's position is the value.
             *
             * @param label Field label, also the ImGui id.
             * @param member The field to edit.
             * @param low Range minimum.
             * @param high Range maximum.
             * @param format printf-style display format.
             * @param tooltip Explanation shown on hover, or nullptr for none.
             * @return Whether the field changed this frame.
             */
            template <typename Field>
            bool fraction(const char* label, Field Parameters::*member, float low, float high,
                          const char* format, const char* tooltip = nullptr)
            {
                return edit(member, tooltip, 1.0f,
                            [&](float& value, bool is_mixed)
                            {
                                return ImGui::SliderFloat(label, &value, low, high,
                                                          is_mixed ? "-" : format);
                            });
            }

            /**
             * @brief An integer-valued field, drawn as a drag.
             *
             * Separate from @ref number because the engine's counts and indices are
             * unsigned or `std::size_t` and must not round-trip through a float: a seed of
             * 16'777'217 would come back one short.
             *
             * @param label Field label, also the ImGui id.
             * @param member The field to edit.
             * @param speed Drag sensitivity in units per pixel.
             * @param low Lower clamp.
             * @param high Upper clamp.
             * @param tooltip Explanation shown on hover, or nullptr for none.
             * @return Whether the field changed this frame.
             */
            template <typename Field>
            bool integer(const char* label, Field Parameters::*member, float speed, int low,
                         int high, const char* tooltip = nullptr)
            {
                const bool is_mixed = mixed(member);
                int value = static_cast<int>(values_.*member);
                const bool changed = ImGui::DragInt(label, &value, speed, low, high,
                                                    is_mixed ? "-" : "%d");
                finish_row(tooltip, false);
                if (!changed)
                    return false;
                if (value < low)
                    value = low;
                if (value > high)
                    value = high;
                assign(member, static_cast<Field>(value));
                return true;
            }

            /**
             * @brief A boolean field, drawn as a checkbox.
             *
             * The one widget ImGui can render as genuinely indeterminate, so a mixed
             * selection shows its mixed-value box rather than a dash.
             *
             * @param label Field label, also the ImGui id.
             * @param member The field to edit.
             * @param tooltip Explanation shown on hover, or nullptr for none.
             * @return Whether the field changed this frame.
             */
            bool toggle(const char* label, bool Parameters::*member, const char* tooltip = nullptr)
            {
                const bool is_mixed = mixed(member);
                bool value = values_.*member;
                if (is_mixed)
                    ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
                const bool changed = ImGui::Checkbox(label, &value);
                if (is_mixed)
                    ImGui::PopItemFlag();
                finish_row(tooltip, false);
                if (!changed)
                    return false;
                assign(member, value);
                return true;
            }

            /**
             * @brief A three-component vector field, drawn as X/Y/Z drags.
             *
             * @param label Field label, also the ImGui id.
             * @param member The field to edit.
             * @param speed Drag sensitivity in units per pixel.
             * @param low Lower clamp per component; equal to @p high leaves it unbounded.
             * @param high Upper clamp per component.
             * @param format printf-style display format, carrying the unit suffix.
             * @param tooltip Explanation shown on hover, or nullptr for none.
             * @return Whether the field changed this frame.
             */
            bool vector(const char* label, Vector3 Parameters::*member, float speed, float low,
                        float high, const char* format, const char* tooltip = nullptr)
            {
                const bool is_mixed = mixed(member);
                const Vector3& current = values_.*member;
                float components[3] = {to_float(current.x), to_float(current.y),
                                       to_float(current.z)};
                const bool changed = ImGui::DragFloat3(label, components, speed, low, high,
                                                       is_mixed ? "-" : format);
                finish_row(tooltip, false);
                if (!changed)
                    return false;
                assign(member, Vector3{to_scalar(components[0]), to_scalar(components[1]),
                                       to_scalar(components[2])});
                return true;
            }

            /**
             * @brief One component of a vector field, for the fields whose axes mean
             * different things.
             *
             * A sphere's `ShapeParameters` stores its radius in `parameters.x` and leaves the other
             * two unused; drawing all three would offer two controls that change nothing.
             *
             * @param label Field label, also the ImGui id.
             * @param member The vector field one component of which is edited.
             * @param axis 0, 1 or 2 for x, y or z.
             * @param speed Drag sensitivity in units per pixel.
             * @param low Lower clamp; equal to @p high leaves it unbounded.
             * @param high Upper clamp.
             * @param format printf-style display format, carrying the unit suffix.
             * @param tooltip Explanation shown on hover, or nullptr for none.
             * @return Whether the component changed this frame.
             */
            bool vector_component(const char* label, Vector3 Parameters::*member, int axis,
                                  float speed, float low, float high, const char* format,
                                  const char* tooltip = nullptr)
            {
                const auto axis_of = [axis](const Vector3& v)
                { return axis == 0 ? v.x : (axis == 1 ? v.y : v.z); };
                const Scalar current = axis_of(values_.*member);
                bool is_mixed = false;
                for (std::size_t i = 1; i < targets_.size() && !is_mixed; ++i)
                {
                    const Parameters other = (world_.*access_.read)(targets_[i]);
                    is_mixed = axis_of(other.*member) != current;
                }
                float value = to_float(current);
                const bool changed = ImGui::DragFloat(label, &value, speed, low, high,
                                                     is_mixed ? "-" : format);
                finish_row(tooltip, false);
                if (!changed)
                    return false;
                // Only the one axis travels: the other two stay whatever each entity had.
                values_.*member = with_axis(values_.*member, axis, to_scalar(value));
                for (const Simulation::EntityId id : targets_)
                {
                    Parameters parameters = (world_.*access_.read)(id);
                    parameters.*member = with_axis(parameters.*member, axis, to_scalar(value));
                    (world_.*access_.write)(id, parameters);
                }
                return true;
            }

            /**
             * @brief A linear-colour field, drawn as a swatch with a picker.
             *
             * A swatch cannot show "no colour", so a mixed selection draws the primary
             * entity's colour with a `mixed` label beside it.
             *
             * @param label Field label, also the ImGui id.
             * @param member The field to edit.
             * @param tooltip Explanation shown on hover, or nullptr for none.
             * @return Whether the field changed this frame.
             */
            bool color(const char* label, Vector3 Parameters::*member,
                       const char* tooltip = nullptr)
            {
                const bool is_mixed = mixed(member);
                const Vector3& current = values_.*member;
                float components[3] = {to_float(current.x), to_float(current.y),
                                       to_float(current.z)};
                const bool changed = ImGui::ColorEdit3(label, components);
                finish_row(tooltip, is_mixed);
                if (!changed)
                    return false;
                assign(member, Vector3{to_scalar(components[0]), to_scalar(components[1]),
                                       to_scalar(components[2])});
                return true;
            }

            /**
             * @brief An enumerated field, drawn as a combo over @p names.
             *
             * @param label Field label, also the ImGui id.
             * @param member The field to edit; its underlying value indexes @p names.
             * @param names The option labels, in enumerator order.
             * @param count How many options @p names holds.
             * @param tooltip Explanation shown on hover, or nullptr for none.
             * @return Whether the field changed this frame.
             */
            template <typename Field>
            bool choice(const char* label, Field Parameters::*member, const char* const* names,
                        int count, const char* tooltip = nullptr)
            {
                const bool is_mixed = mixed(member);
                int index = static_cast<int>(values_.*member);
                if (index < 0 || index >= count)
                    index = 0;
                bool changed = false;
                if (ImGui::BeginCombo(label, is_mixed ? "-" : names[index]))
                {
                    for (int option = 0; option < count; ++option)
                    {
                        if (!ImGui::Selectable(names[option], !is_mixed && option == index))
                            continue;
                        index = option;
                        changed = true;
                    }
                    ImGui::EndCombo();
                }
                if (tooltip != nullptr && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", tooltip);
                if (!changed)
                    return false;
                // A combo commits on the frame the option is picked and is never "active"
                // across frames, so the whole gesture is one recorded step.
                context_.history.record(world_);
                assign(member, static_cast<Field>(index));
                return true;
            }

            /**
             * @brief A fixed-buffer text field, for the inline labels the engine stores inline.
             *
             * @tparam N The buffer's capacity, deduced from the member.
             * @param label Field label, also the ImGui id.
             * @param member The character buffer to edit.
             * @param tooltip Explanation shown on hover, or nullptr for none.
             * @return Whether the field changed this frame.
             */
            template <std::size_t N>
            bool text(const char* label, char (Parameters::*member)[N],
                      const char* tooltip = nullptr)
            {
                char buffer[N];
                std::snprintf(buffer, N, "%s", values_.*member);
                const bool changed = ImGui::InputText(label, buffer, N);
                bool is_mixed = false;
                for (std::size_t i = 1; i < targets_.size() && !is_mixed; ++i)
                {
                    const Parameters other = (world_.*access_.read)(targets_[i]);
                    is_mixed = std::string(other.*member) != std::string(values_.*member);
                }
                finish_row(tooltip, is_mixed);
                if (!changed)
                    return false;
                track_item_undo(context_, world_);
                std::snprintf(values_.*member, N, "%s", buffer);
                for (const Simulation::EntityId id : targets_)
                {
                    Parameters parameters = (world_.*access_.read)(id);
                    std::snprintf(parameters.*member, N, "%s", buffer);
                    (world_.*access_.write)(id, parameters);
                }
                return true;
            }

            /**
             * @brief Installs @p parameters on every target, as one undo step.
             *
             * The whole-component path behind Reset and Paste Values, and the escape hatch
             * for a section that edits its parameters some other way — the Decal's texture
             * slots load through the asset library rather than through a field method.
             *
             * @param parameters The values to install everywhere.
             */
            void write_all(const Parameters& parameters)
            {
                context_.history.record(world_);
                values_ = parameters;
                for (const Simulation::EntityId id : targets_)
                    (world_.*access_.write)(id, parameters);
            }

            /**
             * @brief Installs the primary entity's current values on it alone.
             *
             * For the fields a section edits outside the field methods, where fanning out
             * to the selection would overwrite paths the other entities authored
             * themselves.
             */
            void write_primary()
            {
                (world_.*access_.write)(primary_, values_);
            }

            /** @brief The primary entity's values, mutable for the paths above. */
            Parameters& mutable_values() noexcept { return values_; }

        private:
            /**
             * @brief The shared body of every float-backed field method.
             *
             * @param member The field being edited.
             * @param tooltip Hover text, or nullptr.
             * @param scale Display units per stored unit; see @ref number.
             * @param draw Draws the widget over a float buffer; told whether the selection is
             *             mixed, so it can substitute a dash for its display format.
             * @return Whether the field changed this frame.
             */
            template <typename Field, typename Draw>
            bool edit(Field Parameters::*member, const char* tooltip, float scale, Draw draw)
            {
                const bool is_mixed = mixed(member);
                float value = static_cast<float>(values_.*member) * scale;
                const bool changed = draw(value, is_mixed);
                finish_row(tooltip, false);
                if (!changed)
                    return false;
                assign(member, static_cast<Field>(value / scale));
                return true;
            }

            /** @brief @p v with one axis replaced; the write half of @ref vector_component. */
            static Vector3 with_axis(const Vector3& v, int axis, Scalar value)
            {
                return Vector3{axis == 0 ? value : v.x, axis == 1 ? value : v.y,
                               axis == 2 ? value : v.z};
            }

            /**
             * @brief Closes a field row: its undo bracket, tooltip, and mixed marker.
             *
             * @param tooltip Hover text, or nullptr.
             * @param show_mixed Whether to append the `mixed` label, for widgets that
             *                   cannot show indeterminacy themselves.
             */
            void finish_row(const char* tooltip, bool show_mixed)
            {
                track_item_undo(context_, world_);
                if (tooltip != nullptr && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", tooltip);
                if (!show_mixed)
                    return;
                ImGui::SameLine();
                ImGui::TextDisabled("mixed");
            }

            /**
             * @brief Folds a new field value into the primary copy and every target.
             *
             * Only the one field is written: each target's other fields are read back and
             * put straight down again, so a mixed selection stays mixed everywhere the user
             * did not touch.
             */
            template <typename Field>
            void assign(Field Parameters::*member, const Field& value)
            {
                values_.*member = value;
                for (const Simulation::EntityId id : targets_)
                {
                    Parameters parameters = (world_.*access_.read)(id);
                    parameters.*member = value;
                    (world_.*access_.write)(id, parameters);
                }
            }

            EditorContext& context_;
            Simulation::IWorldEditor& world_;
            ComponentAccess<Parameters> access_;
            Simulation::EntityId primary_;
            Parameters values_;
            std::vector<Simulation::EntityId> targets_;
        };
    } // namespace Editor
} // namespace SushiEngine
