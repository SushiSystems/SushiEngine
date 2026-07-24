/**************************************************************************/
/* bindings_json.hpp                                                     */
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
 * @file bindings_json.hpp
 * @brief (De)serializing binding sets — the engine-provided persistence, not a file policy.
 *
 * The engine hands a game the functions to turn an @ref InputContext's bindings into JSON and
 * back; where the file lives is the game's business (the editor rides its own Preferences
 * store). Reads are tolerant field-by-field with defaulted values, exactly the
 * `render_settings_to_json` pattern in the editor's Preferences: a stale or partial document
 * loads to the context's compiled-in defaults without throwing, and an action whose entry is
 * missing keeps its defaults, so an update that adds an action degrades gracefully. Unknown
 * actions in an older document are preserved on round-trip through the `previous`-merging
 * overload of @ref bindings_to_json.
 *
 * This is the only input header that includes nlohmann/json; the core action layer stays
 * dependency-free, so a headless build that never persists bindings pulls in no JSON.
 */

#include <string>

#include <nlohmann/json.hpp>

#include <SushiEngine/input/action_map.hpp>
#include <SushiEngine/input/bindings.hpp>
#include <SushiEngine/input/controls.hpp>

namespace SushiEngine
{
    namespace Input
    {
        namespace detail
        {
            inline nlohmann::json control_path_to_json(const ControlPath& path)
            {
                return nlohmann::json{{"family", static_cast<std::uint16_t>(path.family)},
                                      {"control", path.control}};
            }

            inline ControlPath control_path_from_json(const nlohmann::json& node)
            {
                ControlPath path;
                path.family = static_cast<DeviceFamily>(node.value("family", std::uint16_t{0}));
                path.control = node.value("control", std::uint16_t{0});
                return path;
            }

            inline nlohmann::json deadzone_to_json(const Deadzone& deadzone)
            {
                return nlohmann::json{{"shape", static_cast<std::uint8_t>(deadzone.shape)},
                                      {"inner", deadzone.inner},
                                      {"outer", deadzone.outer}};
            }

            inline Deadzone deadzone_from_json(const nlohmann::json& node)
            {
                Deadzone deadzone;
                deadzone.shape = static_cast<Deadzone::Shape>(node.value("shape", std::uint8_t{0}));
                deadzone.inner = node.value("inner", 0.0f);
                deadzone.outer = node.value("outer", 1.0f);
                return deadzone;
            }

            inline nlohmann::json chord_to_json(const ChordGate& chord)
            {
                nlohmann::json modifiers = nlohmann::json::array();
                for (std::uint8_t index = 0; index < chord.count; ++index)
                    modifiers.push_back(control_path_to_json(chord.modifiers[index]));
                return modifiers;
            }

            inline ChordGate chord_from_json(const nlohmann::json& node)
            {
                ChordGate chord;
                if (node.is_array())
                    for (const nlohmann::json& modifier : node)
                        chord.require(control_path_from_json(modifier));
                return chord;
            }

            inline nlohmann::json binding_to_json(const Binding& binding)
            {
                return nlohmann::json{{"control", control_path_to_json(binding.control)},
                                      {"chord", chord_to_json(binding.chord)},
                                      {"deadzone", deadzone_to_json(binding.deadzone)},
                                      {"scale", binding.scale},
                                      {"invert", binding.invert}};
            }

            inline Binding binding_from_json(const nlohmann::json& node)
            {
                Binding binding;
                binding.control = control_path_from_json(node.value("control", nlohmann::json::object()));
                binding.chord = chord_from_json(node.value("chord", nlohmann::json::array()));
                binding.deadzone = deadzone_from_json(node.value("deadzone", nlohmann::json::object()));
                binding.scale = node.value("scale", 1.0f);
                binding.invert = node.value("invert", false);
                return binding;
            }

            inline nlohmann::json composite1d_to_json(const CompositeAxis1D& composite)
            {
                return nlohmann::json{{"negative", control_path_to_json(composite.negative)},
                                      {"positive", control_path_to_json(composite.positive)},
                                      {"chord", chord_to_json(composite.chord)},
                                      {"scale", composite.scale}};
            }

            inline CompositeAxis1D composite1d_from_json(const nlohmann::json& node)
            {
                CompositeAxis1D composite;
                composite.negative = control_path_from_json(node.value("negative", nlohmann::json::object()));
                composite.positive = control_path_from_json(node.value("positive", nlohmann::json::object()));
                composite.chord = chord_from_json(node.value("chord", nlohmann::json::array()));
                composite.scale = node.value("scale", 1.0f);
                return composite;
            }

            inline nlohmann::json vector2_binding_to_json(const Vector2Binding& binding)
            {
                return nlohmann::json{{"x", control_path_to_json(binding.x_axis)},
                                      {"y", control_path_to_json(binding.y_axis)},
                                      {"chord", chord_to_json(binding.chord)},
                                      {"deadzone", deadzone_to_json(binding.deadzone)},
                                      {"invert_x", binding.invert_x},
                                      {"invert_y", binding.invert_y},
                                      {"scale", binding.scale}};
            }

            inline Vector2Binding vector2_binding_from_json(const nlohmann::json& node)
            {
                Vector2Binding binding;
                binding.x_axis = control_path_from_json(node.value("x", nlohmann::json::object()));
                binding.y_axis = control_path_from_json(node.value("y", nlohmann::json::object()));
                binding.chord = chord_from_json(node.value("chord", nlohmann::json::array()));
                binding.deadzone = deadzone_from_json(node.value("deadzone", nlohmann::json::object()));
                binding.invert_x = node.value("invert_x", false);
                binding.invert_y = node.value("invert_y", false);
                binding.scale = node.value("scale", 1.0f);
                return binding;
            }

            inline nlohmann::json composite2d_to_json(const CompositeAxis2D& composite)
            {
                return nlohmann::json{{"up", control_path_to_json(composite.up)},
                                      {"down", control_path_to_json(composite.down)},
                                      {"left", control_path_to_json(composite.left)},
                                      {"right", control_path_to_json(composite.right)},
                                      {"chord", chord_to_json(composite.chord)},
                                      {"scale", composite.scale},
                                      {"normalize", composite.normalize}};
            }

            inline CompositeAxis2D composite2d_from_json(const nlohmann::json& node)
            {
                CompositeAxis2D composite;
                composite.up = control_path_from_json(node.value("up", nlohmann::json::object()));
                composite.down = control_path_from_json(node.value("down", nlohmann::json::object()));
                composite.left = control_path_from_json(node.value("left", nlohmann::json::object()));
                composite.right = control_path_from_json(node.value("right", nlohmann::json::object()));
                composite.chord = chord_from_json(node.value("chord", nlohmann::json::array()));
                composite.scale = node.value("scale", 1.0f);
                composite.normalize = node.value("normalize", true);
                return composite;
            }

            inline nlohmann::json action_to_json(const Action& action)
            {
                nlohmann::json buttons = nlohmann::json::array();
                for (const Binding& binding : action.button_bindings)
                    buttons.push_back(binding_to_json(binding));

                nlohmann::json axis1d = nlohmann::json::array();
                for (const Binding& binding : action.axis1d_bindings)
                    axis1d.push_back(binding_to_json(binding));

                nlohmann::json composites1d = nlohmann::json::array();
                for (const CompositeAxis1D& composite : action.axis1d_composites)
                    composites1d.push_back(composite1d_to_json(composite));

                nlohmann::json vector2 = nlohmann::json::array();
                for (const Vector2Binding& binding : action.axis2d_bindings)
                    vector2.push_back(vector2_binding_to_json(binding));

                nlohmann::json composites2d = nlohmann::json::array();
                for (const CompositeAxis2D& composite : action.axis2d_composites)
                    composites2d.push_back(composite2d_to_json(composite));

                return nlohmann::json{{"type", static_cast<std::uint8_t>(action.type)},
                                      {"buttons", buttons},
                                      {"axis1d", axis1d},
                                      {"composites1d", composites1d},
                                      {"vector2", vector2},
                                      {"composites2d", composites2d}};
            }

            // Rebuilds an action's bindings from @p node into temporaries, committing them only
            // if the whole parse succeeds — so a malformed entry leaves the compiled-in defaults
            // in place rather than a half-applied set.
            inline void apply_action_from_json(Action& action, const nlohmann::json& node)
            {
                std::vector<Binding> buttons;
                std::vector<Binding> axis1d;
                std::vector<CompositeAxis1D> composites1d;
                std::vector<Vector2Binding> vector2;
                std::vector<CompositeAxis2D> composites2d;

                for (const nlohmann::json& item : node.value("buttons", nlohmann::json::array()))
                    buttons.push_back(binding_from_json(item));
                for (const nlohmann::json& item : node.value("axis1d", nlohmann::json::array()))
                    axis1d.push_back(binding_from_json(item));
                for (const nlohmann::json& item : node.value("composites1d", nlohmann::json::array()))
                    composites1d.push_back(composite1d_from_json(item));
                for (const nlohmann::json& item : node.value("vector2", nlohmann::json::array()))
                    vector2.push_back(vector2_binding_from_json(item));
                for (const nlohmann::json& item : node.value("composites2d", nlohmann::json::array()))
                    composites2d.push_back(composite2d_from_json(item));

                action.button_bindings = std::move(buttons);
                action.axis1d_bindings = std::move(axis1d);
                action.axis1d_composites = std::move(composites1d);
                action.axis2d_bindings = std::move(vector2);
                action.axis2d_composites = std::move(composites2d);
            }
        } // namespace detail

        /**
         * @brief Serializes @p context's bindings to a JSON object keyed by action name.
         * @param context The context whose bindings to write.
         * @return A JSON object mapping each action name to its serialized bindings.
         */
        inline nlohmann::json bindings_to_json(const InputContext& context)
        {
            nlohmann::json document = nlohmann::json::object();
            for (const std::unique_ptr<Action>& action : context.actions())
                document[action->name] = detail::action_to_json(*action);
            return document;
        }

        /**
         * @brief Serializes @p context's bindings, preserving unknown entries from @p previous.
         *
         * Entries in @p previous whose action names are not in @p context (an action removed by a
         * downgrade, or a sibling context's data sharing the file) are carried through unchanged,
         * so a round-trip never silently drops another version's bindings.
         *
         * @param context  The context whose current bindings to write.
         * @param previous A previously loaded document to merge unknown entries from.
         * @return The merged JSON object.
         */
        inline nlohmann::json bindings_to_json(const InputContext& context, const nlohmann::json& previous)
        {
            nlohmann::json document = previous.is_object() ? previous : nlohmann::json::object();
            for (const std::unique_ptr<Action>& action : context.actions())
                document[action->name] = detail::action_to_json(*action);
            return document;
        }

        /**
         * @brief Applies a serialized @p document onto @p context's actions.
         *
         * For each action the context defines, if @p document carries an entry, that action's
         * bindings are replaced with the document's; otherwise the action keeps its compiled-in
         * defaults. A malformed entry is ignored (defaults kept), and the whole call is
         * exception-safe: a document that is not an object, or any per-action parse failure,
         * leaves the affected actions at their defaults rather than throwing.
         *
         * @param context  The context to update in place (already built with its defaults).
         * @param document The loaded JSON object, keyed by action name.
         */
        inline void bindings_from_json(InputContext& context, const nlohmann::json& document)
        {
            if (!document.is_object())
                return;

            for (const std::unique_ptr<Action>& action : context.actions())
            {
                const auto entry = document.find(action->name);
                if (entry == document.end())
                    continue; // no override for this action; keep defaults.
                try
                {
                    detail::apply_action_from_json(*action, *entry);
                }
                catch (const nlohmann::json::exception&)
                {
                    // Stale/partial entry: leave this action at its compiled-in defaults.
                }
            }
        }
    } // namespace Input
} // namespace SushiEngine
