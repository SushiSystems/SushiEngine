/**************************************************************************/
/* animator_controller_json.hpp                                          */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
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
 * @file animator_controller_json.hpp
 * @brief (De)serializing an authored @ref ControllerDesc as JSON (phase A9).
 *
 * The Animator is authored in the editor and persisted as JSON (design §4.3, like
 * `.sushiscene`); at load it compiles to the flat `.sushictrl` blob. This header is the
 * persistence seam: `ControllerDesc` ⇄ JSON, human-readable (enums as names, nested blend trees
 * as nested objects). It is what the editor's save/load and undo/redo ride on — an undo step is
 * a serialized snapshot restored. Like `input/bindings_json.hpp`, this is the *only* animation
 * header that pulls in nlohmann/json, so the core stack stays dependency-free; a headless build
 * that never persists controllers includes none of it. Asset references serialize as their ids
 * (-1 for none); a project layer that maps ids to paths sits above this.
 */

#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/blend_tree.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        namespace detail
        {
            inline const char* parameter_type_name(ParameterType type)
            {
                switch (type)
                {
                    case ParameterType::Float: return "Float";
                    case ParameterType::Int: return "Int";
                    case ParameterType::Bool: return "Bool";
                    case ParameterType::Trigger: return "Trigger";
                }
                return "Float";
            }
            inline ParameterType parameter_type_from(const std::string& name)
            {
                if (name == "Int") return ParameterType::Int;
                if (name == "Bool") return ParameterType::Bool;
                if (name == "Trigger") return ParameterType::Trigger;
                return ParameterType::Float;
            }

            inline const char* comparator_name(Comparator comparator)
            {
                switch (comparator)
                {
                    case Comparator::Greater: return "Greater";
                    case Comparator::Less: return "Less";
                    case Comparator::Equals: return "Equals";
                    case Comparator::NotEquals: return "NotEquals";
                    case Comparator::If: return "If";
                    case Comparator::IfNot: return "IfNot";
                }
                return "Greater";
            }
            inline Comparator comparator_from(const std::string& name)
            {
                if (name == "Less") return Comparator::Less;
                if (name == "Equals") return Comparator::Equals;
                if (name == "NotEquals") return Comparator::NotEquals;
                if (name == "If") return Comparator::If;
                if (name == "IfNot") return Comparator::IfNot;
                return Comparator::Greater;
            }

            inline const char* interruption_name(InterruptionSource source)
            {
                switch (source)
                {
                    case InterruptionSource::None: return "None";
                    case InterruptionSource::CurrentState: return "CurrentState";
                    case InterruptionSource::NextState: return "NextState";
                }
                return "None";
            }
            inline InterruptionSource interruption_from(const std::string& name)
            {
                if (name == "CurrentState") return InterruptionSource::CurrentState;
                if (name == "NextState") return InterruptionSource::NextState;
                return InterruptionSource::None;
            }

            inline const char* blend_mode_name(LayerBlendMode mode)
            {
                return mode == LayerBlendMode::Additive ? "Additive" : "Override";
            }
            inline LayerBlendMode blend_mode_from(const std::string& name)
            {
                return name == "Additive" ? LayerBlendMode::Additive : LayerBlendMode::Override;
            }

            inline const char* blend_tree_type_name(BlendTreeType type)
            {
                switch (type)
                {
                    case BlendTreeType::Simple1D: return "Simple1D";
                    case BlendTreeType::SimpleDirectional2D: return "SimpleDirectional2D";
                    case BlendTreeType::FreeformDirectional2D: return "FreeformDirectional2D";
                    case BlendTreeType::FreeformCartesian2D: return "FreeformCartesian2D";
                    case BlendTreeType::Direct: return "Direct";
                }
                return "Simple1D";
            }
            inline BlendTreeType blend_tree_type_from(const std::string& name)
            {
                if (name == "SimpleDirectional2D") return BlendTreeType::SimpleDirectional2D;
                if (name == "FreeformDirectional2D") return BlendTreeType::FreeformDirectional2D;
                if (name == "FreeformCartesian2D") return BlendTreeType::FreeformCartesian2D;
                if (name == "Direct") return BlendTreeType::Direct;
                return BlendTreeType::Simple1D;
            }

            inline std::int64_t asset_to_json(AssetId id)
            {
                return id == INVALID_ASSET ? -1 : static_cast<std::int64_t>(id);
            }
            inline AssetId asset_from_json(const nlohmann::json& value)
            {
                const std::int64_t raw = value.is_number() ? value.get<std::int64_t>() : -1;
                return raw < 0 ? INVALID_ASSET : static_cast<AssetId>(raw);
            }

            inline nlohmann::json condition_to_json(const ConditionDesc& condition)
            {
                return nlohmann::json{{"parameter", condition.parameter},
                                      {"comparator", comparator_name(condition.comparator)},
                                      {"threshold", condition.threshold}};
            }
            inline nlohmann::json transition_to_json(const TransitionDesc& transition)
            {
                nlohmann::json conditions = nlohmann::json::array();
                for (const ConditionDesc& c : transition.conditions)
                    conditions.push_back(condition_to_json(c));
                return nlohmann::json{{"destination", transition.destination},
                                      {"has_exit_time", transition.has_exit_time},
                                      {"exit_time", transition.exit_time},
                                      {"duration", transition.duration},
                                      {"offset", transition.offset},
                                      {"interruption", interruption_name(transition.interruption)},
                                      {"conditions", conditions}};
            }
            inline TransitionDesc transition_from_json(const nlohmann::json& json)
            {
                TransitionDesc transition;
                transition.destination = json.value("destination", std::string{});
                transition.has_exit_time = json.value("has_exit_time", false);
                transition.exit_time = json.value("exit_time", 1.0f);
                transition.duration = json.value("duration", 0.0f);
                transition.offset = json.value("offset", 0.0f);
                transition.interruption =
                    interruption_from(json.value("interruption", std::string{"None"}));
                if (json.contains("conditions"))
                    for (const nlohmann::json& c : json.at("conditions"))
                    {
                        ConditionDesc condition;
                        condition.parameter = c.value("parameter", std::string{});
                        condition.comparator =
                            comparator_from(c.value("comparator", std::string{"Greater"}));
                        condition.threshold = c.value("threshold", 0.0f);
                        transition.conditions.push_back(condition);
                    }
                return transition;
            }

            inline nlohmann::json blend_node_to_json(const BlendTreeNodeDesc& node);

            inline nlohmann::json blend_child_to_json(const BlendChildDesc& child)
            {
                nlohmann::json json{{"clip", asset_to_json(child.clip)},
                                    {"threshold", child.threshold},
                                    {"position_x", child.position_x},
                                    {"position_y", child.position_y},
                                    {"parameter", child.parameter},
                                    {"speed", child.speed}};
                if (child.child_node)
                    json["node"] = blend_node_to_json(*child.child_node);
                return json;
            }

            inline nlohmann::json blend_node_to_json(const BlendTreeNodeDesc& node)
            {
                nlohmann::json children = nlohmann::json::array();
                for (const BlendChildDesc& child : node.children)
                    children.push_back(blend_child_to_json(child));
                return nlohmann::json{{"type", blend_tree_type_name(node.type)},
                                      {"parameter_x", node.parameter_x},
                                      {"parameter_y", node.parameter_y},
                                      {"normalize", node.normalize},
                                      {"children", children}};
            }

            inline std::shared_ptr<BlendTreeNodeDesc> blend_node_from_json(const nlohmann::json& json);

            inline BlendChildDesc blend_child_from_json(const nlohmann::json& json)
            {
                BlendChildDesc child;
                child.clip = asset_from_json(json.value("clip", nlohmann::json(-1)));
                child.threshold = json.value("threshold", 0.0f);
                child.position_x = json.value("position_x", 0.0f);
                child.position_y = json.value("position_y", 0.0f);
                child.parameter = json.value("parameter", std::string{});
                child.speed = json.value("speed", 1.0f);
                if (json.contains("node") && !json.at("node").is_null())
                    child.child_node = blend_node_from_json(json.at("node"));
                return child;
            }

            inline std::shared_ptr<BlendTreeNodeDesc> blend_node_from_json(const nlohmann::json& json)
            {
                auto node = std::make_shared<BlendTreeNodeDesc>();
                node->type = blend_tree_type_from(json.value("type", std::string{"Simple1D"}));
                node->parameter_x = json.value("parameter_x", std::string{});
                node->parameter_y = json.value("parameter_y", std::string{});
                node->normalize = json.value("normalize", true);
                if (json.contains("children"))
                    for (const nlohmann::json& c : json.at("children"))
                        node->children.push_back(blend_child_from_json(c));
                return node;
            }
        } // namespace detail

        /**
         * @brief Serializes an authored controller to JSON.
         * @param desc The controller to serialize.
         * @return A JSON document that @ref controller_from_json reads back to an equal desc.
         */
        inline nlohmann::json controller_to_json(const ControllerDesc& desc)
        {
            nlohmann::json parameters = nlohmann::json::array();
            for (const ParameterDesc& p : desc.parameters)
                parameters.push_back({{"name", p.name},
                                      {"type", detail::parameter_type_name(p.type)},
                                      {"default", p.default_value}});

            nlohmann::json layers = nlohmann::json::array();
            for (const LayerDesc& layer : desc.layers)
            {
                nlohmann::json states = nlohmann::json::array();
                for (const StateDesc& state : layer.states)
                {
                    nlohmann::json transitions = nlohmann::json::array();
                    for (const TransitionDesc& t : state.transitions)
                        transitions.push_back(detail::transition_to_json(t));
                    nlohmann::json events = nlohmann::json::array();
                    for (const StateEventDesc& e : state.events)
                        events.push_back({{"normalized_time", e.normalized_time},
                                          {"name", e.name},
                                          {"payload", e.payload}});
                    nlohmann::json json_state{{"name", state.name},
                                              {"clip", detail::asset_to_json(state.clip)},
                                              {"speed", state.speed},
                                              {"speed_parameter", state.speed_parameter},
                                              {"cycle_offset", state.cycle_offset},
                                              {"transitions", transitions},
                                              {"events", events}};
                    if (state.blend_tree)
                        json_state["blend_tree"] = detail::blend_node_to_json(*state.blend_tree);
                    states.push_back(json_state);
                }
                nlohmann::json any_transitions = nlohmann::json::array();
                for (const TransitionDesc& t : layer.any_state_transitions)
                    any_transitions.push_back(detail::transition_to_json(t));
                layers.push_back({{"name", layer.name},
                                  {"weight", layer.weight},
                                  {"mask", detail::asset_to_json(layer.mask)},
                                  {"blend_mode", detail::blend_mode_name(layer.blend_mode)},
                                  {"weight_parameter", layer.weight_parameter},
                                  {"default_state", layer.default_state},
                                  {"states", states},
                                  {"any_state_transitions", any_transitions}});
            }

            return nlohmann::json{{"parameters", parameters}, {"layers", layers}};
        }

        /**
         * @brief Reads an authored controller from JSON (tolerant, field-by-field defaults).
         * @param json A document produced by @ref controller_to_json (older/partial ones degrade).
         * @return The reconstructed controller description.
         */
        inline ControllerDesc controller_from_json(const nlohmann::json& json)
        {
            ControllerDesc desc;
            if (json.contains("parameters"))
                for (const nlohmann::json& p : json.at("parameters"))
                {
                    ParameterDesc parameter;
                    parameter.name = p.value("name", std::string{});
                    parameter.type = detail::parameter_type_from(p.value("type", std::string{"Float"}));
                    parameter.default_value = p.value("default", 0.0f);
                    desc.parameters.push_back(parameter);
                }
            if (json.contains("layers"))
                for (const nlohmann::json& l : json.at("layers"))
                {
                    LayerDesc layer;
                    layer.name = l.value("name", std::string{});
                    layer.weight = l.value("weight", 1.0f);
                    layer.mask = detail::asset_from_json(l.value("mask", nlohmann::json(-1)));
                    layer.blend_mode = detail::blend_mode_from(l.value("blend_mode", std::string{"Override"}));
                    layer.weight_parameter = l.value("weight_parameter", std::string{});
                    layer.default_state = l.value("default_state", std::string{});
                    if (l.contains("states"))
                        for (const nlohmann::json& s : l.at("states"))
                        {
                            StateDesc state;
                            state.name = s.value("name", std::string{});
                            state.clip = detail::asset_from_json(s.value("clip", nlohmann::json(-1)));
                            state.speed = s.value("speed", 1.0f);
                            state.speed_parameter = s.value("speed_parameter", std::string{});
                            state.cycle_offset = s.value("cycle_offset", 0.0f);
                            if (s.contains("blend_tree") && !s.at("blend_tree").is_null())
                                state.blend_tree = detail::blend_node_from_json(s.at("blend_tree"));
                            if (s.contains("transitions"))
                                for (const nlohmann::json& t : s.at("transitions"))
                                    state.transitions.push_back(detail::transition_from_json(t));
                            if (s.contains("events"))
                                for (const nlohmann::json& e : s.at("events"))
                                {
                                    StateEventDesc event;
                                    event.normalized_time = e.value("normalized_time", 0.0f);
                                    event.name = e.value("name", std::string{});
                                    event.payload = e.value("payload", 0);
                                    state.events.push_back(event);
                                }
                            layer.states.push_back(state);
                        }
                    if (l.contains("any_state_transitions"))
                        for (const nlohmann::json& t : l.at("any_state_transitions"))
                            layer.any_state_transitions.push_back(detail::transition_from_json(t));
                    desc.layers.push_back(layer);
                }
            return desc;
        }
    } // namespace Animation
} // namespace SushiEngine
