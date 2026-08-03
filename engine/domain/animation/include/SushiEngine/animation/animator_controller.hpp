/**************************************************************************/
/* animator_controller.hpp                                               */
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
 * @file animator_controller.hpp
 * @brief The compiled Animator controller: a flat, index-linked blob and its view.
 *
 * The Animator is authored as a graph (states, transitions, parameters) and compiled to a
 * relocatable `.sushictrl` blob of POD record arrays — parameters, layers, states,
 * transitions, conditions, events — cross-referenced by index spans (design §4.3).
 * Evaluation is a data interpreter over the view (@ref animator_step), no virtual dispatch
 * and no allocation, so it is device-visible and the OCP seam for new state kinds. A3 states
 * hold a single clip; blend-tree states arrive in A4 as a new record kind, not a new class.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <SushiEngine/animation/asset_id.hpp>
#include <SushiEngine/animation/blend_tree.hpp>
#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp> // detail::align_up

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief A controller parameter's type. */
        enum class ParameterType : std::uint32_t
        {
            Float = 0,
            Int = 1,
            Bool = 2,
            Trigger = 3
        };

        /** @brief How a condition compares a parameter to its threshold. */
        enum class Comparator : std::uint32_t
        {
            Greater = 0, /**< param > threshold (float/int). */
            Less = 1,    /**< param < threshold (float/int). */
            Equals = 2,  /**< param == threshold (int). */
            NotEquals = 3, /**< param != threshold (int). */
            If = 4,      /**< bool/trigger is true. */
            IfNot = 5    /**< bool is false. */
        };

        /** @brief When a transition may interrupt one already in progress. */
        enum class InterruptionSource : std::uint32_t
        {
            None = 0,          /**< Never interrupt an active transition. */
            CurrentState = 1,  /**< The current state's transitions may interrupt. */
            NextState = 2      /**< The destination state's transitions may interrupt. */
        };

        /** @brief How a layer folds onto the layers beneath it. */
        enum class LayerBlendMode : std::uint32_t
        {
            Override = 0, /**< Blend toward the layer's pose by weight × mask (nlerp). */
            Additive = 1  /**< Add the layer's baked delta pose scaled by weight × mask (FMA). */
        };

        // MAX_PARAMETERS and MAX_LAYERS are defined in animator_components.hpp (the low-level
        // column header this file's blob shares them with), reached via blend_tree.hpp above.

        // --- Cooked POD records (blob storage) ----------------------------------------

        /** @brief One parameter: its name hash, type, and default. */
        struct ParameterRecord
        {
            NameHash name = 0;
            std::uint32_t type = 0;
            float default_value = 0.0f;
        };

        /** @brief One layer: its state span, any-state transition span, default state, weight. */
        struct LayerRecord
        {
            std::uint32_t state_base = 0;
            std::uint32_t state_count = 0;
            std::uint32_t any_transition_base = 0;
            std::uint32_t any_transition_count = 0;
            std::int32_t default_state = 0;
            float weight = 1.0f;
            AssetId mask = INVALID_ASSET;       /**< Avatar mask asset gating this layer, or none. */
            std::uint32_t blend_mode = 0;       /**< @ref LayerBlendMode. */
            std::int32_t weight_parameter = -1; /**< Parameter driving the layer weight, or -1. */
            std::uint32_t pad = 0;
        };

        /** @brief One state: its clip or blend tree, speed, and transition/event spans. */
        struct StateRecord
        {
            AssetId clip = INVALID_ASSET;
            float speed = 1.0f;
            std::int32_t speed_parameter = -1; /**< Parameter index scaling speed, or -1. */
            float cycle_offset = 0.0f;
            std::uint32_t transition_base = 0;
            std::uint32_t transition_count = 0;
            std::uint32_t event_base = 0;
            std::uint32_t event_count = 0;
            NameHash name = 0;
            std::int32_t blend_tree = -1; /**< Root blend-tree node index, or -1 for a single clip. */
            std::uint32_t pad = 0;
        };

        /** @brief One transition: destination, conditions, exit time, and crossfade shape. */
        struct TransitionRecord
        {
            std::uint32_t destination_state = 0;
            std::uint32_t condition_base = 0;
            std::uint32_t condition_count = 0;
            std::uint32_t has_exit_time = 0;
            float exit_time = 0.0f;        /**< Normalized source time the exit is armed at. */
            float duration = 0.0f;         /**< Crossfade length, normalized to the source clip. */
            float offset = 0.0f;           /**< Normalized start time in the destination. */
            std::uint32_t interruption = 0; /**< @ref InterruptionSource. */
        };

        /** @brief One condition: a parameter, comparator, and threshold. */
        struct ConditionRecord
        {
            std::uint32_t parameter_index = 0;
            std::uint32_t comparator = 0;
            float threshold = 0.0f;
            std::uint32_t pad = 0;
        };

        /** @brief One animation event on a state's clip: when it fires and what it carries. */
        struct EventRecord
        {
            float normalized_time = 0.0f;
            std::int32_t payload = 0;
            NameHash name = 0;
        };

        /** @brief Magic tag at the head of a `.sushictrl` blob. */
        constexpr char CONTROLLER_BLOB_MAGIC[8] = {'S', 'U', 'S', 'H', 'C', 'T', 'R', 'L'};

        /** @brief Current `.sushictrl` format version (2 adds blend-tree and layer-mask sections). */
        constexpr std::uint32_t CONTROLLER_BLOB_VERSION = 2;

        /** @brief The fixed header at offset 0 of a controller blob. */
        struct ControllerBlobHeader
        {
            char magic[8];
            std::uint32_t version;
            std::uint32_t parameter_count;
            std::uint32_t layer_count;
            std::uint32_t state_count;
            std::uint32_t transition_count;
            std::uint32_t condition_count;
            std::uint32_t event_count;
            std::uint32_t node_count;
            std::uint32_t child_count;
            std::uint32_t pair_count;
            std::uint32_t total_size;
            std::uint32_t parameters_offset;
            std::uint32_t layers_offset;
            std::uint32_t states_offset;
            std::uint32_t transitions_offset;
            std::uint32_t conditions_offset;
            std::uint32_t events_offset;
            std::uint32_t nodes_offset;
            std::uint32_t children_offset;
            std::uint32_t pairs_offset;
        };

        /**
         * @brief A non-owning, immutable view of a compiled controller (aliases a blob).
         *
         * Trivially copyable. The interpreter walks these arrays by the index spans the
         * records carry; nothing here allocates or chases a pointer beyond the flat arrays.
         */
        struct ControllerView
        {
            std::uint32_t parameter_count = 0;
            std::uint32_t layer_count = 0;
            std::uint32_t state_count = 0;
            std::uint32_t node_count = 0;
            std::uint32_t child_count = 0;
            std::uint32_t pair_count = 0;
            const ParameterRecord* parameters = nullptr;
            const LayerRecord* layers = nullptr;
            const StateRecord* states = nullptr;
            const TransitionRecord* transitions = nullptr;
            const ConditionRecord* conditions = nullptr;
            const EventRecord* events = nullptr;
            const BlendTreeNodeRecord* nodes = nullptr;
            const BlendTreeChildRecord* children = nullptr;
            const BlendPairRecord* pairs = nullptr;

            /** @brief Whether the view points at real data. */
            bool valid() const noexcept { return layers != nullptr && layer_count > 0; }

            /**
             * @brief The index of a parameter by name hash.
             * @param name The FNV-1a 64 hash of the parameter name.
             * @return The parameter index, or -1 if none matches.
             */
            int find_parameter(NameHash name) const noexcept
            {
                for (std::uint32_t i = 0; i < parameter_count; ++i)
                    if (parameters[i].name == name)
                        return static_cast<int>(i);
                return -1;
            }
        };

        // --- Host authoring description ------------------------------------------------

        struct ConditionDesc
        {
            std::string parameter;
            Comparator comparator = Comparator::Greater;
            float threshold = 0.0f;
        };

        struct TransitionDesc
        {
            std::string destination;                 /**< Destination state name (in the layer). */
            std::vector<ConditionDesc> conditions;
            bool has_exit_time = false;
            float exit_time = 1.0f;
            float duration = 0.0f;
            float offset = 0.0f;
            InterruptionSource interruption = InterruptionSource::None;
        };

        struct StateEventDesc
        {
            float normalized_time = 0.0f;
            std::string name;
            std::int32_t payload = 0;
        };

        struct BlendTreeNodeDesc; // a child may nest another node

        /**
         * @brief One authored blend-tree child: a clip leaf or a nested node, and its coordinates.
         *
         * Exactly one of @c clip and @c child_node is set. Parameter names are resolved to
         * indices at compile.
         */
        struct BlendChildDesc
        {
            AssetId clip = INVALID_ASSET;                    /**< Leaf clip, or @ref INVALID_ASSET. */
            std::shared_ptr<BlendTreeNodeDesc> child_node;   /**< Nested node, or null. */
            float threshold = 0.0f;                          /**< 1D coordinate. */
            float position_x = 0.0f;                         /**< 2D coordinate, X. */
            float position_y = 0.0f;                         /**< 2D coordinate, Y. */
            std::string parameter;                           /**< Direct parent's driving parameter. */
            float speed = 1.0f;                              /**< Per-child rate multiplier (reserved). */
        };

        /** @brief One authored blend-tree node: its kind, driving parameters, and children. */
        struct BlendTreeNodeDesc
        {
            BlendTreeType type = BlendTreeType::Simple1D;
            std::string parameter_x; /**< 1D / 2D-X driving parameter. */
            std::string parameter_y; /**< 2D-Y driving parameter (empty for 1D / Direct). */
            bool normalize = true;   /**< Direct: normalise child weights to sum 1. */
            std::vector<BlendChildDesc> children;
        };

        struct StateDesc
        {
            std::string name;
            AssetId clip = INVALID_ASSET; /**< The single clip, when @c blend_tree is null. */
            std::shared_ptr<BlendTreeNodeDesc> blend_tree; /**< A blend tree, or null for a clip. */
            float speed = 1.0f;
            std::string speed_parameter; /**< Empty for none. */
            float cycle_offset = 0.0f;
            std::vector<TransitionDesc> transitions;
            std::vector<StateEventDesc> events;
        };

        struct LayerDesc
        {
            std::string name;
            float weight = 1.0f;
            std::vector<StateDesc> states;
            std::vector<TransitionDesc> any_state_transitions;
            std::string default_state;                        /**< Empty defaults to the first state. */
            AssetId mask = INVALID_ASSET;                     /**< Avatar mask gating this layer, or none. */
            LayerBlendMode blend_mode = LayerBlendMode::Override;
            std::string weight_parameter;                     /**< Parameter driving layer weight, or empty. */
        };

        struct ParameterDesc
        {
            std::string name;
            ParameterType type = ParameterType::Float;
            float default_value = 0.0f;
        };

        /** @brief A controller as authored: the input to @ref compile_controller_blob. */
        struct ControllerDesc
        {
            std::vector<ParameterDesc> parameters;
            std::vector<LayerDesc> layers;
        };

        namespace detail
        {
            /** @brief Resolves a state name to its index within one layer, or -1. */
            inline int find_state(const LayerDesc& layer, const std::string& name)
            {
                for (std::size_t i = 0; i < layer.states.size(); ++i)
                    if (layer.states[i].name == name)
                        return static_cast<int>(i);
                return -1;
            }

            /** @brief Resolves a parameter name to its index, or -1. */
            inline int find_parameter(const ControllerDesc& desc, const std::string& name)
            {
                for (std::size_t i = 0; i < desc.parameters.size(); ++i)
                    if (desc.parameters[i].name == name)
                        return static_cast<int>(i);
                return -1;
            }

            /** @brief Whether a node kind reads a precomputed gradient-band pair table. */
            inline bool is_freeform(BlendTreeType type)
            {
                return type == BlendTreeType::FreeformDirectional2D ||
                       type == BlendTreeType::FreeformCartesian2D;
            }

            /**
             * @brief Bakes the gradient-band pair table for a freeform node's children.
             *
             * For every ordered pair (i, j) appends the vector from child i to child j in the
             * node's blend metric and `1/|delta|²` — cartesian difference, or
             * `(signed-angle, magnitude-difference)` for the directional variant.
             */
            inline void bake_pairs(const std::vector<BlendChildDesc>& children, bool directional,
                                   std::vector<BlendPairRecord>& pairs)
            {
                const std::size_t n = children.size();
                for (std::size_t i = 0; i < n; ++i)
                {
                    const float ix = children[i].position_x;
                    const float iy = children[i].position_y;
                    const float mag_i = std::sqrt(ix * ix + iy * iy);
                    for (std::size_t j = 0; j < n; ++j)
                    {
                        BlendPairRecord pair;
                        if (i != j)
                        {
                            const float jx = children[j].position_x;
                            const float jy = children[j].position_y;
                            if (directional)
                            {
                                const float mag_j = std::sqrt(jx * jx + jy * jy);
                                pair.delta_x = signed_angle_2d(ix, iy, jx, jy);
                                pair.delta_y = mag_j - mag_i;
                            }
                            else
                            {
                                pair.delta_x = jx - ix;
                                pair.delta_y = jy - iy;
                            }
                            const float len_sq = pair.delta_x * pair.delta_x + pair.delta_y * pair.delta_y;
                            pair.inv_length_sq = len_sq > 1e-12f ? 1.0f / len_sq : 0.0f;
                        }
                        pairs.push_back(pair);
                    }
                }
            }

            /**
             * @brief Flattens an authored blend-tree node into the global node/child/pair arrays.
             *
             * Subtree children are flattened first so this node's child records stay contiguous.
             * A Simple1D node's children are emitted sorted by threshold (the resolver assumes it).
             *
             * @return The flattened node's index, or -1 if a referenced parameter name is unknown.
             */
            inline int flatten_blend_tree(const ControllerDesc& controller,
                                          const BlendTreeNodeDesc& node,
                                          std::vector<BlendTreeNodeRecord>& nodes,
                                          std::vector<BlendTreeChildRecord>& children,
                                          std::vector<BlendPairRecord>& pairs)
            {
                if (node.children.size() > MAX_BLEND_TREE_CHILDREN)
                    return -1;

                std::vector<BlendChildDesc> ordered = node.children;
                if (node.type == BlendTreeType::Simple1D)
                    std::sort(ordered.begin(), ordered.end(),
                              [](const BlendChildDesc& a, const BlendChildDesc& b)
                              { return a.threshold < b.threshold; });

                std::vector<int> child_node_index(ordered.size(), -1);
                for (std::size_t k = 0; k < ordered.size(); ++k)
                {
                    if (ordered[k].child_node)
                    {
                        const int index = flatten_blend_tree(controller, *ordered[k].child_node, nodes,
                                                             children, pairs);
                        if (index < 0)
                            return -1;
                        child_node_index[k] = index;
                    }
                }

                const int self = static_cast<int>(nodes.size());
                BlendTreeNodeRecord record;
                record.type = static_cast<std::uint32_t>(node.type);
                record.parameter_x =
                    node.parameter_x.empty() ? -1 : find_parameter(controller, node.parameter_x);
                record.parameter_y =
                    node.parameter_y.empty() ? -1 : find_parameter(controller, node.parameter_y);
                if ((!node.parameter_x.empty() && record.parameter_x < 0) ||
                    (!node.parameter_y.empty() && record.parameter_y < 0))
                    return -1;
                record.normalize = node.normalize ? 1u : 0u;
                record.child_base = static_cast<std::uint32_t>(children.size());
                record.child_count = static_cast<std::uint32_t>(ordered.size());
                record.pair_base = static_cast<std::uint32_t>(pairs.size());
                if (is_freeform(node.type))
                    bake_pairs(ordered, node.type == BlendTreeType::FreeformDirectional2D, pairs);
                nodes.push_back(record);

                for (std::size_t k = 0; k < ordered.size(); ++k)
                {
                    BlendTreeChildRecord child;
                    if (child_node_index[k] >= 0)
                    {
                        child.child_node = child_node_index[k];
                        child.clip = -1;
                    }
                    else
                    {
                        child.clip = ordered[k].clip == INVALID_ASSET
                                         ? -1
                                         : static_cast<std::int32_t>(ordered[k].clip);
                        child.child_node = -1;
                    }
                    child.threshold = ordered[k].threshold;
                    child.position_x = ordered[k].position_x;
                    child.position_y = ordered[k].position_y;
                    child.parameter =
                        ordered[k].parameter.empty() ? -1 : find_parameter(controller, ordered[k].parameter);
                    child.speed = ordered[k].speed;
                    children.push_back(child);
                }
                return self;
            }
        } // namespace detail

        /**
         * @brief Compiles a controller description into a relocatable `.sushictrl` blob.
         *
         * Flattens layers, states, transitions, conditions, and events into index-linked POD
         * arrays: a state's transition indices are into one global transition array, a
         * transition's conditions into one global condition array, and so on. State and
         * parameter names in the desc are resolved to indices here, so the runtime never
         * matches a string. Any-State transitions compile to a per-layer span.
         *
         * @param desc The authored controller.
         * @param out  Receives the blob bytes; cleared first. Empty on failure.
         * @return True on success; false if a referenced state/parameter name is unknown, or
         *         a layer/parameter count exceeds its cap.
         */
        inline bool compile_controller_blob(const ControllerDesc& desc, std::vector<std::byte>& out)
        {
            out.clear();
            if (desc.layers.empty() || desc.layers.size() > MAX_LAYERS ||
                desc.parameters.size() > MAX_PARAMETERS)
                return false;

            std::vector<ParameterRecord> parameters;
            std::vector<LayerRecord> layers;
            std::vector<StateRecord> states;
            std::vector<TransitionRecord> transitions;
            std::vector<ConditionRecord> conditions;
            std::vector<EventRecord> events;
            std::vector<BlendTreeNodeRecord> nodes;
            std::vector<BlendTreeChildRecord> children;
            std::vector<BlendPairRecord> pairs;

            for (const ParameterDesc& p : desc.parameters)
            {
                ParameterRecord record;
                record.name = hash_name(p.name.c_str());
                record.type = static_cast<std::uint32_t>(p.type);
                record.default_value = p.default_value;
                parameters.push_back(record);
            }

            // Compiles a transition list against a layer, appending to the global arrays.
            const auto compile_transitions =
                [&](const LayerDesc& layer, const std::vector<TransitionDesc>& source,
                    std::uint32_t& base, std::uint32_t& count) -> bool
            {
                base = static_cast<std::uint32_t>(transitions.size());
                count = 0;
                for (const TransitionDesc& t : source)
                {
                    // "Exit" is the reserved sink of the editor's graph: at this flat level it
                    // returns to the layer's entry (default) state, so exit transitions compile
                    // and behave rather than dangling.
                    int destination = detail::find_state(layer, t.destination);
                    if (destination < 0 && (t.destination == "Exit" || t.destination == "exit"))
                        destination = layer.default_state.empty()
                                          ? 0
                                          : detail::find_state(layer, layer.default_state);
                    if (destination < 0)
                        return false;
                    TransitionRecord record;
                    record.destination_state = static_cast<std::uint32_t>(destination);
                    record.has_exit_time = t.has_exit_time ? 1u : 0u;
                    record.exit_time = t.exit_time;
                    record.duration = t.duration;
                    record.offset = t.offset;
                    record.interruption = static_cast<std::uint32_t>(t.interruption);
                    record.condition_base = static_cast<std::uint32_t>(conditions.size());
                    record.condition_count = static_cast<std::uint32_t>(t.conditions.size());
                    for (const ConditionDesc& c : t.conditions)
                    {
                        const int parameter = detail::find_parameter(desc, c.parameter);
                        if (parameter < 0)
                            return false;
                        ConditionRecord condition;
                        condition.parameter_index = static_cast<std::uint32_t>(parameter);
                        condition.comparator = static_cast<std::uint32_t>(c.comparator);
                        condition.threshold = c.threshold;
                        conditions.push_back(condition);
                    }
                    transitions.push_back(record);
                    ++count;
                }
                return true;
            };

            for (const LayerDesc& layer : desc.layers)
            {
                LayerRecord layer_record;
                layer_record.weight = layer.weight;
                layer_record.mask = layer.mask;
                layer_record.blend_mode = static_cast<std::uint32_t>(layer.blend_mode);
                layer_record.weight_parameter =
                    layer.weight_parameter.empty() ? -1
                                                   : detail::find_parameter(desc, layer.weight_parameter);
                if (!layer.weight_parameter.empty() && layer_record.weight_parameter < 0)
                    return false;
                layer_record.state_base = static_cast<std::uint32_t>(states.size());
                layer_record.state_count = static_cast<std::uint32_t>(layer.states.size());
                const int default_state =
                    layer.default_state.empty() ? 0 : detail::find_state(layer, layer.default_state);
                if (default_state < 0)
                    return false;
                layer_record.default_state = default_state;

                if (!compile_transitions(layer, layer.any_state_transitions,
                                         layer_record.any_transition_base,
                                         layer_record.any_transition_count))
                    return false;

                for (const StateDesc& state : layer.states)
                {
                    StateRecord state_record;
                    state_record.clip = state.clip;
                    state_record.speed = state.speed;
                    state_record.speed_parameter =
                        state.speed_parameter.empty()
                            ? -1
                            : detail::find_parameter(desc, state.speed_parameter);
                    state_record.cycle_offset = state.cycle_offset;
                    state_record.name = hash_name(state.name.c_str());
                    if (state.blend_tree)
                    {
                        const int root = detail::flatten_blend_tree(desc, *state.blend_tree, nodes,
                                                                    children, pairs);
                        if (root < 0)
                            return false;
                        state_record.blend_tree = root;
                    }
                    if (!compile_transitions(layer, state.transitions, state_record.transition_base,
                                             state_record.transition_count))
                        return false;
                    state_record.event_base = static_cast<std::uint32_t>(events.size());
                    state_record.event_count = static_cast<std::uint32_t>(state.events.size());
                    for (const StateEventDesc& e : state.events)
                    {
                        EventRecord event;
                        event.normalized_time = e.normalized_time;
                        event.name = hash_name(e.name.c_str());
                        event.payload = e.payload;
                        events.push_back(event);
                    }
                    states.push_back(state_record);
                }
                layers.push_back(layer_record);
            }

            // Lay out the six arrays at aligned offsets after the header.
            const auto section = [](std::size_t& cursor, std::size_t bytes) -> std::size_t
            {
                const std::size_t offset = cursor;
                cursor = detail::align_up(cursor + bytes, 16);
                return offset;
            };
            std::size_t cursor = detail::align_up(sizeof(ControllerBlobHeader), 16);
            const std::size_t parameters_offset =
                section(cursor, parameters.size() * sizeof(ParameterRecord));
            const std::size_t layers_offset = section(cursor, layers.size() * sizeof(LayerRecord));
            const std::size_t states_offset = section(cursor, states.size() * sizeof(StateRecord));
            const std::size_t transitions_offset =
                section(cursor, transitions.size() * sizeof(TransitionRecord));
            const std::size_t conditions_offset =
                section(cursor, conditions.size() * sizeof(ConditionRecord));
            const std::size_t events_offset = section(cursor, events.size() * sizeof(EventRecord));
            const std::size_t nodes_offset = section(cursor, nodes.size() * sizeof(BlendTreeNodeRecord));
            const std::size_t children_offset =
                section(cursor, children.size() * sizeof(BlendTreeChildRecord));
            const std::size_t pairs_offset = section(cursor, pairs.size() * sizeof(BlendPairRecord));
            const std::size_t total_size = cursor;

            out.assign(total_size, std::byte{0});
            std::byte* base = out.data();

            ControllerBlobHeader header{};
            std::memcpy(header.magic, CONTROLLER_BLOB_MAGIC, sizeof(header.magic));
            header.version = CONTROLLER_BLOB_VERSION;
            header.parameter_count = static_cast<std::uint32_t>(parameters.size());
            header.layer_count = static_cast<std::uint32_t>(layers.size());
            header.state_count = static_cast<std::uint32_t>(states.size());
            header.transition_count = static_cast<std::uint32_t>(transitions.size());
            header.condition_count = static_cast<std::uint32_t>(conditions.size());
            header.event_count = static_cast<std::uint32_t>(events.size());
            header.node_count = static_cast<std::uint32_t>(nodes.size());
            header.child_count = static_cast<std::uint32_t>(children.size());
            header.pair_count = static_cast<std::uint32_t>(pairs.size());
            header.total_size = static_cast<std::uint32_t>(total_size);
            header.parameters_offset = static_cast<std::uint32_t>(parameters_offset);
            header.layers_offset = static_cast<std::uint32_t>(layers_offset);
            header.states_offset = static_cast<std::uint32_t>(states_offset);
            header.transitions_offset = static_cast<std::uint32_t>(transitions_offset);
            header.conditions_offset = static_cast<std::uint32_t>(conditions_offset);
            header.events_offset = static_cast<std::uint32_t>(events_offset);
            header.nodes_offset = static_cast<std::uint32_t>(nodes_offset);
            header.children_offset = static_cast<std::uint32_t>(children_offset);
            header.pairs_offset = static_cast<std::uint32_t>(pairs_offset);
            std::memcpy(base, &header, sizeof(header));

            const auto copy = [&](std::size_t offset, const void* data, std::size_t bytes)
            {
                if (bytes > 0)
                    std::memcpy(base + offset, data, bytes);
            };
            copy(parameters_offset, parameters.data(), parameters.size() * sizeof(ParameterRecord));
            copy(layers_offset, layers.data(), layers.size() * sizeof(LayerRecord));
            copy(states_offset, states.data(), states.size() * sizeof(StateRecord));
            copy(transitions_offset, transitions.data(),
                 transitions.size() * sizeof(TransitionRecord));
            copy(conditions_offset, conditions.data(), conditions.size() * sizeof(ConditionRecord));
            copy(events_offset, events.data(), events.size() * sizeof(EventRecord));
            copy(nodes_offset, nodes.data(), nodes.size() * sizeof(BlendTreeNodeRecord));
            copy(children_offset, children.data(), children.size() * sizeof(BlendTreeChildRecord));
            copy(pairs_offset, pairs.data(), pairs.size() * sizeof(BlendPairRecord));
            return true;
        }

        /**
         * @brief Validates and views a `.sushictrl` blob.
         * @param data First byte of the blob (must outlive the returned view).
         * @param size Bytes available at @p data.
         * @return The controller view, or a default (invalid) view.
         */
        inline ControllerView load_controller_blob(const std::byte* data, std::size_t size) noexcept
        {
            ControllerView view{};
            if (data == nullptr || size < sizeof(ControllerBlobHeader))
                return view;
            ControllerBlobHeader header{};
            std::memcpy(&header, data, sizeof(header));
            if (std::memcmp(header.magic, CONTROLLER_BLOB_MAGIC, sizeof(header.magic)) != 0 ||
                header.version != CONTROLLER_BLOB_VERSION || header.total_size > size ||
                header.layer_count == 0)
                return view;
            view.parameter_count = header.parameter_count;
            view.layer_count = header.layer_count;
            view.state_count = header.state_count;
            view.node_count = header.node_count;
            view.child_count = header.child_count;
            view.pair_count = header.pair_count;
            view.parameters =
                reinterpret_cast<const ParameterRecord*>(data + header.parameters_offset);
            view.layers = reinterpret_cast<const LayerRecord*>(data + header.layers_offset);
            view.states = reinterpret_cast<const StateRecord*>(data + header.states_offset);
            view.transitions =
                reinterpret_cast<const TransitionRecord*>(data + header.transitions_offset);
            view.conditions =
                reinterpret_cast<const ConditionRecord*>(data + header.conditions_offset);
            view.events = reinterpret_cast<const EventRecord*>(data + header.events_offset);
            view.nodes = reinterpret_cast<const BlendTreeNodeRecord*>(data + header.nodes_offset);
            view.children =
                reinterpret_cast<const BlendTreeChildRecord*>(data + header.children_offset);
            view.pairs = reinterpret_cast<const BlendPairRecord*>(data + header.pairs_offset);
            return view;
        }
    } // namespace Animation
} // namespace SushiEngine
