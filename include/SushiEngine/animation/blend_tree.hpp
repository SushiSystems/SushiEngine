/**************************************************************************/
/* blend_tree.hpp                                                        */
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
 * @file blend_tree.hpp
 * @brief Blend-tree records and the weight-resolution interpreter (phase A4).
 *
 * A blend-tree state does not hold one clip but a small tree of nodes that, read against
 * the parameter block, resolves to a weighted set of clip contributions (design §5.2 step
 * 1). Five node kinds are supported — 1D, 2D simple-directional, 2D freeform-directional,
 * 2D freeform-cartesian, and direct — and a child may itself be a nested node, so the tree
 * flattens to two POD arrays (nodes + children) plus a per-pair gradient-band table
 * precomputed at compile for the freeform kinds. @ref resolve_blend_tree walks those arrays
 * with no allocation and no virtual dispatch, so it is the OCP seam for new node kinds (a
 * new kind is a new enum plus one case, not a class). The records live here; the outer
 * `.sushictrl` blob that carries them and the state that points into it live in
 * @ref animator_controller.hpp.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/animation/animator_components.hpp>
#include <SushiEngine/animation/asset_id.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief The blend function a blend-tree node applies to its children. */
        enum class BlendTreeType : std::uint32_t
        {
            Simple1D = 0,              /**< One parameter, segment-lerp over sorted thresholds. */
            SimpleDirectional2D = 1,   /**< Two parameters, angular sectors around an optional centre. */
            FreeformDirectional2D = 2, /**< Two parameters, polar gradient-band interpolation. */
            FreeformCartesian2D = 3,   /**< Two parameters, cartesian gradient-band interpolation. */
            Direct = 4                 /**< One parameter per child drives that child's weight directly. */
        };

        /** @brief Blend-tree children per node (design §4.5). */
        constexpr std::uint32_t MAX_BLEND_TREE_CHILDREN = 16;

        /** @brief Upper bound on flattened contributions one state resolves to. */
        constexpr std::uint32_t MAX_BLEND_CONTRIBUTIONS = 64;

        /**
         * @brief One blend-tree node: its kind, driving parameters, and child/pair spans.
         *
         * @c parameter_x / @c parameter_y index the controller parameter table (the axes a 1D
         * or 2D node reads); a Direct node reads a parameter per child instead. @c pair_base
         * points into the precomputed gradient-band table (@ref BlendPairRecord),
         * @c child_count × @c child_count entries laid out row-major, used only by the freeform
         * kinds.
         */
        struct BlendTreeNodeRecord
        {
            std::uint32_t type = 0;         /**< @ref BlendTreeType. */
            std::int32_t parameter_x = -1;  /**< Parameter driving the 1D / 2D-X axis, or -1. */
            std::int32_t parameter_y = -1;  /**< Parameter driving the 2D-Y axis, or -1. */
            std::uint32_t child_base = 0;   /**< First child in the child array. */
            std::uint32_t child_count = 0;  /**< Children this node blends. */
            std::uint32_t pair_base = 0;    /**< First gradient-band pair (freeform kinds). */
            std::uint32_t normalize = 1;    /**< Direct: normalise child weights to sum 1. */
            float angle_scale = 1.0f;       /**< Reserved directional axis scaling (see resolver). */
        };

        /**
         * @brief One blend-tree child: a clip leaf or a nested node, and its blend coordinates.
         *
         * Exactly one of @c clip and @c child_node is set (the other is -1): a leaf plays a
         * clip, a branch recurses into another node. @c threshold is the 1D coordinate;
         * @c position_x / @c position_y are the 2D coordinate; @c parameter is the driving
         * parameter for a Direct parent. @c speed is the child's playback-rate multiplier
         * (reserved for de-synced blends; the sync-by-normalised-time default ignores it).
         */
        struct BlendTreeChildRecord
        {
            std::int32_t clip = -1;         /**< Clip @ref AssetId as int32, or -1 for a node. */
            std::int32_t child_node = -1;   /**< Nested node index, or -1 for a clip leaf. */
            float threshold = 0.0f;         /**< 1D blend coordinate. */
            float position_x = 0.0f;        /**< 2D blend coordinate, X. */
            float position_y = 0.0f;        /**< 2D blend coordinate, Y. */
            std::int32_t parameter = -1;    /**< Direct parent: the parameter driving this child. */
            float speed = 1.0f;             /**< Per-child playback-rate multiplier (reserved). */
            std::uint32_t pad = 0;
        };

        /**
         * @brief One precomputed gradient-band pair vector for a freeform 2D node.
         *
         * For ordered pair (i, j) at row-major index `pair_base + i * child_count + j`:
         * @c delta_x / @c delta_y is the vector from child i to child j in the node's blend
         * metric (cartesian difference, or `(signed-angle, magnitude-difference)` for
         * directional), and @c inv_length_sq is `1 / |delta|²` (0 when i == j or degenerate).
         * Baked at compile so the runtime never recomputes the pairwise geometry.
         */
        struct BlendPairRecord
        {
            float delta_x = 0.0f;
            float delta_y = 0.0f;
            float inv_length_sq = 0.0f;
            float pad = 0.0f;
        };

        /** @brief One resolved clip contribution: which clip, at what weight and rate. */
        struct BlendContribution
        {
            AssetId clip = INVALID_ASSET; /**< The clip to sample. */
            float weight = 0.0f;          /**< Its share of the pose (weights sum to ~1). */
            float speed = 1.0f;           /**< Its playback-rate multiplier (reserved). */
        };

        namespace detail
        {
            /** @brief Reads a controller parameter as a float, whatever its stored type. */
            inline float read_parameter_float(const AnimatorParameterBlock& parameters,
                                              std::int32_t index) noexcept
            {
                if (index < 0 || static_cast<std::uint32_t>(index) >= MAX_PARAMETERS)
                    return 0.0f;
                const ParameterValue& value = parameters.values[static_cast<std::uint32_t>(index)];
                if (value.type == 1)
                    return static_cast<float>(value.as_int);
                if (value.type == 2 || value.type == 3)
                    return value.as_uint != 0u ? 1.0f : 0.0f;
                return value.as_float;
            }

            /** @brief The signed angle (radians, in (-pi, pi]) from vector a to vector b. */
            inline float signed_angle_2d(float ax, float ay, float bx, float by) noexcept
            {
                const float cross = ax * by - ay * bx;
                const float dot = ax * bx + ay * by;
                return std::atan2(cross, dot);
            }

            /** @brief Weights a Simple1D node: segment-lerp over its sorted child thresholds. */
            inline void weights_1d(const BlendTreeChildRecord* children, std::uint32_t count,
                                   float x, float* out) noexcept
            {
                for (std::uint32_t i = 0; i < count; ++i)
                    out[i] = 0.0f;
                if (count == 0)
                    return;
                if (count == 1 || x <= children[0].threshold)
                {
                    out[0] = 1.0f;
                    return;
                }
                if (x >= children[count - 1].threshold)
                {
                    out[count - 1] = 1.0f;
                    return;
                }
                for (std::uint32_t i = 0; i + 1 < count; ++i)
                {
                    const float lo = children[i].threshold;
                    const float hi = children[i + 1].threshold;
                    if (x >= lo && x <= hi)
                    {
                        const float span = hi - lo;
                        const float a = span > 0.0f ? (x - lo) / span : 0.0f;
                        out[i] = 1.0f - a;
                        out[i + 1] = a;
                        return;
                    }
                }
            }

            /** @brief Weights a Direct node: each child's weight is its driving parameter. */
            inline void weights_direct(const BlendTreeChildRecord* children, std::uint32_t count,
                                       const AnimatorParameterBlock& parameters, bool normalize,
                                       float* out) noexcept
            {
                float sum = 0.0f;
                for (std::uint32_t i = 0; i < count; ++i)
                {
                    float w = read_parameter_float(parameters, children[i].parameter);
                    if (w < 0.0f)
                        w = 0.0f;
                    out[i] = w;
                    sum += w;
                }
                if (normalize && sum > 1e-8f)
                    for (std::uint32_t i = 0; i < count; ++i)
                        out[i] /= sum;
            }

            /**
             * @brief Weights a freeform 2D node by the gradient-band metric (Johansen's method).
             *
             * For each child i, the weight is `min over j of clamp(1 - dot(d_is, d_ij) / |d_ij|²)`,
             * where d_ij is the precomputed pair vector and d_is is child i → sample in the same
             * metric — cartesian difference, or `(signed-angle, magnitude-difference)` for the
             * directional variant. At a sample equal to a child position that child alone survives.
             */
            inline void weights_freeform_2d(const BlendTreeChildRecord* children,
                                            const BlendPairRecord* pairs, std::uint32_t count,
                                            float sx, float sy, bool directional, float* out) noexcept
            {
                const float sample_mag = std::sqrt(sx * sx + sy * sy);
                float sum = 0.0f;
                for (std::uint32_t i = 0; i < count; ++i)
                {
                    const float ix = children[i].position_x;
                    const float iy = children[i].position_y;
                    float dis_x;
                    float dis_y;
                    if (directional)
                    {
                        const float mag_i = std::sqrt(ix * ix + iy * iy);
                        dis_x = signed_angle_2d(ix, iy, sx, sy);
                        dis_y = sample_mag - mag_i;
                    }
                    else
                    {
                        dis_x = sx - ix;
                        dis_y = sy - iy;
                    }
                    float weight = 1.0f;
                    for (std::uint32_t j = 0; j < count; ++j)
                    {
                        if (j == i)
                            continue;
                        const BlendPairRecord& pair = pairs[i * count + j];
                        const float d = dis_x * pair.delta_x + dis_y * pair.delta_y;
                        float h = 1.0f - d * pair.inv_length_sq;
                        if (h <= 0.0f)
                        {
                            weight = 0.0f;
                            break;
                        }
                        if (h < weight)
                            weight = h;
                    }
                    out[i] = weight;
                    sum += weight;
                }
                if (sum > 1e-8f)
                    for (std::uint32_t i = 0; i < count; ++i)
                        out[i] /= sum;
                else if (count > 0)
                    out[0] = 1.0f;
            }

            /**
             * @brief Weights a Simple-Directional 2D node: angular bracket around an optional centre.
             *
             * The children partition into an optional centre (position ~ origin) and directional
             * children on a ring. The sample's angle brackets two ring children (angularly
             * interpolated); its magnitude, against the interpolated ring magnitude, splits weight
             * between the ring and the centre.
             */
            inline void weights_simple_directional(const BlendTreeChildRecord* children,
                                                   std::uint32_t count, float sx, float sy,
                                                   float* out) noexcept
            {
                for (std::uint32_t i = 0; i < count; ++i)
                    out[i] = 0.0f;
                if (count == 0)
                    return;

                std::int32_t centre = -1;
                for (std::uint32_t i = 0; i < count; ++i)
                {
                    const float mag = std::sqrt(children[i].position_x * children[i].position_x +
                                                children[i].position_y * children[i].position_y);
                    if (mag < 1e-5f)
                    {
                        centre = static_cast<std::int32_t>(i);
                        break;
                    }
                }

                const float sample_mag = std::sqrt(sx * sx + sy * sy);
                if (sample_mag < 1e-6f)
                {
                    // At the origin the centre owns the pose; with no centre, share the ring evenly.
                    if (centre >= 0)
                        out[centre] = 1.0f;
                    else
                        for (std::uint32_t i = 0; i < count; ++i)
                            out[i] = 1.0f / static_cast<float>(count);
                    return;
                }

                // Bracket the sample angle by the two nearest ring children on either side.
                std::int32_t lo = -1;
                std::int32_t hi = -1;
                float lo_angle = -3.30f; // below -pi
                float hi_angle = 3.30f;  // above  pi
                for (std::uint32_t i = 0; i < count; ++i)
                {
                    if (static_cast<std::int32_t>(i) == centre)
                        continue;
                    const float angle = signed_angle_2d(sx, sy, children[i].position_x,
                                                         children[i].position_y);
                    if (angle <= 0.0f && angle > lo_angle)
                    {
                        lo_angle = angle;
                        lo = static_cast<std::int32_t>(i);
                    }
                    if (angle >= 0.0f && angle < hi_angle)
                    {
                        hi_angle = angle;
                        hi = static_cast<std::int32_t>(i);
                    }
                }
                if (lo < 0)
                    lo = hi;
                if (hi < 0)
                    hi = lo;
                if (lo < 0)
                {
                    if (centre >= 0)
                        out[centre] = 1.0f;
                    return;
                }

                float ring_lo;
                float ring_hi;
                if (lo == hi)
                {
                    ring_lo = 1.0f;
                    ring_hi = 0.0f;
                }
                else
                {
                    const float span = hi_angle - lo_angle;
                    const float a = span > 1e-6f ? (0.0f - lo_angle) / span : 0.0f;
                    ring_lo = 1.0f - a;
                    ring_hi = a;
                }

                const float mag_lo = std::sqrt(children[lo].position_x * children[lo].position_x +
                                               children[lo].position_y * children[lo].position_y);
                const float mag_hi = std::sqrt(children[hi].position_x * children[hi].position_x +
                                               children[hi].position_y * children[hi].position_y);
                const float ring_mag = ring_lo * mag_lo + ring_hi * mag_hi;
                float radial = ring_mag > 1e-6f ? sample_mag / ring_mag : 1.0f;
                if (radial > 1.0f)
                    radial = 1.0f;

                if (centre >= 0)
                {
                    out[centre] = 1.0f - radial;
                    out[lo] += radial * ring_lo;
                    out[hi] += radial * ring_hi;
                }
                else
                {
                    out[lo] += ring_lo;
                    out[hi] += ring_hi;
                }
            }

            /**
             * @brief Recursively resolves one node into weighted clip contributions.
             * @param inherited The weight this node carries from its parent (1 at the root).
             */
            inline void resolve_node(const BlendTreeNodeRecord* nodes,
                                     const BlendTreeChildRecord* children,
                                     const BlendPairRecord* pairs, std::uint32_t node_index,
                                     const AnimatorParameterBlock& parameters, float inherited,
                                     BlendContribution* out, std::uint32_t& out_count,
                                     std::uint32_t max_out) noexcept
            {
                const BlendTreeNodeRecord& node = nodes[node_index];
                std::uint32_t child_count = node.child_count;
                if (child_count > MAX_BLEND_TREE_CHILDREN)
                    child_count = MAX_BLEND_TREE_CHILDREN;
                const BlendTreeChildRecord* child = children + node.child_base;

                float weights[MAX_BLEND_TREE_CHILDREN];
                const float x = read_parameter_float(parameters, node.parameter_x);
                const float y = read_parameter_float(parameters, node.parameter_y);
                switch (static_cast<BlendTreeType>(node.type))
                {
                    case BlendTreeType::Simple1D:
                        weights_1d(child, child_count, x, weights);
                        break;
                    case BlendTreeType::SimpleDirectional2D:
                        weights_simple_directional(child, child_count, x, y, weights);
                        break;
                    case BlendTreeType::FreeformDirectional2D:
                        weights_freeform_2d(child, pairs + node.pair_base, child_count, x, y, true,
                                            weights);
                        break;
                    case BlendTreeType::FreeformCartesian2D:
                        weights_freeform_2d(child, pairs + node.pair_base, child_count, x, y, false,
                                            weights);
                        break;
                    case BlendTreeType::Direct:
                        weights_direct(child, child_count, parameters, node.normalize != 0, weights);
                        break;
                    default:
                        for (std::uint32_t i = 0; i < child_count; ++i)
                            weights[i] = 0.0f;
                        break;
                }

                for (std::uint32_t i = 0; i < child_count; ++i)
                {
                    const float w = inherited * weights[i];
                    if (w <= 1e-6f)
                        continue;
                    if (child[i].child_node >= 0)
                    {
                        resolve_node(nodes, children, pairs,
                                     static_cast<std::uint32_t>(child[i].child_node), parameters, w,
                                     out, out_count, max_out);
                    }
                    else if (child[i].clip >= 0 && out_count < max_out)
                    {
                        out[out_count].clip = static_cast<AssetId>(child[i].clip);
                        out[out_count].weight = w;
                        out[out_count].speed = child[i].speed;
                        ++out_count;
                    }
                }
            }
        } // namespace detail

        /**
         * @brief Resolves a blend tree to weighted clip contributions against the parameters.
         *
         * @param nodes      The controller's flattened blend-tree node array.
         * @param children   The controller's flattened child array.
         * @param pairs      The controller's precomputed gradient-band pair table.
         * @param root_node  The state's root node index.
         * @param parameters The animator's parameter block.
         * @param out        Receives the contributions (cleared logically via @p out_count).
         * @param max_out    Capacity of @p out.
         * @return The number of contributions written (0 if the tree is empty).
         */
        inline std::uint32_t resolve_blend_tree(const BlendTreeNodeRecord* nodes,
                                                const BlendTreeChildRecord* children,
                                                const BlendPairRecord* pairs,
                                                std::uint32_t root_node,
                                                const AnimatorParameterBlock& parameters,
                                                BlendContribution* out, std::uint32_t max_out) noexcept
        {
            std::uint32_t count = 0;
            if (nodes != nullptr && children != nullptr && max_out > 0)
                detail::resolve_node(nodes, children, pairs, root_node, parameters, 1.0f, out, count,
                                     max_out);
            return count;
        }
    } // namespace Animation
} // namespace SushiEngine
