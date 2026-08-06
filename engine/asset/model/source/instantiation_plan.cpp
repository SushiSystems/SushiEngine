/**************************************************************************/
/* instantiation_plan.cpp                                                 */
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

/**
 * @file instantiation_plan.cpp
 * @brief The pure decision that turns a glTF node graph and its settings into entities.
 *
 * The translation unit `docs/design/model_import.md` §3 places every hard import decision in:
 * which node becomes which entity, when a node splits, how a dropped pivot folds its transform
 * into its children, and how a name collision resolves. It links no device and no editor, which
 * is what makes all of that testable on a machine with no GPU.
 *
 * Two passes. The first classifies each description node — what it becomes, what it could not
 * also become, and whether it survives — and reads no other node. The second emits entities in
 * description order, which inherits the parent-before-child invariant
 * `Geometry::import_gltf_scene` already guarantees rather than re-establishing it here.
 */

#include <SushiEngine/model/instantiation_plan.hpp>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace SushiEngine
{
    namespace Model
    {
        namespace
        {
            constexpr float DEGREES_TO_RADIANS = 0.017453292519943295f;

            /** @brief A local placement, in the decomposed form a node and an entity both use. */
            struct LocalTransform
            {
                Vector3f translation{0.0f, 0.0f, 0.0f};
                Quaternionf rotation{0.0f, 0.0f, 0.0f, 1.0f};
                Vector3f scale{1.0f, 1.0f, 1.0f};
            };

            /** @brief Componentwise product, which is how a scale multiplies a scale or a point. */
            Vector3f multiply(const Vector3f& a, const Vector3f& b) noexcept
            {
                return Vector3f{a.x * b.x, a.y * b.y, a.z * b.z};
            }

            /**
             * @brief The single transform equivalent to applying @p child and then @p parent.
             *
             * Standard translation-rotation-scale composition. A parent scale combined with a
             * child rotation is a shear no decomposed transform can hold, so the composed scale
             * is the componentwise product — the approximation every engine storing a transform
             * this way makes, and exact for the uniform scales a pivot carries.
             *
             * @param parent The outer transform.
             * @param child  The inner transform, expressed in @p parent's frame.
             * @return The composition, expressed in @p parent's own frame.
             */
            LocalTransform compose(const LocalTransform& parent, const LocalTransform& child)
            {
                LocalTransform result;
                result.translation =
                    parent.translation +
                    rotate(parent.rotation, multiply(parent.scale, child.translation));
                result.rotation = normalize(mul(parent.rotation, child.rotation));
                result.scale = multiply(parent.scale, child.scale);
                return result;
            }

            /**
             * @brief The rotation a per-axis correction in degrees describes.
             *
             * Z, then Y, then X — the order `applications/editor/source/ui/panel_widgets.cpp`
             * reads and writes the Inspector's rotation fields in, so a correction typed into an
             * asset's settings and one typed into the Inspector mean the same thing.
             *
             * @param degrees Rotation about each axis, in degrees.
             * @return The unit quaternion; identity when @p degrees is zero.
             */
            Quaternionf rotation_from_euler_degrees(const Vector3f& degrees)
            {
                const Quaternionf about_x = quaternion_axis_angle(Vector3f{1.0f, 0.0f, 0.0f},
                                                                  degrees.x * DEGREES_TO_RADIANS);
                const Quaternionf about_y = quaternion_axis_angle(Vector3f{0.0f, 1.0f, 0.0f},
                                                                  degrees.y * DEGREES_TO_RADIANS);
                const Quaternionf about_z = quaternion_axis_angle(Vector3f{0.0f, 0.0f, 1.0f},
                                                                  degrees.z * DEGREES_TO_RADIANS);
                return normalize(mul(mul(about_z, about_y), about_x));
            }

            /** @brief Whether a scale changes nothing, so a node carrying it can be passed over. */
            bool is_unit_scale(const Vector3f& scale) noexcept
            {
                return scale.x == 1.0f && scale.y == 1.0f && scale.z == 1.0f;
            }

            /** @brief The names one parent's children hold, each with the next suffix to try. */
            using SiblingNames = std::unordered_map<std::string, std::uint32_t>;

            /**
             * @brief A name no sibling already holds, suffixed only when it has to be.
             *
             * Uniqueness is per parent rather than global: two wheels may each hold a node named
             * `Tire`, and renaming one of them would misdescribe the file. The suffix search
             * resumes where the previous collision left off and steps over a suffix the file
             * itself already spells, so `Wheel`, `Wheel` becomes `Wheel`, `Wheel (1)`.
             *
             * @param taken The parent's name table; updated with whatever this call claims.
             * @param base  The name the entity would like to have.
             * @return The claimed name.
             */
            std::string claim_sibling_name(SiblingNames& taken, const std::string& base)
            {
                std::uint32_t suffix = 1;
                {
                    const auto found = taken.find(base);
                    if (found == taken.end())
                    {
                        taken.emplace(base, 1u);
                        return base;
                    }
                    suffix = found->second;
                }

                std::string candidate = base + " (" + std::to_string(suffix) + ")";
                while (taken.find(candidate) != taken.end())
                {
                    ++suffix;
                    candidate = base + " (" + std::to_string(suffix) + ")";
                }
                taken[base] = suffix + 1;
                taken.emplace(candidate, 1u);
                return candidate;
            }

            /** @brief What one description node becomes, decided before anything is emitted. */
            struct NodeClassification
            {
                std::string base_name;

                /** @brief What the node's entity carries; None when it splits or carries nothing. */
                PlannedComponent component = PlannedComponent::None;

                /** @brief Primitives to emit as children; zero unless the node splits. */
                std::uint32_t split_primitives = 0;

                /** @brief Whether the node contributes geometry, on itself or on split children. */
                bool carries_geometry = false;

                /** @brief Whether `preserve_pivots == false` removes this node. */
                bool dropped = false;
            };

            /** @brief The node's own name, or one built from its file index so no row is blank. */
            std::string base_name_of(const Geometry::GLTFNodeDescription& node)
            {
                if (!node.name.empty())
                    return node.name;
                return "node " + std::to_string(node.source_index);
            }

            /**
             * @brief Applies the per-node rules of §5, recording what could not be carried across.
             *
             * Everything that depends on one node alone: whether its mesh survives, which of
             * mesh, light and camera wins when it declares more than one, and whether it is a
             * pivot the settings drop. Nothing here reads another node, which is what lets the
             * emitting pass then run as a single forward sweep.
             *
             * @param description The file's node graph.
             * @param settings    The asset's `.meta` settings.
             * @param report      Receives the skipped counts and their warnings.
             * @return One classification per node, in description order.
             */
            std::vector<NodeClassification> classify_nodes(
                const Geometry::GLTFSceneDescription& description,
                const ModelImportSettings& settings, ModelImportReport& report)
            {
                std::vector<NodeClassification> classifications(description.nodes.size());
                for (std::size_t i = 0; i < description.nodes.size(); ++i)
                {
                    const Geometry::GLTFNodeDescription& node = description.nodes[i];
                    NodeClassification& classification = classifications[i];
                    classification.base_name = base_name_of(node);

                    const bool has_mesh = node.mesh >= 0 && node.primitive_count > 0;
                    const bool skinned = has_mesh && node.skin >= 0;
                    if (skinned)
                    {
                        ++report.skinned_nodes_skipped;
                        report.warnings.push_back(
                            "Node '" + classification.base_name +
                            "' carries a skin, so its mesh is not imported. The node is kept as a "
                            "plain transform; rigged models are imported through the Crowd "
                            "component instead.");
                    }

                    bool light_available = false;
                    if (node.light >= 0 &&
                        static_cast<std::size_t>(node.light) < description.lights.size() &&
                        settings.import_lights)
                    {
                        const Geometry::GLTFLightDescription& light =
                            description.lights[static_cast<std::size_t>(node.light)];
                        if (light.kind == Geometry::GLTFLightKind::Directional)
                        {
                            ++report.lights_skipped_directional;
                            report.warnings.push_back(
                                "Node '" + classification.base_name +
                                "' carries a directional light, which is a scene property in this "
                                "engine rather than a component. The node is imported as a plain "
                                "transform and the scene's own sun is left alone.");
                        }
                        else
                        {
                            light_available = true;
                        }
                    }

                    const bool camera_available =
                        node.camera >= 0 &&
                        static_cast<std::size_t>(node.camera) < description.cameras.size() &&
                        settings.import_cameras;

                    classification.carries_geometry = has_mesh && !skinned;
                    if (classification.carries_geometry)
                    {
                        if (node.primitive_count > 1)
                            classification.split_primitives = node.primitive_count;
                        else
                            classification.component = PlannedComponent::Shape;
                    }
                    else if (light_available)
                    {
                        classification.component = PlannedComponent::Light;
                    }
                    else if (camera_available)
                    {
                        classification.component = PlannedComponent::Camera;
                    }

                    // An entity carries one of geometry, a light and a camera; glTF lets a node
                    // declare several. Geometry wins because it is what an artist sees, and
                    // whatever lost is named rather than dropped without a word.
                    if (classification.carries_geometry && (light_available || camera_available))
                        report.warnings.push_back(
                            "Node '" + classification.base_name +
                            "' carries geometry as well as a light or a camera. The entity keeps "
                            "its geometry; the rest of the node is not imported.");
                    else if (!classification.carries_geometry && light_available &&
                             camera_available)
                        report.warnings.push_back(
                            "Node '" + classification.base_name +
                            "' carries both a light and a camera. The entity keeps the light; the "
                            "camera is not imported.");

                    // A node the file declares nothing on is a pivot. The file's own roots are
                    // never dropped: dropping one would move what the file placed at the top.
                    classification.dropped = !settings.preserve_pivots && node.parent >= 0 &&
                                             node.mesh < 0 && node.light < 0 && node.camera < 0 &&
                                             node.skin < 0;
                }
                return classifications;
            }

            /**
             * @brief Warns once per node the file scales that has collider-carrying descendants.
             *
             * `resolve_collider` sizes a collider by its entity's own scale rather than its world
             * scale, so a scale the file puts on an ancestor never reaches it. The importer does
             * not work around that; §11 has it report it, so an artist finds out at import rather
             * than when something falls through the floor.
             *
             * @param description     The file's node graph.
             * @param classifications What each node became.
             * @param report          Receives one warning per offending node.
             */
            void warn_about_scaled_collider_ancestors(
                const Geometry::GLTFSceneDescription& description,
                const std::vector<NodeClassification>& classifications, ModelImportReport& report)
            {
                // Reverse description order visits every child before its parent, so one sweep
                // lifts "a collider hangs below here" all the way up each chain.
                std::vector<bool> collider_below(description.nodes.size(), false);
                for (std::size_t i = description.nodes.size(); i-- > 0;)
                {
                    const std::int32_t parent = description.nodes[i].parent;
                    if (parent < 0)
                        continue;
                    if (classifications[i].carries_geometry || collider_below[i])
                        collider_below[static_cast<std::size_t>(parent)] = true;
                }

                for (std::size_t i = 0; i < description.nodes.size(); ++i)
                {
                    if (!collider_below[i] || is_unit_scale(description.nodes[i].scale))
                        continue;
                    report.warnings.push_back(
                        "The file scales node '" + classifications[i].base_name +
                        "', and a generated collider is sized by its own entity's scale rather "
                        "than its world scale. Colliders below this node are cooked without that "
                        "scale.");
                }
            }
        } // namespace

        ModelInstantiationPlan plan_model_instantiation(
            const Geometry::GLTFSceneDescription& description, const ModelImportSettings& settings,
            const std::string& file_stem, ModelImportReport& report)
        {
            report = ModelImportReport{};
            report.nodes = static_cast<std::uint32_t>(description.nodes.size());
            report.materials = description.material_count;

            ModelInstantiationPlan plan;
            if (description.nodes.empty())
                return plan;

            const std::vector<NodeClassification> classifications =
                classify_nodes(description, settings, report);

            std::size_t root_count = 0;
            for (const Geometry::GLTFNodeDescription& node : description.nodes)
                if (node.parent < 0)
                    ++root_count;

            const Quaternionf root_correction =
                rotation_from_euler_degrees(settings.root_rotation_degrees);

            // One selectable, movable, deletable thing always comes out, so a file with several
            // roots gains a synthetic one named after itself and a file with one does not.
            const bool synthetic_root = root_count > 1;
            std::unordered_map<std::int32_t, SiblingNames> names_by_parent;
            if (synthetic_root)
            {
                PlannedEntity root;
                root.name = file_stem.empty() ? std::string("Model") : file_stem;
                root.rotation = root_correction;
                names_by_parent[-1].emplace(root.name, 1u);
                plan.entities.push_back(root);
            }

            // Where a node's own entity attaches, or where a dropped node's children attach.
            std::vector<std::int32_t> attach_parent(description.nodes.size(), -1);
            // What a dropped node leaves for each child to absorb; identity for a survivor.
            std::vector<LocalTransform> child_fold(description.nodes.size());

            for (std::size_t i = 0; i < description.nodes.size(); ++i)
            {
                const Geometry::GLTFNodeDescription& node = description.nodes[i];
                const NodeClassification& classification = classifications[i];

                std::int32_t parent_entity = synthetic_root ? 0 : -1;
                LocalTransform inherited;
                if (node.parent >= 0)
                {
                    const std::size_t parent = static_cast<std::size_t>(node.parent);
                    parent_entity = attach_parent[parent];
                    inherited = child_fold[parent];
                }

                // The scale factor multiplies every local translation, which scales the composed
                // world position without ever entering the parent chain and compounding there.
                LocalTransform local;
                local.translation = node.translation * settings.scale_factor;
                local.rotation = node.rotation;
                local.scale = node.scale;
                const LocalTransform placement = compose(inherited, local);

                if (classification.dropped)
                {
                    ++report.pivots_dropped;
                    attach_parent[i] = parent_entity;
                    child_fold[i] = placement;
                    continue;
                }

                PlannedEntity entity;
                entity.name =
                    claim_sibling_name(names_by_parent[parent_entity], classification.base_name);
                entity.parent = parent_entity;
                entity.translation = placement.translation;
                entity.rotation = placement.rotation;
                entity.scale = placement.scale;
                entity.source_node = node.source_index;
                entity.component = classification.component;

                // The correction rotates the whole imported subtree, so it belongs on its root
                // and nowhere else: applying it per node would compound it once per level.
                if (parent_entity < 0)
                    entity.rotation = normalize(mul(root_correction, entity.rotation));

                if (classification.component == PlannedComponent::Shape)
                {
                    // The factor lands on the entity that carries geometry because that is where
                    // `resolve_collider` reads a scale from.
                    entity.scale = multiply(entity.scale,
                                            Vector3f{settings.scale_factor, settings.scale_factor,
                                                     settings.scale_factor});
                    entity.generate_collider = settings.generate_colliders;
                    ++report.primitives_imported;
                }
                else if (classification.component == PlannedComponent::Light)
                {
                    entity.light = node.light;
                    ++report.lights_imported;
                }
                else if (classification.component == PlannedComponent::Camera)
                {
                    entity.camera = node.camera;
                    ++report.cameras_imported;
                }

                plan.entities.push_back(entity);
                const std::int32_t entity_index =
                    static_cast<std::int32_t>(plan.entities.size()) - 1;
                attach_parent[i] = entity_index;

                // An entity holds one mesh and one material, so a node with several primitives
                // cannot be one entity. It stays the visible pivot and its primitives hang off
                // it at identity, which keeps the pivot the artist authored where it was.
                for (std::uint32_t primitive = 0; primitive < classification.split_primitives;
                     ++primitive)
                {
                    PlannedEntity part;
                    part.name = claim_sibling_name(names_by_parent[entity_index],
                                                   classification.base_name + " (" +
                                                       std::to_string(primitive) + ")");
                    part.parent = entity_index;
                    part.scale = Vector3f{settings.scale_factor, settings.scale_factor,
                                          settings.scale_factor};
                    part.component = PlannedComponent::Shape;
                    part.source_node = node.source_index;
                    part.primitive = primitive;
                    part.generate_collider = settings.generate_colliders;
                    plan.entities.push_back(part);
                    ++report.primitives_imported;
                }
            }

            if (settings.generate_colliders)
                warn_about_scaled_collider_ancestors(description, classifications, report);

            report.entities = static_cast<std::uint32_t>(plan.entities.size());
            return plan;
        }
    } // namespace Model
} // namespace SushiEngine
