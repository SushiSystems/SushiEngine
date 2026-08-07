/**************************************************************************/
/* prefab_output.cpp                                                      */
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

#include <SushiEngine/model_import/prefab_output.hpp>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <vector>

#include <nlohmann/json.hpp>

#include <SushiEngine/model/import_settings.hpp>
#include <SushiEngine/model/import_settings_io.hpp>

#include "prefab_serializer.hpp"

namespace SushiEngine
{
    namespace ModelImport
    {
        using SushiEngine::Geometry::GLTFSceneDescription;
        using SushiEngine::Model::ModelImportReport;
        using SushiEngine::Model::ModelInstantiationPlan;
        using SushiEngine::Model::PlannedComponent;
        using SushiEngine::Model::PlannedEntity;
        using SushiEngine::Simulation::EntityId;
        using SushiEngine::Simulation::IWorldEditor;
        using SushiEngine::Simulation::NULL_ENTITY;

        namespace
        {
            /** @brief Radians to degrees, for the cone angles a light states in radians. */
            constexpr float DEGREES_PER_RADIAN = 57.29577951308232f;

            /**
             * @brief The planned entity's local transform, widened to the scene's scalar.
             *
             * Written out rather than left to an implicit conversion: the planner works in
             * float and an entity transform does not, and a widening that a later reader has
             * to infer is one they will eventually infer wrongly.
             */
            SushiEngine::Simulation::EntityTransform transform_of(const PlannedEntity& entity)
            {
                SushiEngine::Simulation::EntityTransform transform;
                transform.position = Vector3{static_cast<Scalar>(entity.translation.x),
                                             static_cast<Scalar>(entity.translation.y),
                                             static_cast<Scalar>(entity.translation.z)};
                transform.rotation = Quaternion{static_cast<Scalar>(entity.rotation.x),
                                                static_cast<Scalar>(entity.rotation.y),
                                                static_cast<Scalar>(entity.rotation.z),
                                                static_cast<Scalar>(entity.rotation.w)};
                transform.scale = Vector3{static_cast<Scalar>(entity.scale.x),
                                          static_cast<Scalar>(entity.scale.y),
                                          static_cast<Scalar>(entity.scale.z)};
                return transform;
            }

            /** @brief Applies the file's light @p index to @p id. */
            void apply_light(IWorldEditor& world, EntityId id,
                             const GLTFSceneDescription& description, std::int32_t index)
            {
                if (index < 0 ||
                    static_cast<std::size_t>(index) >= description.lights.size())
                    return;
                const auto& source = description.lights[static_cast<std::size_t>(index)];

                SushiEngine::Simulation::LightParameters light;
                light.color = Vector3{static_cast<Scalar>(source.color[0]),
                                      static_cast<Scalar>(source.color[1]),
                                      static_cast<Scalar>(source.color[2])};
                light.intensity = source.intensity;
                light.range = source.range;
                light.is_spot = source.kind == SushiEngine::Geometry::GLTFLightKind::Spot;
                // glTF states cone half-angles in radians and the authoring form takes
                // degrees; the planner drops directional lights before this is reached, which
                // is why there is no branch for them here.
                light.inner_degrees = source.spot_inner_cone_radians * DEGREES_PER_RADIAN;
                light.outer_degrees = source.spot_outer_cone_radians * DEGREES_PER_RADIAN;

                world.set_has_light(id, true);
                world.set_light_parameters(id, light);
            }

            /** @brief Applies the file's camera @p index to @p id. */
            void apply_camera(IWorldEditor& world, EntityId id,
                              const GLTFSceneDescription& description, std::int32_t index)
            {
                if (index < 0 ||
                    static_cast<std::size_t>(index) >= description.cameras.size())
                    return;
                const auto& source = description.cameras[static_cast<std::size_t>(index)];

                SushiEngine::Simulation::CameraParameters camera =
                    world.camera_parameters(id);
                camera.vertical_fov_radians = source.vertical_field_of_view_radians;
                camera.near_plane = source.near_plane;
                if (source.far_plane > 0.0f)
                    camera.far_plane = source.far_plane;
                // Inactive: importing a model must not take over the view the artist is
                // looking through, and a file with two cameras would otherwise fight itself.
                camera.active = false;
                world.set_camera_parameters(id, camera);
            }
        } // namespace

        EntityId instantiate_plan(IWorldEditor& world, const ModelInstantiationPlan& plan,
                                  const GLTFSceneDescription& description,
                                  const std::string& source_path)
        {
            if (plan.entities.empty())
                return NULL_ENTITY;

            std::vector<EntityId> created;
            created.reserve(plan.entities.size());

            for (const PlannedEntity& entity : plan.entities)
            {
                const bool is_camera = entity.component == PlannedComponent::Camera;
                const EntityId id =
                    is_camera ? world.create_camera(entity.name) : world.create(entity.name);
                created.push_back(id);

                // The planner guarantees parents precede their children, so the parent is
                // always in `created` by the time its child is read.
                if (entity.parent >= 0 &&
                    static_cast<std::size_t>(entity.parent) < created.size())
                    world.set_parent(id, created[static_cast<std::size_t>(entity.parent)]);
                // After parenting, not before: `set_parent` preserves the entity's world pose
                // by recomputing its local transform, which would divide out the very local
                // transform the plan states.
                world.set_transform(id, transform_of(entity));

                switch (entity.component)
                {
                    case PlannedComponent::Shape:
                    {
                        SushiEngine::Simulation::ShapeParameters shape =
                            world.shape_parameters(id);
                        shape.mesh_path = source_path;
                        shape.source_node = entity.source_node;
                        shape.primitive = entity.primitive;
                        // `mesh` is left invalid on purpose: it is a handle from whichever
                        // session imported the file, and this one may have no renderer at all.
                        // `resolve_scene_assets` derives it from the three fields above.
                        //
                        // The Renderer is not optional decoration here: drawing gates on
                        // `visible && has_shape && has_renderer`, and `create` deliberately
                        // makes a bare entity with neither. Without this the geometry resolves
                        // and reports no error, then never appears.
                        world.set_has_renderer(id, true);
                        world.set_has_shape(id, true);
                        world.set_shape_parameters(id, shape);
                        break;
                    }
                    case PlannedComponent::Light:
                        apply_light(world, id, description, entity.light);
                        break;
                    case PlannedComponent::Camera:
                        apply_camera(world, id, description, entity.camera);
                        break;
                    case PlannedComponent::None:
                        // A pivot: a transform and nothing to draw, which is exactly what
                        // `create` already produces — no Renderer, no Shape. Nothing to do.
                        break;
                }
            }

            return created.front();
        }

        bool write_model_prefab(const std::string& asset_path, ModelImportReport& report)
        {
            report = ModelImportReport{};

            GLTFSceneDescription description;
            if (!SushiEngine::Geometry::import_gltf_scene(asset_path.c_str(), description))
                return false;

            // A missing `.meta` is the ordinary case for an asset nobody has configured, not a
            // failure: the settings keep their defaults and the import proceeds with them.
            SushiEngine::Model::ModelImportSettings settings;
            (void)SushiEngine::Model::load_model_import_settings(asset_path, settings);

            const std::string stem = std::filesystem::path(asset_path).stem().string();
            const ModelInstantiationPlan plan =
                SushiEngine::Model::plan_model_instantiation(description, settings, stem, report);
            if (plan.entities.empty())
                return false;

            // A scratch world rather than emitting the entity records directly. The record
            // shape has exactly one writer -- `Detail::write_entity_record` -- and this keeps
            // it that way: a field added to an entity is carried into imported prefabs the day
            // it is added, with no edit here. It costs one simulation per import, which is an
            // authoring action rather than a frame.
            const auto simulation = SushiEngine::Simulation::create_simulation();
            if (simulation == nullptr)
                return false;
            IWorldEditor& world = simulation->world();
            for (const EntityId id : world.entities())
                world.destroy(id);

            const EntityId root = instantiate_plan(world, plan, description, asset_path);
            if (root == NULL_ENTITY)
                return false;

            const nlohmann::json document = SushiEngine::Scene::capture_prefab(world, root);
            if (document["entities"].empty())
                return false;

            // Written beside the source with the extension appended, the convention `.meta`
            // already follows, so a `.glb` and a `.gltf` of the same stem cannot collide.
            const std::string target = asset_path + ".sushiprefab";
            std::ofstream file(target);
            if (!file || !(file << document.dump(2)))
            {
                // A half-written prefab is worse than none: every scene referencing it would
                // fail to parse it on its next open. Every failure above this line happens
                // before the file is touched, and this one removes what it managed to write.
                file.close();
                std::error_code error;
                std::filesystem::remove(target, error);
                return false;
            }
            return true;
        }
    } // namespace ModelImport
} // namespace SushiEngine
