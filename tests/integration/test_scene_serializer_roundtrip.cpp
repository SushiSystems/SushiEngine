/**************************************************************************/
/* test_scene_serializer_roundtrip.cpp                                    */
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

// The editor's capture_scene/apply_scene round-trip backs four separate user
// features at once — Save/Load, Undo/Redo, and the Play→Stop restore — so a
// component the capture forgets is a component all four silently destroy.
// These tests pin the components that were once missing (lights, decals,
// materials and their texture paths) through the real simulation, plus the
// Authoring::CommandHistory path that turns the same snapshots into undo steps.
//
// The soft-body cases at the end also pin the *shape* the capture takes, which is
// the one place this format is not uniform: a cooked asset is megabytes, so a
// scene file inlines it (the file has to open on its own) while an in-memory
// snapshot names it into a caller-owned blob table (fifty undo steps must not
// hold fifty cooks). Both halves are asserted, and so is the refusal in between —
// a snapshot whose asset cannot be resolved restores no body at all.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/cooking/soft_body_cooker.hpp>
#include <SushiEngine/simulation/simulation.hpp>

#include <SushiEngine/authoring/command_history.hpp>
#include "scene_blob_table.hpp"
#include "scene_serializer.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    /** @brief Empties the world, demo seeds included, so a test builds from zero. */
    void clear_world(IWorldEditor& world)
    {
        for (const EntityId id : world.entities())
            world.destroy(id);
    }

    /** @brief The first entity carrying @p name, or NULL_ENTITY. */
    EntityId find_by_name(IWorldEditor& world, const std::string& name)
    {
        for (const EntityId id : world.entities())
            if (world.name(id) == name)
                return id;
        return NULL_ENTITY;
    }

    /** @brief The authored light every test round-trips. */
    LightParameters reference_light()
    {
        LightParameters p;
        p.color = Vector3{0.25, 0.5, 0.75};
        p.intensity = 12.5f;
        p.range = 42.0f;
        p.is_spot = true;
        p.inner_degrees = 15.5f;
        p.outer_degrees = 30.25f;
        p.casts_shadows = true;
        return p;
    }

    /** @brief The authored decal, ids and paths both set (the live-handle convention). */
    DecalParameters reference_decal()
    {
        DecalParameters p;
        p.color = Vector3{0.125, 0.25, 0.375};
        p.half_extents = Vector3{2.0, 3.0, 0.5};
        p.opacity = 0.625f;
        p.albedo_map = 7u;
        p.orm_map = 9u;
        p.albedo_map_path = "textures/decal_albedo.png";
        p.orm_map_path = "textures/decal_orm.png";
        return p;
    }

    /** @brief A material that differs from the default in every serialized region. */
    Render::Material reference_material()
    {
        Render::Material m;
        m.metallic = 0.75f;
        m.roughness = 0.125f;
        m.albedo_map = 5u;
        m.parallax_steps = 32u;
        m.emissive_enabled = true;
        m.emissive = Vector3{1.0, 2.0, 3.0};
        m.emissive_intensity = 4.5f;
        m.main_transform.tiling_x = 2.0f;
        m.main_transform.tiling_y = 3.0f;
        m.main_transform.offset_x = 0.25f;
        m.main_transform.offset_y = 0.5f;
        m.clearcoat = 0.5f;
        m.surface_type = Render::SurfaceType::Transparent;
        m.blend_mode = Render::BlendMode::Additive;
        m.cull_mode = Render::MaterialCullMode::Back;
        m.render_queue = 3000;
        m.cast_shadows = false;
        m.weather_wettable = true;
        return m;
    }

    /** @brief The texture-path record that goes with @ref reference_material. */
    MaterialTexturePaths reference_material_paths()
    {
        MaterialTexturePaths p;
        p.albedo_map = "textures/brick.png";
        p.normal_map = "textures/brick_n.png";
        return p;
    }

    void expect_vector3_equal(const Vector3& actual, const Vector3& expected)
    {
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.x), static_cast<double>(expected.x));
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.y), static_cast<double>(expected.y));
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.z), static_cast<double>(expected.z));
    }

    void expect_light_equal(const LightParameters& actual, const LightParameters& expected)
    {
        expect_vector3_equal(actual.color, expected.color);
        EXPECT_FLOAT_EQ(actual.intensity, expected.intensity);
        EXPECT_FLOAT_EQ(actual.range, expected.range);
        EXPECT_EQ(actual.is_spot, expected.is_spot);
        EXPECT_FLOAT_EQ(actual.inner_degrees, expected.inner_degrees);
        EXPECT_FLOAT_EQ(actual.outer_degrees, expected.outer_degrees);
        EXPECT_EQ(actual.casts_shadows, expected.casts_shadows);
    }

    void expect_decal_equal(const DecalParameters& actual, const DecalParameters& expected)
    {
        expect_vector3_equal(actual.color, expected.color);
        expect_vector3_equal(actual.half_extents, expected.half_extents);
        EXPECT_FLOAT_EQ(actual.opacity, expected.opacity);
        EXPECT_EQ(actual.albedo_map, expected.albedo_map);
        EXPECT_EQ(actual.orm_map, expected.orm_map);
        EXPECT_EQ(actual.albedo_map_path, expected.albedo_map_path);
        EXPECT_EQ(actual.orm_map_path, expected.orm_map_path);
    }

    void expect_material_equal(const Render::Material& actual,
                               const Render::Material& expected)
    {
        EXPECT_FLOAT_EQ(actual.metallic, expected.metallic);
        EXPECT_FLOAT_EQ(actual.roughness, expected.roughness);
        EXPECT_EQ(actual.albedo_map, expected.albedo_map);
        EXPECT_EQ(actual.parallax_steps, expected.parallax_steps);
        EXPECT_EQ(actual.emissive_enabled, expected.emissive_enabled);
        expect_vector3_equal(actual.emissive, expected.emissive);
        EXPECT_FLOAT_EQ(actual.emissive_intensity, expected.emissive_intensity);
        EXPECT_FLOAT_EQ(actual.main_transform.tiling_x, expected.main_transform.tiling_x);
        EXPECT_FLOAT_EQ(actual.main_transform.tiling_y, expected.main_transform.tiling_y);
        EXPECT_FLOAT_EQ(actual.main_transform.offset_x, expected.main_transform.offset_x);
        EXPECT_FLOAT_EQ(actual.main_transform.offset_y, expected.main_transform.offset_y);
        EXPECT_FLOAT_EQ(actual.clearcoat, expected.clearcoat);
        EXPECT_EQ(actual.surface_type, expected.surface_type);
        EXPECT_EQ(actual.blend_mode, expected.blend_mode);
        EXPECT_EQ(actual.cull_mode, expected.cull_mode);
        EXPECT_EQ(actual.render_queue, expected.render_queue);
        EXPECT_EQ(actual.cast_shadows, expected.cast_shadows);
        EXPECT_EQ(actual.weather_wettable, expected.weather_wettable);
    }

    void expect_material_paths_equal(const MaterialTexturePaths& actual,
                                     const MaterialTexturePaths& expected)
    {
        EXPECT_EQ(actual.albedo_map, expected.albedo_map);
        EXPECT_EQ(actual.metallic_roughness_map, expected.metallic_roughness_map);
        EXPECT_EQ(actual.normal_map, expected.normal_map);
        EXPECT_EQ(actual.height_map, expected.height_map);
        EXPECT_EQ(actual.occlusion_map, expected.occlusion_map);
        EXPECT_EQ(actual.emissive_map, expected.emissive_map);
        EXPECT_EQ(actual.detail_albedo_map, expected.detail_albedo_map);
        EXPECT_EQ(actual.detail_normal_map, expected.detail_normal_map);
        EXPECT_EQ(actual.detail_mask_map, expected.detail_mask_map);
    }

    /** @brief Builds the three-entity authored scene every test starts from. */
    void build_reference_scene(IWorldEditor& world)
    {
        clear_world(world);

        const EntityId light = world.create_light("KeyLight");
        world.set_light_parameters(light, reference_light());

        const EntityId decal = world.create_decal("Splash");
        world.set_decal_parameters(decal, reference_decal());

        const EntityId box = world.create("BrickBox");
        world.set_material(box, reference_material());
        world.set_material_texture_paths(box, reference_material_paths());
    }

    void expect_reference_scene(IWorldEditor& world)
    {
        const EntityId light = find_by_name(world, "KeyLight");
        ASSERT_NE(light, NULL_ENTITY);
        ASSERT_TRUE(world.has_light(light));
        expect_light_equal(world.light_parameters(light), reference_light());

        const EntityId decal = find_by_name(world, "Splash");
        ASSERT_NE(decal, NULL_ENTITY);
        ASSERT_TRUE(world.has_decal(decal));
        expect_decal_equal(world.decal_parameters(decal), reference_decal());

        const EntityId box = find_by_name(world, "BrickBox");
        ASSERT_NE(box, NULL_ENTITY);
        expect_material_equal(world.material(box), reference_material());
        expect_material_paths_equal(world.material_texture_paths(box),
                                    reference_material_paths());
    }
}

TEST(Integration_SceneSerializer, LightsDecalsMaterialsSurviveCaptureApply)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    build_reference_scene(world);
    const nlohmann::json snapshot = Scene::capture_scene(world);

    // Wreck everything the snapshot should restore: delete the light, gut the
    // decal, reset the material.
    world.destroy(find_by_name(world, "KeyLight"));
    world.set_decal_parameters(find_by_name(world, "Splash"), DecalParameters{});
    world.set_material(find_by_name(world, "BrickBox"), Render::Material{});
    world.set_material_texture_paths(find_by_name(world, "BrickBox"),
                                     MaterialTexturePaths{});

    Scene::apply_scene(world, snapshot);
    expect_reference_scene(world);
}

TEST(Integration_SceneSerializer, DefaultMaterialStaysOutOfTheCapture)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    clear_world(world);
    world.create("PlainBox");

    // A never-touched material must not bloat the file: the entry carries no
    // material block, and the round-trip still yields a default material.
    const nlohmann::json snapshot = Scene::capture_scene(world);
    ASSERT_TRUE(snapshot.contains("entities"));
    const nlohmann::json& entities = snapshot["entities"];
    ASSERT_EQ(entities.size(), 1u);
    EXPECT_FALSE(entities.front().contains("material"));
    EXPECT_FALSE(entities.front().contains("material_texture_paths"));

    Scene::apply_scene(world, snapshot);
    const EntityId box = find_by_name(world, "PlainBox");
    ASSERT_NE(box, NULL_ENTITY);
    expect_material_equal(world.material(box), Render::Material{});
}

TEST(Integration_SceneSerializer, UndoRestoresLightsDecalsMaterials)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    build_reference_scene(world);

    // The editor's delete path: snapshot, then mutate. Undo must bring back the
    // whole authored scene, not the entity skeletons.
    Authoring::CommandHistory history;
    history.record(world);
    world.destroy(find_by_name(world, "KeyLight"));
    world.destroy(find_by_name(world, "Splash"));
    world.set_material(find_by_name(world, "BrickBox"), Render::Material{});

    ASSERT_TRUE(history.undo(world));
    expect_reference_scene(world);

    // And redo must re-apply the deletion without resurrecting stale state.
    ASSERT_TRUE(history.redo(world));
    EXPECT_EQ(find_by_name(world, "KeyLight"), NULL_ENTITY);
    EXPECT_EQ(find_by_name(world, "Splash"), NULL_ENTITY);
    expect_material_equal(world.material(find_by_name(world, "BrickBox")),
                          Render::Material{});
}

namespace
{
    /** @brief An environment with authored (non-default) values across every serialized domain. */
    SushiEngine::Render::Environment reference_environment()
    {
        SushiEngine::Render::Environment environment;
        environment.sun.intensity = 17.5f;
        environment.fog.enabled = true;
        environment.fog.density = 0.042f;
        environment.fog.height_falloff = 0.0011f;
        environment.fog_volume_count = 1;
        environment.fog_volumes[0].density = 0.09f;
        environment.fog_volumes[0].edge_falloff = 0.5f;
        environment.fog_volumes[0].shape = SushiEngine::Render::FogVolumeShape::Ellipsoid;
        environment.gi.enabled = true;
        environment.gi.intensity = 1.75f;
        environment.gi.normal_bias = 0.9f;
        environment.exposure = 0.31f;
        environment.atmosphere_nest.enabled = true;
        environment.atmosphere_nest.surface_albedo = 0.27f;
        environment.atmosphere_nest.surface_moisture_availability = 0.62f;
        environment.atmosphere_nest.max_steps_per_frame = 7;
        environment.atmosphere_nest.cloud_critical_humidity = 0.83f;
        return environment;
    }

    void expect_environment_equal(const SushiEngine::Render::Environment& actual,
                                  const SushiEngine::Render::Environment& expected)
    {
        EXPECT_FLOAT_EQ(actual.sun.intensity, expected.sun.intensity);
        EXPECT_EQ(actual.fog.enabled, expected.fog.enabled);
        EXPECT_FLOAT_EQ(actual.fog.density, expected.fog.density);
        EXPECT_FLOAT_EQ(actual.fog.height_falloff, expected.fog.height_falloff);
        EXPECT_EQ(actual.fog_volume_count, expected.fog_volume_count);
        EXPECT_FLOAT_EQ(actual.fog_volumes[0].density, expected.fog_volumes[0].density);
        EXPECT_FLOAT_EQ(actual.fog_volumes[0].edge_falloff,
                        expected.fog_volumes[0].edge_falloff);
        EXPECT_EQ(actual.fog_volumes[0].shape, expected.fog_volumes[0].shape);
        EXPECT_EQ(actual.gi.enabled, expected.gi.enabled);
        EXPECT_FLOAT_EQ(actual.gi.intensity, expected.gi.intensity);
        EXPECT_FLOAT_EQ(actual.gi.normal_bias, expected.gi.normal_bias);
        EXPECT_FLOAT_EQ(actual.exposure, expected.exposure);
        EXPECT_EQ(actual.atmosphere_nest.enabled, expected.atmosphere_nest.enabled);
        EXPECT_FLOAT_EQ(actual.atmosphere_nest.surface_albedo,
                        expected.atmosphere_nest.surface_albedo);
        EXPECT_FLOAT_EQ(actual.atmosphere_nest.surface_moisture_availability,
                        expected.atmosphere_nest.surface_moisture_availability);
        EXPECT_EQ(actual.atmosphere_nest.max_steps_per_frame,
                  expected.atmosphere_nest.max_steps_per_frame);
        EXPECT_FLOAT_EQ(actual.atmosphere_nest.cloud_critical_humidity,
                        expected.atmosphere_nest.cloud_critical_humidity);
    }
} // namespace

TEST(Integration_SceneSerializer, EnvironmentSurvivesCaptureApply)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    // The environment is scene content: fog, GI, exposure, and the nest's physics
    // must ride the same capture the entities do, or undo/save/play silently reset
    // the weather an author tuned.
    world.set_environment(reference_environment());
    const nlohmann::json snapshot = Scene::capture_scene(world);
    ASSERT_TRUE(snapshot.contains("environment"));

    world.set_environment(SushiEngine::Render::Environment{});
    Scene::apply_scene(world, snapshot);
    expect_environment_equal(world.environment(), reference_environment());
}

TEST(Integration_SceneSerializer, UndoRestoresTheEnvironment)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    world.set_environment(reference_environment());

    // The editor's environment-edit path: snapshot, then write (see
    // commit_environment_edit) — one Ctrl+Z must bring the authored sky back.
    Authoring::CommandHistory history;
    history.record(world);
    world.set_environment(SushiEngine::Render::Environment{});

    ASSERT_TRUE(history.undo(world));
    expect_environment_equal(world.environment(), reference_environment());

    // Redo re-applies the reset, so the pair is symmetric.
    ASSERT_TRUE(history.redo(world));
    expect_environment_equal(world.environment(), SushiEngine::Render::Environment{});
}

namespace
{
    /** @brief A unit box as a closed, outward-wound triangle mesh — the cooker's input. */
    Geometry::TriangleMesh box_mesh(float half_x, float half_y, float half_z)
    {
        Geometry::TriangleMesh mesh;
        const float corners[8][3] = {
            {-half_x, -half_y, -half_z}, {half_x, -half_y, -half_z}, {half_x, half_y, -half_z},
            {-half_x, half_y, -half_z},  {-half_x, -half_y, half_z}, {half_x, -half_y, half_z},
            {half_x, half_y, half_z},    {-half_x, half_y, half_z}};
        for (const auto& corner : corners)
        {
            mesh.positions.push_back(corner[0]);
            mesh.positions.push_back(corner[1]);
            mesh.positions.push_back(corner[2]);
        }
        const std::uint32_t faces[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                            {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                            {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};
        for (const auto& face : faces)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /**
     * @brief A real `.sushisoft` blob, cooked once and shared by every case here.
     *
     * Cooked rather than synthesized: the asset is what a soft body *is*, and a
     * hand-written stand-in would round-trip through a path production does not have.
     * The parameters are the cheapest cook the pipeline accepts, since what is under
     * test is whether the bytes survive rather than what they describe.
     */
    const std::vector<std::byte>& cooked_box()
    {
        static const std::vector<std::byte> bytes = []
        {
            Physics::Cooking::SoftBodyCooker cooker;
            Physics::Cooking::CookingParameters parameters;
            parameters.fidelity = 0.0f;
            parameters.voxel_resolution = 6;
            parameters.target_tetrahedron_count = Physics::Cooking::DERIVE_FROM_FIDELITY;
            parameters.simulation_level_count = 2;
            parameters.distance_field_resolution = 8;
            parameters.surface_conforming_passes = 0;
            parameters.accuracy_lattice_order = 2;
            parameters.cook_soft_body = true;
            parameters.cook_collision = false;

            const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
            std::vector<std::byte> out;
            cooker.cook(box.view(), parameters, nullptr, nullptr, out);
            return out;
        }();
        return bytes;
    }

    /**
     * @brief The authored soft body every case round-trips.
     *
     * Every field differs from its default, including all eight material parameters:
     * a capture that forgets one of them restores a body that looks right and behaves
     * like a different substance, which is the failure this whole file exists to catch.
     */
    SoftBodyParameters reference_soft_body()
    {
        SoftBodyParameters p;
        p.asset = cooked_box();
        // The authored simulation level. Whether the cook's chain actually reached a
        // second level is `test_soft_body_cooker`'s subject; here it is a number the
        // capture either carries or loses.
        p.level = 1;
        p.thickness = Scalar(0.037);
        p.self_collision = true;
        p.cosmetic = true;
        p.material.young_modulus = Scalar(3.25e6);
        p.material.poisson_ratio = Scalar(0.27);
        p.material.density = Scalar(1234.5);
        p.material.damping = Scalar(0.75);
        p.material.yield_stress = Scalar(4.5e7);
        p.material.plastic_creep = Scalar(0.35);
        p.material.maximum_plastic_strain = Scalar(0.19);
        p.material.fracture_stress = Scalar(6.75e8);
        return p;
    }

    void expect_soft_body_equal(const SoftBodyParameters& actual,
                                const SoftBodyParameters& expected)
    {
        // Size first, then the bytes as one boolean: a byte-by-byte report over a
        // multi-megabyte cook would bury every other failure in the file.
        EXPECT_EQ(actual.asset.size(), expected.asset.size());
        EXPECT_TRUE(actual.asset == expected.asset)
            << "the cooked asset came back as different bytes";
        EXPECT_EQ(actual.level, expected.level);
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.thickness),
                         static_cast<double>(expected.thickness));
        EXPECT_EQ(actual.self_collision, expected.self_collision);
        EXPECT_EQ(actual.cosmetic, expected.cosmetic);
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.material.young_modulus),
                         static_cast<double>(expected.material.young_modulus));
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.material.poisson_ratio),
                         static_cast<double>(expected.material.poisson_ratio));
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.material.density),
                         static_cast<double>(expected.material.density));
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.material.damping),
                         static_cast<double>(expected.material.damping));
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.material.yield_stress),
                         static_cast<double>(expected.material.yield_stress));
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.material.plastic_creep),
                         static_cast<double>(expected.material.plastic_creep));
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.material.maximum_plastic_strain),
                         static_cast<double>(expected.material.maximum_plastic_strain));
        EXPECT_DOUBLE_EQ(static_cast<double>(actual.material.fracture_stress),
                         static_cast<double>(expected.material.fracture_stress));
    }

    /** @brief The one authored soft body every case starts from, named "Jelly". */
    EntityId build_soft_body_scene(IWorldEditor& world)
    {
        clear_world(world);
        const EntityId body = world.create_soft_body("Jelly", cooked_box());
        if (body != NULL_ENTITY)
            world.set_soft_body_parameters(body, reference_soft_body());
        return body;
    }
} // namespace

TEST(Integration_SceneSerializer, ASoftBodySurvivesCaptureApplyThroughABlobTable)
{
    ASSERT_FALSE(cooked_box().empty()) << "the cook this whole section depends on produced none";

    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    ASSERT_NE(build_soft_body_scene(world), NULL_ENTITY);

    Scene::SceneBlobTable blobs;
    const nlohmann::json snapshot = Scene::capture_scene(world, &blobs);
    EXPECT_EQ(blobs.size(), 1u) << "the cooked asset never reached the table";

    // The point of the table: the snapshot names the blob instead of holding it, so
    // fifty undo steps over one body cost fifty integers rather than fifty cooks.
    ASSERT_TRUE(snapshot.contains("entities"));
    ASSERT_EQ(snapshot["entities"].size(), 1u);
    const nlohmann::json& entry = snapshot["entities"].front();
    ASSERT_TRUE(entry.value("has_soft_body", false));
    ASSERT_TRUE(entry.contains("soft_body"));
    EXPECT_TRUE(entry["soft_body"].contains("asset_hash"));
    EXPECT_FALSE(entry["soft_body"].contains("asset"));

    clear_world(world);
    Scene::apply_scene(world, snapshot, &blobs);

    const EntityId restored = find_by_name(world, "Jelly");
    ASSERT_NE(restored, NULL_ENTITY);
    ASSERT_TRUE(world.has_soft_body(restored));
    expect_soft_body_equal(world.soft_body_parameters(restored), reference_soft_body());
}

TEST(Integration_SceneSerializer, ASoftBodySurvivesTheSceneFileByValue)
{
    ASSERT_FALSE(cooked_box().empty());

    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    ASSERT_NE(build_soft_body_scene(world), NULL_ENTITY);

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sushiengine_soft_body_roundtrip.sushiscene";
    std::error_code error;
    std::filesystem::remove(path, error);

    ASSERT_TRUE(Scene::save_scene(world, path.string()));

    // A scene file has to open on its own machine and on anyone else's, so the blob
    // is in it rather than named by it — the opposite of the snapshot path above.
    {
        std::ifstream file(path.string());
        ASSERT_TRUE(static_cast<bool>(file));
        nlohmann::json written;
        ASSERT_NO_THROW(file >> written);
        ASSERT_TRUE(written.contains("entities"));
        ASSERT_EQ(written["entities"].size(), 1u);
        const nlohmann::json& entry = written["entities"].front();
        ASSERT_TRUE(entry.contains("soft_body"));
        EXPECT_TRUE(entry["soft_body"].contains("asset"));
        EXPECT_FALSE(entry["soft_body"].contains("asset_hash"));
    }

    clear_world(world);
    ASSERT_TRUE(Scene::load_scene(world, path.string()));

    const EntityId restored = find_by_name(world, "Jelly");
    ASSERT_NE(restored, NULL_ENTITY);
    ASSERT_TRUE(world.has_soft_body(restored));
    expect_soft_body_equal(world.soft_body_parameters(restored), reference_soft_body());

    std::filesystem::remove(path, error);
    // save_scene may drop an atmosphere sidecar beside the scene; the test owns both.
    std::filesystem::remove(std::filesystem::path(path.string() + ".atmos"), error);
}

TEST(Integration_SceneSerializer, ASoftBodyWithNoResolvableAssetComesBackWithoutABody)
{
    ASSERT_FALSE(cooked_box().empty());

    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    ASSERT_NE(build_soft_body_scene(world), NULL_ENTITY);

    Scene::SceneBlobTable blobs;
    const nlohmann::json snapshot = Scene::capture_scene(world, &blobs);

    // A snapshot that outlived its table. The entity must come back as an entity and
    // nothing more: a soft body holding an empty blob is one the physics will never
    // build, and it would sit in the scene looking authored.
    clear_world(world);
    Scene::SceneBlobTable unrelated;
    Scene::apply_scene(world, snapshot, &unrelated);

    const EntityId restored = find_by_name(world, "Jelly");
    ASSERT_NE(restored, NULL_ENTITY);
    EXPECT_FALSE(world.has_soft_body(restored));
    EXPECT_TRUE(world.soft_body_parameters(restored).asset.empty());

    // And with no table offered at all, which is what a hash-bearing snapshot handed
    // to the file path would be.
    clear_world(world);
    Scene::apply_scene(world, snapshot);

    const EntityId again = find_by_name(world, "Jelly");
    ASSERT_NE(again, NULL_ENTITY);
    EXPECT_FALSE(world.has_soft_body(again));
}

TEST(Integration_SceneSerializer, UndoRestoresASoftBody)
{
    ASSERT_FALSE(cooked_box().empty());

    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    ASSERT_NE(build_soft_body_scene(world), NULL_ENTITY);

    // The editor's delete path, on the component that used to be destroyed by it.
    Authoring::CommandHistory history;
    history.record(world);
    world.destroy(find_by_name(world, "Jelly"));

    ASSERT_TRUE(history.undo(world));
    const EntityId restored = find_by_name(world, "Jelly");
    ASSERT_NE(restored, NULL_ENTITY);
    ASSERT_TRUE(world.has_soft_body(restored));
    expect_soft_body_equal(world.soft_body_parameters(restored), reference_soft_body());

    ASSERT_TRUE(history.redo(world));
    EXPECT_EQ(find_by_name(world, "Jelly"), NULL_ENTITY);
}
