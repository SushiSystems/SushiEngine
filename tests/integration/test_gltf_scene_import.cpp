/**************************************************************************/
/* test_gltf_scene_import.cpp                                             */
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

// What `import_gltf_scene` promises about a file's node graph: parents precede their children,
// the file's own node index comes back as the join key, local transforms survive in both the
// form a file may state them in, and a punctual light and a camera arrive with their values.
// Most of it runs against glTF the test writes itself, which it can do because the importer
// never loads buffers: a hierarchy that carries no mesh is a valid file with no binary blob.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>

#include <SushiEngine/gltf/scene_import.hpp>

namespace
{
    std::string asset(const char* name)
    {
        return std::string(SE_TEST_ASSET_DIR) + "/" + name;
    }

    // A hierarchy with one root, two children, a grandchild, a spot light and a camera. No
    // buffers: import_gltf_scene reads structure only, so nothing here needs vertex data.
    const char* HIERARCHY_GLTF = R"json({
      "asset": { "version": "2.0" },
      "extensionsUsed": [ "KHR_lights_punctual" ],
      "extensions": {
        "KHR_lights_punctual": {
          "lights": [
            { "name": "Lamp", "type": "spot", "color": [1, 0.5, 0.25], "intensity": 3,
              "range": 12, "spot": { "innerConeAngle": 0.2, "outerConeAngle": 0.6 } }
          ]
        }
      },
      "cameras": [
        { "name": "Shot", "type": "perspective",
          "perspective": { "yfov": 0.8, "znear": 0.1, "zfar": 500 } }
      ],
      "scene": 0,
      "scenes": [ { "nodes": [ 0 ] } ],
      "nodes": [
        { "name": "Root", "children": [ 1, 2 ], "translation": [1, 0, 0] },
        { "name": "Left", "children": [ 3 ], "translation": [0, 2, 0] },
        { "name": "Right", "camera": 0 },
        { "name": "Deep",
          "extensions": { "KHR_lights_punctual": { "light": 0 } } }
      ]
    })json";

    // One node stating its transform as a matrix rather than as translation, rotation and
    // scale — the other form glTF permits. Column-major: a quarter turn about +Z, a uniform
    // scale of two, and a translation of (5, 6, 7).
    const char* MATRIX_NODE_GLTF = R"json({
      "asset": { "version": "2.0" },
      "scene": 0,
      "scenes": [ { "nodes": [ 0 ] } ],
      "nodes": [
        { "name": "Baked",
          "matrix": [0, 2, 0, 0, -2, 0, 0, 0, 0, 0, 2, 0, 5, 6, 7, 1] }
      ]
    })json";

    // Writes `contents` to `name` in the working directory and removes it on destruction, so a
    // run leaves nothing behind and two runs cannot see each other's files.
    class ScratchGLTF
    {
        public:
            explicit ScratchGLTF(const char* name, const char* contents) : path_(name)
            {
                std::ofstream stream(path_, std::ios::binary);
                stream << contents;
            }

            ~ScratchGLTF() { std::remove(path_.c_str()); }

            const char* path() const noexcept { return path_.c_str(); }

        private:
            std::string path_;
    };
} // namespace

TEST(Integration_GLTFSceneImport, AMissingFileFailsAndLeavesTheDescriptionEmpty)
{
    SushiEngine::Geometry::GLTFSceneDescription description;
    description.nodes.resize(3);
    EXPECT_FALSE(SushiEngine::Geometry::import_gltf_scene(
        asset("this_file_does_not_exist.gltf").c_str(), description));
    EXPECT_TRUE(description.nodes.empty());
}

TEST(Integration_GLTFSceneImport, ANullPathFails)
{
    SushiEngine::Geometry::GLTFSceneDescription description;
    EXPECT_FALSE(SushiEngine::Geometry::import_gltf_scene(nullptr, description));
}

TEST(Integration_GLTFSceneImport, EveryParentPrecedesItsChildAndEverySourceIndexIsUnique)
{
    const ScratchGLTF file("test_scene_import_hierarchy.gltf", HIERARCHY_GLTF);
    SushiEngine::Geometry::GLTFSceneDescription description;
    ASSERT_TRUE(SushiEngine::Geometry::import_gltf_scene(file.path(), description));
    ASSERT_EQ(description.nodes.size(), 4u);

    for (std::size_t i = 0; i < description.nodes.size(); ++i)
    {
        EXPECT_LT(description.nodes[i].parent, static_cast<std::int32_t>(i));
        EXPECT_GE(description.nodes[i].parent, -1);
        for (std::size_t j = 0; j < i; ++j)
            EXPECT_NE(description.nodes[j].source_index, description.nodes[i].source_index);
    }
}

TEST(Integration_GLTFSceneImport, TheTreeShapeAndTheLocalTransformsSurvive)
{
    const ScratchGLTF file("test_scene_import_shape.gltf", HIERARCHY_GLTF);
    SushiEngine::Geometry::GLTFSceneDescription description;
    ASSERT_TRUE(SushiEngine::Geometry::import_gltf_scene(file.path(), description));
    ASSERT_EQ(description.nodes.size(), 4u);

    EXPECT_EQ(description.nodes[0].name, "Root");
    EXPECT_EQ(description.nodes[0].parent, -1);
    EXPECT_FLOAT_EQ(description.nodes[0].translation.x, 1.0f);

    EXPECT_EQ(description.nodes[1].name, "Left");
    EXPECT_EQ(description.nodes[1].parent, 0);
    EXPECT_FLOAT_EQ(description.nodes[1].translation.y, 2.0f);

    // "Deep" hangs off "Left", so a depth-first walk must place it before "Right" would be if
    // the walk were breadth-first. Pinning the order pins the parent-before-child guarantee.
    EXPECT_EQ(description.nodes[2].name, "Deep");
    EXPECT_EQ(description.nodes[2].parent, 1);

    EXPECT_EQ(description.nodes[3].name, "Right");
    EXPECT_EQ(description.nodes[3].parent, 0);
}

TEST(Integration_GLTFSceneImport, AMatrixNodeDecomposesIntoTranslationRotationAndScale)
{
    const ScratchGLTF file("test_scene_import_matrix.gltf", MATRIX_NODE_GLTF);
    SushiEngine::Geometry::GLTFSceneDescription description;
    ASSERT_TRUE(SushiEngine::Geometry::import_gltf_scene(file.path(), description));
    ASSERT_EQ(description.nodes.size(), 1u);

    const SushiEngine::Geometry::GLTFNodeDescription& node = description.nodes[0];
    EXPECT_FLOAT_EQ(node.translation.x, 5.0f);
    EXPECT_FLOAT_EQ(node.translation.y, 6.0f);
    EXPECT_FLOAT_EQ(node.translation.z, 7.0f);
    EXPECT_NEAR(node.scale.x, 2.0f, 1e-5f);
    EXPECT_NEAR(node.scale.y, 2.0f, 1e-5f);
    EXPECT_NEAR(node.scale.z, 2.0f, 1e-5f);
    EXPECT_NEAR(node.rotation.x, 0.0f, 1e-5f);
    EXPECT_NEAR(node.rotation.y, 0.0f, 1e-5f);
    EXPECT_NEAR(node.rotation.z, 0.70710678f, 1e-5f);
    EXPECT_NEAR(node.rotation.w, 0.70710678f, 1e-5f);
}

TEST(Integration_GLTFSceneImport, APunctualLightAndACameraComeBackWithTheirValues)
{
    const ScratchGLTF file("test_scene_import_light.gltf", HIERARCHY_GLTF);
    SushiEngine::Geometry::GLTFSceneDescription description;
    ASSERT_TRUE(SushiEngine::Geometry::import_gltf_scene(file.path(), description));
    ASSERT_EQ(description.nodes.size(), 4u);

    ASSERT_EQ(description.lights.size(), 1u);
    EXPECT_EQ(description.lights[0].kind, SushiEngine::Geometry::GLTFLightKind::Spot);
    EXPECT_FLOAT_EQ(description.lights[0].intensity, 3.0f);
    EXPECT_FLOAT_EQ(description.lights[0].range, 12.0f);
    EXPECT_FLOAT_EQ(description.lights[0].spot_outer_cone_radians, 0.6f);
    EXPECT_EQ(description.nodes[2].light, 0);

    ASSERT_EQ(description.cameras.size(), 1u);
    EXPECT_EQ(description.cameras[0].kind, SushiEngine::Geometry::GLTFCameraKind::Perspective);
    EXPECT_FLOAT_EQ(description.cameras[0].vertical_field_of_view_radians, 0.8f);
    EXPECT_EQ(description.nodes[3].camera, 0);
}

TEST(Integration_GLTFSceneImport, ARiggedAssetInTheTreeReportsItsSkin)
{
    SushiEngine::Geometry::GLTFSceneDescription description;
    ASSERT_TRUE(
        SushiEngine::Geometry::import_gltf_scene(asset("rigged_chain.gltf").c_str(), description));
    EXPECT_GT(description.skin_count, 0u);
    EXPECT_FALSE(description.nodes.empty());
}
