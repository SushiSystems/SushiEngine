# Model thumbnail renderer (Phase 3a) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a new, self-contained render-tier primitive — `IWindowRenderer::create_mesh_thumbnail_renderer()` /
`IMeshThumbnailRenderer` — that loads a `.gltf`/`.glb` file and renders it, flat/unlit, into a small
RGBA8 image read back to the CPU. This is the engine-only half of Phase 3 (model thumbnails for the
Project panel); the editor-tier consumer (`ModelThumbnailCache` and its integration) is a separate
plan, 3b, written after this one lands so its task briefs can cite this plan's real, committed
interfaces instead of guessed ones.

**Architecture:** A new `VulkanMeshThumbnailRenderer` owns an *isolated* asset stack (its own
`SamplerCache`, `DescriptorHeap`, `MeshRegistry`, `TextureLibrary` — never the main renderer's) plus
a small offscreen color+depth target and one fixed, hand-built graphics pipeline (no
`GraphicsPipelineFactory` — that class exists to reuse work across many diverse pipelines built over
a session; this renderer only ever builds the one pipeline it needs, once, so a direct
`vkCreateGraphicsPipelines` call is the simpler, more honest fit). `render_thumbnail()` imports the
glTF into that isolated stack, computes a fixed three-quarter camera framing the model's bounding box
(the one piece of pure, GPU-independent math in this plan, extracted and unit-tested per the approved
spec), draws every imported primitive with flat headlight+ambient shading and its base-color texture,
and reads the result back into a `FrameImage`.

**Tech Stack:** C++17, Vulkan 1.3 (core sync2, dynamic rendering), VMA (`vk_mem_alloc.h`), GLSL
(compiled to SPIR-V headers via this repo's `sushiengine_compile_shader` CMake macro), GoogleTest.

## Global Constraints

- **Format scope:** `.gltf`/`.glb` only, via the existing `Render::Assets::import_gltf` entry point
  (`engine/presentation/render/source/material/gltf_importer.hpp`). Nothing in this plan touches
  `.fbx`/`.obj` — no loader for either exists anywhere in this codebase.
- **Shading fidelity:** one fixed directional "headlight" (from roughly the camera's direction) plus
  a flat ambient term, sampling each primitive's base-color texture (tinted by `Material::albedo`).
  No shadows, no IBL, no atmosphere, no post-processing, no `pbr.frag` reuse — `pbr.frag` (669 lines)
  unconditionally binds ~15 scene resources this isolated renderer has none of.
- **Camera:** a fixed three-quarter isometric angle, auto-framing the model's world-space bounding
  box with margin — no per-model tuning, no user-authored angle.
- **Asset isolation:** this renderer's `SamplerCache`/`DescriptorHeap`/`MeshRegistry`/`TextureLibrary`
  are its own, constructed the same way `Assets::AssetLibrary` constructs the main renderer's copies
  (`engine/presentation/render/source/material/asset_library.cpp:64-81` is the template), and are
  never shared with, and never touch, the main renderer's live scene assets.
- **This machine cannot run builds.** No task's implementer or reviewer runs `se build`, `se test`,
  `se editor`, cmake, or ninja. Every task is verified by reading the code and reasoning about it by
  hand — the user builds and tests the whole branch after every task is complete. Where a step below
  says "verify by reading," that is not optional; it is the step.
- **No engine-tier code may depend on anything above its own tier**
  (`cmake/EngineLayers.cmake`'s `foundation < domain < asset < presentation < world < application`
  order). Task 1's camera-framing math lives in `engine/domain/geometry` (domain tier) and returns
  `SushiEngine::Matrix4`/`SushiEngine::Vector3` — both `foundation`-tier types
  (`engine/foundation/core`), so this is a same-tier-or-below dependency, not an upward one.
- **A few API surfaces below are marked "verify against the live header before writing this exact
  call."** They are the small number of things this plan's own research could not fully confirm
  (an exact accessor method name, an operator/free-function name) despite five research passes into
  this codebase. Treat each one as a required verification step, not an assumption to skip.

---

### Task 1: Bounding-box and three-quarter camera-framing math (pure, testable)

**Files:**
- Modify: `engine/domain/geometry/CMakeLists.txt`
- Create: `engine/domain/geometry/include/SushiEngine/geometry/mesh_thumbnail_camera.hpp`
- Create: `engine/domain/geometry/source/mesh_thumbnail_camera.cpp`
- Modify: `tests/CMakeLists.txt`
- Create: `tests/unit/test_mesh_thumbnail_camera.cpp`

**Interfaces:**
- Produces: `SushiEngine::Geometry::AABB3` (fields: `Vector3 min`, `Vector3 max`, `bool
  initialized`), `SushiEngine::Geometry::expand_aabb(AABB3&, const Vector3&)`,
  `SushiEngine::Geometry::ThumbnailCamera` (fields: `Matrix4 view`, `Matrix4 projection`),
  `SushiEngine::Geometry::three_quarter_camera_for_bounds(const AABB3&, float aspect_ratio) ->
  ThumbnailCamera`. Task 2 calls `expand_aabb` while importing a glTF; Task 5's
  `VulkanMeshThumbnailRenderer::render_thumbnail` calls `three_quarter_camera_for_bounds` with the
  bounds Task 2's extended `import_gltf` reports.

- [ ] **Step 0: Confirm the exact `Vector3`/`Matrix4` API before writing Step 1**

Open `engine/foundation/core/include/SushiEngine/core/blas_placeholder.hpp` and confirm:
(a) `Vector3T<T>` (aliased as `Vector3 = Vector3T<Float>`, `Float = double`) exposes public `x`/`y`/`z`
fields you can read and construct with `Vector3{x, y, z}` — this plan's code below only ever uses
field access and aggregate construction, deliberately avoiding any assumption about whether `Vector3`
overloads arithmetic operators, so this should already match regardless; (b) the free functions
`Matrix4 look_at(const Vector3& eye, const Vector3& center, const Vector3& up)` (around line 431) and
`Matrix4 perspective(Float fovy_radians, Float aspect, Float near_plane, Float far_plane)` (around
line 406) exist with these exact names and parameter orders, and are re-exported into the top-level
`SushiEngine` namespace via `engine/foundation/core/include/SushiEngine/core/types.hpp` (so they are
callable, unqualified, as `look_at(...)`/`perspective(...)` from inside `namespace
SushiEngine::Geometry`). If either free function's name or parameter order differs from this, adjust
Step 3's code accordingly before committing — this is the one piece of this task's API surface not
already confirmed byte-for-byte by this plan's own research.

- [ ] **Step 1: Add the source file to `geometry`'s module and confirm its `core` dependency**

`engine/domain/geometry/CMakeLists.txt` currently reads:

```cmake
# geometry — the triangle mesh, the distance hierarchy every cooking stage queries, and
# the field bake over it. It depends on nothing, deliberately: the renderer and the physics
# both read a mesh's distance field and neither may own it, and a cooker that needed a
# device would fail on a build machine (docs/design/physics_system.md §3.4).
sushiengine_add_module(NAME geometry LAYER domain
    SOURCES
        source/mesh_utilities.cpp
        source/meshlet.cpp
        source/mesh_distance_query.cpp
        source/signed_distance_field.cpp)
```

Add the new source file, and add `PUBLIC_DEPENDS core` since this task's new header uses
`SushiEngine::Vector3`/`SushiEngine::Matrix4` from the `core` module (the existing file has no
`PUBLIC_DEPENDS` at all — if `core`'s types were reaching this module transitively through some
other path before, declaring the dependency explicitly here is still correct and makes the real
requirement visible rather than accidental):

```cmake
# geometry — the triangle mesh, the distance hierarchy every cooking stage queries, and
# the field bake over it. Depends only on core, for the Vector3/Matrix4 types the
# thumbnail-camera math (mesh_thumbnail_camera.cpp) and mesh vertex data already need.
sushiengine_add_module(NAME geometry LAYER domain
    SOURCES
        source/mesh_utilities.cpp
        source/meshlet.cpp
        source/mesh_distance_query.cpp
        source/signed_distance_field.cpp
        source/mesh_thumbnail_camera.cpp
    PUBLIC_DEPENDS core)
```

- [ ] **Step 2: Write the header**

Create `engine/domain/geometry/include/SushiEngine/geometry/mesh_thumbnail_camera.hpp`:

```cpp
/**************************************************************************/
/* mesh_thumbnail_camera.hpp                                              */
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
 * @file mesh_thumbnail_camera.hpp
 * @brief A world-space bounding box, and the fixed camera that frames one for a thumbnail.
 *
 * Neither the render module's `Geometry::MeshRegistry` mesh record nor its glTF importer
 * carries a min/max bounding box today (only a bounding radius) — this is the one new piece
 * of real algorithmic content the Project panel's model-thumbnail renderer needs, and it has
 * no device dependency, so it lives here rather than behind Vulkan, the same reason the rest
 * of this module exists.
 */

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Geometry
    {
        /**
         * @brief An axis-aligned world-space bounding box, built incrementally via @ref expand_aabb.
         *
         * @c initialized distinguishes "never expanded" from "expanded to include the origin" —
         * a box that starts at @c {0,0,0}..{0,0,0} by construction would otherwise silently claim
         * a valid zero-size box before a single point had ever been added to it.
         */
        struct AABB3
        {
            Vector3 min{0.0, 0.0, 0.0};
            Vector3 max{0.0, 0.0, 0.0};
            bool initialized = false;
        };

        /**
         * @brief Grows @p bounds to include @p point, initializing it on the first call.
         * @param bounds The box to grow; read and written in place.
         * @param point  The point @p bounds must include afterward.
         */
        void expand_aabb(AABB3& bounds, const Vector3& point);

        /** @brief A camera's view and projection matrices, ready for @c CameraView. */
        struct ThumbnailCamera
        {
            Matrix4 view;
            Matrix4 projection;
        };

        /**
         * @brief A fixed three-quarter isometric camera that frames @p bounds with margin.
         *
         * Positions the eye along a fixed elevated diagonal direction from the box's center, at
         * a distance computed from the box's bounding-sphere radius and a fixed vertical field
         * of view, so the whole box fits the frame with room to spare — no per-model tuning.
         * @param bounds       The world-space box to frame. An uninitialized (never-expanded) box
         *   is treated as a single point at the origin.
         * @param aspect_ratio The target image's width divided by its height.
         * @return The camera's view and projection matrices.
         */
        ThumbnailCamera three_quarter_camera_for_bounds(const AABB3& bounds, float aspect_ratio);
    } // namespace Geometry
} // namespace SushiEngine
```

- [ ] **Step 3: Write the implementation**

Create `engine/domain/geometry/source/mesh_thumbnail_camera.cpp`:

```cpp
/**************************************************************************/
/* mesh_thumbnail_camera.cpp                                              */
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

#include "SushiEngine/geometry/mesh_thumbnail_camera.hpp"

#include <algorithm>
#include <cmath>

namespace SushiEngine
{
    namespace Geometry
    {
        namespace
        {
            // 35 degrees: tight enough that a model fills most of the frame without its
            // silhouette clipping the corners at the fixed three-quarter angle below.
            constexpr float THUMBNAIL_FOV_Y_RADIANS = 0.6109f;
            // Extra distance past the tightest fit, so the model's silhouette never touches
            // the frame edge.
            constexpr double THUMBNAIL_MARGIN_FACTOR = 1.35;
            // How far above the horizontal the fixed viewing direction sits; larger values
            // look down on the model more steeply.
            constexpr double THUMBNAIL_ELEVATION = 0.8;

            double dot3(const Vector3& a, const Vector3& b)
            {
                return a.x * b.x + a.y * b.y + a.z * b.z;
            }

            double length3(const Vector3& v)
            {
                return std::sqrt(dot3(v, v));
            }

            Vector3 normalized3(const Vector3& v)
            {
                const double len = length3(v);
                if (len <= 0.0)
                    return Vector3{0.0, 0.0, 1.0};
                return Vector3{v.x / len, v.y / len, v.z / len};
            }
        } // namespace

        void expand_aabb(AABB3& bounds, const Vector3& point)
        {
            if (!bounds.initialized)
            {
                bounds.min = point;
                bounds.max = point;
                bounds.initialized = true;
                return;
            }
            bounds.min.x = std::min(bounds.min.x, point.x);
            bounds.min.y = std::min(bounds.min.y, point.y);
            bounds.min.z = std::min(bounds.min.z, point.z);
            bounds.max.x = std::max(bounds.max.x, point.x);
            bounds.max.y = std::max(bounds.max.y, point.y);
            bounds.max.z = std::max(bounds.max.z, point.z);
        }

        ThumbnailCamera three_quarter_camera_for_bounds(const AABB3& bounds, float aspect_ratio)
        {
            const Vector3 center{
                (bounds.min.x + bounds.max.x) * 0.5,
                (bounds.min.y + bounds.max.y) * 0.5,
                (bounds.min.z + bounds.max.z) * 0.5};
            const Vector3 half_extents{
                (bounds.max.x - bounds.min.x) * 0.5,
                (bounds.max.y - bounds.min.y) * 0.5,
                (bounds.max.z - bounds.min.z) * 0.5};
            // A conservative bounding-sphere radius from the box's half-diagonal; never zero,
            // so a degenerate (single-point) box still gets a sane, non-zero camera distance.
            const double radius = std::max(length3(half_extents), 0.001);

            const Vector3 direction = normalized3(Vector3{1.0, THUMBNAIL_ELEVATION, 1.0});
            const double half_fov = static_cast<double>(THUMBNAIL_FOV_Y_RADIANS) * 0.5;
            const double distance = (radius / std::sin(half_fov)) * THUMBNAIL_MARGIN_FACTOR;

            const Vector3 eye{
                center.x + direction.x * distance,
                center.y + direction.y * distance,
                center.z + direction.z * distance};
            const Vector3 up{0.0, 1.0, 0.0};

            ThumbnailCamera camera;
            camera.view = look_at(eye, center, up);
            const float near_plane = static_cast<float>(std::max(distance - radius * 1.5, 0.01));
            const float far_plane = static_cast<float>(distance + radius * 4.0);
            camera.projection =
                perspective(THUMBNAIL_FOV_Y_RADIANS, aspect_ratio, near_plane, far_plane);
            return camera;
        }
    } // namespace Geometry
} // namespace SushiEngine
```

- [ ] **Step 4: Write the failing test**

Create `tests/unit/test_mesh_thumbnail_camera.cpp`:

```cpp
/**************************************************************************/
/* test_mesh_thumbnail_camera.cpp                                        */
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

// Unit_MeshThumbnailCamera: the model-thumbnail pipeline's bounding-box accumulation and its
// fixed three-quarter camera framing, checked without any device — expand_aabb against known
// point sequences, and the camera against properties that must hold for ANY valid bounds
// (looks away from a degenerate box, moves farther back for a larger box, and actually uses
// the aspect ratio it's given) rather than a single hand-computed matrix, since the exact
// matrix layout is an implementation detail of Matrix4::look_at/perspective this test does not
// own.

#include <cmath>

#include <gtest/gtest.h>

#include "SushiEngine/geometry/mesh_thumbnail_camera.hpp"

using namespace SushiEngine::Geometry;

TEST(Unit_MeshThumbnailCamera, ExpandAabbFromEmptyTakesFirstPoint)
{
    AABB3 bounds;
    expand_aabb(bounds, SushiEngine::Vector3{1.0, 2.0, 3.0});
    EXPECT_TRUE(bounds.initialized);
    EXPECT_DOUBLE_EQ(bounds.min.x, 1.0);
    EXPECT_DOUBLE_EQ(bounds.min.y, 2.0);
    EXPECT_DOUBLE_EQ(bounds.min.z, 3.0);
    EXPECT_DOUBLE_EQ(bounds.max.x, 1.0);
    EXPECT_DOUBLE_EQ(bounds.max.y, 2.0);
    EXPECT_DOUBLE_EQ(bounds.max.z, 3.0);
}

TEST(Unit_MeshThumbnailCamera, ExpandAabbGrowsToEncloseNewPoints)
{
    AABB3 bounds;
    expand_aabb(bounds, SushiEngine::Vector3{0.0, 0.0, 0.0});
    expand_aabb(bounds, SushiEngine::Vector3{-2.0, 5.0, 1.0});
    expand_aabb(bounds, SushiEngine::Vector3{3.0, -1.0, 0.5});
    EXPECT_DOUBLE_EQ(bounds.min.x, -2.0);
    EXPECT_DOUBLE_EQ(bounds.min.y, -1.0);
    EXPECT_DOUBLE_EQ(bounds.min.z, 0.0);
    EXPECT_DOUBLE_EQ(bounds.max.x, 3.0);
    EXPECT_DOUBLE_EQ(bounds.max.y, 5.0);
    EXPECT_DOUBLE_EQ(bounds.max.z, 1.0);
}

TEST(Unit_MeshThumbnailCamera, LargerBoundsProduceAFartherEyeThanSmallerBounds)
{
    AABB3 small_bounds;
    expand_aabb(small_bounds, SushiEngine::Vector3{-1.0, -1.0, -1.0});
    expand_aabb(small_bounds, SushiEngine::Vector3{1.0, 1.0, 1.0});

    AABB3 large_bounds;
    expand_aabb(large_bounds, SushiEngine::Vector3{-10.0, -10.0, -10.0});
    expand_aabb(large_bounds, SushiEngine::Vector3{10.0, 10.0, 10.0});

    const ThumbnailCamera small_camera = three_quarter_camera_for_bounds(small_bounds, 1.0f);
    const ThumbnailCamera large_camera = three_quarter_camera_for_bounds(large_bounds, 1.0f);

    // Both boxes are centered on the origin, so a look_at view matrix's translation column
    // (m[12..14] in this column-major layout: -dot(right,eye), -dot(up,eye), -dot(forward,eye))
    // grows in magnitude exactly as the eye moves farther from that shared center — this holds
    // regardless of the exact fixed viewing direction/margin constants chosen above.
    auto translation_magnitude = [](const SushiEngine::Matrix4& m)
    {
        return std::sqrt(m.m[12] * m.m[12] + m.m[13] * m.m[13] + m.m[14] * m.m[14]);
    };
    EXPECT_GT(translation_magnitude(large_camera.view), translation_magnitude(small_camera.view));
}

TEST(Unit_MeshThumbnailCamera, AspectRatioActuallyReachesTheProjectionMatrix)
{
    AABB3 bounds;
    expand_aabb(bounds, SushiEngine::Vector3{-1.0, -1.0, -1.0});
    expand_aabb(bounds, SushiEngine::Vector3{1.0, 1.0, 1.0});

    const ThumbnailCamera square = three_quarter_camera_for_bounds(bounds, 1.0f);
    const ThumbnailCamera wide = three_quarter_camera_for_bounds(bounds, 2.0f);

    // A deliberately weak assertion (not-equal rather than a specific element/direction): the
    // exact convention Matrix4::perspective uses for folding aspect ratio into its matrix is
    // that function's own implementation detail, not something this test should have to know.
    // What must hold regardless is that the aspect ratio parameter actually changes the result.
    EXPECT_NE(square.projection.m[0], wide.projection.m[0]);
}

TEST(Unit_MeshThumbnailCamera, DegenerateSinglePointBoundsProduceAFiniteCamera)
{
    AABB3 bounds;
    expand_aabb(bounds, SushiEngine::Vector3{5.0, 5.0, 5.0});

    const ThumbnailCamera camera = three_quarter_camera_for_bounds(bounds, 1.0f);
    for (int i = 0; i < 16; ++i)
    {
        EXPECT_TRUE(std::isfinite(camera.view.m[i])) << "view.m[" << i << "]";
        EXPECT_TRUE(std::isfinite(camera.projection.m[i])) << "projection.m[" << i << "]";
    }
}
```

- [ ] **Step 5: Wire the test file into the test binary**

In `tests/CMakeLists.txt`, add the new test file next to `unit/test_thumbnail_lru.cpp` (from Phase
2's Task 3):

```cmake
    unit/test_thumbnail_lru.cpp
    unit/test_mesh_thumbnail_camera.cpp
```

- [ ] **Step 6: Verify by reading, not by building**

Per Global Constraints, this machine cannot run `se test`. Trace
`LargerBoundsProduceAFartherEyeThanSmallerBounds` by hand: both boxes share the center `{0,0,0}`, so
`look_at(eye, {0,0,0}, up)`'s translation terms are `-dot(basis_axis, eye)` for each of the view
matrix's three orthonormal basis vectors — confirm that the `distance` computed for the 20-unit-wide
box (radius ≈ 10·√3, since `half_extents = {10,10,10}`) is meaningfully larger than for the 2-unit-wide
box (radius ≈ √3), and that a larger `distance` along the same fixed unit `direction` produces a
proportionally larger-magnitude `eye`, and therefore a larger-magnitude translation column.

- [ ] **Step 7: Commit**

```bash
git add engine/domain/geometry/CMakeLists.txt \
        engine/domain/geometry/include/SushiEngine/geometry/mesh_thumbnail_camera.hpp \
        engine/domain/geometry/source/mesh_thumbnail_camera.cpp \
        tests/CMakeLists.txt tests/unit/test_mesh_thumbnail_camera.cpp
git commit -m "feat(geometry): add bounding-box and three-quarter camera framing math"
```

---

### Task 2: Extend `import_gltf` to optionally report the imported model's world-space bounds

**Files:**
- Modify: `engine/presentation/render/source/material/gltf_importer.hpp`
- Modify: `engine/presentation/render/source/material/gltf_importer.cpp`

**Interfaces:**
- Consumes: `SushiEngine::Geometry::AABB3`, `SushiEngine::Geometry::expand_aabb` (Task 1).
- Produces: `import_gltf`'s new trailing parameter, `Geometry::AABB3* out_bounds = nullptr`. Task 5's
  `VulkanMeshThumbnailRenderer::render_thumbnail` passes a non-null `AABB3*` and reads the result;
  every existing caller (e.g. `applications/editor/source/scene/scene_commands.cpp`'s scene
  drag-and-drop path) keeps compiling unchanged since the new parameter defaults to `nullptr`.

- [ ] **Step 1: Locate the current declaration and add the parameter**

In `engine/presentation/render/source/material/gltf_importer.hpp`, find `import_gltf`'s declaration
(its current signature, confirmed by this plan's own research, is `std::size_t import_gltf(const
char* path, Geometry::MeshRegistry& meshes, TextureLibrary& textures, MeshId* out_meshes,
Render::Material* out_materials, std::size_t capacity);`). Add
`#include <SushiEngine/geometry/mesh_thumbnail_camera.hpp>` to this header's includes (for
`Geometry::AABB3`), and add the new trailing defaulted parameter:

```cpp
std::size_t import_gltf(const char* path, Geometry::MeshRegistry& meshes,
                        TextureLibrary& textures, MeshId* out_meshes,
                        Render::Material* out_materials, std::size_t capacity,
                        Geometry::AABB3* out_bounds = nullptr);
```

Leave `import_gltf_scene_meshes` and `import_gltf_skinned_mesh` (or whatever the skinned entry point
is named) untouched — only `import_gltf` is this plan's concern, since it is the entry point that
bakes every node's world transform into its vertices, which is exactly what "one bounding box for the
whole imported model" needs.

- [ ] **Step 2: Accumulate bounds during the existing per-vertex transform loop**

Open `engine/presentation/render/source/material/gltf_importer.cpp` and read `import_gltf`'s body in
full before editing — this plan's research located its structure at approximately lines 398-436: a
loop that builds each primitive's `PrimitiveGeometry geometry` (via `read_primitive_geometry`), then
(for `import_gltf` specifically, not the scene-meshes variant) a loop over `geometry.vertices` that
bakes the current node's world transform into each vertex's position (reported at lines ~404-426),
before the primitive is uploaded via `meshes.add_mesh(...)` and the loop continues to the next
primitive.

Add the parameter to the function's definition, matching Step 1's declaration (drop the default
value here — defaults belong on the declaration only). Inside the existing per-vertex
transform-bake loop, immediately after each vertex's world-space position is finalized for this
primitive, add:

```cpp
if (out_bounds != nullptr)
{
    Geometry::expand_aabb(
        *out_bounds,
        Vector3{static_cast<double>(vertex.position[0]),
               static_cast<double>(vertex.position[1]),
               static_cast<double>(vertex.position[2])});
}
```

(matching whatever the loop's actual per-vertex variable is named — `vertex` above is illustrative;
use the real local variable name from the loop you are editing.) This accumulates one running
`AABB3` across every primitive and every node the function processes in this call, which is exactly
the "one box for the whole imported model" this plan needs — do not reset `*out_bounds` between
primitives, since the caller owns a single `AABB3` for the whole `import_gltf` call and expects it
still `initialized == false` only if the function wrote nothing (e.g. the file failed to parse).

- [ ] **Step 3: Verify by reading, not by building**

Per Global Constraints, this machine cannot build. Re-read the edited function end-to-end and confirm:
(a) the new parameter's default only appears once, on the header declaration, never repeated on the
`.cpp` definition (a C++ rule, not a style preference — a repeated default is a compile error); (b)
`out_bounds` is dereferenced only when non-null, at every call site you added, not just the first;
(c) every existing call site of `import_gltf` in the tree (search for `import_gltf(` across
`applications/` and `engine/`) still compiles unchanged, since it doesn't pass the new trailing
argument.

- [ ] **Step 4: Commit**

```bash
git add engine/presentation/render/source/material/gltf_importer.hpp \
        engine/presentation/render/source/material/gltf_importer.cpp
git commit -m "feat(render): report a glTF import's world-space bounds, opt-in"
```

---

### Task 3: `IMeshThumbnailRenderer` interface and `IWindowRenderer::create_mesh_thumbnail_renderer()`

**Files:**
- Create: `engine/presentation/render/include/SushiEngine/render/mesh_thumbnail_renderer.hpp`
- Modify: `engine/presentation/render/include/SushiEngine/render/window_renderer.hpp`

**Interfaces:**
- Consumes: `Render::FrameImage` (`engine/presentation/render/include/SushiEngine/render/scene_view.hpp`
  — the existing `{width, height, rgba}` struct `ISceneView::read_output` already uses; reused here
  rather than inventing a second output type).
- Produces: `SushiEngine::Render::IMeshThumbnailRenderer` (pure virtual, one method,
  `render_thumbnail`) and `IWindowRenderer::create_mesh_thumbnail_renderer()`. Task 5's
  `VulkanMeshThumbnailRenderer` implements the interface; Task 6 implements the factory method.

- [ ] **Step 1: Write the interface header**

Create `engine/presentation/render/include/SushiEngine/render/mesh_thumbnail_renderer.hpp`:

```cpp
/**************************************************************************/
/* mesh_thumbnail_renderer.hpp                                            */
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
 * @file mesh_thumbnail_renderer.hpp
 * @brief One mesh, one flat/unlit draw, one readback — nothing else.
 *
 * @c ISceneView::render is the production forward path: lighting, environment, TAA, picking,
 * decals, particles. A thumbnail needs none of that, and standing up a full @c ISceneView per
 * resident thumbnail would be both wasteful and awkward to pool. @c IMeshThumbnailRenderer is
 * the deliberately small alternative: load one glTF, frame it with a fixed camera, draw it flat,
 * read the pixels back. See
 * docs/superpowers/specs/2026-08-07-project-panel-model-thumbnails-design.md.
 */

#include <cstdint>

#include "scene_view.hpp"

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief Renders one glTF/GLB model, flat/unlit, into a small offscreen image.
         *
         * Owns an asset stack isolated from the renderer's main scene assets (its own mesh
         * registry, texture library, and bindless heap) — loading or discarding a thumbnail
         * never touches what the live scene has loaded for the same file.
         */
        class IMeshThumbnailRenderer
        {
            public:
                virtual ~IMeshThumbnailRenderer() = default;

                /**
                 * @brief Loads @p path and renders it into a @p width x @p height RGBA8 image.
                 *
                 * The camera is a fixed three-quarter angle auto-framing the model's bounding
                 * box; shading is flat headlight-plus-ambient sampling each primitive's
                 * base-color texture. @p out_image is left untouched on failure.
                 *
                 * @param path   A `.gltf`/`.glb` file path.
                 * @param width  Output image width in pixels.
                 * @param height Output image height in pixels.
                 * @param out_image Receives the rendered result on success.
                 * @return @c true on success; @c false on any load or render failure (an
                 *   unsupported/corrupt file, a model with no position data, more primitives
                 *   than this renderer's fixed capacity, or a Vulkan error).
                 */
                virtual bool render_thumbnail(const char* path, std::uint32_t width,
                                              std::uint32_t height, FrameImage& out_image) = 0;
        };
    } // namespace Render
} // namespace SushiEngine
```

- [ ] **Step 2: Add the factory method to `IWindowRenderer`**

In `engine/presentation/render/include/SushiEngine/render/window_renderer.hpp`, add
`#include "mesh_thumbnail_renderer.hpp"` to this header's includes (alongside whatever brings in
`ISceneView`/`create_scene_view()`'s declaration today). Then find `create_scene_view()`'s
declaration (confirmed by this plan's research to read exactly
`virtual std::unique_ptr<ISceneView> create_scene_view() = 0;`) and add the new method directly
after it:

```cpp
        virtual std::unique_ptr<ISceneView> create_scene_view() = 0;

        /**
         * @brief Creates an offscreen mesh-thumbnail renderer on this renderer's device.
         *
         * Unlike create_scene_view(), which stands up the full production scene pipeline
         * (lighting, environment, TAA, picking), this is a small, purpose-built renderer for
         * the Project panel's model thumbnails: one mesh, one flat/unlit draw, one readback.
         * It owns an isolated asset stack (its own descriptor heap, sampler cache, mesh
         * registry, texture library) so loading or evicting a thumbnail never touches this
         * renderer's own live scene assets.
         *
         * @return An owning handle to the new mesh-thumbnail renderer.
         */
        virtual std::unique_ptr<IMeshThumbnailRenderer> create_mesh_thumbnail_renderer() = 0;
```

- [ ] **Step 3: Verify by reading, not by building**

Per Global Constraints, this machine cannot build. Confirm `mesh_thumbnail_renderer.hpp`'s
`#include "scene_view.hpp"` resolves relative to its own directory (both files sit in
`engine/presentation/render/include/SushiEngine/render/`) and that `FrameImage` is indeed a
publicly visible type in `scene_view.hpp` (not nested inside `ISceneView` or otherwise
inaccessible from outside it) — re-open `scene_view.hpp` and confirm `FrameImage`'s declaration
sits at namespace scope, not as a private nested type.

- [ ] **Step 4: Commit**

```bash
git add engine/presentation/render/include/SushiEngine/render/mesh_thumbnail_renderer.hpp \
        engine/presentation/render/include/SushiEngine/render/window_renderer.hpp
git commit -m "feat(render): declare IMeshThumbnailRenderer and its IWindowRenderer factory method"
```

---

### Task 4: Flat/unlit mesh-thumbnail shader pair

**Files:**
- Create: `engine/presentation/render/shaders/mesh_thumbnail.vert`
- Create: `engine/presentation/render/shaders/mesh_thumbnail.frag`
- Modify: `engine/presentation/render/CMakeLists.txt`

**Interfaces:**
- Produces: two SPIR-V-embedding C++ headers generated at build time, exposing
  `SushiEngine::Render::Shaders::mesh_thumbnail_vert_spv` and `::mesh_thumbnail_frag_spv` (naming
  mirrors the existing `mesh_vert_spv`/`pbr_frag_spv` symbols this same CMakeLists.txt already
  generates). Task 5's `VulkanMeshThumbnailRenderer` includes both generated headers and passes
  their SPIR-V bytes to `vkCreateShaderModule`.

- [ ] **Step 1: Write the vertex shader**

Create `engine/presentation/render/shaders/mesh_thumbnail.vert`:

```glsl
#version 450

// Locations match MeshVertex's 60-byte layout exactly (engine/domain/geometry/include/
// SushiEngine/geometry/mesh_vertex.hpp): position @0, normal @1, tangent @2 (unused here),
// uv0 @3, uv1 @4 (unused here), color @5 (unused here). Vulkan does not require a pipeline's
// vertex input attributes to be contiguous, so skipping the unused locations costs nothing.
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 3) in vec2 in_uv0;

layout(location = 0) out vec3 out_world_position;
layout(location = 1) out vec3 out_world_normal;
layout(location = 2) out vec2 out_uv0;

layout(push_constant) uniform Push
{
    mat4 model;
    mat4 view_projection;
    vec4 albedo;
    int albedo_texture_index; // -1 = no texture; flat albedo tint only.
} pc;

void main()
{
    vec4 world_position = pc.model * vec4(in_position, 1.0);
    out_world_position = world_position.xyz;
    // No non-uniform scale in this pipeline's usage (import_gltf already bakes node transforms
    // into vertex positions, so pc.model is always identity here) -- a plain 3x3 rotation of
    // the normal is correct without a separate inverse-transpose normal matrix.
    out_world_normal = mat3(pc.model) * in_normal;
    out_uv0 = in_uv0;
    gl_Position = pc.view_projection * world_position;
}
```

- [ ] **Step 2: Write the fragment shader**

Create `engine/presentation/render/shaders/mesh_thumbnail.frag`:

```glsl
#version 450
#extension GL_EXT_nonuniform_qualifier : require

// The isolated thumbnail renderer's own bindless heap, at the same set/binding this codebase's
// main bindless heap already uses for its texture array (DescriptorHeap::TEXTURE_BINDING == 0;
// set 1 mirrors pbr.frag's convention of reserving set 0 for the pipeline's own per-draw data).
layout(set = 1, binding = 0) uniform sampler2D bindless_textures[];

layout(location = 0) in vec3 in_world_position;
layout(location = 1) in vec3 in_world_normal;
layout(location = 2) in vec2 in_uv0;

layout(location = 0) out vec4 out_color;

layout(push_constant) uniform Push
{
    mat4 model;
    mat4 view_projection;
    vec4 albedo;
    int albedo_texture_index;
} pc;

void main()
{
    vec4 base_color = pc.albedo;
    if (pc.albedo_texture_index >= 0)
        base_color *= texture(bindless_textures[nonuniformEXT(pc.albedo_texture_index)], in_uv0);

    // One fixed headlight from roughly the camera's own viewing direction, plus a flat ambient
    // floor -- deliberately not physically based (no shadows, no IBL, no atmosphere): this is
    // the "simple/unlit-leaning" fidelity level the approved design chose for model thumbnails,
    // over the far heavier machinery pbr.frag would otherwise require.
    const vec3 light_direction = normalize(vec3(1.0, 0.8, 1.0));
    const float ambient = 0.35;
    vec3 normal = normalize(in_world_normal);
    float headlight = max(dot(normal, light_direction), 0.0);
    float shade = clamp(ambient + headlight * (1.0 - ambient), 0.0, 1.0);

    out_color = vec4(base_color.rgb * shade, base_color.a);
}
```

- [ ] **Step 3: Wire both shaders into the CMake build**

In `engine/presentation/render/CMakeLists.txt`, find the existing `sushiengine_compile_shader(...)`
calls (e.g. the `mesh.vert`/`pbr.frag` pair this plan's research already confirmed) and add two more,
following the exact same pattern:

```cmake
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_thumbnail.vert"
    mesh_thumbnail_vert_spv MESH_THUMBNAIL_VERT_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_thumbnail.frag"
    mesh_thumbnail_frag_spv MESH_THUMBNAIL_FRAG_HEADER)
```

Then find where the existing generated headers (e.g. `${MESH_VERT_HEADER}`, `${PBR_FRAG_HEADER}`)
are added to the `sushiengine_render` target's sources (search this same file for
`${MESH_VERT_HEADER}` to find that list) and add `${MESH_THUMBNAIL_VERT_HEADER}` and
`${MESH_THUMBNAIL_FRAG_HEADER}` to it — this is what actually puts each shader's compile rule in the
target's build graph; declaring the `sushiengine_compile_shader(...)` call alone does not.

- [ ] **Step 4: Verify by reading, not by building**

Per Global Constraints, this machine cannot compile these shaders. Re-read both files against
`MeshVertex`'s confirmed 60-byte layout (`position[3]` @0, `normal[3]` @12, `tangent[4]` @24,
`uv0[2]` @40, `uv1[2]` @48, `color[4]` @56) and confirm the vertex shader's three `in_*` locations
(0, 1, 3) match the attributes Task 5's pipeline creation will declare — Task 5 is responsible for
the `VkVertexInputAttributeDescription` array with the matching offsets; this step only confirms the
shader-side location numbers are internally consistent with that plan.

- [ ] **Step 5: Commit**

```bash
git add engine/presentation/render/shaders/mesh_thumbnail.vert \
        engine/presentation/render/shaders/mesh_thumbnail.frag \
        engine/presentation/render/CMakeLists.txt
git commit -m "feat(render): add the flat/unlit mesh-thumbnail shader pair"
```

---

### Task 5: `VulkanMeshThumbnailRenderer` — isolated asset stack, offscreen targets, pipeline, render+readback

**Files:**
- Create: `engine/presentation/render/source/rhi/vulkan/vulkan_mesh_thumbnail_renderer.hpp`
- Create: `engine/presentation/render/source/rhi/vulkan/vulkan_mesh_thumbnail_renderer.cpp`

**Interfaces:**
- Consumes: `SushiEngine::Render::IMeshThumbnailRenderer` (Task 3), the extended `import_gltf` (Task
  2), `SushiEngine::Geometry::three_quarter_camera_for_bounds`/`AABB3` (Task 1), the generated
  `mesh_thumbnail_vert_spv`/`mesh_thumbnail_frag_spv` headers (Task 4), and the existing
  `Vulkan::VulkanDevice`, `Resources::SamplerCache`, `Resources::DescriptorHeap`,
  `Geometry::MeshRegistry`, `Assets::TextureLibrary` classes (all already in the tree; their
  constructors are quoted exactly below).
- Produces: `SushiEngine::Render::Vulkan::VulkanMeshThumbnailRenderer`, which Task 6's
  `VulkanWindowRenderer::create_mesh_thumbnail_renderer()` constructs and returns.

This is one cohesive class and one cohesive review gate — its pieces (isolated asset construction,
offscreen target creation, pipeline creation, the render+readback sequence, teardown) only make sense
reviewed together, matching how Phase 2's `ThumbnailCache` was one task despite being the plan's
largest. There is no unit test for this task: every line of it touches Vulkan, which this codebase's
test binary cannot reach (see Global Constraints). Verification is the by-hand trace in Step 3.

- [ ] **Step 1: Write the header**

Create `engine/presentation/render/source/rhi/vulkan/vulkan_mesh_thumbnail_renderer.hpp`:

```cpp
/**************************************************************************/
/* vulkan_mesh_thumbnail_renderer.hpp                                     */
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
 * @file vulkan_mesh_thumbnail_renderer.hpp
 * @brief The Vulkan implementation of IMeshThumbnailRenderer.
 *
 * Owns an asset stack isolated from the main renderer's (its own sampler cache, bindless
 * descriptor heap, mesh registry, and texture library, built the same way
 * Assets::AssetLibrary builds the main renderer's copies) plus one fixed, hand-built
 * graphics pipeline. Unlike the renderer's other pipelines, this one is never rebuilt
 * against different shaders or reused across a diverse pipeline set, so it is created
 * directly with vkCreateGraphicsPipelines rather than through GraphicsPipelineFactory --
 * that class exists to amortize work across many pipelines over a session, which does not
 * apply to a renderer that only ever builds the one pipeline it needs, once.
 */

#include <cstdint>

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <SushiEngine/render/mesh_thumbnail_renderer.hpp>

#include "../../geometry/mesh_registry.hpp"
#include "../../material/texture_library.hpp"
#include "../../resources/descriptor_heap.hpp"
#include "../../resources/sampler_cache.hpp"
#include "vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            /** @brief Vulkan implementation of IMeshThumbnailRenderer; see the file doc comment. */
            class VulkanMeshThumbnailRenderer final : public IMeshThumbnailRenderer
            {
                public:
                    /**
                     * @brief Builds this renderer's isolated asset stack and pipeline.
                     * @param device The live device this renderer's own asset stack and
                     *   offscreen resources are built against.
                     * @throws std::runtime_error on any Vulkan resource creation failure.
                     */
                    explicit VulkanMeshThumbnailRenderer(VulkanDevice& device);

                    /** @brief Frees the offscreen targets, the pipeline, and this renderer's
                     *  own isolated asset stack. */
                    ~VulkanMeshThumbnailRenderer() override;

                    VulkanMeshThumbnailRenderer(const VulkanMeshThumbnailRenderer&) = delete;
                    VulkanMeshThumbnailRenderer& operator=(const VulkanMeshThumbnailRenderer&) = delete;

                    bool render_thumbnail(const char* path, std::uint32_t width,
                                          std::uint32_t height, FrameImage& out_image) override;

                private:
                    // A model with more primitives than this is treated as a load failure --
                    // out_meshes/out_materials are fixed-capacity arrays, not vectors, matching
                    // import_gltf's own existing (unchanged by this plan) signature.
                    static constexpr std::size_t MAX_PRIMITIVES = 64;
                    static constexpr std::uint32_t HEAP_TEXTURE_CAPACITY = 256;
                    static constexpr std::uint32_t HEAP_BUFFER_CAPACITY = 16;
                    static constexpr std::size_t TEXTURE_BUDGET_BYTES = 64u * 1024u * 1024u;
                    static constexpr VkFormat COLOR_FORMAT = VK_FORMAT_R8G8B8A8_UNORM;
                    static constexpr VkFormat DEPTH_FORMAT = VK_FORMAT_D32_SFLOAT;

                    /** @brief One draw call's worth of per-primitive push-constant data. */
                    struct Push
                    {
                        float model[16];
                        float view_projection[16];
                        float albedo[4];
                        std::int32_t albedo_texture_index;
                    };

                    void create_pipeline();
                    void destroy_pipeline();
                    void ensure_targets(std::uint32_t width, std::uint32_t height);
                    void destroy_targets();
                    bool copy_output_to_cpu(std::uint32_t width, std::uint32_t height,
                                            FrameImage& out_image);

                    VulkanDevice& device_;

                    // This renderer's own asset stack, isolated from the main renderer's --
                    // see the class doc comment.
                    Resources::SamplerCache samplers_;
                    Resources::DescriptorHeap heap_;
                    Geometry::MeshRegistry meshes_;
                    Assets::TextureLibrary textures_;

                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;

                    VkCommandPool command_pool_ = VK_NULL_HANDLE;

                    // Offscreen render targets, sized to the largest request seen so far and
                    // reused across calls; recreated only when a larger size is requested.
                    VkImage color_image_ = VK_NULL_HANDLE;
                    VmaAllocation color_allocation_ = VK_NULL_HANDLE;
                    VkImageView color_view_ = VK_NULL_HANDLE;
                    VkImage depth_image_ = VK_NULL_HANDLE;
                    VmaAllocation depth_allocation_ = VK_NULL_HANDLE;
                    VkImageView depth_view_ = VK_NULL_HANDLE;
                    std::uint32_t target_width_ = 0;
                    std::uint32_t target_height_ = 0;
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
```

- [ ] **Step 2: Write the implementation**

Create `engine/presentation/render/source/rhi/vulkan/vulkan_mesh_thumbnail_renderer.cpp`:

```cpp
/**************************************************************************/
/* vulkan_mesh_thumbnail_renderer.cpp                                     */
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

#include "vulkan_mesh_thumbnail_renderer.hpp"

#include <cstring>
#include <stdexcept>

#include <SushiEngine/geometry/mesh_thumbnail_camera.hpp>
#include <SushiEngine/material/material.hpp>

#include "../../material/gltf_importer.hpp"
#include "mesh_thumbnail.frag.h"
#include "mesh_thumbnail.vert.h"
#include "vulkan_utils.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            namespace
            {
                void write_matrix(const Matrix4& source, float destination[16])
                {
                    for (int i = 0; i < 16; ++i)
                        destination[i] = static_cast<float>(source.m[i]);
                }
            } // namespace

            VulkanMeshThumbnailRenderer::VulkanMeshThumbnailRenderer(VulkanDevice& device)
                : device_(device)
                , samplers_(device)
                , heap_(device, HEAP_TEXTURE_CAPACITY, HEAP_BUFFER_CAPACITY)
                , meshes_(device)
                , textures_(device, heap_, samplers_, TEXTURE_BUDGET_BYTES)
            {
                VkCommandPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
                pool_info.queueFamilyIndex = device_.graphics_queue_family();
                Vulkan::check(
                    vkCreateCommandPool(device_.device(), &pool_info, nullptr, &command_pool_),
                    "vkCreateCommandPool(mesh thumbnail)");

                create_pipeline();
            }

            VulkanMeshThumbnailRenderer::~VulkanMeshThumbnailRenderer()
            {
                destroy_targets();
                destroy_pipeline();
                vkDestroyCommandPool(device_.device(), command_pool_, nullptr);
            }

            void VulkanMeshThumbnailRenderer::create_pipeline()
            {
                VkShaderModuleCreateInfo vert_info{};
                vert_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
                vert_info.codeSize = sizeof(Shaders::mesh_thumbnail_vert_spv);
                vert_info.pCode = Shaders::mesh_thumbnail_vert_spv;
                VkShaderModule vert_module = VK_NULL_HANDLE;
                Vulkan::check(
                    vkCreateShaderModule(device_.device(), &vert_info, nullptr, &vert_module),
                    "vkCreateShaderModule(mesh_thumbnail.vert)");

                VkShaderModuleCreateInfo frag_info{};
                frag_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
                frag_info.codeSize = sizeof(Shaders::mesh_thumbnail_frag_spv);
                frag_info.pCode = Shaders::mesh_thumbnail_frag_spv;
                VkShaderModule frag_module = VK_NULL_HANDLE;
                if (vkCreateShaderModule(device_.device(), &frag_info, nullptr, &frag_module) !=
                    VK_SUCCESS)
                {
                    vkDestroyShaderModule(device_.device(), vert_module, nullptr);
                    throw std::runtime_error(
                        "SushiEngine: vkCreateShaderModule(mesh_thumbnail.frag) failed");
                }

                // Set 1 is this renderer's own bindless heap (matching mesh_thumbnail.frag's
                // `layout(set = 1, binding = 0)`); set 0 is reserved but carries no bindings
                // today, since every per-draw value this pipeline needs travels as a push
                // constant -- kept as an empty set 0 rather than renumbering the heap to set 0,
                // so the shader-side set numbers match pbr.frag's own set-1-for-the-heap
                // convention exactly.
                VkDescriptorSetLayoutCreateInfo empty_set_info{};
                empty_set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                VkDescriptorSetLayout empty_set_layout = VK_NULL_HANDLE;
                if (vkCreateDescriptorSetLayout(device_.device(), &empty_set_info, nullptr,
                                                &empty_set_layout) != VK_SUCCESS)
                {
                    vkDestroyShaderModule(device_.device(), frag_module, nullptr);
                    vkDestroyShaderModule(device_.device(), vert_module, nullptr);
                    throw std::runtime_error(
                        "SushiEngine: vkCreateDescriptorSetLayout(mesh thumbnail empty set 0) "
                        "failed");
                }

                VkDescriptorSetLayout set_layouts[2] = {empty_set_layout, heap_.layout()};
                VkPushConstantRange push_range{};
                push_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
                push_range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                layout_info.setLayoutCount = 2;
                layout_info.pSetLayouts = set_layouts;
                layout_info.pushConstantRangeCount = 1;
                layout_info.pPushConstantRanges = &push_range;
                const VkResult layout_result = vkCreatePipelineLayout(
                    device_.device(), &layout_info, nullptr, &pipeline_layout_);
                vkDestroyDescriptorSetLayout(device_.device(), empty_set_layout, nullptr);
                if (layout_result != VK_SUCCESS)
                {
                    vkDestroyShaderModule(device_.device(), frag_module, nullptr);
                    vkDestroyShaderModule(device_.device(), vert_module, nullptr);
                    throw std::runtime_error(
                        "SushiEngine: vkCreatePipelineLayout(mesh thumbnail) failed");
                }

                VkPipelineShaderStageCreateInfo stages[2]{};
                stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
                stages[0].module = vert_module;
                stages[0].pName = "main";
                stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
                stages[1].module = frag_module;
                stages[1].pName = "main";

                // Matches MeshVertex's confirmed 60-byte layout: position@0 (offset 0),
                // normal@1 (offset 12), uv0@3 (offset 40) -- locations 2/4/5 (tangent/uv1/color)
                // are declared in MeshVertex but unused by mesh_thumbnail.vert, so they are
                // simply omitted here; Vulkan does not require contiguous attribute locations.
                VkVertexInputBindingDescription binding{};
                binding.binding = 0;
                binding.stride = 60; // sizeof(SushiEngine::Geometry::MeshVertex)
                binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

                VkVertexInputAttributeDescription attributes[3]{};
                attributes[0] = {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0};
                attributes[1] = {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 12};
                attributes[2] = {3, 0, VK_FORMAT_R32G32_SFLOAT, 40};

                VkPipelineVertexInputStateCreateInfo vertex_input{};
                vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
                vertex_input.vertexBindingDescriptionCount = 1;
                vertex_input.pVertexBindingDescriptions = &binding;
                vertex_input.vertexAttributeDescriptionCount = 3;
                vertex_input.pVertexAttributeDescriptions = attributes;

                VkPipelineInputAssemblyStateCreateInfo input_assembly{};
                input_assembly.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
                input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

                VkPipelineViewportStateCreateInfo viewport_state{};
                viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
                viewport_state.viewportCount = 1;
                viewport_state.scissorCount = 1;

                VkPipelineRasterizationStateCreateInfo rasterization{};
                rasterization.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
                rasterization.polygonMode = VK_POLYGON_MODE_FILL;
                rasterization.cullMode = VK_CULL_MODE_BACK_BIT;
                rasterization.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
                rasterization.lineWidth = 1.0f;

                VkPipelineMultisampleStateCreateInfo multisample{};
                multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
                multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

                // Reverse-Z (per Matrix4::perspective's documented convention): depth clears to
                // 0.0 and passes when the new fragment's depth is >= what's already there.
                VkPipelineDepthStencilStateCreateInfo depth_stencil{};
                depth_stencil.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
                depth_stencil.depthTestEnable = VK_TRUE;
                depth_stencil.depthWriteEnable = VK_TRUE;
                depth_stencil.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;

                VkPipelineColorBlendAttachmentState color_blend_attachment{};
                color_blend_attachment.colorWriteMask =
                    VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                    VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

                VkPipelineColorBlendStateCreateInfo color_blend{};
                color_blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
                color_blend.attachmentCount = 1;
                color_blend.pAttachments = &color_blend_attachment;

                const VkDynamicState dynamic_states[2] = {VK_DYNAMIC_STATE_VIEWPORT,
                                                          VK_DYNAMIC_STATE_SCISSOR};
                VkPipelineDynamicStateCreateInfo dynamic_state{};
                dynamic_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                dynamic_state.dynamicStateCount = 2;
                dynamic_state.pDynamicStates = dynamic_states;

                VkFormat color_format = COLOR_FORMAT;
                VkPipelineRenderingCreateInfo rendering_info{};
                rendering_info.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
                rendering_info.colorAttachmentCount = 1;
                rendering_info.pColorAttachmentFormats = &color_format;
                rendering_info.depthAttachmentFormat = DEPTH_FORMAT;

                VkGraphicsPipelineCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                pipeline_info.pNext = &rendering_info;
                pipeline_info.stageCount = 2;
                pipeline_info.pStages = stages;
                pipeline_info.pVertexInputState = &vertex_input;
                pipeline_info.pInputAssemblyState = &input_assembly;
                pipeline_info.pViewportState = &viewport_state;
                pipeline_info.pRasterizationState = &rasterization;
                pipeline_info.pMultisampleState = &multisample;
                pipeline_info.pDepthStencilState = &depth_stencil;
                pipeline_info.pColorBlendState = &color_blend;
                pipeline_info.pDynamicState = &dynamic_state;
                pipeline_info.layout = pipeline_layout_;

                const VkResult pipeline_result = vkCreateGraphicsPipelines(
                    device_.device(), VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &pipeline_);
                vkDestroyShaderModule(device_.device(), frag_module, nullptr);
                vkDestroyShaderModule(device_.device(), vert_module, nullptr);
                if (pipeline_result != VK_SUCCESS)
                {
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                    pipeline_layout_ = VK_NULL_HANDLE;
                    throw std::runtime_error(
                        "SushiEngine: vkCreateGraphicsPipelines(mesh thumbnail) failed");
                }
            }

            void VulkanMeshThumbnailRenderer::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
            }

            void VulkanMeshThumbnailRenderer::ensure_targets(std::uint32_t width,
                                                              std::uint32_t height)
            {
                if (width <= target_width_ && height <= target_height_ &&
                    color_image_ != VK_NULL_HANDLE)
                    return;

                destroy_targets();

                VkImageCreateInfo color_info{};
                color_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                color_info.imageType = VK_IMAGE_TYPE_2D;
                color_info.format = COLOR_FORMAT;
                color_info.extent = {width, height, 1};
                color_info.mipLevels = 1;
                color_info.arrayLayers = 1;
                color_info.samples = VK_SAMPLE_COUNT_1_BIT;
                color_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                color_info.usage =
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
                VmaAllocationCreateInfo alloc_info{};
                alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &color_info, &alloc_info,
                                            &color_image_, &color_allocation_, nullptr),
                              "vmaCreateImage(mesh thumbnail color)");

                VkImageViewCreateInfo color_view_info{};
                color_view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                color_view_info.image = color_image_;
                color_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                color_view_info.format = COLOR_FORMAT;
                color_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                color_view_info.subresourceRange.levelCount = 1;
                color_view_info.subresourceRange.layerCount = 1;
                Vulkan::check(
                    vkCreateImageView(device_.device(), &color_view_info, nullptr, &color_view_),
                    "vkCreateImageView(mesh thumbnail color)");

                VkImageCreateInfo depth_info = color_info;
                depth_info.format = DEPTH_FORMAT;
                depth_info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
                Vulkan::check(vmaCreateImage(device_.allocator(), &depth_info, &alloc_info,
                                            &depth_image_, &depth_allocation_, nullptr),
                              "vmaCreateImage(mesh thumbnail depth)");

                VkImageViewCreateInfo depth_view_info{};
                depth_view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                depth_view_info.image = depth_image_;
                depth_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                depth_view_info.format = DEPTH_FORMAT;
                depth_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
                depth_view_info.subresourceRange.levelCount = 1;
                depth_view_info.subresourceRange.layerCount = 1;
                Vulkan::check(
                    vkCreateImageView(device_.device(), &depth_view_info, nullptr, &depth_view_),
                    "vkCreateImageView(mesh thumbnail depth)");

                target_width_ = width;
                target_height_ = height;
            }

            void VulkanMeshThumbnailRenderer::destroy_targets()
            {
                if (depth_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), depth_view_, nullptr);
                if (depth_image_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), depth_image_, depth_allocation_);
                if (color_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), color_view_, nullptr);
                if (color_image_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), color_image_, color_allocation_);
                depth_view_ = VK_NULL_HANDLE;
                depth_image_ = VK_NULL_HANDLE;
                depth_allocation_ = VK_NULL_HANDLE;
                color_view_ = VK_NULL_HANDLE;
                color_image_ = VK_NULL_HANDLE;
                color_allocation_ = VK_NULL_HANDLE;
                target_width_ = 0;
                target_height_ = 0;
            }

            bool VulkanMeshThumbnailRenderer::render_thumbnail(const char* path,
                                                                std::uint32_t width,
                                                                std::uint32_t height,
                                                                FrameImage& out_image)
            {
                MeshId mesh_ids[MAX_PRIMITIVES];
                Render::Material materials[MAX_PRIMITIVES];
                Geometry::AABB3 bounds{};
                const std::size_t count = Assets::import_gltf(
                    path, meshes_, textures_, mesh_ids, materials, MAX_PRIMITIVES, &bounds);
                if (count == 0 || !bounds.initialized)
                    return false;

                ensure_targets(width, height);

                const Geometry::ThumbnailCamera camera = Geometry::three_quarter_camera_for_bounds(
                    bounds, static_cast<float>(width) / static_cast<float>(height));
                // import_gltf already bakes every node's world transform into its vertices, so
                // every primitive this call produced shares one consistent model space and is
                // drawn with an identity model matrix -- see the vertex shader's own comment.
                Matrix4 identity{};
                identity.m[0] = 1.0f;
                identity.m[5] = 1.0f;
                identity.m[10] = 1.0f;
                identity.m[15] = 1.0f;
                Matrix4 view_projection = mul(camera.projection, camera.view);

                VkCommandBufferAllocateInfo command_alloc{};
                command_alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                command_alloc.commandPool = command_pool_;
                command_alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                command_alloc.commandBufferCount = 1;
                VkCommandBuffer command = VK_NULL_HANDLE;
                Vulkan::check(
                    vkAllocateCommandBuffers(device_.device(), &command_alloc, &command),
                    "vkAllocateCommandBuffers(mesh thumbnail)");

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                Vulkan::check(vkBeginCommandBuffer(command, &begin),
                              "vkBeginCommandBuffer(mesh thumbnail)");

                VkImageMemoryBarrier2 to_attachment[2]{};
                to_attachment[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_attachment[0].srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                to_attachment[0].dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
                to_attachment[0].dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
                to_attachment[0].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                to_attachment[0].newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                to_attachment[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_attachment[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_attachment[0].image = color_image_;
                to_attachment[0].subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                to_attachment[1].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_attachment[1].srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                to_attachment[1].dstStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT;
                to_attachment[1].dstAccessMask =
                    VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
                to_attachment[1].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                to_attachment[1].newLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
                to_attachment[1].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_attachment[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_attachment[1].image = depth_image_;
                to_attachment[1].subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
                VkDependencyInfo to_attachment_dependency{};
                to_attachment_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                to_attachment_dependency.imageMemoryBarrierCount = 2;
                to_attachment_dependency.pImageMemoryBarriers = to_attachment;
                vkCmdPipelineBarrier2(command, &to_attachment_dependency);

                VkRenderingAttachmentInfo color_attachment{};
                color_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                color_attachment.imageView = color_view_;
                color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                color_attachment.clearValue.color = {{0.0f, 0.0f, 0.0f, 0.0f}};

                VkRenderingAttachmentInfo depth_attachment{};
                depth_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                depth_attachment.imageView = depth_view_;
                depth_attachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
                depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                // Reverse-Z: clears to 0.0, not 1.0.
                depth_attachment.clearValue.depthStencil = {0.0f, 0};

                VkRenderingInfo rendering_info{};
                rendering_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
                rendering_info.renderArea = {{0, 0}, {width, height}};
                rendering_info.layerCount = 1;
                rendering_info.colorAttachmentCount = 1;
                rendering_info.pColorAttachments = &color_attachment;
                rendering_info.pDepthAttachment = &depth_attachment;
                vkCmdBeginRendering(command, &rendering_info);

                VkViewport viewport{0.0f, 0.0f, static_cast<float>(width),
                                    static_cast<float>(height), 0.0f, 1.0f};
                VkRect2D scissor{{0, 0}, {width, height}};
                vkCmdSetViewport(command, 0, 1, &viewport);
                vkCmdSetScissor(command, 0, 1, &scissor);

                vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
                VkDescriptorSet heap_set = heap_.set();
                vkCmdBindDescriptorSets(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        pipeline_layout_, 1, 1, &heap_set, 0, nullptr);

                for (std::size_t i = 0; i < count; ++i)
                {
                    // TODO(verify): confirm MeshRegistry's exact public accessor name for a
                    // Mesh record by MeshId, and confirm whether Mesh is nested inside
                    // MeshRegistry (Geometry::MeshRegistry::Mesh, assumed below) or a
                    // free-standing sibling type (Geometry::Mesh) -- this plan's research
                    // confirmed the Mesh struct's fields (vertices/indices/vertex_count/
                    // index_count) but not its exact scope or the accessor method's name; check
                    // mesh_registry.hpp and adjust the type and the call below if either differs
                    // from what's written here.
                    const Geometry::MeshRegistry::Mesh& mesh = meshes_.mesh(mesh_ids[i]);

                    Push push{};
                    write_matrix(identity, push.model);
                    write_matrix(view_projection, push.view_projection);
                    push.albedo[0] = static_cast<float>(materials[i].albedo.x);
                    push.albedo[1] = static_cast<float>(materials[i].albedo.y);
                    push.albedo[2] = static_cast<float>(materials[i].albedo.z);
                    push.albedo[3] = materials[i].base_alpha;
                    push.albedo_texture_index =
                        materials[i].albedo_map != INVALID_TEXTURE
                            ? static_cast<std::int32_t>(materials[i].albedo_map)
                            : -1;
                    vkCmdPushConstants(command, pipeline_layout_,
                                       VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                       0, sizeof(Push), &push);

                    VkDeviceSize vertex_offset = 0;
                    vkCmdBindVertexBuffers(command, 0, 1, &mesh.vertices, &vertex_offset);
                    vkCmdBindIndexBuffer(command, mesh.indices, 0, VK_INDEX_TYPE_UINT32);
                    vkCmdDrawIndexed(command, mesh.index_count, 1, 0, 0, 0);
                }

                vkCmdEndRendering(command);

                VkImageMemoryBarrier2 to_transfer_src{};
                to_transfer_src.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_transfer_src.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
                to_transfer_src.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
                to_transfer_src.dstStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
                to_transfer_src.dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
                to_transfer_src.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                to_transfer_src.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
                to_transfer_src.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_transfer_src.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_transfer_src.image = color_image_;
                to_transfer_src.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                VkDependencyInfo to_transfer_src_dependency{};
                to_transfer_src_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                to_transfer_src_dependency.imageMemoryBarrierCount = 1;
                to_transfer_src_dependency.pImageMemoryBarriers = &to_transfer_src;
                vkCmdPipelineBarrier2(command, &to_transfer_src_dependency);

                const VkDeviceSize readback_size = VkDeviceSize(width) * height * 4;
                VkBufferCreateInfo readback_info{};
                readback_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                readback_info.size = readback_size;
                readback_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
                VmaAllocationCreateInfo readback_alloc_info{};
                readback_alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
                readback_alloc_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                            VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VkBuffer readback_buffer = VK_NULL_HANDLE;
                VmaAllocation readback_allocation = VK_NULL_HANDLE;
                VmaAllocationInfo readback_mapped{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &readback_info,
                                              &readback_alloc_info, &readback_buffer,
                                              &readback_allocation, &readback_mapped),
                              "vmaCreateBuffer(mesh thumbnail readback)");

                VkBufferImageCopy copy{};
                copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copy.imageSubresource.layerCount = 1;
                copy.imageExtent = {width, height, 1};
                vkCmdCopyImageToBuffer(command, color_image_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                       readback_buffer, 1, &copy);

                Vulkan::check(vkEndCommandBuffer(command),
                              "vkEndCommandBuffer(mesh thumbnail)");

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                VkFence fence = VK_NULL_HANDLE;
                Vulkan::check(vkCreateFence(device_.device(), &fence_info, nullptr, &fence),
                              "vkCreateFence(mesh thumbnail)");

                VkCommandBufferSubmitInfo command_submit{};
                command_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                command_submit.commandBuffer = command;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &command_submit;
                Vulkan::check(
                    vkQueueSubmit2(device_.graphics_queue(), 1, &submit, fence),
                    "vkQueueSubmit2(mesh thumbnail)");
                vkWaitForFences(device_.device(), 1, &fence, VK_TRUE, UINT64_MAX);

                vmaInvalidateAllocation(device_.allocator(), readback_allocation, 0,
                                        readback_size);
                out_image.width = width;
                out_image.height = height;
                out_image.rgba.resize(static_cast<std::size_t>(readback_size));
                std::memcpy(out_image.rgba.data(), readback_mapped.pMappedData,
                           static_cast<std::size_t>(readback_size));

                vkDestroyFence(device_.device(), fence, nullptr);
                vkFreeCommandBuffers(device_.device(), command_pool_, 1, &command);
                vmaDestroyBuffer(device_.allocator(), readback_buffer, readback_allocation);
                return true;
            }
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
```

- [ ] **Step 3: Verify by reading, not by building**

Per Global Constraints, this machine cannot build. Trace and confirm each of the following against
the actual, live headers you can read in this tree:

1. **`MeshRegistry::Mesh` accessor and scope** — open `engine/presentation/render/source/geometry/mesh_registry.hpp`
   and find the actual public method that returns a `Mesh` (or a reference/pointer to one) given a
   `MeshId`, and confirm whether `Mesh` is nested inside `MeshRegistry` (as this plan's code
   assumes, `Geometry::MeshRegistry::Mesh`) or a free-standing sibling type (`Geometry::Mesh`). If
   either the accessor's name or `Mesh`'s scope differs from what's written above (the
   `TODO(verify)` comment marks this exactly), correct both the type and the call — this is the one
   API surface this plan's research could not fully confirm despite five research passes into this
   codebase.
2. **`VulkanDevice`'s accessors** — confirm `device_.allocator()`, `device_.device()`,
   `device_.graphics_queue()`, and `device_.graphics_queue_family()` exist with these exact names
   (all four are used elsewhere in this codebase — `texture_library.cpp` and `pipeline_cache.cpp`,
   both already read during this plan's research — so this should already match; re-confirm rather
   than assume).
3. **`mul(a, b)`'s multiplication order** — open `blas_placeholder.hpp` and confirm `Matrix4 mul(const
   Matrix4&, const Matrix4&)` computes its first argument times its second in the conventional
   column-vector sense (`clip = projection * view * model * position`, right-multiplied) — this
   plan's code computes `view_projection = mul(camera.projection, camera.view)` on that assumption;
   if `mul`'s actual convention is reversed, swap the two arguments.
4. **`Push` struct layout matches both shaders' `Push` blocks exactly** — 16+16+4+1 = 37 `float`-sized
   slots is not naturally 16-byte-aligned as a whole (`std430`/`push_constant` block layout rules
   apply); re-check `mesh_thumbnail.vert`/`.frag`'s `Push` block against the C++ `Push` struct's
   actual size and alignment (`sizeof(Push)`) and confirm the shader compiler you're relying on
   (`sushiengine_shader_compiler`) and this C++ struct agree byte-for-byte — a mismatch here would
   silently corrupt values without any Vulkan validation error, since push constants are raw bytes
   with no runtime type checking.
5. **Cleanup ordering on every construction-failure path in `create_pipeline`** — confirm no Vulkan
   handle is destroyed twice and none is leaked, by re-tracing each `throw` site against exactly
   which of `vert_module`/`frag_module`/`empty_set_layout`/`pipeline_layout_` had already been
   created at that point.
6. **`Vulkan::check` and `vulkan_utils.hpp`** — confirm this header (used throughout this file) is
   actually the private engine-internal helper other render-tier `.cpp` files already use (this
   plan's research found it referenced by `texture_library.cpp`'s own `Vulkan::check(...)` calls);
   confirm its exact include path relative to this new file's location
   (`engine/presentation/render/source/rhi/vulkan/`).

- [ ] **Step 4: Commit**

```bash
git add engine/presentation/render/source/rhi/vulkan/vulkan_mesh_thumbnail_renderer.hpp \
        engine/presentation/render/source/rhi/vulkan/vulkan_mesh_thumbnail_renderer.cpp
git commit -m "feat(render): add VulkanMeshThumbnailRenderer"
```

---

### Task 6: Wire `create_mesh_thumbnail_renderer()` into `VulkanWindowRenderer`

**Files:**
- Modify: `engine/presentation/render/source/rhi/vulkan/vulkan_window_renderer.hpp`
- Modify: `engine/presentation/render/source/rhi/vulkan/vulkan_window_renderer.cpp`

**Interfaces:**
- Consumes: `SushiEngine::Render::Vulkan::VulkanMeshThumbnailRenderer` (Task 5).
- Produces: the working `IWindowRenderer::create_mesh_thumbnail_renderer()` implementation Phase 3b
  (the editor-tier plan, written after this one lands) will call from `main.cpp`.

- [ ] **Step 1: Declare the override**

In `engine/presentation/render/source/rhi/vulkan/vulkan_window_renderer.hpp`, find
`create_scene_view()`'s declaration (confirmed by this plan's research at line 103, alongside the
other overrides) and add the new one directly after it:

```cpp
                    std::unique_ptr<ISceneView> create_scene_view() override;
                    std::unique_ptr<IMeshThumbnailRenderer> create_mesh_thumbnail_renderer() override;
```

Add `#include "vulkan_mesh_thumbnail_renderer.hpp"` to this header's includes.

- [ ] **Step 2: Implement it**

In `engine/presentation/render/source/rhi/vulkan/vulkan_window_renderer.cpp`, find
`create_scene_view()`'s implementation (confirmed by this plan's research at lines 466-469:
`return std::unique_ptr<ISceneView>(new VulkanSceneView(device_, *assets_));`) and add the new method
directly after it, following the exact same one-line-forwarding shape:

```cpp
std::unique_ptr<IMeshThumbnailRenderer> VulkanWindowRenderer::create_mesh_thumbnail_renderer()
{
    return std::unique_ptr<IMeshThumbnailRenderer>(new VulkanMeshThumbnailRenderer(device_));
}
```

Unlike `create_scene_view()`, this does not forward `*assets_` — `VulkanMeshThumbnailRenderer`
builds its own isolated asset stack from `device_` alone (Task 5), which is the entire point of the
isolation this plan's approved design requires.

- [ ] **Step 3: Verify by reading, not by building**

Per Global Constraints, this machine cannot build. Confirm `VulkanWindowRenderer`'s `device_` member
(the same `VulkanDevice device_;` `create_scene_view()` already reads) is accessible from this new
method with the same visibility `create_scene_view()` already has (both are members of the same
class, so this should hold trivially — confirm rather than assume). Confirm the new
`#include "vulkan_mesh_thumbnail_renderer.hpp"` does not create a circular include (`vulkan_window_renderer.hpp`
including a header that itself includes `vulkan_window_renderer.hpp` back) — it should not, since
`vulkan_mesh_thumbnail_renderer.hpp` only needs `vulkan_device.hpp`, not the window renderer itself.

- [ ] **Step 4: Commit**

```bash
git add engine/presentation/render/source/rhi/vulkan/vulkan_window_renderer.hpp \
        engine/presentation/render/source/rhi/vulkan/vulkan_window_renderer.cpp
git commit -m "feat(render): implement IWindowRenderer::create_mesh_thumbnail_renderer"
```

---

## Manual verification (after the branch builds)

This plan adds no editor-facing surface — Phase 3b consumes it. Once the user has built the branch,
the only meaningful verification available before 3b lands is that the engine module and the render
module still build clean and the new unit tests (Task 1) pass under `se test`. A true end-to-end
check (does a real `.glb` file actually render a recognizable flat-shaded thumbnail) is only possible
once Phase 3b's editor integration exists to call `create_mesh_thumbnail_renderer()` from a running
`se editor` session — that verification belongs to Phase 3b's plan, not this one.
