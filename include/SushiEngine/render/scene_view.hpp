/**************************************************************************/
/* scene_view.hpp                                                         */
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
 * @file scene_view.hpp
 * @brief An offscreen 3D view a host samples into its UI.
 *
 * A scene view renders a camera's view of a set of mesh instances (plus a ground
 * grid) into an offscreen colour target and exposes that target for sampling — the
 * editor draws it with ImGui::Image inside a Viewport panel. The renderer keeps the
 * Vulkan image; the host registers it with its UI backend through the neutral
 * SceneViewTexture handles (a VkSampler and VkImageView as void*). The view is
 * double-buffered internally so the frame being sampled is never the frame being
 * drawn; render() reports which slot it just produced.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/render/light.hpp>
#include <SushiEngine/render/render_settings.hpp>

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief A camera's world-to-clip transform plus its world position.
         *
         * @c view and @c projection shade the meshes; @c world_position is the camera's
         * absolute (ECEF-anchored) eye in double precision, which the atmosphere/planet
         * pass needs to place the planet relative to the camera without single-precision
         * blow-up at planet scale. @c near_plane / @c far_plane linearise the sampled
         * depth in the sky pass for aerial perspective.
         */
        struct CameraView
        {
            Mat4 view; /**< World-to-camera. */
            Mat4 projection; /**< Camera-to-clip (Vulkan depth 0..1, Y-flipped). */
            WorldVector3 world_position{}; /**< Absolute eye position, metres. */
            float near_plane = 0.1f;  /**< Near clip distance, for depth linearisation. */
            float far_plane = 1000.0f; /**< Far clip distance, for depth linearisation. */
        };

        /**
         * @brief Which of the renderer's built-in unit meshes an instance draws with.
         *
         * A render-side enum rather than a reuse of `Simulation::PrimitiveKind`, so
         * this header stays free of any dependency on the sim seam — the editor's
         * per-frame copy from `RenderInstance` to `MeshInstance` (see editor/main.cpp)
         * maps one to the other, the same place colour and transform are copied.
         */
        enum class MeshKind : std::uint32_t
        {
            Box,
            Sphere,
            Cylinder,
        };

        /** @brief One mesh drawn this frame: its world transform, material, and pick id. */
        struct MeshInstance
        {
            Mat4 model;              /**< Object-to-world transform. */
            Vector3 color;              /**< Base colour; also seeds @ref material.albedo. */
            std::uint32_t id = 0;    /**< Picking id written to the id target (0 = none). */
            MeshKind kind = MeshKind::Box; /**< Which unit mesh to draw this instance with. */
            Vector3 shape_params{Vector3{0.5, 0.5, 0.5}}; /**< Box: half-extents. Sphere: radius in x. Cylinder: radius in x, half-height in y. */
            Material material{}; /**< PBR metallic-roughness surface this instance shades with. */
            /**
             * @brief An imported mesh to draw instead of the primitive named by @ref kind.
             *
             * INVALID_MESH — the default — draws the primitive, so an instance that has
             * never seen an imported asset behaves exactly as it did before glTF import
             * existed. When set, @ref kind and @ref shape_params are ignored: an imported
             * mesh carries its own geometry and its own scale.
             */
            MeshId mesh = INVALID_MESH;
        };

        /**
         * @brief One simulated soft-body grid's world-space points, ready to triangulate and draw.
         *
         * A non-owning view: `vertices` points at `rows * cols` row-major points
         * (matching `Physics::ClothGrid`) owned by the caller for the duration of
         * the `render()` call that receives it. The scene view triangulates the grid
         * (see `triangulate_cloth_grid`) and draws it as a shaded, pickable mesh
         * rather than a wireframe.
         */
        struct ClothStrandView
        {
            std::uint32_t rows = 0;  /**< Grid rows. */
            std::uint32_t cols = 0;  /**< Grid columns. */
            const Vector3* vertices = nullptr; /**< Row-major world-space points, rows * cols long. */
            Vector3 color{Vector3{0.85, 0.85, 0.9}}; /**< Base colour. */
            std::uint32_t id = 0;    /**< Picking id written to the id target (0 = none). */
        };

        /** @brief The id a pick returns when no instance covers the sampled pixel. */
        constexpr std::uint32_t NO_PICK = 0;

        /**
         * @brief One render pass's measured GPU time from the last completed frame.
         *
         * @c name points at storage the scene view owns and is valid until the next
         * render(); the host copies it if it needs to keep it.
         */
        struct ScenePassTiming
        {
            const char* name = "";     /**< The pass name as registered in the render graph. */
            float milliseconds = 0.0f; /**< GPU time between the pass's two timestamps. */
        };

        /** @brief Native handles a UI backend needs to sample one target slot. */
        struct SceneViewTexture
        {
            void* sampler = nullptr;    /**< VkSampler for the offscreen colour image. */
            void* image_view = nullptr; /**< VkImageView of the offscreen colour image. */
        };

        /**
         * @brief An offscreen camera view of a mesh scene, sampled into a UI panel.
         *
         * Owns its colour and depth targets and the pipelines that draw into them.
         * resize() matches the target to the panel; render() draws one frame from a
         * camera; the host reads the just-drawn slot's texture to display it.
         */
        class ISceneView
        {
            public:
                virtual ~ISceneView() = default;

                /**
                 * @brief Resizes the offscreen targets; a no-op if unchanged.
                 *
                 * Invalidates previously returned textures for every slot, so the host
                 * must re-register them with its UI backend after a resize.
                 *
                 * @param width  New target width in pixels (clamped to >= 1).
                 * @param height New target height in pixels (clamped to >= 1).
                 */
                virtual void resize(std::uint32_t width, std::uint32_t height) = 0;

                /** @brief Current target width in pixels. */
                virtual std::uint32_t width() const noexcept = 0;

                /** @brief Current target height in pixels. */
                virtual std::uint32_t height() const noexcept = 0;

                /**
                 * @brief Sets how the view trades fidelity against frame time.
                 *
                 * Takes effect from the next render(); the view keeps its own copy, so
                 * the host may pass a temporary. Changing the anti-aliasing mode or the
                 * render scale never reallocates the targets the host samples, so no
                 * texture the host registered is invalidated.
                 *
                 * @param settings The requested quality, anti-aliasing, and scaling.
                 */
                virtual void set_settings(const RenderSettings& settings) = 0;

                /** @brief The settings the next frame will be drawn with. */
                virtual const RenderSettings& settings() const noexcept = 0;

                /**
                 * @brief The internal resolution the last frame was actually rendered at.
                 *
                 * Equal to width()/height() unless the render scale or the dynamic
                 * resolution controller reduced it; the temporal resolve upscales from
                 * here to the output size. Reported so a host can surface what the
                 * governor decided.
                 *
                 * @param width  Receives the internal render width in pixels.
                 * @param height Receives the internal render height in pixels.
                 */
                virtual void render_resolution(std::uint32_t& width,
                                               std::uint32_t& height) const noexcept = 0;

                /**
                 * @brief Draws one frame: the ground grid plus every instance.
                 *
                 * Records and submits an offscreen pass into the next slot and leaves
                 * its colour image ready to sample. Same-queue ordering makes the
                 * result visible to the UI submit that follows.
                 *
                 * @param camera      The view, projection, and world eye to render from.
                 * @param environment The sun, planet, atmosphere, clouds, and stars to
                 *                    light and surround the scene with this frame.
                 * @param instances   Pointer to the mesh instances to draw.
                 * @param count       Number of instances.
                 * @param selected_id The instance id to highlight, or NO_PICK for none.
                 * @param strands       Pointer to the soft-body wireframes to draw, or
                 *                      nullptr for none.
                 * @param strand_count  Number of entries in @p strands.
                 * @param lights        Pointer to the punctual lights to shade with, or
                 *                      nullptr for none; culled into the froxel grid.
                 * @param light_count   Number of entries in @p lights.
                 * @param decals        Pointer to the projected decals, or nullptr for none;
                 *                      culled into the same froxel grid.
                 * @param decal_count   Number of entries in @p decals.
                 * @param show_grid     Draw the editor reference grid overlay. Off for a
                 *                      shipped runtime; the Scene viewport turns it on.
                 */
                virtual void render(const CameraView& camera, const Environment& environment,
                                    const MeshInstance* instances,
                                    std::size_t count, std::uint32_t selected_id,
                                    const ClothStrandView* strands = nullptr,
                                    std::size_t strand_count = 0,
                                    const PunctualLight* lights = nullptr,
                                    std::size_t light_count = 0,
                                    const Decal* decals = nullptr,
                                    std::size_t decal_count = 0,
                                    bool show_grid = false) = 0;

                /**
                 * @brief The instance id drawn at a pixel of the last rendered frame.
                 *
                 * Reads back the id target the render pass wrote, so a host maps a
                 * click in the panel to the entity under the cursor. Coordinates are in
                 * target pixels; out-of-range or empty pixels return NO_PICK.
                 *
                 * @param x Pixel x in [0, width()).
                 * @param y Pixel y in [0, height()).
                 * @return The instance id at that pixel, or NO_PICK.
                 */
                virtual std::uint32_t pick(std::uint32_t x, std::uint32_t y) = 0;

                /** @brief Number of double-buffered target slots. */
                virtual std::uint32_t slot_count() const noexcept = 0;

                /**
                 * @brief The sampler/view pair for a target slot.
                 * @param slot Slot index in [0, slot_count()).
                 * @return The native handles to register with the UI backend.
                 */
                virtual SceneViewTexture texture(std::uint32_t slot) const noexcept = 0;

                /** @brief The slot produced by the most recent render(). */
                virtual std::uint32_t current_slot() const noexcept = 0;

                /**
                 * @brief Number of per-pass GPU timings available.
                 *
                 * Zero until enough frames have completed for a timed submit to be read
                 * back, and zero for the whole run on a device without timestamp queries.
                 */
                virtual std::size_t pass_timing_count() const noexcept = 0;

                /**
                 * @brief One pass's GPU time from the most recently resolved frame.
                 * @param index Timing index in [0, pass_timing_count()).
                 * @return The pass's name and measured milliseconds.
                 */
                virtual ScenePassTiming pass_timing(std::size_t index) const noexcept = 0;

                /**
                 * @brief The GPU-driven cull counts from the last completed frame.
                 *
                 * How many mesh instances survived the cull and how many were tested, read
                 * back from the cull pass a frame late. Both zero on a frame that took the
                 * classic path (a lower tier, the path disabled, or an object selected) and
                 * on a view whose backend does not cull on the GPU — the default here.
                 *
                 * @param drawn  Receives the instances that survived and were drawn.
                 * @param tested Receives the instances the cull considered.
                 */
                virtual void cull_statistics(std::uint32_t& drawn,
                                             std::uint32_t& tested) const noexcept
                {
                    drawn = 0;
                    tested = 0;
                }
        };
    } // namespace Render
} // namespace SushiEngine
