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
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/environment/environment.hpp>
#include <SushiEngine/environment/light.hpp>
#include <SushiEngine/render/deformable_mesh.hpp>
#include <SushiEngine/render/render_settings.hpp>
#include <SushiEngine/ui/draw_list.hpp>

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
         * per-frame copy from `RenderInstance` to `MeshInstance` (see applications/editor/source/main.cpp)
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
         * @brief One skinned character to skin and draw this frame — the extract seam.
         *
         * The renderer's view of a skinned instance (design §3, "SkinnedGeometry channel"):
         * a mesh that carries a skin vertex stream, the object-to-world transform, and the
         * object-space joint palettes (current and previous frame) the compute skinning pass
         * consumes. The renderer never sees clips, controllers, or the evaluator — only
         * palette floats and a mesh, exactly as it sees cloth vertices. @c palette and
         * @c previous_palette point at @c joint_count matrices of 16 column-major floats each
         * (`JointMatrix`), object space; @c previous_palette may be null on an instance's
         * first frame, in which case the skinning pass reuses @c palette for prev-position.
         */
        struct SkinnedInstance
        {
            Mat4 model;                               /**< Object-to-world transform. */
            const void* palette = nullptr;            /**< joint_count × 16 floats, this frame. */
            const void* previous_palette = nullptr;   /**< Same, last frame (null = reuse current). */
            std::uint32_t joint_count = 0;            /**< Joints in the palettes. */
            std::uint32_t id = 0;                     /**< Picking id (0 = none). */
            MeshId mesh = INVALID_MESH;               /**< A skinned mesh (carries a skin stream). */
            Material material{};                      /**< Surface to shade with. */
            /**
             * @brief Per-target morph weights, in the mesh's target order, or null for none.
             *
             * Design §6.5/§12.1: the SkinningPass blends `Σ weight × delta` into the base
             * position before joint skinning. Length is @c morph_target_count, which must not
             * exceed the mesh's own target count (extras are ignored, a short array pads with
             * zero weight for the remaining targets).
             */
            const float* morph_weights = nullptr;
            std::uint32_t morph_target_count = 0;     /**< Entries in @c morph_weights. */
            /**
             * @brief Opt into dual-quaternion skinning instead of linear-blend (design §12.4).
             *
             * `SkinningSystem` always derives a dual-quaternion palette alongside the linear
             * one (cheap, and correct for any rigid — no non-uniform-scale — joint palette), so
             * this is a pure per-instance switch, not a cost an instance pays only when set.
             * Fixes the "candy wrapper" pinch a large bend shows under linear-blend skinning;
             * see `animation/dual_quaternion_skinning.hpp` for the verified blend math this
             * flips on. Previous-frame motion vectors always sample the linear-blend previous
             * palette regardless of this flag — the sub-pixel difference is not worth a second
             * previous-frame dual-quaternion palette.
             */
            bool use_dual_quaternion_skinning = false;
        };

        /**
         * @brief One VFX emitter to simulate and draw this frame — the particle extract seam.
         *
         * The renderer's view of a cosmetic (GPU-simulated) emitter. As with @ref SkinnedInstance
         * it never sees the authoring types: @c compiled points at the emitter's flattened POD
         * parameters (a `VFX::CompiledEmitter`), and @c curve_luts / @c gradient_luts at the
         * effect's baked look-up-table atlases the compiled offsets index into — all opaque
         * bytes to the renderer, which uploads them and lets the compute passes interpret them.
         * @c spawn_count is how many particles the host decided to emit this frame (rate over
         * time plus bursts, advanced host-side), so the emit shader stays a pure allocator.
         */
        struct ParticleEmitterView
        {
            Mat4 model;                            /**< Emitter object-to-world transform. */
            const void* compiled = nullptr;        /**< `VFX::CompiledEmitter*`, opaque here. */
            const float* curve_luts = nullptr;     /**< The effect's baked scalar-curve atlas. */
            const float* gradient_luts = nullptr;  /**< The effect's baked RGBA gradient atlas. */
            std::uint32_t curve_lut_floats = 0;    /**< Length of @ref curve_luts in floats. */
            std::uint32_t gradient_lut_floats = 0; /**< Length of @ref gradient_luts in floats. */
            std::uint32_t spawn_count = 0;         /**< Particles to emit this frame (host-computed). */
            std::uint32_t seed = 0;                /**< Emitter RNG seed. */
            float dt = 0.0f;                       /**< Simulation timestep this frame, seconds. */
            std::uint32_t id = 0;                  /**< Picking id (0 = none). */
        };

        /**
         * @brief One already-simulated particle drawn as a camera-facing billboard.
         *
         * The extract seam for the CPU-deterministic path: unlike @ref ParticleEmitterView (an
         * emitter the GPU simulates), these are final world-space particles the sim advanced on
         * its fixed tick, handed to the renderer to billboard directly — the particle analogue of
         * @ref DeformableMeshView's already-simulated vertices.
         */
        struct ParticleBillboard
        {
            Vector3 position;                /**< World-space centre. */
            Vector3 color{Vector3{1, 1, 1}}; /**< Linear-RGB tint. */
            float size = 0.1f;               /**< World-space size. */
            float alpha = 1.0f;              /**< Opacity. */
            float rotation = 0.0f;           /**< Roll, radians. */
        };

        /** @brief The id a pick returns when no instance covers the sampled pixel. */
        constexpr std::uint32_t NO_PICK = 0;

        /**
         * @brief The frame's 2D UI overlay: already-resolved screen-space geometry.
         *
         * Non-owning, like every other view here: the arrays must outlive the `render()`
         * call and no longer. The rectangles are in pixels with a top-left origin and are
         * already laid out — anchors, pivots and parent chains were resolved by whoever
         * built the list, so the renderer neither knows nor needs the layout rules.
         *
         * @ref width and @ref height are the screen the layout was solved against, which is
         * what turns those pixels into clip space. They are carried rather than taken from
         * the render target because an editor viewport solves its UI against the viewport it
         * is drawn into, not against the window.
         */
        struct UIView
        {
            const UI::UIDrawRect* rects = nullptr;
            std::size_t rect_count = 0;
            const UI::UITextRun* texts = nullptr;
            std::size_t text_count = 0;
            float width = 0.0f;  /**< Screen width the layout was solved against, pixels. */
            float height = 0.0f; /**< Screen height the layout was solved against, pixels. */

            /** @brief Whether this frame has any UI to draw at all. */
            bool empty() const noexcept
            {
                return (rect_count == 0 && text_count == 0) || width <= 0.0f || height <= 0.0f;
            }
        };

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

        /**
         * @brief One finished frame copied back to host memory, tightly packed RGBA8.
         *
         * Exists for the golden-image harness (`docs/slop/cross_platform_engineering_plan.md`
         * RHI0), which is the only thing in the engine that needs to *look* at what the
         * renderer drew rather than display it. Deliberately the view's output image and
         * not an HDR intermediate: the hash has to be over the pixels a person would have
         * seen, so that a golden mismatch is always a visible difference.
         */
        struct FrameImage
        {
            std::uint32_t width = 0;  /**< Image width in pixels. */
            std::uint32_t height = 0; /**< Image height in pixels. */
            /** @brief @ref width × @ref height RGBA8 pixels, row-major, no padding. */
            std::vector<std::uint8_t> rgba;
        };

        /**
         * @brief One texture one pass wrote, and a hash of what it held afterwards.
         *
         * The whole-frame @ref FrameImage says an image changed; this says which pass
         * changed it. @c pass and @c resource together are the identity, so a golden
         * survives passes being reordered but not renamed — renaming a pass is a change
         * to what the frame is made of, and the harness is right to notice.
         */
        struct PassOutputHash
        {
            std::string pass;     /**< The pass name the graph registered. */
            std::string resource; /**< The written texture's debug name. */
            std::uint64_t hash = 0;
        };

        /**
         * @brief A frame's per-pass hashes, and what the capture could not reach.
         *
         * The counts are part of the result rather than a query beside it, because a
         * caller recording a reference from an incomplete capture would be recording one
         * that cannot notice the passes it never saw. Making them impossible to receive
         * without seeing is the whole reason this is a struct.
         */
        struct PassCaptureReport
        {
            /** @brief One entry per captured output, in the order the frame produced them. */
            std::vector<PassOutputHash> passes;
            /** @brief Outputs the frame could not afford; the capture was incomplete. */
            std::uint32_t dropped_by_budget = 0;
            /** @brief Outputs whose format or usage made them un-copyable. */
            std::uint32_t dropped_by_format = 0;
            std::uint64_t bytes_used = 0;
            std::uint64_t bytes_budget = 0;
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
                 * @param deformable       Pointer to the host-simulated surfaces to shade and
                 *                         draw, or nullptr for none.
                 * @param deformable_count Number of entries in @p deformable.
                 * @param lights        Pointer to the punctual lights to shade with, or
                 *                      nullptr for none; culled into the froxel grid.
                 * @param light_count   Number of entries in @p lights.
                 * @param decals        Pointer to the projected decals, or nullptr for none;
                 *                      culled into the same froxel grid.
                 * @param decal_count   Number of entries in @p decals.
                 * @param show_grid     Draw the editor reference grid overlay. Off for a
                 *                      shipped runtime; the Scene viewport turns it on.
                 * @param skinned       Pointer to the skinned characters to skin and draw, or
                 *                      nullptr for none; the compute skinning pass deforms them.
                 * @param skinned_count Number of entries in @p skinned.
                 * @param emitters      Pointer to the cosmetic particle emitters to simulate and
                 *                      draw, or nullptr for none; the compute particle passes
                 *                      emit, integrate, and billboard them.
                 * @param emitter_count Number of entries in @p emitters.
                 * @param billboards    Pointer to already-simulated deterministic particles to
                 *                      billboard directly, or nullptr for none.
                 * @param billboard_count Number of entries in @p billboards.
                 * @param ui            The 2D UI overlay to composite over the finished image,
                 *                      or nullptr for none. Drawn after tone mapping and
                 *                      anti-aliasing, so it is neither tonemapped nor blurred.
                 */
                virtual void render(const CameraView& camera, const Environment& environment,
                                    const MeshInstance* instances,
                                    std::size_t count, std::uint32_t selected_id,
                                    const DeformableMeshView* deformable = nullptr,
                                    std::size_t deformable_count = 0,
                                    const PunctualLight* lights = nullptr,
                                    std::size_t light_count = 0,
                                    const Decal* decals = nullptr,
                                    std::size_t decal_count = 0,
                                    bool show_grid = false,
                                    const SkinnedInstance* skinned = nullptr,
                                    std::size_t skinned_count = 0,
                                    const ParticleEmitterView* emitters = nullptr,
                                    std::size_t emitter_count = 0,
                                    const ParticleBillboard* billboards = nullptr,
                                    std::size_t billboard_count = 0,
                                    const UIView* ui = nullptr) = 0;

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

                /**
                 * @brief Copies a finished slot's output image back to host memory.
                 *
                 * The seam the golden-image harness reads through, and the only reason it
                 * exists — nothing that displays a frame needs this, which is why it is a
                 * capability with a default rather than an obligation on every backend.
                 *
                 * Synchronous by design: it waits for @p slot's submit and then does its own
                 * one-shot copy, so a caller may read any slot at any time without knowing
                 * how deep the frame chain runs. That makes it unfit for anything per-frame,
                 * which is correct — a harness renders a fixed number of frames and looks at
                 * one of them.
                 *
                 * @param slot  The slot to read; normally @ref current_slot after a render().
                 * @param image Receives the pixels; untouched when the read is refused.
                 * @return Whether the image was produced. False when the backend cannot read
                 *         back at all (the default) or when @p slot has never been rendered.
                 */
                virtual bool read_output(std::uint32_t slot, FrameImage& image)
                {
                    (void)slot;
                    (void)image;
                    return false;
                }

                /**
                 * @brief Turns per-pass output hashing on or off for following frames.
                 *
                 * A debug instrument, off by default and refusable: a backend that cannot
                 * capture returns false and is not thereby broken. Enabling it costs
                 * bandwidth and perturbs how the frame's transients are allocated, so it
                 * is for a golden run and a bisect, not for a shipping frame.
                 *
                 * Takes effect from the next render(): the frame in progress has already
                 * decided what its resources are.
                 *
                 * **Invalidates every texture() previously handed out, exactly as resize()
                 * does, and discards any accumulated temporal history.** A backend may have
                 * to rebuild its targets to make them legal copy sources — a usage flag is
                 * fixed at creation — and that is the price of not carrying the flag in
                 * builds that never capture. A host that registered textures with a UI
                 * backend must re-register them; a caller that renders a fixed number of
                 * frames from cold, which is what a harness does, pays nothing.
                 *
                 * @param enabled Whether following frames should capture.
                 * @return Whether the backend honoured the request.
                 */
                virtual bool enable_pass_capture(bool enabled)
                {
                    (void)enabled;
                    return false;
                }

                /**
                 * @brief Reads the per-pass hashes of a rendered slot.
                 *
                 * @param slot   The slot to read; normally @ref current_slot after render().
                 * @param report Receives the hashes and what the capture could not reach;
                 *               untouched when the read is refused.
                 * @return Whether hashes were produced. False when the backend cannot
                 *         capture, when capture is off, or when @p slot holds no capture.
                 */
                virtual bool read_pass_hashes(std::uint32_t slot, PassCaptureReport& report)
                {
                    (void)slot;
                    (void)report;
                    return false;
                }
        };
    } // namespace Render
} // namespace SushiEngine
