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

namespace SushiEngine
{
    namespace render
    {
        /** @brief A camera's world-to-clip transform, split into its two matrices. */
        struct CameraView
        {
            Mat4 view; /**< World-to-camera. */
            Mat4 projection; /**< Camera-to-clip (Vulkan depth 0..1, Y-flipped). */
        };

        /** @brief One mesh drawn this frame: its world transform, colour, and pick id. */
        struct MeshInstance
        {
            Mat4 model;              /**< Object-to-world transform. */
            Vec3 color;              /**< Base colour. */
            std::uint32_t id = 0;    /**< Picking id written to the id target (0 = none). */
        };

        /** @brief The id a pick returns when no instance covers the sampled pixel. */
        constexpr std::uint32_t NO_PICK = 0;

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
                 * @brief Draws one frame: the ground grid plus every instance.
                 *
                 * Records and submits an offscreen pass into the next slot and leaves
                 * its colour image ready to sample. Same-queue ordering makes the
                 * result visible to the UI submit that follows.
                 *
                 * @param camera      The view and projection to render from.
                 * @param instances   Pointer to the mesh instances to draw.
                 * @param count       Number of instances.
                 * @param selected_id The instance id to highlight, or NO_PICK for none.
                 */
                virtual void render(const CameraView& camera, const MeshInstance* instances,
                                    std::size_t count, std::uint32_t selected_id) = 0;

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
        };
    } // namespace render
} // namespace SushiEngine
