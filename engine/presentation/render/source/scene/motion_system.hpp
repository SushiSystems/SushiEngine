/**************************************************************************/
/* motion_system.hpp                                                      */
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
 * @file motion_system.hpp
 * @brief Where each drawn object was last frame, packed for the vertex shader.
 *
 * A motion vector is the difference between where a vertex is now and where the same
 * vertex was one frame ago, so the geometry pass needs both transforms. The current
 * one already travels in the push constant; the previous one is looked up here by the
 * object's picking id and packed into a per-frame array the draw indexes with — the
 * same shape as the material array, and for the same reason: the push constant stays
 * fixed-size no matter how much per-object state the renderer grows.
 *
 * Everything packed here is camera-relative against the *previous* frame's eye, to
 * match the previous world-to-clip the temporal block carries. That pairing is what
 * makes the camera's own translation show up in the motion vector — a static object
 * does move across the screen when the camera does — while keeping planet-scale metres
 * out of single precision on both sides of the subtraction.
 */

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Scene
        {
            /**
             * @brief Packs the previous frame's transform for every object drawn this frame.
             *
             * Per frame slot rather than shared, because the buffer is host-written each
             * frame and a slot still in flight may be reading the one before it.
             * Non-copyable: it owns Vulkan buffers.
             */
            class MotionSystem
            {
                public:
                    /**
                     * @brief Allocates one packing buffer per frame slot.
                     * @param device      The live Vulkan device.
                     * @param frame_slots How many frames may be in flight.
                     */
                    MotionSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots);
                    ~MotionSystem();

                    MotionSystem(const MotionSystem&) = delete;
                    MotionSystem& operator=(const MotionSystem&) = delete;

                    /**
                     * @brief Starts a frame: retires last frame's transforms as the history.
                     * @param slot Which frame slot is being recorded.
                     * @param eye  This frame's camera world position, metres.
                     */
                    void begin_frame(std::uint32_t slot, const double eye[3]);

                    /**
                     * @brief Records an object's transform and packs where it was last frame.
                     *
                     * An object with no history — spawned this frame, or the first frame
                     * of a view — packs its current transform, which still yields the
                     * camera's own contribution to its screen motion and nothing else.
                     *
                     * @param entity_id The object's picking id, which is what keys history.
                     * @param model     This frame's object-to-world transform, absolute.
                     * @return The index the draw carries into the frame's motion array.
                     */
                    std::uint32_t push(std::uint32_t entity_id, const Mat4& model);

                    /**
                     * @brief Packs motion for geometry the CPU already made camera-relative.
                     *
                     * Soft bodies rebuild their vertices around the eye every frame, so
                     * their draw carries no model matrix to look a history up with. What
                     * is packed instead is the shift between the two frames' eyes, which
                     * is the part of their screen motion that is knowable — their own
                     * deformation is not, and the neighbourhood clamp absorbs it.
                     *
                     * @return The index the draw carries into the frame's motion array.
                     */
                    std::uint32_t push_camera_relative();

                    /** @brief Copies the packed array into this slot's buffer. */
                    void upload();

                    /** @brief Closes the frame, promoting what was pushed to next frame's history. */
                    void end_frame();

                    /** @brief The buffer holding this slot's packed previous transforms. */
                    VkBuffer buffer() const noexcept;

                    /** @brief Bytes of that buffer the descriptor must expose. */
                    VkDeviceSize buffer_range() const noexcept;

                private:
                    /** @brief One frame slot's packing buffer. */
                    struct Slot
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize capacity = 0;
                    };

                    /** @brief One object's absolute transform, kept until the next frame. */
                    struct Record
                    {
                        std::uint32_t id = 0;
                        Mat4 model{};
                    };

                    void grow(Slot& slot, VkDeviceSize bytes);
                    std::uint32_t append(const Mat4& previous_absolute);

                    Vulkan::VulkanDevice& device_;
                    std::vector<Slot> slots_;
                    std::vector<float> packed_;
                    std::vector<Record> current_;
                    std::vector<Record> previous_;
                    std::unordered_map<std::uint32_t, std::uint32_t> previous_index_;
                    double eye_[3] = {0.0, 0.0, 0.0};
                    double previous_eye_[3] = {0.0, 0.0, 0.0};
                    std::uint32_t current_slot_ = 0;
            };
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
