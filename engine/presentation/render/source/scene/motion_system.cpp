/**************************************************************************/
/* motion_system.cpp                                                      */
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

#include "scene/motion_system.hpp"

#include <algorithm>
#include <cstring>

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            namespace
            {
                /** @brief Floats one packed transform occupies. */
                constexpr std::size_t MATRIX_FLOATS = 16;

                /** @brief Transforms a slot buffer starts out able to hold. */
                constexpr std::size_t INITIAL_CAPACITY = 256;
            } // namespace

            MotionSystem::MotionSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots)
                : device_(device)
            {
                slots_.resize(frame_slots);
                for (Slot& slot : slots_)
                    grow(slot, INITIAL_CAPACITY * MATRIX_FLOATS * sizeof(float));
            }

            MotionSystem::~MotionSystem()
            {
                for (Slot& slot : slots_)
                    if (slot.buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.buffer, slot.allocation);
                slots_.clear();
            }

            void MotionSystem::grow(Slot& slot, VkDeviceSize bytes)
            {
                if (bytes <= slot.capacity)
                    return;
                if (slot.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), slot.buffer, slot.allocation);

                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = bytes;
                buffer_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                              VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo info{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                              &slot.buffer, &slot.allocation, &info),
                              "vmaCreateBuffer(motion)");
                slot.mapped = info.pMappedData;
                slot.capacity = bytes;
            }

            void MotionSystem::begin_frame(std::uint32_t slot, const double eye[3])
            {
                current_slot_ = slot;
                packed_.clear();

                previous_.swap(current_);
                current_.clear();
                previous_index_.clear();
                previous_index_.reserve(previous_.size());
                for (std::uint32_t i = 0; i < previous_.size(); ++i)
                    previous_index_[previous_[i].id] = i;

                for (int i = 0; i < 3; ++i)
                    eye_[i] = eye[i];
            }

            std::uint32_t MotionSystem::append(const Mat4& previous_absolute)
            {
                const std::uint32_t index =
                    static_cast<std::uint32_t>(packed_.size() / MATRIX_FLOATS);
                packed_.resize(packed_.size() + MATRIX_FLOATS);
                float* out = packed_.data() + index * MATRIX_FLOATS;
                for (std::size_t i = 0; i < MATRIX_FLOATS; ++i)
                    out[i] = static_cast<float>(previous_absolute.m[i]);
                // The eye is subtracted in double, before the float cast, exactly as the
                // push constant's current transform does — and against the *previous*
                // eye, because this transform is consumed by the previous world-to-clip.
                out[12] = static_cast<float>(previous_absolute.m[12] - previous_eye_[0]);
                out[13] = static_cast<float>(previous_absolute.m[13] - previous_eye_[1]);
                out[14] = static_cast<float>(previous_absolute.m[14] - previous_eye_[2]);
                return index;
            }

            std::uint32_t MotionSystem::push(std::uint32_t entity_id, const Mat4& model)
            {
                current_.push_back(Record{entity_id, model});

                // An id of zero is the "no pick" sentinel every unnamed draw carries, so
                // it identifies nothing and must not be allowed to match another draw's
                // history; such geometry is static, and its current transform is the
                // right answer for where it was.
                const Mat4* previous = &model;
                if (entity_id != 0)
                {
                    const auto found = previous_index_.find(entity_id);
                    if (found != previous_index_.end())
                        previous = &previous_[found->second].model;
                }
                return append(*previous);
            }

            std::uint32_t MotionSystem::push_camera_relative()
            {
                // The vertices arrive relative to this frame's eye, so the transform that
                // carries them into the previous frame's camera-relative space is the
                // shift between the two eyes — and append() will subtract the previous
                // eye from the translation column, so it is added here first.
                Mat4 shift;
                shift.m[12] = eye_[0];
                shift.m[13] = eye_[1];
                shift.m[14] = eye_[2];
                return append(shift);
            }

            void MotionSystem::upload()
            {
                Slot& slot = slots_[current_slot_];
                // Always keep at least one entry: a zero-range storage buffer is not a
                // legal descriptor, and a frame with nothing to draw still writes a set.
                const std::size_t floats = std::max<std::size_t>(packed_.size(), MATRIX_FLOATS);
                grow(slot, floats * sizeof(float));
                if (!packed_.empty())
                    std::memcpy(slot.mapped, packed_.data(), packed_.size() * sizeof(float));
            }

            void MotionSystem::end_frame()
            {
                for (int i = 0; i < 3; ++i)
                    previous_eye_[i] = eye_[i];
            }

            VkBuffer MotionSystem::buffer() const noexcept
            {
                return slots_[current_slot_].buffer;
            }

            VkDeviceSize MotionSystem::buffer_range() const noexcept
            {
                const std::size_t floats = std::max<std::size_t>(packed_.size(), MATRIX_FLOATS);
                return static_cast<VkDeviceSize>(floats * sizeof(float));
            }
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
