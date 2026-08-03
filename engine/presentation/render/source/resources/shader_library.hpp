/**************************************************************************/
/* shader_library.hpp                                                     */
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
 * @file shader_library.hpp
 * @brief Named shader modules, with source watching and in-process recompilation.
 *
 * Every shader ships as SPIR-V embedded at build time, so a release build needs no
 * compiler and no shader files on disk. When the source tree that produced those
 * words is still present — a developer build — the library also watches it: poll()
 * recompiles any file whose timestamp moved and bumps revision(), and the owner of
 * the pipelines rebuilds them. A compile error leaves the previous module in place
 * and reports on stderr, so a typo never takes the editor down.
 */

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <vulkan/vulkan.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Resources
        {
            /** @brief Which pipeline stage a shader is compiled for. */
            enum class ShaderStage : std::uint32_t
            {
                Vertex,
                Fragment,
                Compute,
                Task,
                Mesh,
            };

            /** @brief One shader's build-time SPIR-V and the source that produced it. */
            struct ShaderSource
            {
                const char* name = nullptr;          /**< Lookup key, e.g. "pbr.frag". */
                ShaderStage stage = ShaderStage::Fragment;
                const std::uint32_t* words = nullptr; /**< Embedded SPIR-V. */
                std::size_t word_count = 0;
                const char* file = nullptr;          /**< Source file name inside the watch directory. */
            };

            /**
             * @brief Owns every shader module and reloads the ones whose source changes.
             *
             * Non-copyable: it owns VkShaderModule handles, which are destroyed and
             * recreated on reload. A caller must idle the device before calling poll()
             * if it holds pipelines built from these modules.
             */
            class ShaderLibrary
            {
                public:
                    /**
                     * @brief Registers the shader set and binds an optional watch directory.
                     *
                     * The catalogue is taken at construction rather than added afterwards so
                     * that anything built from this library — pipelines, compute jobs — can
                     * be constructed in the same initialiser list and still find its module.
                     *
                     * @param device           The live Vulkan device.
                     * @param source_directory Directory holding the GLSL sources; hot reload
                     *                         is off when empty or absent.
                     * @param sources          The shaders to register.
                     * @param count            Number of entries in @p sources.
                     */
                    ShaderLibrary(Vulkan::VulkanDevice& device, std::string source_directory,
                                  const ShaderSource* sources, std::size_t count);
                    ~ShaderLibrary();

                    ShaderLibrary(const ShaderLibrary&) = delete;
                    ShaderLibrary& operator=(const ShaderLibrary&) = delete;

                    /**
                     * @brief The module for a registered shader, created on first request.
                     * @param name The name the shader was registered under.
                     * @return The module, or VK_NULL_HANDLE if the name is unknown.
                     */
                    VkShaderModule module(const char* name);

                    /**
                     * @brief Recompiles any watched source whose file has changed.
                     *
                     * Destroys the modules it replaces, so the caller must have idled the
                     * device and must rebuild every pipeline built from them.
                     *
                     * @return true when at least one shader was replaced.
                     */
                    bool poll();

                    /** @brief Counter incremented by every successful reload. */
                    std::uint32_t revision() const noexcept { return revision_; }

                    /** @brief Whether a watch directory was found and reloading is active. */
                    bool watching() const noexcept { return watching_; }

                private:
                    /** @brief One registered shader: its source of truth and current module. */
                    struct Entry
                    {
                        std::string name;
                        std::string path;
                        ShaderStage stage = ShaderStage::Fragment;
                        const std::uint32_t* embedded = nullptr;
                        std::size_t embedded_count = 0;
                        std::vector<std::uint32_t> reloaded;
                        VkShaderModule module = VK_NULL_HANDLE;
                    };

                    void add(const ShaderSource& source);
                    Entry* find(const char* name);
                    VkShaderModule build(Entry& entry);
                    bool compile(const Entry& entry, std::vector<std::uint32_t>& words) const;

                    Vulkan::VulkanDevice& device_;
                    std::string source_directory_;
                    std::vector<Entry> entries_;
                    std::int64_t directory_stamp_ = 0;
                    std::uint32_t revision_ = 0;
                    bool watching_ = false;
                    bool glslang_initialized_ = false;
            };
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
