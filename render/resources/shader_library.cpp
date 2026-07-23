/**************************************************************************/
/* shader_library.cpp                                                     */
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

#include "resources/shader_library.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

#include <glslang/Public/ResourceLimits.h>
#include <glslang/Public/ShaderLang.h>
#include <glslang/SPIRV/GlslangToSpv.h>

#include "resources/glsl_includer.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            namespace
            {
                /**
                 * @brief Maps a library stage onto the glslang language enum.
                 * @param stage The registered stage.
                 * @return The glslang stage to parse with.
                 */
                EShLanguage glslang_stage(ShaderStage stage) noexcept
                {
                    switch (stage)
                    {
                        case ShaderStage::Vertex:
                            return EShLangVertex;
                        case ShaderStage::Compute:
                            return EShLangCompute;
                        case ShaderStage::Task:
                            return EShLangTask;
                        case ShaderStage::Mesh:
                            return EShLangMesh;
                        case ShaderStage::Fragment:
                        default:
                            return EShLangFragment;
                    }
                }

                /**
                 * @brief The newest last-write time across a shader directory.
                 *
                 * Watching the directory as a whole rather than each shader's own file is
                 * what makes an edited `.glsl` include reload every shader that pulls it
                 * in — the dependency is invisible to us, the timestamp is not.
                 *
                 * @param directory Directory to scan, non-recursively.
                 * @return Ticks since the filesystem clock epoch, or 0 when unreadable.
                 */
                std::int64_t newest_write_time(const std::string& directory)
                {
                    std::error_code error;
                    std::int64_t newest = 0;
                    for (const std::filesystem::directory_entry& file :
                         std::filesystem::directory_iterator(directory, error))
                    {
                        std::error_code stat_error;
                        const auto time = file.last_write_time(stat_error);
                        if (stat_error)
                            continue;
                        const std::int64_t stamp = time.time_since_epoch().count();
                        if (stamp > newest)
                            newest = stamp;
                    }
                    return error ? 0 : newest;
                }
            } // namespace

            ShaderLibrary::ShaderLibrary(Vulkan::VulkanDevice& device,
                                         std::string source_directory,
                                         const ShaderSource* sources, std::size_t count)
                : device_(device), source_directory_(std::move(source_directory))
            {
                std::error_code error;
                watching_ = !source_directory_.empty() &&
                            std::filesystem::is_directory(source_directory_, error) && !error;
                if (watching_)
                {
                    glslang_initialized_ = glslang::InitializeProcess();
                    directory_stamp_ = newest_write_time(source_directory_);
                }

                entries_.reserve(count);
                for (std::size_t i = 0; i < count; ++i)
                    add(sources[i]);
            }

            ShaderLibrary::~ShaderLibrary()
            {
                for (Entry& entry : entries_)
                    if (entry.module != VK_NULL_HANDLE)
                        vkDestroyShaderModule(device_.device(), entry.module, nullptr);
                entries_.clear();
                if (glslang_initialized_)
                    glslang::FinalizeProcess();
            }

            void ShaderLibrary::add(const ShaderSource& source)
            {
                if (source.name == nullptr)
                    return;
                Entry entry;
                entry.name = source.name;
                entry.stage = source.stage;
                entry.embedded = source.words;
                entry.embedded_count = source.word_count;
                if (watching_ && source.file != nullptr)
                    entry.path = (std::filesystem::path(source_directory_) / source.file).string();
                entries_.push_back(std::move(entry));
            }

            ShaderLibrary::Entry* ShaderLibrary::find(const char* name)
            {
                if (name == nullptr)
                    return nullptr;
                for (Entry& entry : entries_)
                    if (entry.name == name)
                        return &entry;
                return nullptr;
            }

            VkShaderModule ShaderLibrary::build(Entry& entry)
            {
                const std::uint32_t* words =
                    entry.reloaded.empty() ? entry.embedded : entry.reloaded.data();
                const std::size_t count =
                    entry.reloaded.empty() ? entry.embedded_count : entry.reloaded.size();
                if (words == nullptr || count == 0)
                    return VK_NULL_HANDLE;

                VkShaderModuleCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
                info.codeSize = count * sizeof(std::uint32_t);
                info.pCode = words;
                VkShaderModule module = VK_NULL_HANDLE;
                Vulkan::check(vkCreateShaderModule(device_.device(), &info, nullptr, &module),
                              "vkCreateShaderModule");
                return module;
            }

            VkShaderModule ShaderLibrary::module(const char* name)
            {
                Entry* entry = find(name);
                if (entry == nullptr)
                    return VK_NULL_HANDLE;
                if (entry->module == VK_NULL_HANDLE)
                    entry->module = build(*entry);
                return entry->module;
            }

            bool ShaderLibrary::compile(const Entry& entry,
                                        std::vector<std::uint32_t>& words) const
            {
                std::ifstream stream(entry.path, std::ios::binary);
                if (!stream)
                    return false;
                std::ostringstream buffer;
                buffer << stream.rdbuf();
                const std::string text = buffer.str();

                const EShLanguage stage = glslang_stage(entry.stage);
                glslang::TShader shader(stage);
                const char* source = text.c_str();
                shader.setStrings(&source, 1);
                shader.setEnvInput(glslang::EShSourceGlsl, stage, glslang::EShClientVulkan, 100);
                shader.setEnvClient(glslang::EShClientVulkan, glslang::EShTargetVulkan_1_3);
                shader.setEnvTarget(glslang::EShTargetSpv, glslang::EShTargetSpv_1_6);

                const EShMessages messages =
                    static_cast<EShMessages>(EShMsgSpvRules | EShMsgVulkanRules);
                GlslIncluder includer(source_directory_);
                if (!shader.parse(GetDefaultResources(), 100, false, messages, includer))
                {
                    std::fprintf(stderr, "SushiEngine: shader reload failed for %s:\n%s\n",
                                 entry.name.c_str(), shader.getInfoLog());
                    return false;
                }

                glslang::TProgram program;
                program.addShader(&shader);
                if (!program.link(messages))
                {
                    std::fprintf(stderr, "SushiEngine: shader link failed for %s:\n%s\n",
                                 entry.name.c_str(), program.getInfoLog());
                    return false;
                }

                words.clear();
                glslang::GlslangToSpv(*program.getIntermediate(stage), words);
                return !words.empty();
            }

            bool ShaderLibrary::poll()
            {
                if (!watching_ || !glslang_initialized_)
                    return false;

                const std::int64_t stamp = newest_write_time(source_directory_);
                if (stamp == 0 || stamp == directory_stamp_)
                    return false;
                directory_stamp_ = stamp;

                bool reloaded_any = false;
                for (Entry& entry : entries_)
                {
                    if (entry.path.empty())
                        continue;

                    std::vector<std::uint32_t> words;
                    if (!compile(entry, words))
                        continue;

                    entry.reloaded = std::move(words);
                    if (entry.module != VK_NULL_HANDLE)
                        vkDestroyShaderModule(device_.device(), entry.module, nullptr);
                    entry.module = build(entry);
                    reloaded_any = true;
                }

                if (reloaded_any)
                    ++revision_;
                return reloaded_any;
            }
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
