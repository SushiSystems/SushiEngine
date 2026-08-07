/**************************************************************************/
/* imgui_backend.cpp                                                      */
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

#include "imgui_backend.hpp"

#include <cstdint>
#include <stdexcept>

#include <vulkan/vulkan.h>

#include <SDL.h>

#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_vulkan.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            /**
             * @brief A generous descriptor pool ImGui allocates its font/image sets from.
             *
             * This vendored ImGui's `ImGui_ImplVulkan_AddTexture(VkImageView, VkImageLayout)`
             * (backends/imgui_impl_vulkan.cpp) allocates one `VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE`
             * set per call against `DescriptorSetLayoutTexture` — not a single combined
             * image+sampler set, despite the three-argument overload's name; that overload just
             * forwards to the two-argument one and drops the sampler on the floor
             * (`IM_UNUSED(sampler)`). Every texture the editor registers — the font atlas, every
             * `ViewportPanel` render target, and now every resident Project panel thumbnail
             * (`ThumbnailCache::RESIDENT_CAPACITY` = 256) — allocates one such set from this
             * pool. Separately, `ImGui_ImplVulkan_CreateDeviceObjects` allocates exactly two
             * fixed `VK_DESCRIPTOR_TYPE_SAMPLER` sets once (its shared linear/nearest samplers),
             * also from this same pool. Sized well past the 256-thumbnail ceiling plus the font
             * atlas and viewport panels' existing consumption, with headroom.
             */
            VkDescriptorPool create_descriptor_pool(VkDevice device)
            {
                constexpr std::uint32_t max_sets = 384;
                const VkDescriptorPoolSize sizes[] = {
                    {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 376},
                    {VK_DESCRIPTOR_TYPE_SAMPLER, 8},
                };
                VkDescriptorPoolCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
                info.maxSets = max_sets;
                info.poolSizeCount = 2;
                info.pPoolSizes = sizes;

                VkDescriptorPool pool = VK_NULL_HANDLE;
                if (vkCreateDescriptorPool(device, &info, nullptr, &pool) != VK_SUCCESS)
                    throw std::runtime_error("SushiEngine: ImGui descriptor pool creation failed");
                return pool;
            }
        } // namespace

        ImGuiBackend::ImGuiBackend(Platform::IPlatformWindow& window,
                                   SushiEngine::Render::IWindowRenderer& renderer)
            : renderer_(renderer)
        {
            const SushiEngine::Render::NativeDeviceHandles handles = renderer.native_handles();
            VkDevice device = static_cast<VkDevice>(handles.device);
            VkDescriptorPool pool = create_descriptor_pool(device);
            descriptor_pool_ = pool;

            IMGUI_CHECKVERSION();
            ImGui::CreateContext();
            ImGuiIO& io = ImGui::GetIO();
            io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
            io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
            ImGui::StyleColorsDark();

            ImGui_ImplSDL2_InitForVulkan(static_cast<SDL_Window*>(window.native_handle()));

            // Vulkan 1.3 dynamic rendering: ImGui records into the swapchain image the
            // window renderer has already opened, so it needs the color format rather
            // than a render pass. The format storage must outlive the Init call.
            static VkFormat color_format = static_cast<VkFormat>(renderer.color_format());

            ImGui_ImplVulkan_InitInfo init{};
            init.ApiVersion = VK_API_VERSION_1_3;
            init.Instance = static_cast<VkInstance>(handles.instance);
            init.PhysicalDevice = static_cast<VkPhysicalDevice>(handles.physical_device);
            init.Device = device;
            init.QueueFamily = handles.graphics_queue_family;
            init.Queue = static_cast<VkQueue>(handles.graphics_queue);
            init.DescriptorPool = pool;
            init.MinImageCount = renderer.min_image_count();
            init.ImageCount = renderer.image_count();
            init.UseDynamicRendering = true;
            init.PipelineInfoMain.PipelineRenderingCreateInfo.sType =
                VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
            init.PipelineInfoMain.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
            init.PipelineInfoMain.PipelineRenderingCreateInfo.pColorAttachmentFormats = &color_format;

            if (!ImGui_ImplVulkan_Init(&init))
            {
                ImGui_ImplSDL2_Shutdown();
                ImGui::DestroyContext();
                vkDestroyDescriptorPool(device, pool, nullptr);
                throw std::runtime_error("SushiEngine: ImGui_ImplVulkan_Init failed");
            }

            // Route the window's raw events into ImGui, so the app loop stays SDL-free.
            window.set_event_handler([](const void* event)
            {
                ImGui_ImplSDL2_ProcessEvent(static_cast<const SDL_Event*>(event));
            });
        }

        ImGuiBackend::~ImGuiBackend()
        {
            const SushiEngine::Render::NativeDeviceHandles handles = renderer_.native_handles();
            ImGui_ImplVulkan_Shutdown();
            ImGui_ImplSDL2_Shutdown();
            ImGui::DestroyContext();
            vkDestroyDescriptorPool(static_cast<VkDevice>(handles.device),
                                    static_cast<VkDescriptorPool>(descriptor_pool_), nullptr);
        }

        void ImGuiBackend::new_frame()
        {
            ImGui_ImplVulkan_NewFrame();
            ImGui_ImplSDL2_NewFrame();
            ImGui::NewFrame();
        }

        void ImGuiBackend::render(void* command_buffer)
        {
            ImGui::Render();
            ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(),
                                            static_cast<VkCommandBuffer>(command_buffer));
        }

        ImTextureID ImGuiBackend::register_texture(void* sampler, void* image_view)
        {
            VkDescriptorSet set = ImGui_ImplVulkan_AddTexture(
                static_cast<VkSampler>(sampler), static_cast<VkImageView>(image_view),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            return static_cast<ImTextureID>(reinterpret_cast<std::uintptr_t>(set));
        }

        void ImGuiBackend::unregister_texture(ImTextureID texture)
        {
            if (texture == 0)
                return;
            ImGui_ImplVulkan_RemoveTexture(
                reinterpret_cast<VkDescriptorSet>(static_cast<std::uintptr_t>(texture)));
        }
    } // namespace Editor
} // namespace SushiEngine
