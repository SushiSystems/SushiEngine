/**************************************************************************/
/* asset_library.cpp                                                      */
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

#include "material/asset_library.hpp"

#include "material/gltf_importer.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "shader_catalogue.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Assets
        {
            namespace
            {
                /** @brief Slots reserved in the bindless texture heap. */
                constexpr std::uint32_t HEAP_TEXTURES = 4096;

                /** @brief Slots reserved in the bindless storage-buffer heap. */
                constexpr std::uint32_t HEAP_BUFFERS = 256;

#ifndef SUSHI_PIPELINE_CACHE_DIR
#define SUSHI_PIPELINE_CACHE_DIR "."
#endif

                /** @brief File the driver's compiled pipeline blob is carried across runs in. */
                constexpr const char* PIPELINE_CACHE_PATH =
                    SUSHI_PIPELINE_CACHE_DIR "/sushi_pipeline_cache.bin";

                /**
                 * @brief Device memory the resident texture set is held under.
                 *
                 * A fixed budget rather than a fraction of VRAM: it is the number the
                 * streaming residency decisions are made against, and a predictable one
                 * keeps those decisions reproducible across machines.
                 */
                constexpr std::size_t TEXTURE_BUDGET_BYTES = 512u * 1024u * 1024u;
            } // namespace

            AssetLibrary::AssetLibrary(Vulkan::VulkanDevice& device)
                : device_(device),
                  shaders_(device, SUSHI_SHADER_SOURCE_DIR, shader_catalogue(),
                           shader_catalogue_count()),
                  pipeline_cache_(device, PIPELINE_CACHE_PATH),
                  pipelines_(device, pipeline_cache_), samplers_(device),
                  heap_(device, HEAP_TEXTURES, HEAP_BUFFERS), layout_(device, heap_),
                  meshes_(device),
                  textures_(device, heap_, samplers_, TEXTURE_BUDGET_BYTES),
                  noise_(device, shaders_, pipelines_, samplers_, heap_)
            {
            }

            AssetLibrary::~AssetLibrary() = default;

            TextureId AssetLibrary::load_texture(const char* path, TextureColorSpace color_space)
            {
                return textures_.load(path, color_space);
            }

            void AssetLibrary::release_texture(TextureId texture)
            {
                textures_.release(texture);
            }

            std::size_t AssetLibrary::load_gltf(const char* path, MeshId* meshes,
                                                Render::Material* materials, std::size_t count)
            {
                return import_gltf(path, meshes_, textures_, meshes, materials, count);
            }

            std::size_t AssetLibrary::load_gltf_skinned_mesh(const char* path,
                                                             std::size_t skin_index,
                                                             MeshId* meshes,
                                                             Render::Material* materials,
                                                             std::size_t count)
            {
                return import_gltf_skinned_mesh(path, skin_index, meshes_, textures_, meshes,
                                                materials, count);
            }

            std::size_t AssetLibrary::resident_texture_bytes() const noexcept
            {
                return textures_.resident_bytes();
            }

            std::uint32_t AssetLibrary::morph_target_count(MeshId mesh) const noexcept
            {
                return meshes_.morph_target_count(mesh);
            }

            void AssetLibrary::step_atmosphere(const AtmosphereParameters& parameters,
                                               const AtmosphereForcing& forcing)
            {
                // A nest with no parent solution has nothing driving it, so "is anyone
                // publishing forcing" is the real enable: a scene with no weather provider
                // never builds one and never pays its hundred-odd megabytes, without the
                // renderer needing to know what a weather provider is.
                if (!parameters.enabled || !forcing.valid())
                {
                    // Left standing rather than torn down: switching weather off and on again is
                    // an editor action, and re-allocating a hundred megabytes on the frame it
                    // happens would be a visible hitch for no gain. The nest simply stops
                    // stepping, and its mirror stops advancing with it.
                    return;
                }
                if (!atmosphere_)
                    atmosphere_ = std::make_unique<Atmosphere::AtmosphereNest>(
                        device_, shaders_, pipelines_, samplers_, AtmosphereNestSize{});
                atmosphere_->step(parameters, forcing);
            }

            AtmosphereMirror AssetLibrary::atmosphere_mirror() const noexcept
            {
                return atmosphere_ ? atmosphere_->atmosphere_mirror() : AtmosphereMirror{};
            }

            bool AssetLibrary::update()
            {
                textures_.update();
                pipelines_.tick();
                if (!shaders_.watching() || !shaders_.poll())
                    return false;
                // Every pipeline was built from the modules that just changed, so the
                // device is idled once and the cached pipeline libraries are dropped; the
                // views rebuild their own pipelines from the new SPIR-V.
                vkDeviceWaitIdle(device_.device());
                pipelines_.clear_libraries();
                // The nest is device-level, so no view will rebuild it on their behalf.
                if (atmosphere_)
                    atmosphere_->rebuild_pipelines();
                return true;
            }
        } // namespace Assets
    } // namespace Render
} // namespace SushiEngine
