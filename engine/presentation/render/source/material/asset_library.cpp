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

#include <algorithm>

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

#ifndef SUSHIENGINE_PIPELINE_CACHE_DIR
#define SUSHIENGINE_PIPELINE_CACHE_DIR "."
#endif

                /** @brief Compiled-in fallback used when the caller passes no path. */
                const std::string DEFAULT_PIPELINE_CACHE_PATH =
                    std::string(SUSHIENGINE_PIPELINE_CACHE_DIR) + "/sushi_pipeline_cache.bin";

                /**
                 * @brief Device memory the resident texture set is held under.
                 *
                 * A fixed budget rather than a fraction of VRAM: it is the number the
                 * streaming residency decisions are made against, and a predictable one
                 * keeps those decisions reproducible across machines.
                 */
                constexpr std::size_t TEXTURE_BUDGET_BYTES = 512u * 1024u * 1024u;
            } // namespace

            AssetLibrary::AssetLibrary(Vulkan::VulkanDevice& device,
                                       std::string shader_source_directory,
                                       std::string pipeline_cache_path)
                : device_(device),
                  shaders_(device,
                           shader_source_directory.empty() ? SUSHIENGINE_SHADER_SOURCE_DIR
                                                            : std::move(shader_source_directory),
                           shader_catalogue(), shader_catalogue_count()),
                  pipeline_cache_(device, pipeline_cache_path.empty()
                                              ? DEFAULT_PIPELINE_CACHE_PATH
                                              : std::move(pipeline_cache_path)),
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
                if (path == nullptr || meshes == nullptr || materials == nullptr || count == 0)
                    return 0;

                // A cache hit has to be able to fill every slot the caller asked for, or it
                // falls through and imports for real -- every real call site today always
                // asks for exactly one mesh (see gltf_cache_'s own comment), so this only
                // matters if a future caller ever asks for more than an earlier call did.
                const std::string key(path);
                const auto cached = gltf_cache_.find(key);
                if (cached != gltf_cache_.end() && cached->second.meshes.size() >= count)
                {
                    std::copy_n(cached->second.meshes.begin(), count, meshes);
                    std::copy_n(cached->second.materials.begin(), count, materials);
                    return count;
                }

                const std::size_t imported =
                    import_gltf(path, meshes_, textures_, meshes, materials, count);
                if (imported > 0)
                    gltf_cache_[key] = GltfImportCacheEntry{
                        std::vector<MeshId>(meshes, meshes + imported),
                        std::vector<Render::Material>(materials, materials + imported)};
                return imported;
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

            namespace
            {
                /** @brief Whether two discretizations describe the same grid. */
                bool same_nest_size(const AtmosphereNestSize& a, const AtmosphereNestSize& b)
                {
                    return a.cells_x == b.cells_x && a.cells_z == b.cells_z &&
                           a.levels == b.levels && a.spacing_m == b.spacing_m &&
                           a.top_m == b.top_m;
                }
            } // namespace

            void AssetLibrary::stage_atmosphere(const AtmosphereNestParameters& parameters,
                                                const AtmosphereForcing& forcing,
                                                const AtmosphereNestSize& size)
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
                // Latched, not built and not stepped. `forcing` borrows the parent solution's
                // samples from the `Environment` the caller is holding, which outlives this
                // frame's rendering — the same lifetime the previous, immediate call already
                // relied on. Construction moves to the flush with the step, because a tier
                // change destroys an image this frame's views may already have submitted reads
                // of, and the flush is the point where that is known to be over.
                staged_atmosphere_.parameters = parameters;
                staged_atmosphere_.forcing = forcing;
                staged_atmosphere_.size = size;
                staged_atmosphere_.pending = true;
            }

            void AssetLibrary::note_atmosphere_reader(VkSemaphore timeline, std::uint64_t value)
            {
                if (timeline == VK_NULL_HANDLE || !atmosphere_)
                    return;
                // One entry per view per frame, so the list is three long at its worst; a view
                // that submits several times registers only its last value, which is the one
                // that covers the rest.
                for (VkSemaphoreSubmitInfo& reader : atmosphere_readers_)
                    if (reader.semaphore == timeline)
                    {
                        reader.value = std::max(reader.value, value);
                        return;
                    }
                VkSemaphoreSubmitInfo reader{};
                reader.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
                reader.semaphore = timeline;
                reader.value = value;
                // The step must not begin writing before the reader has finished reading, and a
                // reader's sampling can happen in any stage of its own submission.
                reader.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
                atmosphere_readers_.push_back(reader);
            }

            void AssetLibrary::flush_atmosphere()
            {
                if (!staged_atmosphere_.pending)
                {
                    atmosphere_readers_.clear();
                    return;
                }
                staged_atmosphere_.pending = false;

                // A tier change re-discretizes the atmosphere, and the grid is fixed for a
                // nest's lifetime — so the nest is rebuilt, and a rebuilt nest starts from its
                // base state. The running weather is lost, which is why this is a tier and not a
                // slider, and why the editor says so beside the setting.
                //
                // Idling the device first: the views of the frames still in flight hold
                // descriptors pointing at the extinction volume about to be freed, and this is
                // the one place with no cheaper way to know they are done with it. A tier change
                // is a rare, deliberate act and a hitch is the right price for it — the same
                // judgement the enable/disable path already makes in the other direction by
                // *not* tearing the nest down.
                if (atmosphere_ && !same_nest_size(atmosphere_->size(), staged_atmosphere_.size))
                {
                    vkDeviceWaitIdle(device_.device());
                    atmosphere_.reset();
                    atmosphere_readers_.clear();
                }
                if (!atmosphere_)
                    atmosphere_ = std::make_unique<Atmosphere::AtmosphereNest>(
                        device_, shaders_, pipelines_, samplers_, staged_atmosphere_.size);
                atmosphere_->step(staged_atmosphere_.parameters, staged_atmosphere_.forcing,
                                  atmosphere_readers_.data(),
                                  std::uint32_t(atmosphere_readers_.size()));
                atmosphere_readers_.clear();
            }

            AtmosphereMirror AssetLibrary::atmosphere_mirror() const noexcept
            {
                return atmosphere_ ? atmosphere_->atmosphere_mirror() : AtmosphereMirror{};
            }

            AtmosphereStepCost AssetLibrary::atmosphere_step_cost() const noexcept
            {
                return atmosphere_ ? atmosphere_->step_cost() : AtmosphereStepCost{};
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
