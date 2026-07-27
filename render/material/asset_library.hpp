/**************************************************************************/
/* asset_library.hpp                                                      */
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
 * @file asset_library.hpp
 * @brief The device's shared asset store, and the services every view draws on.
 *
 * Textures, meshes, the bindless heap, the pipeline and sampler caches, and the
 * shader library are all device-level: two viewports drawing the same imported
 * model must share one upload and one pipeline, not each hold their own. This owns
 * them, so a scene view holds only what genuinely varies per view — its targets, its
 * per-frame allocators, and its passes.
 *
 * It is also the implementation of the public IAssetLibrary seam, which is how a
 * host loads a texture or a glTF file without seeing a Vulkan type.
 */

#include <cstddef>
#include <memory>

#include <SushiEngine/render/material.hpp>

#include "geometry/mesh_registry.hpp"
#include "material/texture_library.hpp"
#include "resources/descriptor_heap.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_layout.hpp"
#include "atmosphere/atmosphere_nest.hpp"
#include "textures/cloud_noise.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Assets
        {
            /**
             * @brief Everything shared between the views on one device.
             *
             * Construction order is the dependency order: caches, then the heap, then
             * the stores that register into it, then the noise volumes that need a
             * pipeline to generate themselves. Non-copyable.
             */
            class AssetLibrary final : public IAssetLibrary
            {
                public:
                    /**
                     * @brief Brings up every device-level service.
                     * @param device The live Vulkan device.
                     */
                    explicit AssetLibrary(Vulkan::VulkanDevice& device);
                    ~AssetLibrary() override;

                    AssetLibrary(const AssetLibrary&) = delete;
                    AssetLibrary& operator=(const AssetLibrary&) = delete;

                    TextureId load_texture(const char* path,
                                           TextureColorSpace color_space) override;
                    void release_texture(TextureId texture) override;
                    std::size_t load_gltf(const char* path, MeshId* meshes,
                                          Render::Material* materials,
                                          std::size_t count) override;
                    std::size_t load_gltf_skinned_mesh(const char* path, std::size_t skin_index,
                                                       MeshId* meshes,
                                                       Render::Material* materials,
                                                       std::size_t count) override;
                    std::size_t resident_texture_bytes() const noexcept override;
                    std::uint32_t morph_target_count(MeshId mesh) const noexcept override;

                    /** @brief The shader modules every pipeline is built from. */
                    Resources::ShaderLibrary& shaders() noexcept { return shaders_; }

                    /** @brief The factory pipelines are created through. */
                    Resources::GraphicsPipelineFactory& pipelines() noexcept { return pipelines_; }

                    /** @brief The shared sampler cache. */
                    Resources::SamplerCache& samplers() noexcept { return samplers_; }

                    /** @brief The global bindless descriptor heap. */
                    Resources::DescriptorHeap& heap() noexcept { return heap_; }

                    /** @brief The descriptor and pipeline layout every scene pass shares. */
                    Scene::SceneLayout& layout() noexcept { return layout_; }

                    /** @brief The mesh store: built-in primitives and imported meshes. */
                    Geometry::MeshRegistry& meshes() noexcept { return meshes_; }

                    /** @brief The texture store. */
                    TextureLibrary& textures() noexcept { return textures_; }

                    /** @brief The volumetric cloud noise set. */
                    Textures::CloudNoise& cloud_noise() noexcept { return noise_; }

                    /**
                     * @brief Advances the one regional atmosphere, if the scene runs one.
                     *
                     * Device-level rather than per-view because the editor renders three scene
                     * views and there is one atmosphere; see `atmosphere/atmosphere_nest.hpp`.
                     * Idempotent within a frame — the nest takes the difference against the
                     * clock it last saw — so every view may call it and only the first does
                     * work. Brings the nest up on the first call that asks for it, so a scene
                     * that never enables weather never pays its hundred-odd megabytes.
                     *
                     * @param parameters The authored physics.
                     * @param forcing    The parent solution, the observer, and the clock.
                     */
                    void step_atmosphere(const AtmosphereParameters& parameters,
                                         const AtmosphereForcing& forcing);

                    /**
                     * @brief The nest's asynchronous query mirror, for the simulation.
                     *
                     * Invalid until a step has been read back, and invalid forever in a scene
                     * that never runs the nest — which the simulation reads as "answer from the
                     * base state", not as an error.
                     */
                    AtmosphereMirror atmosphere_mirror() const noexcept override;

                    /** @brief The live nest, or null when the scene has never enabled one. */
                    Atmosphere::AtmosphereNest* atmosphere() noexcept { return atmosphere_.get(); }

                    /**
                     * @brief Advances per-frame asset work: texture streaming, shader reload.
                     * @return true when shaders were reloaded and pipelines must be rebuilt.
                     */
                    bool update();

                private:
                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary shaders_;
                    Resources::PipelineCache pipeline_cache_;
                    Resources::GraphicsPipelineFactory pipelines_;
                    Resources::SamplerCache samplers_;
                    Resources::DescriptorHeap heap_;
                    Scene::SceneLayout layout_;
                    Geometry::MeshRegistry meshes_;
                    TextureLibrary textures_;
                    Textures::CloudNoise noise_;
                    // Constructed on first use, not with the library: at High the nest's fields
                    // are ~110 MB of VRAM, and a scene that never turns weather on should not
                    // pay for an atmosphere it does not have.
                    std::unique_ptr<Atmosphere::AtmosphereNest> atmosphere_;
            };
        } // namespace Assets
    } // namespace Render
} // namespace SushiEngine
