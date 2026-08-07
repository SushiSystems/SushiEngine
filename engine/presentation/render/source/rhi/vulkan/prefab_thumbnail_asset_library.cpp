/**************************************************************************/
/* prefab_thumbnail_asset_library.cpp                                     */
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

#include "prefab_thumbnail_asset_library.hpp"

#include <stdexcept>

#include "../../material/gltf_importer.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            PrefabThumbnailAssetLibrary::PrefabThumbnailAssetLibrary(VulkanDevice& device)
                : samplers_(device)
                , heap_(device, HEAP_TEXTURE_CAPACITY, HEAP_BUFFER_CAPACITY)
                , meshes_(device)
                , textures_(device, heap_, samplers_, TEXTURE_BUDGET_BYTES)
            {
                if (!heap_.available())
                    throw std::runtime_error(
                        "SushiEngine: PrefabThumbnailAssetLibrary requires descriptor indexing "
                        "support, which this device lacks");
            }

            TextureId PrefabThumbnailAssetLibrary::load_texture(const char* path,
                                                                TextureColorSpace color_space)
            {
                return textures_.load(path, color_space);
            }

            void PrefabThumbnailAssetLibrary::release_texture(TextureId texture)
            {
                textures_.release(texture);
            }

            std::size_t PrefabThumbnailAssetLibrary::load_gltf(const char* path, MeshId* meshes,
                                                               Material* materials,
                                                               std::size_t count)
            {
                return Assets::import_gltf(path, meshes_, textures_, meshes, materials, count);
            }

            std::size_t PrefabThumbnailAssetLibrary::load_gltf_scene(const char* path,
                                                                     ImportedPrimitive* out,
                                                                     std::size_t capacity)
            {
                return Assets::import_gltf_scene_meshes(path, meshes_, textures_, out, capacity);
            }

            std::size_t PrefabThumbnailAssetLibrary::load_gltf_skinned_mesh(
                const char* path, std::size_t skin_index, MeshId* meshes, Material* materials,
                std::size_t count)
            {
                return Assets::import_gltf_skinned_mesh(path, skin_index, meshes_, textures_, meshes,
                                                materials, count);
            }

            std::size_t PrefabThumbnailAssetLibrary::resident_texture_bytes() const noexcept
            {
                return textures_.resident_bytes();
            }

            std::uint32_t PrefabThumbnailAssetLibrary::morph_target_count(MeshId mesh) const noexcept
            {
                // This renderer's flat/unlit shading never evaluates morph targets -- a prefab
                // entity carrying a morphable mesh still draws its base (unmorphed) shape,
                // which is an acceptable simplification for a 128x128 thumbnail.
                (void)mesh;
                return 0;
            }
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
