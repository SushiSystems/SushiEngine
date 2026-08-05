/**************************************************************************/
/* asset_library_interface.hpp                                            */
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
 * @file asset_library_interface.hpp
 * @brief The seam a host loads textures and meshes through, without seeing a device type.
 *
 * The ids it hands back are the ones a @ref SushiEngine::Render::Material holds, so the
 * authoring side describes a surface in terms of assets it never has to know how to
 * upload. Implementing it is the renderer's job; naming it is all a host needs to do.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/environment/atmosphere_nest.hpp>
#include <SushiEngine/material/material.hpp>

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief The renderer's texture and mesh asset store.
         *
         * Assets live on the device and are shared by every view drawing them, so the
         * library is owned once per device rather than per viewport. Ids stay valid
         * until released; releasing one that is still referenced by a material leaves
         * that slot reading as unset rather than sampling freed memory.
         */
        class IAssetLibrary : public IAtmosphereMirror
        {
            public:
                virtual ~IAssetLibrary() = default;

                /**
                 * @brief The GPU atmosphere's readback, when this library runs one.
                 *
                 * `IAssetLibrary` *is* the mirror source rather than merely offering one, so a
                 * host binds the renderer to the simulation in a single line and there is no
                 * intermediate object whose lifetime someone has to reason about. An
                 * implementation with no atmosphere answers with an invalid mirror, which the
                 * simulation reads as "answer from the base state" — not as an error.
                 */
                AtmosphereMirror atmosphere_mirror() const noexcept override
                {
                    return AtmosphereMirror{};
                }

                /**
                 * @brief What the atmosphere's last step cost on the GPU, per stage.
                 *
                 * Beside the mirror because it answers the other half of "is the atmosphere
                 * healthy": the mirror says what the weather *is*, this says whether the tier
                 * can afford to produce it. An implementation with no atmosphere, or a device
                 * without timestamp queries, answers with `measured == false` — which a reader
                 * must distinguish from a step that cost nothing.
                 */
                virtual AtmosphereStepCost atmosphere_step_cost() const noexcept
                {
                    return AtmosphereStepCost{};
                }

                /**
                 * @brief Loads an image file and registers it as a sampled texture.
                 *
                 * A full mip chain is generated on the GPU. Loading the same path twice
                 * returns the same id rather than uploading twice.
                 *
                 * @param path       Filesystem path to a PNG, JPEG, TGA, BMP, or HDR image.
                 * @param color_space How the file's values are to be interpreted.
                 * @return The texture id, or INVALID_TEXTURE if the file could not be read.
                 */
                virtual TextureId load_texture(const char* path,
                                               TextureColorSpace color_space) = 0;

                /**
                 * @brief Drops a reference to a texture, freeing it at zero.
                 * @param texture The id to release; INVALID_TEXTURE is ignored.
                 */
                virtual void release_texture(TextureId texture) = 0;

                /**
                 * @brief Imports a glTF 2.0 file's meshes and materials.
                 *
                 * Every primitive becomes one mesh; its material is converted to the
                 * authoring form above, including the `KHR_materials_*` extensions that
                 * map onto the advanced lobes. Missing tangents are generated. Importing the
                 * same path twice returns the meshes and materials already resident rather
                 * than re-parsing and re-uploading, the same as @ref load_texture -- so
                 * several entities naming the same file end up sharing one mesh.
                 *
                 * @param path      Filesystem path to a .gltf or .glb file.
                 * @param meshes    Receives one id per imported primitive.
                 * @param materials Receives the material for each entry in @p meshes.
                 * @param count     Capacity of @p meshes and @p materials.
                 * @return Number of primitives imported, or 0 on failure.
                 */
                virtual std::size_t load_gltf(const char* path, MeshId* meshes,
                                              Material* materials, std::size_t count) = 0;

                /**
                 * @brief Imports one glTF skin's triangle primitives as skinned meshes.
                 *
                 * Like @ref load_gltf, but for a rigged primitive: vertices stay in the skin's
                 * bind space (no node-transform bake — the animation evaluator's per-frame
                 * joint palette supplies both pose and world placement), and each mesh carries
                 * a parallel skin stream (joint indices remapped into the cooked skeleton's
                 * order, weights) that the compute skinning pass reads.
                 *
                 * @param path       Filesystem path to a .gltf or .glb file.
                 * @param skin_index Which skin to import (must match the index used to import
                 *                   the matching @c SkeletonAsset/@c ClipAsset for this file).
                 * @param meshes     Receives one id per imported primitive.
                 * @param materials  Receives the material for each entry in @p meshes.
                 * @param count      Capacity of @p meshes and @p materials.
                 * @return Number of primitives imported, or 0 on failure.
                 */
                virtual std::size_t load_gltf_skinned_mesh(const char* path,
                                                           std::size_t skin_index, MeshId* meshes,
                                                           Material* materials,
                                                           std::size_t count) = 0;

                /**
                 * @brief Bytes of texture memory currently resident on the device.
                 *
                 * The streaming budget is enforced against this; the editor surfaces it.
                 */
                virtual std::size_t resident_texture_bytes() const noexcept = 0;

                /**
                 * @brief Morph (blend-shape) targets attached to an imported mesh.
                 *
                 * @ref load_gltf_skinned_mesh uploads a primitive's `primitive.targets[]` deltas
                 * under this count; the editor's blend-shape sliders size themselves from it.
                 * @param mesh The mesh to query.
                 * @return Target count, or 0 if @p mesh has none (including an invalid id).
                 */
                virtual std::uint32_t morph_target_count(MeshId mesh) const noexcept = 0;
        };
    } // namespace Render
} // namespace SushiEngine
