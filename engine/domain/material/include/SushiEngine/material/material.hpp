/**************************************************************************/
/* material.hpp                                                           */
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
 * @file material.hpp
 * @brief The authored surface description: maps, scalars, and rendering state.
 *
 * A metallic-roughness material in the glTF sense, extended with the detail set and
 * the advanced lobes. Every map is optional — an unset id means "use the scalar or
 * tint next to it" — so a material with no textures at all is still valid and shades
 * exactly as it did before textures existed. Maps sample through a shared main
 * tiling/offset, with the detail set carrying its own.
 *
 * This is the authoring form, held by value on a MeshInstance and edited in the
 * inspector. The renderer packs it into its own GPU layout each frame; nothing here
 * is GPU-layout-sensitive, so fields may be added without touching a shader's
 * struct packing.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/atmosphere_nest.hpp>

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief A texture registered with the renderer's texture library.
         *
         * Opaque to the authoring side: the renderer maps it to a bindless heap slot.
         */
        using TextureId = std::uint32_t;

        /** @brief The id an unset texture slot carries. */
        constexpr TextureId INVALID_TEXTURE = 0xFFFFFFFFu;

        /**
         * @brief A mesh registered with the renderer's mesh library.
         *
         * Set on a MeshInstance to draw an imported mesh instead of one of the
         * built-in primitives.
         */
        using MeshId = std::uint32_t;

        /** @brief The id an instance carries when it draws a built-in primitive. */
        constexpr MeshId INVALID_MESH = 0xFFFFFFFFu;

        /** @brief How a material's alpha is interpreted. */
        enum class SurfaceType : std::uint32_t
        {
            Opaque,      /**< Alpha ignored. */
            Cutout,      /**< Alpha thresholded against @ref Material::alpha_cutoff. */
            Transparent, /**< Alpha-blended, depth-tested, no depth write. */
            Fade,        /**< Alpha fades the whole surface including its specular. */
        };

        /**
         * @brief Whether a surface type draws through the alpha-blended transparent pass.
         *
         * Opaque and Cutout draw fully covered (Cutout discards below its alpha
         * threshold instead of blending) and belong to the depth-writing opaque pass.
         * Transparent and Fade both blend by alpha with no depth write, so both route
         * to the same pass and pipeline; @ref MATERIAL_FADE_SPECULAR in the material
         * flags tells the shader which of the two it is shading.
         *
         * @param type The material's surface type.
         * @return True for Transparent and Fade, false for Opaque and Cutout.
         */
        inline bool is_alpha_blended(SurfaceType type) noexcept
        {
            return type == SurfaceType::Transparent || type == SurfaceType::Fade;
        }

        /** @brief Which faces a material rasterises. */
        enum class MaterialCullMode : std::uint32_t
        {
            Off,   /**< Double-sided. */
            Front, /**< Cull front faces. */
            Back,  /**< Cull back faces. */
        };

        /** @brief How a transparent material combines with what is behind it. */
        enum class BlendMode : std::uint32_t
        {
            Alpha,         /**< src.a * src + (1 - src.a) * dst. */
            Premultiplied, /**< src + (1 - src.a) * dst. */
            Additive,      /**< src + dst. */
            Multiply,      /**< src * dst. */
        };

        /** @brief How a sampler behaves outside [0, 1]. */
        enum class TextureWrap : std::uint32_t
        {
            Repeat,
            Clamp,
            Mirror,
        };

        /**
         * @brief A map's tiling and offset — Unity's ST vector.
         *
         * Applied as `uv * tiling + offset` before every sample in its set.
         */
        struct TextureTransform
        {
            float tiling_x = 1.0f;
            float tiling_y = 1.0f;
            float offset_x = 0.0f;
            float offset_y = 0.0f;
        };

        /**
         * @brief A physically-based surface: its maps, its scalars, and its state.
         *
         * The scalar fields double as tints when the matching map is set, matching
         * glTF's factor semantics — `albedo` multiplies the base-colour map, and
         * `metallic`/`roughness` multiply the packed map's blue and green channels.
         */
        struct Material
        {
            Vector3 albedo{Vector3{0.8, 0.8, 0.8}}; /**< Base colour tint / F0 tint when metallic. */
            float base_alpha = 1.0f;                /**< Base colour alpha; drives cutout and blending. */
            TextureId albedo_map = INVALID_TEXTURE; /**< sRGB base colour + alpha. */

            float metallic = 0.0f;  /**< 0 = dielectric, 1 = metal. */
            float roughness = 0.6f; /**< Microfacet roughness in (0, 1]. */
            /** @brief Packed metallic-roughness: G = roughness, B = metallic (glTF order). */
            TextureId metallic_roughness_map = INVALID_TEXTURE;
            /** @brief The MR map is an ORM map whose red channel is ambient occlusion. */
            bool packed_occlusion = false;

            TextureId normal_map = INVALID_TEXTURE; /**< Tangent-space normal; Z reconstructed. */
            float normal_scale = 1.0f;              /**< Bump strength. */

            TextureId height_map = INVALID_TEXTURE; /**< Height for parallax occlusion mapping. */
            float height_scale = 0.02f;             /**< Displacement depth in UV-space units. */
            std::uint32_t parallax_steps = 16;      /**< March steps; 0 disables POM. */
            bool parallax_shadows = false;          /**< Self-shadow the surface along the light ray. */
            bool parallax_silhouette_clip = false;  /**< Discard where the ray leaves the height field. */

            TextureId occlusion_map = INVALID_TEXTURE; /**< Ambient occlusion, red channel. */
            float occlusion_strength = 1.0f;           /**< Blend toward unoccluded at 0. */

            Vector3 emissive{Vector3{0.0, 0.0, 0.0}}; /**< Emissive colour, HDR. */
            float emissive_intensity = 1.0f;          /**< Multiplies @ref emissive and its map. */
            TextureId emissive_map = INVALID_TEXTURE; /**< sRGB emissive. */
            bool emissive_enabled = false;            /**< Off means the emissive term is skipped. */

            TextureId detail_albedo_map = INVALID_TEXTURE; /**< Overlaid close-up albedo detail. */
            TextureId detail_normal_map = INVALID_TEXTURE; /**< Overlaid close-up normal detail. */
            TextureId detail_mask_map = INVALID_TEXTURE;   /**< Where the detail set applies. */
            float detail_normal_scale = 1.0f;              /**< Detail bump strength. */

            TextureTransform main_transform{};   /**< Tiling/offset for the base map set. */
            TextureTransform detail_transform{}; /**< Tiling/offset for the detail set. */

            float anisotropy = 0.0f;          /**< Stretches the specular lobe; 0 = isotropic. */
            float anisotropy_rotation = 0.0f; /**< Tangent-space rotation of the stretch, turns. */
            float clearcoat = 0.0f;           /**< Strength of a second, smooth specular layer. */
            float clearcoat_roughness = 0.03f;
            Vector3 sheen_color{Vector3{0.0, 0.0, 0.0}}; /**< Retroreflective cloth rim. */
            float sheen_roughness = 0.3f;
            float transmission = 0.0f;                   /**< Fraction of light passing through. */
            Vector3 subsurface_color{Vector3{1.0, 1.0, 1.0}}; /**< Tint of transmitted light. */
            float thickness = 0.0f;                      /**< Distance light travels inside, metres. */
            float ior = 1.5f;                            /**< Index of refraction; drives dielectric F0. */

            SurfaceType surface_type = SurfaceType::Opaque;
            float alpha_cutoff = 0.5f;
            MaterialCullMode cull_mode = MaterialCullMode::Off;
            BlendMode blend_mode = BlendMode::Alpha;
            std::int32_t render_queue = 2000; /**< Sort key; lower draws first. */
            bool cast_shadows = true;
            bool receive_shadows = true;
            bool gpu_instancing = true;

            float anisotropic_filtering = 8.0f;          /**< Sampler anisotropy, 1 = off. */
            TextureWrap wrap_mode = TextureWrap::Repeat; /**< Sampler address mode for every map. */

            /**
             * @brief Opt into weather-driven wetness (design doc §5.3, W5): albedo darken / roughness drop.
             *
             * A material-flag path, no shader fork: `pbr.frag` blends toward a thin water film
             * only where this is set, scaled by the frame's global `Environment::weather.ground_wetness`
             * (the same `Simulation::WeatherColumn` sample driving precipitation VFX and fog). Off
             * by default, like `parallax_shadows`/`emissive_enabled` — an author opts a ground/
             * exterior surface in rather than every material reacting uniformly.
             */
            bool weather_wettable = false;
        };

        /** @brief How a texture's values are interpreted when it is uploaded. */
        enum class TextureColorSpace : std::uint32_t
        {
            Linear, /**< Data maps: normal, metallic-roughness, occlusion, height, masks. */
            Srgb,   /**< Colour maps: base colour, emissive, detail albedo. */
        };

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
                 * map onto the advanced lobes. Missing tangents are generated.
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
