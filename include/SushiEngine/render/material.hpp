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
        class IAssetLibrary
        {
            public:
                virtual ~IAssetLibrary() = default;

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
                 * @brief Bytes of texture memory currently resident on the device.
                 *
                 * The streaming budget is enforced against this; the editor surfaces it.
                 */
                virtual std::size_t resident_texture_bytes() const noexcept = 0;
        };
    } // namespace Render
} // namespace SushiEngine
