/**************************************************************************/
/* light.hpp                                                              */
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
 * @file light.hpp
 * @brief The punctual scene lights the clustered light engine shades against.
 *
 * A punctual light is a *record*, not an object: plain trivially-copyable data that
 * crosses the `render()` boundary once per frame, exactly the way @ref MeshInstance
 * and @ref ClothStrandView do. The renderer never holds a polymorphic light — it packs
 * the list into one storage buffer, culls it against the froxel cluster grid, and the
 * shading pass loops the few lights that reach each pixel. This is the data-oriented
 * mandate: culling, the cluster grid, and shading all depend on the *list*, never on a
 * light knowing how to draw itself.
 *
 * The one directional light (the sun) stays in @ref Environment; it needs no culling
 * and lights every pixel, so it is not a punctual light and is not in this list.
 */

#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/material.hpp> // TextureId, INVALID_TEXTURE

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief Which falloff and cone geometry a punctual light uses.
         *
         * Kept an ordinal enum so the CPU side stays a flat SoA and the GPU side reads
         * it as an integer, with no virtual dispatch anywhere. Area (rect/tube) and IES
         * profiles are deferred to a later increment and would extend this enum, never
         * subclass a light.
         */
        enum class LightType : std::uint32_t
        {
            Point = 0, /**< Omnidirectional, inverse-square within @ref PunctualLight::range. */
            Spot = 1,  /**< A cone about @ref PunctualLight::direction with a soft edge. */
        };

        /**
         * @brief One point or spot light placed in the scene this frame.
         *
         * @c position is absolute world metres — the renderer makes it camera-relative
         * at pack time (eye subtracted in double before the float cast), the same way a
         * mesh's model translation is, so a light stays put at planetary range.
         *
         * @c intensity is a raw radiance scale on the same footing as
         * @ref DirectionalLight::intensity, not a photometric unit: the pipeline's
         * exposure is still the fixed pre-tonemap multiplier, so a punctual light and the
         * sun share one scale. When Phase 9 lands auto-exposure and physical camera units
         * this becomes candela (spot) / lumen (point) with no shading-path change — the
         * falloff is already inverse-square.
         */
        struct PunctualLight
        {
            WorldVector3 position{};                 /**< Absolute world position, metres. */
            Vector3 direction{Vector3{0.0, -1.0, 0.0}}; /**< Unit spot axis, toward the lit region. */
            Vector3 color{Vector3{1.0, 1.0, 1.0}};   /**< Linear light colour. */
            float intensity = 20.0f;                 /**< Radiance scale (HDR; tonemapped later). */
            float range = 10.0f;                     /**< Metres at which the windowed falloff reaches zero. */
            float inner_cone = 0.0f;                 /**< Spot inner half-angle, radians (full brightness inside). */
            float outer_cone = 0.7853982f;           /**< Spot outer half-angle, radians (dark outside). */
            LightType type = LightType::Point;       /**< Point or spot. */
            /**
             * @brief Whether this light writes a shadow map.
             *
             * A shadow-casting spot claims one tile in the shared punctual shadow atlas; a
             * shadow-casting point light claims six (one perspective face per cube
             * direction), budget permitting. The shading pass selects a point light's face
             * from the fragment's direction. A caster that does not fit the tile budget is
             * shaded unshadowed.
             */
            bool casts_shadows = false;
            std::uint32_t id = 0;                    /**< Authoring entity id, for the editor/picking. */
        };

        /**
         * @brief A projected box decal that tints the surfaces it overlaps.
         *
         * Also a *record*, culled into the same froxel cluster grid the lights are (a
         * decal that touches no cluster costs no pixel). The box is an oriented volume:
         * @c right / @c up / @c forward are its unit axes and @c half_extents its half-size
         * along each, so the shading pass projects a fragment into decal space and, when it
         * lands inside the box, blends @c color over the surface albedo by @c opacity. The
         * placement comes from the authoring entity's transform, exactly as a light's does.
         *
         * The tint is the base form (scorch marks, paint, blend patches); optional bindless
         * @c albedo_map / @c orm_map sources project real imagery and surface response along
         * the same box — the shading path is already a projection, so a texture only swaps
         * where the colour (and roughness/metallic) come from.
         */
        struct Decal
        {
            WorldVector3 position{};                   /**< Box centre, absolute world metres. */
            Vector3 right{Vector3{1.0, 0.0, 0.0}};     /**< Unit axis across the surface (u). */
            Vector3 up{Vector3{0.0, 1.0, 0.0}};        /**< Unit axis across the surface (v). */
            Vector3 forward{Vector3{0.0, 0.0, 1.0}};   /**< Unit projection axis (box depth). */
            Vector3 half_extents{Vector3{1.0, 1.0, 1.0}}; /**< Half-size along right/up/forward, metres. */
            Vector3 color{Vector3{0.5, 0.1, 0.1}};     /**< Linear tint blended over the surface albedo. */
            float opacity = 1.0f;                      /**< Blend weight of the tint, [0, 1]. */
            /**
             * @brief Optional bindless albedo map, projected in the decal's local uv.
             *
             * @c INVALID_TEXTURE (the default) means the flat @c color tint. Otherwise the
             * decal's local right/up coordinates index this texture and its rgb replaces the
             * tint, its alpha scaling the blend — imagery decals (posters, blood, scorch) on
             * the same projection path. Resolved to a bindless heap index at pack time,
             * exactly as a material map is.
             */
            TextureId albedo_map = INVALID_TEXTURE;
            /**
             * @brief Optional bindless occlusion-roughness-metallic map (glTF ORM packing).
             *
             * @c INVALID_TEXTURE leaves the surface's own response. Otherwise the decal
             * overrides occlusion (r), roughness (g), and metallic (b) where it lands, so a
             * decal can read as wet, rusted, or polished, not merely recoloured.
             */
            TextureId orm_map = INVALID_TEXTURE;
            std::uint32_t id = 0;                      /**< Authoring entity id. */
        };
    } // namespace Render
} // namespace SushiEngine
