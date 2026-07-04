/**************************************************************************/
/* environment.hpp                                                        */
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
 * @file environment.hpp
 * @brief The scene-level lighting and sky environment the renderer shades against.
 *
 * A neutral seam shared by the simulation (which authors these values) and the
 * renderer (which consumes them): the sun as a directional light, the WGS84 planet
 * it lights, the atmosphere/cloud/star layers drawn around that planet, and the
 * per-surface PBR material. None of these types depend on Vulkan or on the sim; they
 * are plain trivially-copyable data that crosses the `render()` boundary once per
 * frame, the same way `MeshInstance` and `CameraView` do.
 *
 * The renderer draws the planet analytically (a ray-marched ellipsoid, no tessellated
 * terrain), so `Wgs84` carries only the ellipsoid's defining constants; the surface
 * grid the editor shows sits tangent to that ellipsoid at the local origin.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief A distant, parallel-ray light — the sun for the whole scene.
         *
         * @c direction points from the surface toward the light (the direction to the
         * sun), so a surface normal dotted with it gives the incident cosine directly.
         */
        struct DirectionalLight
        {
            Vector3 direction{Vector3{0.0, 0.6, 0.8}}; /**< Unit direction toward the sun. */
            Vector3 color{Vector3{1.0, 0.98, 0.95}};   /**< Linear light colour. */
            float intensity = 20.0f;                   /**< Radiance scale (HDR; tonemapped later). */
        };

        /**
         * @brief The WGS84 reference ellipsoid's defining constants.
         *
         * The renderer needs only the equatorial radius and flattening to intersect a
         * view ray with the planet and to size the atmosphere shell; @c semi_minor and
         * @c mean_radius derive the rest. Kept at double precision because the values
         * are planet-scale metres, well past single-precision's clean range.
         */
        struct Wgs84
        {
            double semi_major = 6378137.0;             /**< Equatorial radius a, metres. */
            double inverse_flattening = 298.257223563; /**< 1/f. */

            /** @brief Polar radius b = a(1 - f), metres. */
            double semi_minor() const noexcept
            {
                return semi_major * (1.0 - 1.0 / inverse_flattening);
            }

            /** @brief IUGG mean radius (2a + b)/3, metres — the atmosphere sphere's radius. */
            double mean_radius() const noexcept
            {
                return (2.0 * semi_major + semi_minor()) / 3.0;
            }
        };

        /**
         * @brief Single-scattering atmosphere parameters (Rayleigh + Mie).
         *
         * The renderer integrates in-scattering along the view ray through a shell of
         * thickness @c height above the planet; @c rayleigh_coefficient and
         * @c mie_coefficient are the per-metre scattering coefficients at sea level,
         * @c mie_anisotropy is the Henyey-Greenstein g for the forward Mie lobe.
         */
        struct AtmosphereParams
        {
            bool enabled = true;                       /**< Draw the atmosphere at all. */
            float height = 100000.0f;                  /**< Shell thickness above the surface, metres. */
            Vector3 rayleigh_coefficient{Vector3{5.8e-6, 13.5e-6, 33.1e-6}}; /**< Per-metre Rayleigh, RGB. */
            float mie_coefficient = 21e-6f;            /**< Per-metre Mie scattering. */
            float mie_anisotropy = 0.76f;              /**< Henyey-Greenstein g in [0, 1). */
            float rayleigh_scale_height = 8000.0f;     /**< Rayleigh density e-folding height, metres. */
            float mie_scale_height = 1200.0f;          /**< Mie density e-folding height, metres. */
        };

        /**
         * @brief The planet surface's albedo, shaded analytically where the ray hits it.
         *
         * A deliberately simple two-tone surface (ocean vs land) plus a roughness so the
         * ground reads as a lit sphere from orbit without a texture or terrain pipeline.
         */
        struct PlanetParams
        {
            Vector3 ground_albedo{Vector3{0.16, 0.20, 0.11}}; /**< Land base colour. */
            Vector3 ocean_color{Vector3{0.02, 0.06, 0.16}};   /**< Ocean base colour. */
            float roughness = 0.9f;                           /**< Surface roughness for its sun highlight. */
        };

        /**
         * @brief A single ray-marched cloud layer between two altitudes.
         *
         * Density comes from procedural fbm noise in the shader; @c coverage biases how
         * much of the sky the clouds fill and @c density scales their opacity.
         */
        struct CloudParams
        {
            bool enabled = true;              /**< Draw clouds at all. */
            float base_altitude = 1500.0f;    /**< Cloud layer bottom, metres above surface. */
            float top_altitude = 4000.0f;     /**< Cloud layer top, metres above surface. */
            float coverage = 0.5f;            /**< Fraction of sky covered, [0, 1]. */
            float density = 0.6f;             /**< Opacity scale, [0, 1]. */
        };

        /**
         * @brief The procedural star field drawn behind a thin/absent atmosphere.
         *
         * Stars are hashed points on the view sphere, brightest where the atmosphere's
         * optical depth toward space is low — so they emerge as the camera climbs out of
         * the air, which is the near-surface-to-space transition the whole scene targets.
         */
        struct StarParams
        {
            bool enabled = true;      /**< Draw stars at all. */
            float brightness = 1.0f;  /**< Overall star radiance scale. */
            float density = 0.5f;     /**< Fraction of cells that hold a visible star, [0, 1]. */
        };

        /**
         * @brief A physically-based metallic-roughness surface material.
         *
         * The Cook-Torrance inputs the mesh shader shades with. @c albedo replaces the
         * old flat per-instance colour; @c metallic and @c roughness drive the BRDF and
         * @c emissive adds unlit radiance (HDR, tonemapped later).
         */
        struct Material
        {
            Vector3 albedo{Vector3{0.8, 0.8, 0.8}}; /**< Base colour (diffuse albedo / F0 tint when metallic). */
            float metallic = 0.0f;                  /**< 0 = dielectric, 1 = metal. */
            float roughness = 0.6f;                 /**< Microfacet roughness in (0, 1]. */
            Vector3 emissive{Vector3{0.0, 0.0, 0.0}}; /**< Self-emitted radiance. */
        };

        /**
         * @brief A reference ellipsoid generalised beyond WGS84 to any body's surface.
         *
         * The same two constants @ref Wgs84 carries — equatorial radius and flattening —
         * but named for use by any planet or moon whose surface the near-field regime
         * ray-marches or meshes, not just Earth. WGS84 is one row of this; Mars, the Moon,
         * and the rest supply their own so the ground pipeline is not Earth-specific.
         */
        struct ReferenceEllipsoid
        {
            double semi_major = 6378137.0;             /**< Equatorial radius a, metres. */
            double inverse_flattening = 298.257223563; /**< 1/f (0 or huge for a sphere). */

            /** @brief Polar radius b = a(1 - f), metres; a for a sphere (inverse_flattening 0). */
            double semi_minor() const noexcept
            {
                return inverse_flattening != 0.0 ? semi_major * (1.0 - 1.0 / inverse_flattening)
                                                 : semi_major;
            }

            /** @brief IUGG mean radius (2a + b)/3, metres. */
            double mean_radius() const noexcept
            {
                return (2.0 * semi_major + semi_minor()) / 3.0;
            }
        };

        /** @brief Maximum solar-system bodies the environment carries to the renderer. */
        constexpr int MAX_CELESTIAL_BODIES = 16;

        /** @brief Maximum catalogued stars the environment carries to the renderer. */
        constexpr int MAX_SKY_STARS = 64;

        /**
         * @brief Which representation regime a body is drawn in this frame.
         *
         * The LOD ladder the camera climbs as it approaches a body: a sub-pixel @c Point,
         * an analytically shaded @c Disk, a textured sphere @c Impostor, a real @c Mesh,
         * and finally the on-@c Surface atmosphere/ground regime. The far-field sky pass
         * handles Point/Disk/Impostor today; Mesh/Surface are the near-field hand-off.
         */
        enum class BodyLod : std::uint32_t
        {
            Point = 0,
            Disk,
            Impostor,
            Mesh,
            Surface,
        };

        /**
         * @brief One solar-system body placed in the observer's sky this frame.
         *
         * The far-field seam: an ephemeris authors the local-frame @c direction to the
         * body, its @c angular_radius, the @c sun_direction that lights it (for the phase
         * terminator), and its @c color / @c brightness, so the sky pass draws a
         * phase-lit disk without a mesh. @c heliocentric_position and @c mean_radius carry
         * the true double-precision scale the near-field mesh/surface regimes and
         * floating-origin rebasing need when the camera leaves Earth. Trivially copyable.
         */
        struct CelestialBody
        {
            Vector3 direction{Vector3{0.0, 1.0, 0.0}};       /**< Unit direction to the body, local ENU frame. */
            Vector3 sun_direction{Vector3{0.0, 1.0, 0.0}};   /**< Unit direction from the body toward the sun, local frame (phase). */
            Vector3 color{Vector3{1.0, 1.0, 1.0}};           /**< Linear RGB surface tint. */
            WorldVector3 heliocentric_position{};            /**< Absolute heliocentric position, metres (fly-out / rebasing). */
            float angular_radius = 0.0f;                     /**< Apparent angular radius, radians. */
            float brightness = 1.0f;                         /**< Relative reflected/emitted radiance scale. */
            float distance_metres = 0.0f;                    /**< Geocentric distance to the body, metres (LOD metric). */
            float mean_radius_metres = 0.0f;                 /**< Physical mean radius, metres. */
            BodyLod lod = BodyLod::Disk;                     /**< Representation regime this frame. */
            std::uint32_t is_star = 0;                       /**< 1 if the body emits (the Sun), else 0. */
        };

        /**
         * @brief One catalogued star placed in the observer's sky this frame.
         *
         * The fixed stars, rotated into the local ENU frame by the same topocentric
         * transform as the bodies, so the constellations rise and set with sidereal time.
         */
        struct SkyStar
        {
            Vector3 direction{Vector3{0.0, 1.0, 0.0}}; /**< Unit direction to the star, local ENU frame. */
            Vector3 color{Vector3{1.0, 1.0, 1.0}};     /**< Linear RGB tint from the star's B-V. */
            float brightness = 1.0f;                   /**< Relative brightness from apparent magnitude. */
            float pad = 0.0f;                          /**< Padding to a clean 32-byte stride. */
        };

        /**
         * @brief Where and when the observer stands, driving the whole sky's orientation.
         *
         * The inputs an ephemeris propagates from: the epoch as a Julian Date and the
         * observer's geodetic position. The scene anchors the ground at the equator, so
         * @c latitude_radians defaults to 0; changing it re-points the celestial sphere.
         */
        struct SkyObserver
        {
            double julian_date = 2451545.0;    /**< Epoch as a Julian Date (J2000 default). */
            double latitude_radians = 0.0;     /**< Observer geodetic latitude, radians. */
            double longitude_radians = 0.0;    /**< Observer east longitude, radians. */
            bool astronomical_sun = true;      /**< Drive the directional light from the ephemeris sun. */
            bool space_mode = false;           /**< Interplanetary regime: bodies placed heliocentric, camera-relative, no ground. */
        };

        /**
         * @brief Everything the renderer needs to light and surround the scene this frame.
         *
         * Authored by the simulation, consumed by `ISceneView::render`. A single instance
         * describes the sun, the planet it lights, the sky layers around it, the far-field
         * solar-system bodies and stars, and the camera's exposure — the whole
         * environment, independent of the drawable meshes.
         */
        struct Environment
        {
            DirectionalLight sun;        /**< The scene's single directional light. */
            Wgs84 planet;                /**< The WGS84 ellipsoid the sun lights. */
            AtmosphereParams atmosphere; /**< The air shell around the planet. */
            PlanetParams surface;        /**< How the planet's ground shades. */
            CloudParams clouds;          /**< The ray-marched cloud layer. */
            StarParams stars;            /**< The space-background star field. */
            Vector3 ambient{Vector3{0.03, 0.04, 0.06}}; /**< Flat ambient floor so shadowed faces are not black. */
            float exposure = 0.18f;      /**< Multiplier applied before tonemapping. */

            SkyObserver observer;                        /**< Where/when the sky is seen from. */
            CelestialBody bodies[MAX_CELESTIAL_BODIES]{}; /**< Far-field solar-system bodies this frame. */
            int body_count = 0;                          /**< Number of populated @ref bodies entries. */
            SkyStar sky_stars[MAX_SKY_STARS]{};          /**< Far-field catalogued stars this frame. */
            int sky_star_count = 0;                      /**< Number of populated @ref sky_stars entries. */
        };
    } // namespace Render
} // namespace SushiEngine
