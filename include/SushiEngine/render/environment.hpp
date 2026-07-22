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
 * renderer (which consumes them): the sun as a directional light, the dominant
 * planet it lights, the atmosphere/cloud/star layers drawn around that planet, and
 * the per-surface PBR material. None of these types depend on Vulkan or on the sim;
 * they are plain trivially-copyable data that crosses the `render()` boundary once
 * per frame, the same way `MeshInstance` and `CameraView` do.
 *
 * The renderer draws the planet analytically (a ray-marched ellipsoid, no tessellated
 * terrain), so `ReferenceEllipsoid` carries only the ellipsoid's defining constants;
 * the surface grid the editor shows sits tangent to Earth's at the local origin.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/material.hpp>

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
         * @brief Ground-hugging volumetric fog, integrated through a froxel volume.
         *
         * A height-graded participating medium the renderer marches into a
         * camera-frustum-aligned 3D volume: its extinction falls off exponentially with
         * altitude, and each froxel gathers the sun (phase-weighted, attenuated by the
         * atmosphere's transmittance LUT) plus a constant ambient fill. Meshes, the ground,
         * and the sky then read the air in front of them as one fetch, so valleys and
         * airfields haze over and the low sun lights the fog. Plain data; @c density is the
         * sea-level extinction per metre and @c height_falloff its exponential rate.
         */
        struct FogParams
        {
            bool enabled = false;                            /**< Draw volumetric fog at all. */
            float density = 0.01f;                           /**< Sea-level extinction, per metre. */
            float height_falloff = 0.0005f;                  /**< Exponential altitude falloff, per metre. */
            Vector3 scattering_color{Vector3{0.55, 0.6, 0.7}}; /**< Scattering tint of the medium. */
            float ambient = 0.2f;                            /**< Constant ambient in-scatter fill, [0, 1]. */
            float phase_anisotropy = 0.4f;                   /**< Henyey-Greenstein g for the sun lobe, [0, 1). */
        };

        /** @brief The primitive a @ref FogVolume adds its density inside. */
        enum class FogVolumeShape : std::uint32_t
        {
            Box = 0,
            Ellipsoid = 1,
        };

        /**
         * @brief An authored local fog primitive that thickens the fog inside its bounds.
         *
         * A box or ellipsoid the fog pass blends into the same froxel grid as the global
         * height fog — valley fog, an airfield ground layer, a fog bank — so it costs no
         * new pass, only extra density where it sits. @c edge_falloff softens the boundary
         * so the primitive fades in rather than cutting a hard edge. Camera-relative when it
         * reaches the shader, so it rebases with everything else at planet scale.
         */
        struct FogVolume
        {
            WorldVector3 center{};                          /**< World centre, metres. */
            Vector3 extent{Vector3{500.0, 120.0, 500.0}};   /**< Half-extents, metres. */
            Vector3 color{Vector3{0.6, 0.65, 0.7}};         /**< Scattering tint inside. */
            float density = 0.03f;                          /**< Extinction inside, per metre. */
            float edge_falloff = 0.35f;                     /**< Soft-edge fraction, [0, 1]. */
            FogVolumeShape shape = FogVolumeShape::Box;      /**< Box or ellipsoid bounds. */
        };

        /** @brief Maximum local fog volumes the environment carries to the renderer. */
        constexpr int MAX_FOG_VOLUMES = 8;

        /**
         * @brief Probe-volume global illumination controls.
         *
         * A camera-relative cascade of diffuse irradiance probes replaces the flat sky
         * ambient with a spatially varying indirect term. @c enabled is the author's
         * switch (the tier also has to allow it); @c intensity scales the gathered
         * indirect diffuse, and @c normal_bias pushes the sample point along the surface
         * normal to keep a probe behind the surface from leaking through it.
         */
        struct GiParams
        {
            bool enabled = false;    /**< Draw probe-volume GI at all. */
            float intensity = 1.0f;  /**< Multiplier on the gathered indirect diffuse. */
            float normal_bias = 0.4f; /**< Metres the sample point is pushed along the normal. */
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
         * @brief Which 3D base volume a cloud's low-frequency shape is carved from.
         *
         * The renderer generates one volume per kind. @c Cumuliform is the isotropic
         * Perlin-Worley billow used for every convective and stratiform genus (its cellular
         * lumps flattened into sheets by the profile's @ref CloudGenusProfile::stratiform
         * axis); @c Cirriform is a wind-stretched anisotropic volume whose filaments streak
         * along the flow, so fibrous cirrus reads as feathers rather than puffs.
         */
        enum class CloudNoiseKind : std::uint32_t
        {
            Cumuliform = 0,
            Cirriform = 1,
        };

        /**
         * @brief The ten WMO cloud genera, ordered high étage → low → vertical.
         *
         * The full tropospheric taxonomy the cloudscape can render. Each genus resolves to
         * a @ref CloudGenusProfile that places it in its étage and sets its morphology, so a
         * deck names a genus and inherits a physically plausible shape. High (ice): cirrus,
         * cirrocumulus, cirrostratus. Middle: altocumulus, altostratus, nimbostratus. Low:
         * stratocumulus, stratus, cumulus. Vertical (deep convection): cumulonimbus.
         */
        enum class CloudGenus : std::uint32_t
        {
            Cirrus = 0,
            Cirrocumulus,
            Cirrostratus,
            Altocumulus,
            Altostratus,
            Nimbostratus,
            Stratocumulus,
            Stratus,
            Cumulus,
            Cumulonimbus,
            Count,
        };

        /** @brief Number of WMO genera in @ref CloudGenus (excludes @c Count). */
        constexpr int CLOUD_GENUS_COUNT = static_cast<int>(CloudGenus::Count);

        /**
         * @brief The intrinsic shape and placement of one cloud genus.
         *
         * The physically-motivated recipe the shader builds a genus from: its étage band
         * (@c base_altitude → @c top_altitude), how much sky it covers, its opacity, the
         * @c stratiform axis (0 = cellular cumuliform lumps, 1 = a flat continuous sheet),
         * the @c anvil spread that flares a deep-convective top outward (cumulonimbus), the
         * detail erosion, the base/detail noise tiling, its drift @c wind, and which base
         * volume (@ref CloudNoiseKind) it samples. Plain data; see @ref cloud_genus_profile
         * for the catalogue of WMO-plausible values.
         */
        struct CloudGenusProfile
        {
            float base_altitude;    /**< Étage bottom, metres above surface. */
            float top_altitude;     /**< Étage top, metres above surface. */
            float coverage;         /**< Default sky fraction the genus covers, [0, 1]. */
            float density;          /**< Opacity scale of the modelled field, [0, 1]. */
            float stratiform;       /**< 0 cellular cumuliform → 1 flat stratiform sheet, [0, 1]. */
            float anvil;            /**< Deep-convective top-spread (cumulonimbus), [0, 1]. */
            float detail_strength;  /**< How hard the Worley detail erodes the base edges, [0, 1]. */
            float shape_scale;      /**< Metres per tile of the base shape noise. */
            float detail_scale;     /**< Metres per tile of the high-frequency detail noise. */
            Vector3 wind;           /**< Advection velocity, metres/second. */
            CloudNoiseKind noise;   /**< Which base volume the genus is carved from. */
        };

        /**
         * @brief The WMO-plausible intrinsic profile for a cloud genus.
         *
         * The genus catalogue: maps each @ref CloudGenus to its étage, coverage, opacity,
         * and morphology. Adding or retuning a genus touches only this function, never the
         * decks that reference it (open for extension, closed for modification).
         *
         * @param genus The genus to look up.
         * @return Its intrinsic @ref CloudGenusProfile (cumulus if @p genus is out of range).
         */
        inline CloudGenusProfile cloud_genus_profile(CloudGenus genus)
        {
            switch (genus)
            {
            case CloudGenus::Cirrus:
                return {8000.0f, 12000.0f, 0.35f, 0.14f, 0.15f, 0.0f, 0.60f, 60000.0f, 9000.0f,
                        Vector3{55.0, 0.0, 20.0}, CloudNoiseKind::Cirriform};
            case CloudGenus::Cirrocumulus:
                return {6500.0f, 8500.0f, 0.40f, 0.20f, 0.10f, 0.0f, 0.60f, 12000.0f, 700.0f,
                        Vector3{50.0, 0.0, 18.0}, CloudNoiseKind::Cumuliform};
            case CloudGenus::Cirrostratus:
                return {7000.0f, 9500.0f, 0.75f, 0.12f, 0.92f, 0.0f, 0.35f, 80000.0f, 9000.0f,
                        Vector3{48.0, 0.0, 16.0}, CloudNoiseKind::Cumuliform};
            case CloudGenus::Altocumulus:
                return {3500.0f, 5000.0f, 0.50f, 0.35f, 0.30f, 0.0f, 0.60f, 16000.0f, 900.0f,
                        Vector3{28.0, 0.0, 10.0}, CloudNoiseKind::Cumuliform};
            case CloudGenus::Altostratus:
                return {3000.0f, 5500.0f, 0.88f, 0.42f, 0.90f, 0.0f, 0.30f, 70000.0f, 5000.0f,
                        Vector3{26.0, 0.0, 9.0}, CloudNoiseKind::Cumuliform};
            case CloudGenus::Nimbostratus:
                return {1000.0f, 5500.0f, 0.95f, 0.70f, 0.85f, 0.0f, 0.35f, 60000.0f, 4200.0f,
                        Vector3{20.0, 0.0, 7.0}, CloudNoiseKind::Cumuliform};
            case CloudGenus::Stratocumulus:
                return {800.0f, 2200.0f, 0.70f, 0.50f, 0.55f, 0.0f, 0.55f, 20000.0f, 900.0f,
                        Vector3{18.0, 0.0, 6.0}, CloudNoiseKind::Cumuliform};
            case CloudGenus::Stratus:
                return {300.0f, 1200.0f, 0.90f, 0.48f, 0.95f, 0.0f, 0.25f, 50000.0f, 2500.0f,
                        Vector3{12.0, 0.0, 4.0}, CloudNoiseKind::Cumuliform};
            case CloudGenus::Cumulus:
                return {1000.0f, 3200.0f, 0.42f, 0.62f, 0.12f, 0.0f, 0.62f, 18000.0f, 1000.0f,
                        Vector3{16.0, 0.0, 6.0}, CloudNoiseKind::Cumuliform};
            case CloudGenus::Cumulonimbus:
                return {800.0f, 14000.0f, 0.35f, 0.90f, 0.10f, 0.70f, 0.60f, 28000.0f, 1200.0f,
                        Vector3{14.0, 0.0, 5.0}, CloudNoiseKind::Cumuliform};
            default:
                return cloud_genus_profile(CloudGenus::Cumulus);
            }
        }

        /**
         * @brief The short WMO name of a cloud genus, for the editor.
         * @param genus The genus to name.
         * @return A static, human-readable label (never null).
         */
        inline const char* cloud_genus_name(CloudGenus genus)
        {
            switch (genus)
            {
            case CloudGenus::Cirrus:        return "Cirrus";
            case CloudGenus::Cirrocumulus:  return "Cirrocumulus";
            case CloudGenus::Cirrostratus:  return "Cirrostratus";
            case CloudGenus::Altocumulus:   return "Altocumulus";
            case CloudGenus::Altostratus:   return "Altostratus";
            case CloudGenus::Nimbostratus:  return "Nimbostratus";
            case CloudGenus::Stratocumulus: return "Stratocumulus";
            case CloudGenus::Stratus:       return "Stratus";
            case CloudGenus::Cumulus:       return "Cumulus";
            case CloudGenus::Cumulonimbus:  return "Cumulonimbus";
            default:                        return "Unknown";
            }
        }

        /**
         * @brief One named cloud deck: a genus placed in the sky with light author tweaks.
         *
         * A deck references a @ref CloudGenus and inherits its @ref cloud_genus_profile,
         * then nudges it: @c coverage_bias adds to the profile coverage (thin or fill the
         * deck) and @c density_scale multiplies its opacity. Several decks coexist, so a
         * real sky (cumulus under cirrus over a stratocumulus break) is a few decks.
         */
        struct CloudDeck
        {
            bool enabled = false;                       /**< Draw this deck at all. */
            CloudGenus genus = CloudGenus::Cumulus;     /**< Which genus this deck instantiates. */
            float coverage_bias = 0.0f;                 /**< Added to the profile coverage, [-1, 1]. */
            float density_scale = 1.0f;                 /**< Multiplies the profile density, [0, 2]. */
        };

        /** @brief Maximum simultaneously-rendered cloud decks the cloudscape carries. */
        constexpr int CLOUD_MAX_DECKS = 6;

        /**
         * @brief A physically-motivated volumetric cloudscape of stacked genus decks.
         *
         * Up to @ref CLOUD_MAX_DECKS decks, each a @ref CloudGenus, modelled as one
         * continuous medium: the shader ray marches the union shell of all enabled decks in
         * a single pass, so a sample's light march toward the Sun crosses every deck above
         * it and the high cirrus shadows the cumulus below it for free — the decks interact
         * because they share one optical medium. The scattering knobs therefore live here,
         * not per deck: Beer extinction, the dual-lobe Henyey-Greenstein phase for the
         * silver lining, the powder term for the dark underside, and the sky-dome ambient
         * fill are properties of the medium. The decks cast their combined shadow onto the
         * analytic ground via @ref ground_shadow_strength. A low-resolution cloud map (see
         * the renderer) bounds the march for empty-space skipping and, from orbit, paints
         * the shell as relief so tall cumulonimbus reads from space; @c evolution_rate
         * animates that map as weather forms and dissipates.
         */
        struct Cloudscape
        {
            bool enabled = true;            /**< Draw clouds at all (master switch). */
            float light_absorption = 0.75f; /**< Beer extinction per unit density along the light march. */
            float forward_scattering = 0.8f;/**< Henyey-Greenstein anisotropy g for the forward lobe, [0, 1). */
            float powder_strength = 0.3f;   /**< Strength of the dark-edge powder term, [0, 1]. */
            float ambient_strength = 0.5f;  /**< Sky-dome ambient fill on the shadowed side, [0, 1]. */
            float ground_shadow_strength = 1.0f; /**< How strongly the deck stack shadows the ground, [0, 1]. */
            float weather_scale = 60000.0f; /**< Metres per tile of the cloud/weather map. */
            float evolution_rate = 0.03f;   /**< How fast the cloud map forms and dissipates, [0, 1]. */

            /** @brief The active decks, defaulting to a plausible fair-weather sky. */
            CloudDeck decks[CLOUD_MAX_DECKS] = {
                CloudDeck{true, CloudGenus::Cumulus, 0.0f, 1.0f},
                CloudDeck{true, CloudGenus::Cirrus, 0.0f, 1.0f},
                CloudDeck{true, CloudGenus::Stratocumulus, -0.2f, 1.0f},
                CloudDeck{false, CloudGenus::Cumulonimbus, 0.0f, 1.0f},
                CloudDeck{false, CloudGenus::Altocumulus, 0.0f, 1.0f},
                CloudDeck{false, CloudGenus::Altostratus, 0.0f, 1.0f},
            };
        };

        /**
         * @brief A one-click sky: the common weather situations a flight sim starts from.
         *
         * Each names a familiar sky the pilot recognizes; @ref cloud_weather_preset expands
         * it to a full @ref Cloudscape (which decks, which genera, the medium tuning). The
         * editor offers these as buttons so the sky is set without touching a single deck.
         */
        enum class WeatherPreset : std::uint32_t
        {
            Clear,       /**< Cloudless blue sky. */
            FairWeather, /**< Scattered fair-weather cumulus under a little cirrus. */
            Overcast,    /**< A grey stratocumulus/altostratus sheet. */
            Storm,       /**< Towering cumulonimbus over a low broken deck. */
            Count
        };

        /** @brief Number of presets in @ref WeatherPreset (excludes @c Count). */
        constexpr int WEATHER_PRESET_COUNT = static_cast<int>(WeatherPreset::Count);

        /**
         * @brief The short name of a weather preset, for the editor buttons.
         * @param preset The preset to name.
         * @return Its display name (empty string if out of range).
         */
        inline const char* weather_preset_name(WeatherPreset preset)
        {
            switch (preset)
            {
            case WeatherPreset::Clear:       return "Clear";
            case WeatherPreset::FairWeather: return "Fair Weather";
            case WeatherPreset::Overcast:    return "Overcast";
            case WeatherPreset::Storm:       return "Storm";
            default:                         return "";
            }
        }

        /**
         * @brief Expand a weather preset into a ready-to-render cloudscape.
         *
         * Starts from the @ref Cloudscape defaults, disables every deck, then enables and
         * tunes only the decks the situation calls for and adjusts the shared medium (a storm
         * absorbs more light and fills darker; overcast fills brighter). Pure data, mirroring
         * @ref cloud_genus_profile — adding a preset touches only this function.
         *
         * @param preset The weather situation to build.
         * @return A @ref Cloudscape configured for @p preset (clouds enabled).
         */
        inline Cloudscape cloud_weather_preset(WeatherPreset preset)
        {
            Cloudscape clouds;
            for (int i = 0; i < CLOUD_MAX_DECKS; ++i)
                clouds.decks[i].enabled = false;

            switch (preset)
            {
            case WeatherPreset::Clear:
                break;
            case WeatherPreset::FairWeather:
                clouds.decks[0] = CloudDeck{true, CloudGenus::Cumulus, -0.1f, 1.0f};
                clouds.decks[1] = CloudDeck{true, CloudGenus::Cirrus, -0.1f, 1.0f};
                break;
            case WeatherPreset::Overcast:
                clouds.decks[0] = CloudDeck{true, CloudGenus::Stratocumulus, 0.3f, 1.0f};
                clouds.decks[1] = CloudDeck{true, CloudGenus::Altostratus, 0.2f, 1.0f};
                clouds.ambient_strength = 0.7f;
                break;
            case WeatherPreset::Storm:
                clouds.decks[0] = CloudDeck{true, CloudGenus::Cumulonimbus, 0.2f, 1.2f};
                clouds.decks[1] = CloudDeck{true, CloudGenus::Stratocumulus, 0.2f, 1.0f};
                clouds.decks[2] = CloudDeck{true, CloudGenus::Nimbostratus, 0.1f, 1.0f};
                clouds.light_absorption = 1.1f;
                clouds.ambient_strength = 0.35f;
                break;
            default:
                break;
            }
            return clouds;
        }

        /**
         * @brief How the Moon and the star field light the scene once the Sun sets.
         *
         * The ephemeris derives @ref Environment::ambient each frame from the Sun's
         * elevation, the Moon's elevation and illuminated fraction, and the star field's
         * presence, scaled by these knobs — so night lighting stays physically motivated
         * (full moon brighter than crescent, moonless night darker) while remaining
         * author-controllable from the environment panel.
         */
        struct NightLighting
        {
            bool enabled = true;         /**< Derive ambient from Sun/Moon/star geometry each frame. */
            float moon_intensity = 0.35f; /**< Ambient scale from a fully-illuminated, overhead Moon. */
            float star_intensity = 0.02f; /**< Ambient floor from the star field on a moonless night. */
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
         * @brief A reference ellipsoid generalised beyond WGS84 to any body's surface.
         *
         * Equatorial radius and flattening, for any planet or moon whose surface the
         * near-field regime ray-marches or meshes. The defaults are WGS84 (Earth); Mars,
         * the Moon, and the rest supply their own via the astro surface presets, so the
         * ground pipeline is not Earth-specific. Kept at double precision because the
         * values are planet-scale metres, well past single-precision's clean range.
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
         * @brief How a body's surface is procedurally patterned by the shader.
         *
         * The "shader first" half of the body LOD ladder: before any mesh exists, the
         * sky pass paints a recognisable surface from the body's normal, pole, and two
         * base colours — cratered brightness noise for @c Rocky, an ocean/land mask for
         * @c EarthLike, latitude bands wobbled by turbulence for @c Banded gas giants,
         * and untinted emission for a @c Star.
         */
        enum class SurfaceStyle : std::uint32_t
        {
            Rocky = 0,
            EarthLike,
            Banded,
            Star,
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
            Vector3 pole{Vector3{0.0, 1.0, 0.0}};            /**< Unit north rotation pole, local frame (bands/flattening). */
            Vector3 color{Vector3{1.0, 1.0, 1.0}};           /**< Linear RGB surface tint. */
            WorldVector3 heliocentric_position{};            /**< Absolute heliocentric position, metres (fly-out / rebasing). */
            float angular_radius = 0.0f;                     /**< Apparent angular radius, radians. */
            float brightness = 1.0f;                         /**< Relative reflected/emitted radiance scale. */
            float distance_metres = 0.0f;                    /**< Geocentric distance to the body, metres (LOD metric). */
            float mean_radius_metres = 0.0f;                 /**< Physical mean radius, metres. */
            BodyLod lod = BodyLod::Disk;                     /**< Representation regime this frame. */
            std::uint32_t body_id = 0;                       /**< The ephemeris body index this entry was filled from. */
            std::uint32_t is_star = 0;                       /**< 1 if the body emits (the Sun), else 0. */
            SurfaceStyle surface_style = SurfaceStyle::Rocky; /**< Procedural pattern the shader paints it with. */
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
            // Which body the observer stands on, as the ephemeris body index (Earth = 3
            // by default). Kept as a plain int, not the astro BodyId enum, so this render
            // seam stays free of an astro dependency; the ephemeris maps it back. Making
            // this the observer's body is what lets the whole sky be built the same way on
            // any planet — the scene is anchored to this body's surface, not always Earth's.
            int observer_body = 3;
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
            ReferenceEllipsoid planet;   /**< The dominant body's ellipsoid (WGS84 by default). */
            WorldVector3 planet_center{0.0, -6378137.0, 0.0}; /**< The dominant body's centre in the scene frame, metres. */
            double planet_surface_reference_metres = 6378137.0; /**< Radius of the sphere the atmosphere/cloud altitudes reference — chosen so altitude is exactly zero at the local ground, not the equatorial radius. */
            Vector3 planet_pole{Vector3{0.0, 1.0, 0.0}};      /**< The dominant body's north pole, scene frame (orients the ellipsoid). */
            SurfaceStyle planet_surface_style = SurfaceStyle::EarthLike; /**< Procedural pattern of the dominant body's ground. */
            AtmosphereParams atmosphere; /**< The air shell around the planet. */
            FogParams fog;               /**< Ground-hugging volumetric fog. */
            FogVolume fog_volumes[MAX_FOG_VOLUMES]{}; /**< Authored local fog primitives. */
            int fog_volume_count = 0;    /**< Number of populated @ref fog_volumes entries. */
            GiParams gi;                 /**< Probe-volume global illumination. */
            PlanetParams surface;        /**< How the planet's ground shades. */
            Cloudscape clouds;           /**< The ray-marched, layered cloudscape. */
            StarParams stars;            /**< The space-background star field. */
            NightLighting night;         /**< How the Moon and stars light a sunless sky. */
            Vector3 ambient{Vector3{0.03, 0.04, 0.06}}; /**< Ambient floor so shadowed faces are not black; the ephemeris drives this dynamically when @ref NightLighting::enabled. */
            float exposure = 0.18f;      /**< Multiplier applied before tonemapping. */

            /**
             * @brief Light the scene from the captured environment instead of a constant.
             *
             * With this off, indirect light is the flat @ref ambient term the renderer
             * has always used. With it on, the sky is captured and prefiltered each time
             * it changes and surfaces read their indirect diffuse and specular from it —
             * which is what makes a metal read as metal rather than as a flat tint.
             */
            bool image_based_lighting = true;

            /** @brief Scales the captured environment's contribution. */
            float ibl_intensity = 1.0f;

            SkyObserver observer;                        /**< Where/when the sky is seen from. */
            bool planet_surface_visible = true;          /**< Draw the analytic ground; the ephemeris clears it past the hand-off altitude, where Earth joins @ref bodies instead. */
            int dominant_body_id = -1;                   /**< Ephemeris index of the body whose surface is the analytic ground this frame, or -1 in deep space; lets the camera ride a moving planet as time animates. */
            WorldVector3 dominant_center_metres{};       /**< The dominant body's centre in the scene frame this frame, metres; the camera tracks its delta so it stays attached to a non-Earth planet as its orbit carries it. */
            CelestialBody bodies[MAX_CELESTIAL_BODIES]{}; /**< Far-field solar-system bodies this frame. */
            int body_count = 0;                          /**< Number of populated @ref bodies entries. */
            float solar_eclipse = 0.0f;                  /**< Fraction of the Sun's disk hidden by a nearer body this frame, [0,1]; dims the directional sun so the sky, the ground, and shaded meshes all dusk toward totality together. Computed once by @ref fill_environment_sky and read by both the sky and PBR passes. */
            SkyStar sky_stars[MAX_SKY_STARS]{};          /**< Far-field catalogued stars this frame. */
            int sky_star_count = 0;                      /**< Number of populated @ref sky_stars entries. */
        };
    } // namespace Render
} // namespace SushiEngine
