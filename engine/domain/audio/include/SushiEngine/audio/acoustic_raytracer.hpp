/**************************************************************************/
/* acoustic_raytracer.hpp                                                */
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
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#ifndef SUSHIENGINE_AUDIO_ACOUSTIC_RAYTRACER_HPP
#define SUSHIENGINE_AUDIO_ACOUSTIC_RAYTRACER_HPP

/**
 * @file acoustic_raytracer.hpp
 * @brief Monte-Carlo ray-traced room acoustics — measured RT60 and a baked impulse response.
 *
 * The physically-derived counterpart to the parametric FDN/I3DL2 reverb: rather than dialing a
 * decay time, this **measures** it from the geometry. Rays are shot from the source over the
 * sphere and traced through specular/diffuse bounces (chosen by each surface's scattering
 * coefficient), losing energy to absorption per frequency band at every reflection; whenever a
 * ray segment passes within a receiver sphere around the listener its running band energy is
 * binned by arrival time. Schroeder backward-integration of that energy-time histogram yields
 * the **RT60 per band**, and the histogram is turned into a decaying-noise **impulse response**
 * the @ref ConvolutionReverb can render — so a signature space reverberates by its true shape.
 *
 * @ref maekawa_diffraction_db adds geometric edge diffraction: when the direct path is blocked
 * it finds the least-detour path over the occluder's silhouette and returns the Maekawa
 * single-edge insertion loss per band — "sound bends over the wall", frequency-dependent.
 *
 * Self-contained (its own closest-hit over @ref AcousticMesh triangles) and dependency-free, so
 * it rides the `audio.hpp` umbrella. IR baking is an offline/level-load step, not a per-frame one.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/audio/acoustic_geometry.hpp>
#include <SushiEngine/audio/acoustic_material.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief The measured reverberation of a space: RT60 per band and a baked IR. */
        struct RoomImpulseResponse
        {
            float rt60[ACOUSTIC_BAND_COUNT] = {0.0f, 0.0f, 0.0f}; /**< Decay time per band (s). */
            std::vector<float> impulse;                            /**< Mono IR at the bake rate. */
            double sample_rate = 48000.0;
            int ray_count = 0;      /**< Rays cast. */
            int detected = 0;       /**< Segment detections registered. */
        };

        /** @brief Parameters for a ray-traced acoustics bake. */
        struct RayTraceParams
        {
            int rays = 8000;             /**< Rays cast from the source. */
            int max_order = 64;          /**< Maximum reflection order per ray. */
            float receiver_radius = 0.5f; /**< Listener detection-sphere radius (m). */
            float speed_of_sound = 343.0f;
            double histogram_seconds = 2.0; /**< Energy-time histogram length. */
            double bin_seconds = 0.005;     /**< Histogram bin width. */
            float energy_floor = 1e-6f;     /**< Stop a ray once its energy drops below this. */
            std::uint64_t seed = 0x9e3779b97f4a7c15ull;
        };

        /** @brief Ray-traced room-acoustics baker. */
        class RayTracedAcoustics
        {
            public:
                /**
                 * @brief Bakes the room impulse response and RT60 for a source/listener pair.
                 * @param mesh     The room geometry with per-triangle materials.
                 * @param source   Source position.
                 * @param listener Listener position.
                 * @param params   Ray-trace parameters.
                 * @return The measured RT60 per band and a synthesized impulse response.
                 */
                RoomImpulseResponse bake(const AcousticMesh& mesh, const AudioVec3& source,
                                         const AudioVec3& listener,
                                         const RayTraceParams& params = RayTraceParams()) const
                {
                    RoomImpulseResponse out;
                    out.sample_rate = 48000.0;
                    out.ray_count = params.rays;

                    const int bins =
                        static_cast<int>(params.histogram_seconds / params.bin_seconds) + 1;
                    std::vector<double> histogram(
                        static_cast<std::size_t>(bins * ACOUSTIC_BAND_COUNT), 0.0);

                    std::uint64_t rng = params.seed;
                    for (int r = 0; r < params.rays; ++r)
                    {
                        float dir[3];
                        sample_sphere(rng, dir);
                        trace_ray(mesh, source, listener, dir, params, histogram, bins, out);
                    }

                    compute_rt60(histogram, bins, params, out);
                    synthesize_impulse(histogram, bins, params, out);
                    return out;
                }

                /**
                 * @brief Maekawa single-edge diffraction insertion loss per band (dB, ≥ 0).
                 *
                 * When the straight source→listener path is blocked, sound reaches the listener by
                 * bending over the occluder's silhouette. This samples the blocking geometry's
                 * bounding box edges, takes the least-detour path (loudest), and returns the Maekawa
                 * attenuation `10·log10(3 + 20·N)` per band, where the Fresnel number `N = 2δ/λ`
                 * grows with the detour δ and frequency. Returns all-zero when the path is clear.
                 *
                 * @param scene    The acoustic scene (for the occlusion test).
                 * @param mesh     The geometry whose bounds define the diffracting edge.
                 * @param source   Source position.
                 * @param listener Listener position.
                 * @param out_db   Filled with the per-band insertion loss (dB).
                 * @param speed_of_sound Speed of sound (m/s).
                 */
                static void maekawa_diffraction_db(const AcousticScene& scene,
                                                   const AcousticMesh& mesh,
                                                   const AudioVec3& source,
                                                   const AudioVec3& listener, float* out_db,
                                                   float speed_of_sound = 343.0f)
                {
                    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                        out_db[b] = 0.0f;
                    if (!scene.occluded(source, listener))
                        return; // path is clear: no diffraction loss

                    AcousticAabb box = mesh_bounds(mesh);
                    const float direct = distance(source, listener);
                    float best_detour = -1.0f;
                    // Candidate bend points: the midpoints of the 12 box edges expanded to the
                    // silhouette; the least detour dominates the diffracted energy.
                    AudioVec3 corners[8];
                    box_corners(box, corners);
                    static const int edges[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0},
                                                     {4, 5}, {5, 7}, {7, 6}, {6, 4},
                                                     {0, 4}, {1, 5}, {2, 6}, {3, 7}};
                    for (int e = 0; e < 12; ++e)
                    {
                        for (int s = 0; s <= 4; ++s)
                        {
                            const float f = s * 0.25f;
                            const AudioVec3 p = lerp(corners[edges[e][0]], corners[edges[e][1]], f);
                            const float detour =
                                distance(source, p) + distance(p, listener) - direct;
                            if (detour > 0.0f && (best_detour < 0.0f || detour < best_detour))
                                best_detour = detour;
                        }
                    }
                    if (best_detour <= 0.0f)
                        best_detour = 0.01f;

                    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                    {
                        const float lambda = speed_of_sound / ACOUSTIC_BAND_HZ[b];
                        const float fresnel = 2.0f * best_detour / lambda;
                        const float att = 10.0f * std::log10(3.0f + 20.0f * fresnel);
                        out_db[b] = att > 0.0f ? att : 0.0f;
                    }
                }

            private:
                static float distance(const AudioVec3& a, const AudioVec3& b) noexcept
                {
                    const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
                    return std::sqrt(dx * dx + dy * dy + dz * dz);
                }

                static AudioVec3 lerp(const AudioVec3& a, const AudioVec3& b, float t) noexcept
                {
                    return AudioVec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                                     a.z + (b.z - a.z) * t};
                }

                static AcousticAabb mesh_bounds(const AcousticMesh& mesh) noexcept
                {
                    AcousticAabb box;
                    for (const AcousticTriangle& t : mesh.triangles())
                    {
                        box.expand(t.a);
                        box.expand(t.b);
                        box.expand(t.c);
                    }
                    return box;
                }

                static void box_corners(const AcousticAabb& box, AudioVec3* c) noexcept
                {
                    for (int i = 0; i < 8; ++i)
                        c[i] = AudioVec3{(i & 1) ? box.max.x : box.min.x,
                                         (i & 2) ? box.max.y : box.min.y,
                                         (i & 4) ? box.max.z : box.min.z};
                }

                static float rand_unit(std::uint64_t& state) noexcept
                {
                    // splitmix64 → [0,1)
                    state += 0x9e3779b97f4a7c15ull;
                    std::uint64_t z = state;
                    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
                    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
                    z = z ^ (z >> 31);
                    return static_cast<float>((z >> 11) * (1.0 / 9007199254740992.0));
                }

                static void sample_sphere(std::uint64_t& rng, float* dir) noexcept
                {
                    const float z = 2.0f * rand_unit(rng) - 1.0f;
                    const float theta = 6.28318530718f * rand_unit(rng);
                    const float r = std::sqrt(1.0f - z * z);
                    dir[0] = r * std::cos(theta);
                    dir[1] = r * std::sin(theta);
                    dir[2] = z;
                }

                // Closest triangle hit for a ray origin+dir (unit). Returns t>0, index, and normal.
                static bool closest_hit(const AcousticMesh& mesh, const AudioVec3& o,
                                        const float* d, float& t_out, int& tri_out,
                                        AudioVec3& n_out) noexcept
                {
                    const std::vector<AcousticTriangle>& tris = mesh.triangles();
                    float best = 1e30f;
                    int best_tri = -1;
                    AudioVec3 best_n{0, 0, 0};
                    for (std::size_t i = 0; i < tris.size(); ++i)
                    {
                        const AcousticTriangle& tr = tris[i];
                        const float e1x = tr.b.x - tr.a.x, e1y = tr.b.y - tr.a.y, e1z = tr.b.z - tr.a.z;
                        const float e2x = tr.c.x - tr.a.x, e2y = tr.c.y - tr.a.y, e2z = tr.c.z - tr.a.z;
                        const float px = d[1] * e2z - d[2] * e2y;
                        const float py = d[2] * e2x - d[0] * e2z;
                        const float pz = d[0] * e2y - d[1] * e2x;
                        const float det = e1x * px + e1y * py + e1z * pz;
                        if (det > -1e-9f && det < 1e-9f)
                            continue;
                        const float inv = 1.0f / det;
                        const float sx = o.x - tr.a.x, sy = o.y - tr.a.y, sz = o.z - tr.a.z;
                        const float u = (sx * px + sy * py + sz * pz) * inv;
                        if (u < 0.0f || u > 1.0f)
                            continue;
                        const float qx = sy * e1z - sz * e1y;
                        const float qy = sz * e1x - sx * e1z;
                        const float qz = sx * e1y - sy * e1x;
                        const float v = (d[0] * qx + d[1] * qy + d[2] * qz) * inv;
                        if (v < 0.0f || u + v > 1.0f)
                            continue;
                        const float t = (e2x * qx + e2y * qy + e2z * qz) * inv;
                        if (t > 1e-4f && t < best)
                        {
                            best = t;
                            best_tri = static_cast<int>(i);
                            best_n = AudioVec3{e1y * e2z - e1z * e2y, e1z * e2x - e1x * e2z,
                                               e1x * e2y - e1y * e2x};
                        }
                    }
                    if (best_tri < 0)
                        return false;
                    const float len = std::sqrt(best_n.x * best_n.x + best_n.y * best_n.y +
                                                best_n.z * best_n.z);
                    if (len > 1e-12f)
                    {
                        best_n.x /= len;
                        best_n.y /= len;
                        best_n.z /= len;
                    }
                    t_out = best;
                    tri_out = best_tri;
                    n_out = best_n;
                    return true;
                }

                // Closest distance from point p to the segment a→b.
                static float point_segment_distance(const AudioVec3& p, const AudioVec3& a,
                                                    const float* d, float seg_len,
                                                    float& t_along) noexcept
                {
                    const float apx = p.x - a.x, apy = p.y - a.y, apz = p.z - a.z;
                    float proj = apx * d[0] + apy * d[1] + apz * d[2];
                    if (proj < 0.0f)
                        proj = 0.0f;
                    if (proj > seg_len)
                        proj = seg_len;
                    t_along = proj;
                    const float cx = a.x + d[0] * proj - p.x;
                    const float cy = a.y + d[1] * proj - p.y;
                    const float cz = a.z + d[2] * proj - p.z;
                    return std::sqrt(cx * cx + cy * cy + cz * cz);
                }

                void trace_ray(const AcousticMesh& mesh, const AudioVec3& source,
                               const AudioVec3& listener, const float* dir0,
                               const RayTraceParams& params, std::vector<double>& histogram,
                               int bins, RoomImpulseResponse& out) const
                {
                    AudioVec3 origin = source;
                    float dir[3] = {dir0[0], dir0[1], dir0[2]};
                    float energy[ACOUSTIC_BAND_COUNT];
                    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                        energy[b] = 1.0f;
                    float path_len = 0.0f;
                    std::uint64_t rng =
                        params.seed ^ (0xd1b54a32d192ull *
                                       static_cast<std::uint64_t>(
                                           static_cast<std::int64_t>(dir0[0] * 1e6f) + 3));

                    for (int order = 0; order < params.max_order; ++order)
                    {
                        float t = 0.0f;
                        int tri = -1;
                        AudioVec3 n{0, 0, 0};
                        if (!closest_hit(mesh, origin, dir, t, tri, n))
                            break; // escaped the room

                        // Detection: does this segment pass within the receiver sphere?
                        float t_along = 0.0f;
                        const float miss =
                            point_segment_distance(listener, origin, dir, t, t_along);
                        if (miss < params.receiver_radius)
                        {
                            const double arrival = (path_len + t_along) / params.speed_of_sound;
                            const int bin = static_cast<int>(arrival / params.bin_seconds);
                            if (bin >= 0 && bin < bins)
                            {
                                for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                                    histogram[static_cast<std::size_t>(bin * ACOUSTIC_BAND_COUNT + b)] +=
                                        energy[b];
                                ++out.detected;
                            }
                        }

                        // Advance to the hit point, lose energy to absorption.
                        origin = AudioVec3{origin.x + dir[0] * t, origin.y + dir[1] * t,
                                           origin.z + dir[2] * t};
                        path_len += t;
                        const AcousticMaterial& mat =
                            mesh.material_for(static_cast<std::size_t>(tri));
                        float max_energy = 0.0f;
                        for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                        {
                            energy[b] *= (1.0f - mat.absorption[b]);
                            if (energy[b] > max_energy)
                                max_energy = energy[b];
                        }
                        if (max_energy < params.energy_floor)
                            break;

                        // Reflect: diffuse (cosine) with probability = mid-band scattering, else specular.
                        float nx = n.x, ny = n.y, nz = n.z;
                        const float facing = dir[0] * nx + dir[1] * ny + dir[2] * nz;
                        if (facing > 0.0f) // flip the normal to oppose the incoming ray
                        {
                            nx = -nx;
                            ny = -ny;
                            nz = -nz;
                        }
                        if (rand_unit(rng) < mat.scattering[1])
                            cosine_hemisphere(rng, nx, ny, nz, dir);
                        else
                        {
                            const float dn = dir[0] * nx + dir[1] * ny + dir[2] * nz;
                            dir[0] -= 2.0f * dn * nx;
                            dir[1] -= 2.0f * dn * ny;
                            dir[2] -= 2.0f * dn * nz;
                        }
                        // Nudge off the surface to avoid self-hit.
                        origin = AudioVec3{origin.x + nx * 1e-3f, origin.y + ny * 1e-3f,
                                           origin.z + nz * 1e-3f};
                    }
                }

                static void cosine_hemisphere(std::uint64_t& rng, float nx, float ny, float nz,
                                              float* out) noexcept
                {
                    const float u1 = rand_unit(rng);
                    const float u2 = rand_unit(rng);
                    const float r = std::sqrt(u1);
                    const float theta = 6.28318530718f * u2;
                    const float x = r * std::cos(theta);
                    const float y = r * std::sin(theta);
                    const float z = std::sqrt(1.0f - u1);
                    // Build a basis around the normal.
                    float tx, ty, tz;
                    if (std::fabs(nx) > 0.9f)
                    {
                        tx = 0.0f;
                        ty = 1.0f;
                        tz = 0.0f;
                    }
                    else
                    {
                        tx = 1.0f;
                        ty = 0.0f;
                        tz = 0.0f;
                    }
                    // b1 = normalize(cross(t, n)), b2 = cross(n, b1)
                    float b1x = ty * nz - tz * ny, b1y = tz * nx - tx * nz, b1z = tx * ny - ty * nx;
                    const float bl = std::sqrt(b1x * b1x + b1y * b1y + b1z * b1z) + 1e-20f;
                    b1x /= bl;
                    b1y /= bl;
                    b1z /= bl;
                    const float b2x = ny * b1z - nz * b1y, b2y = nz * b1x - nx * b1z,
                                b2z = nx * b1y - ny * b1x;
                    out[0] = b1x * x + b2x * y + nx * z;
                    out[1] = b1y * x + b2y * y + ny * z;
                    out[2] = b1z * x + b2z * y + nz * z;
                    const float l =
                        std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]) + 1e-20f;
                    out[0] /= l;
                    out[1] /= l;
                    out[2] /= l;
                }

                static void compute_rt60(const std::vector<double>& histogram, int bins,
                                         const RayTraceParams& params, RoomImpulseResponse& out)
                {
                    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                    {
                        // Schroeder backward energy integration.
                        std::vector<double> schroeder(static_cast<std::size_t>(bins), 0.0);
                        double running = 0.0;
                        for (int i = bins - 1; i >= 0; --i)
                        {
                            running += histogram[static_cast<std::size_t>(i * ACOUSTIC_BAND_COUNT + b)];
                            schroeder[static_cast<std::size_t>(i)] = running;
                        }
                        if (running <= 0.0)
                        {
                            out.rt60[b] = 0.0f;
                            continue;
                        }
                        const double total = schroeder[0];
                        // Fit the -5 dB … -35 dB region and extrapolate to -60 (T30 × 2).
                        double t5 = -1.0, t35 = -1.0;
                        for (int i = 0; i < bins; ++i)
                        {
                            const double db = 10.0 * std::log10(
                                                        (schroeder[static_cast<std::size_t>(i)] /
                                                         total) + 1e-12);
                            const double time = i * params.bin_seconds;
                            if (t5 < 0.0 && db <= -5.0)
                                t5 = time;
                            if (t35 < 0.0 && db <= -35.0)
                            {
                                t35 = time;
                                break;
                            }
                        }
                        if (t5 >= 0.0 && t35 > t5)
                            out.rt60[b] = static_cast<float>((t35 - t5) * 2.0);
                        else
                            out.rt60[b] = 0.0f;
                    }
                }

                static void synthesize_impulse(const std::vector<double>& histogram, int bins,
                                               const RayTraceParams& params, RoomImpulseResponse& out)
                {
                    const int length = static_cast<int>(out.sample_rate * params.histogram_seconds);
                    out.impulse.assign(static_cast<std::size_t>(length), 0.0f);
                    std::uint64_t rng = params.seed ^ 0xabcddcbaull;
                    const int samples_per_bin =
                        static_cast<int>(params.bin_seconds * out.sample_rate);
                    // Band-averaged energy envelope shapes decaying white noise (a common,
                    // perceptually-faithful IR synthesis from an energy histogram).
                    for (int i = 0; i < bins; ++i)
                    {
                        double e = 0.0;
                        for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                            e += histogram[static_cast<std::size_t>(i * ACOUSTIC_BAND_COUNT + b)];
                        const float amp = static_cast<float>(std::sqrt(e / ACOUSTIC_BAND_COUNT));
                        for (int s = 0; s < samples_per_bin; ++s)
                        {
                            const int idx = i * samples_per_bin + s;
                            if (idx >= length)
                                break;
                            out.impulse[static_cast<std::size_t>(idx)] =
                                amp * (2.0f * rand_unit(rng) - 1.0f);
                        }
                    }
                }
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
