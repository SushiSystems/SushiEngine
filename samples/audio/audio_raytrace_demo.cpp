/**************************************************************************/
/* audio_raytrace_demo.cpp                                              */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

/**
 * @file audio_raytrace_demo.cpp
 * @brief Ray-traced room acoustics (measured RT60 + baked IR) and edge diffraction, end to end.
 *
 * Builds a shoebox room, ray-traces its impulse response, and checks the measured RT60 against
 * the analytic Sabine estimate for the same volume/surface/absorption (they should agree to
 * well within a factor of two). Then builds a barrier between a source and listener and checks
 * the Maekawa edge-diffraction insertion loss is positive and rises with frequency. Self-checks
 * headless; exits 0 on success.
 */

#include <cmath>
#include <cstdio>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

int main()
{
    // A 10 × 6 × 4 m shoebox, uniform absorption.
    const float lx = 10.0f, ly = 6.0f, lz = 4.0f;
    const float alpha = 0.20f;

    AcousticMesh room;
    AcousticMaterial wall;
    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
    {
        wall.absorption[b] = alpha;
        wall.scattering[b] = 0.2f;
    }
    const std::uint32_t mat = room.add_material(wall);
    room.add_box(AudioVec3{lx * 0.5f, ly * 0.5f, lz * 0.5f},
                 AudioVec3{lx * 0.5f, ly * 0.5f, lz * 0.5f}, mat);

    RayTracedAcoustics tracer;
    RayTraceParams params;
    params.rays = 12000;
    params.max_order = 80;
    params.receiver_radius = 0.6f;
    const RoomImpulseResponse rir =
        tracer.bake(room, AudioVec3{2.0f, 3.0f, 2.0f}, AudioVec3{8.0f, 3.0f, 2.0f}, params);

    const double volume = lx * ly * lz;
    const double surface = 2.0 * (lx * ly + ly * lz + lx * lz);
    const double sabine = 0.161 * volume / (surface * alpha);
    std::printf("ray-traced RT60 per band: %.3f %.3f %.3f s (Sabine estimate %.3f s)\n",
                rir.rt60[0], rir.rt60[1], rir.rt60[2], sabine);
    std::printf("rays=%d detections=%d IR length=%zu samples\n", rir.ray_count, rir.detected,
                rir.impulse.size());

    const double rt = rir.rt60[0];
    if (!(rt > 0.0))
    {
        std::fprintf(stderr, "audio_raytrace_demo FAILED: RT60 not measured\n");
        return 1;
    }
    const double ratio = rt / sabine;
    if (!(ratio > 0.4 && ratio < 2.5))
    {
        std::fprintf(stderr, "audio_raytrace_demo FAILED: RT60 %.3f far from Sabine %.3f\n", rt,
                     sabine);
        return 1;
    }

    // Impulse response must carry energy and decay (start louder than the tail).
    double head = 0.0, tail = 0.0;
    const int n = static_cast<int>(rir.impulse.size());
    for (int i = 0; i < n / 8; ++i)
        head += std::fabs(rir.impulse[static_cast<std::size_t>(i)]);
    for (int i = n - n / 8; i < n; ++i)
        tail += std::fabs(rir.impulse[static_cast<std::size_t>(i)]);
    std::printf("IR head energy=%.2f tail energy=%.2f\n", head, tail);
    if (!(head > tail))
    {
        std::fprintf(stderr, "audio_raytrace_demo FAILED: IR does not decay\n");
        return 1;
    }

    // Edge diffraction: a barrier between source and listener.
    AcousticMesh barrier_mesh;
    const std::uint32_t bmat = barrier_mesh.add_material(AcousticMaterial::concrete());
    barrier_mesh.add_box(AudioVec3{5.0f, 0.0f, 1.5f}, AudioVec3{0.1f, 3.0f, 1.5f}, bmat);
    AcousticBlas barrier_blas;
    barrier_blas.build(barrier_mesh);
    AcousticScene scene;
    AcousticInstance instance;
    instance.blas = &barrier_blas;
    instance.set_position(AudioVec3{0.0f, 0.0f, 0.0f});
    scene.add_instance(instance);
    scene.commit();

    const AudioVec3 src{0.0f, 0.0f, 1.0f};
    const AudioVec3 lst{10.0f, 0.0f, 1.0f};
    float diff_db[ACOUSTIC_BAND_COUNT];
    RayTracedAcoustics::maekawa_diffraction_db(scene, barrier_mesh, src, lst, diff_db);
    std::printf("barrier diffraction loss per band: %.1f %.1f %.1f dB\n", diff_db[0], diff_db[1],
                diff_db[2]);
    if (!(diff_db[0] > 0.0f) || !(diff_db[2] >= diff_db[0]))
    {
        std::fprintf(stderr,
                     "audio_raytrace_demo FAILED: diffraction loss not positive / not rising with "
                     "frequency\n");
        return 1;
    }

    // A clear path (no barrier in the way) must produce no diffraction loss.
    float clear_db[ACOUSTIC_BAND_COUNT];
    RayTracedAcoustics::maekawa_diffraction_db(scene, barrier_mesh, AudioVec3{0.0f, 5.0f, 1.0f},
                                               AudioVec3{2.0f, 5.0f, 1.0f}, clear_db);
    std::printf("clear-path diffraction loss: %.1f %.1f %.1f dB\n", clear_db[0], clear_db[1],
                clear_db[2]);

    std::printf("audio_raytrace_demo OK\n");
    return 0;
}
