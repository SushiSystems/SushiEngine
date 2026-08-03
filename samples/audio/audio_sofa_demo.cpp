/**************************************************************************/
/* audio_sofa_demo.cpp                                                   */
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
 * @file audio_sofa_demo.cpp
 * @brief The SOFA/HDF5 measured-HRTF path, end to end.
 *
 * Loads a real measured HRIR set (the MIT KEMAR SOFA under assets/hrtf, falling back to a
 * synthetic set baked with @ref write_sofa if it is absent), drives the ambisonic binaural
 * spatializer through @ref BinauralSpatializer::set_hrtf_database, and checks that the
 * measured decode preserves laterality: a source to the left produces more energy at the left
 * ear than the right, and vice versa. Exits 0 on success.
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <SushiEngine/audio/audio.hpp>
#include <SushiEngine/audio/sofa_hrtf.hpp>

using namespace SushiEngine::Audio;

namespace
{
    // Bakes a synthetic SimpleFreeFieldHRIR set (azimuth ring at the horizon) so the demo runs
    // with no external artifact. Each ear gets a delayed, level-shaded impulse from the azimuth.
    bool write_synthetic_sofa(const std::string& path)
    {
        const int measurements = 24;
        const int taps = 64;
        const double sample_rate = 48000.0;
        std::vector<float> ir(static_cast<std::size_t>(measurements * 2 * taps), 0.0f);
        std::vector<float> pos(static_cast<std::size_t>(measurements * 3), 0.0f);
        const double pi = 3.14159265358979323846;
        for (int m = 0; m < measurements; ++m)
        {
            const double az_deg = 360.0 * m / measurements;
            const double az = az_deg * pi / 180.0;
            pos[static_cast<std::size_t>(m * 3 + 0)] = static_cast<float>(az_deg);
            pos[static_cast<std::size_t>(m * 3 + 1)] = 0.0f;   // elevation
            pos[static_cast<std::size_t>(m * 3 + 2)] = 1.4f;   // radius
            const double lateral = std::sin(az); // +1 fully left, -1 fully right
            const int itd = 20;
            const int left_delay = 4 + static_cast<int>((1.0 - lateral) * 0.5 * itd);
            const int right_delay = 4 + static_cast<int>((1.0 + lateral) * 0.5 * itd);
            const float left_gain = static_cast<float>(0.6 + 0.4 * lateral);
            const float right_gain = static_cast<float>(0.6 - 0.4 * lateral);
            ir[static_cast<std::size_t>((m * 2 + 0) * taps + left_delay)] = left_gain;
            ir[static_cast<std::size_t>((m * 2 + 1) * taps + right_delay)] = right_gain;
        }
        return write_sofa(path, measurements, taps, sample_rate, ir.data(), pos.data());
    }

    // Renders a mono tone at a head-relative direction through the spatializer and returns the
    // per-ear energy of the binaural decode.
    void render_direction(BinauralSpatializer& spat, float front, float left, float up,
                          double& left_energy, double& right_energy)
    {
        const int block = 512;
        std::vector<float> mono(static_cast<std::size_t>(block), 0.0f);
        for (int i = 0; i < block; ++i)
            mono[static_cast<std::size_t>(i)] =
                0.5f * static_cast<float>(std::sin(2.0 * 3.14159265 * 440.0 * i / 48000.0));

        left_energy = 0.0;
        right_energy = 0.0;
        std::vector<float> l(static_cast<std::size_t>(block), 0.0f);
        std::vector<float> r(static_cast<std::size_t>(block), 0.0f);
        for (int b = 0; b < 8; ++b) // let the HRIR tail settle across blocks
        {
            for (int i = 0; i < block; ++i)
            {
                l[static_cast<std::size_t>(i)] = 0.0f;
                r[static_cast<std::size_t>(i)] = 0.0f;
            }
            spat.begin_block(block);
            spat.encode(mono.data(), block, front, left, up, 1.0f);
            spat.decode_binaural(l.data(), r.data(), block);
            if (b >= 4) // measure after warm-up
            {
                for (int i = 0; i < block; ++i)
                {
                    left_energy += static_cast<double>(l[static_cast<std::size_t>(i)]) *
                                   l[static_cast<std::size_t>(i)];
                    right_energy += static_cast<double>(r[static_cast<std::size_t>(i)]) *
                                    r[static_cast<std::size_t>(i)];
                }
            }
        }
    }
} // namespace

int main()
{
    const double sample_rate = 48000.0;

    SofaHRTFDatabase db;
    const std::string real_path = "assets/hrtf/mit_kemar_normal_pinna.sofa";
    bool loaded = db.load(real_path, sample_rate);
    if (loaded && db.valid())
    {
        std::printf("loaded real SOFA: %s\n", real_path.c_str());
    }
    else
    {
        const std::string synth = "assets/hrtf/synthetic_ring.sofa";
        if (!write_synthetic_sofa(synth))
        {
            std::fprintf(stderr, "audio_sofa_demo FAILED: could not write synthetic SOFA\n");
            return 1;
        }
        loaded = db.load(synth, sample_rate);
        std::printf("real SOFA absent — baked and loaded synthetic set: %s\n", synth.c_str());
    }

    if (!db.valid())
    {
        std::fprintf(stderr, "audio_sofa_demo FAILED: SOFA load produced no measurements\n");
        return 1;
    }
    std::printf("SOFA: %d measurements, %d taps, %.0f Hz\n", db.measurement_count(),
                db.ir_length(), db.sample_rate());

    // Direct HRIR laterality: a left source must energize the left ear more than the right.
    {
        std::vector<float> lir(static_cast<std::size_t>(db.ir_length()), 0.0f);
        std::vector<float> rir(static_cast<std::size_t>(db.ir_length()), 0.0f);
        db.get_hrir(0.0f, 1.0f, 0.0f, lir.data(), rir.data());
        double le = 0.0, re = 0.0;
        for (int t = 0; t < db.ir_length(); ++t)
        {
            le += static_cast<double>(lir[static_cast<std::size_t>(t)]) * lir[static_cast<std::size_t>(t)];
            re += static_cast<double>(rir[static_cast<std::size_t>(t)]) * rir[static_cast<std::size_t>(t)];
        }
        std::printf("left-source HRIR energy: left ear=%.5f right ear=%.5f\n", le, re);
        if (!(le > re))
        {
            std::fprintf(stderr, "audio_sofa_demo FAILED: HRIR laterality wrong (le %.5f re %.5f)\n",
                         le, re);
            return 1;
        }
    }

    // Full path: the spatializer's measured binaural decode must preserve laterality.
    BinauralSpatializer spat;
    spat.configure(3, sample_rate, 512);
    spat.set_hrtf_database(&db);
    if (!spat.uses_measured_hrtf())
    {
        std::fprintf(stderr, "audio_sofa_demo FAILED: spatializer did not take the database\n");
        return 1;
    }

    double left_l = 0.0, left_r = 0.0, right_l = 0.0, right_r = 0.0;
    render_direction(spat, 0.0f, 1.0f, 0.0f, left_l, left_r);  // source to the left
    render_direction(spat, 0.0f, -1.0f, 0.0f, right_l, right_r); // source to the right
    std::printf("left source  -> ears L=%.4f R=%.4f\n", left_l, left_r);
    std::printf("right source -> ears L=%.4f R=%.4f\n", right_l, right_r);

    if (!(left_l > left_r) || !(right_r > right_l))
    {
        std::fprintf(stderr, "audio_sofa_demo FAILED: binaural decode did not preserve laterality\n");
        return 1;
    }

    std::printf("audio_sofa_demo OK\n");
    return 0;
}
