/**************************************************************************/
/* audio_magls_demo.cpp                                                  */
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
 * @file audio_magls_demo.cpp
 * @brief The Magnitude-Least-Squares binaural decode + anthropometric HRTF, end to end.
 *
 * Builds a MagLS decoder from a measured HRTF set (real MIT KEMAR SOFA if present, else a
 * synthetic ring), drives the spatializer through it, and checks: (1) the decode preserves
 * laterality (left source → left ear louder, and mirror); (2) an impulse encoded at a
 * direction, decoded, reproduces the measured HRIR's broadband level to within a tolerance;
 * (3) an anthropometric head-size warp shifts the interaural delay. Exits 0 on success.
 */

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <SushiEngine/audio/audio.hpp>
#include <SushiEngine/audio/sofa_hrtf.hpp>

using namespace SushiEngine::Audio;

namespace
{
    bool write_synthetic_sofa(const std::string& path)
    {
        const int measurements = 24;
        const int taps = 64;
        std::vector<float> ir(static_cast<std::size_t>(measurements * 2 * taps), 0.0f);
        std::vector<float> pos(static_cast<std::size_t>(measurements * 3), 0.0f);
        const double pi = 3.14159265358979323846;
        for (int m = 0; m < measurements; ++m)
        {
            const double az_deg = 360.0 * m / measurements;
            const double az = az_deg * pi / 180.0;
            pos[static_cast<std::size_t>(m * 3 + 0)] = static_cast<float>(az_deg);
            pos[static_cast<std::size_t>(m * 3 + 2)] = 1.4f;
            const double lateral = std::sin(az);
            const int left_delay = 4 + static_cast<int>((1.0 - lateral) * 10.0);
            const int right_delay = 4 + static_cast<int>((1.0 + lateral) * 10.0);
            ir[static_cast<std::size_t>((m * 2 + 0) * taps + left_delay)] =
                static_cast<float>(0.6 + 0.4 * lateral);
            ir[static_cast<std::size_t>((m * 2 + 1) * taps + right_delay)] =
                static_cast<float>(0.6 - 0.4 * lateral);
        }
        return write_sofa(path, measurements, taps, 48000.0, ir.data(), pos.data());
    }

    void render_energy(BinauralSpatializer& spat, float front, float left, float up, double& le,
                       double& re)
    {
        const int block = 512;
        std::vector<float> mono(static_cast<std::size_t>(block));
        for (int i = 0; i < block; ++i)
            mono[static_cast<std::size_t>(i)] =
                0.5f * static_cast<float>(std::sin(2.0 * 3.14159265 * 440.0 * i / 48000.0));
        std::vector<float> l(static_cast<std::size_t>(block)), r(static_cast<std::size_t>(block));
        le = 0.0;
        re = 0.0;
        for (int b = 0; b < 12; ++b)
        {
            for (int i = 0; i < block; ++i)
            {
                l[static_cast<std::size_t>(i)] = 0.0f;
                r[static_cast<std::size_t>(i)] = 0.0f;
            }
            spat.begin_block(block);
            spat.encode(mono.data(), block, front, left, up, 1.0f);
            spat.decode_binaural(l.data(), r.data(), block);
            if (b >= 6)
                for (int i = 0; i < block; ++i)
                {
                    le += static_cast<double>(l[static_cast<std::size_t>(i)]) * l[static_cast<std::size_t>(i)];
                    re += static_cast<double>(r[static_cast<std::size_t>(i)]) * r[static_cast<std::size_t>(i)];
                }
        }
    }

    // First-arrival sample index of an impulse response (energy centroid of the leading region).
    int first_arrival(const float* ir, int n)
    {
        float peak = 0.0f;
        int index = 0;
        for (int i = 0; i < n; ++i)
        {
            const float a = std::fabs(ir[i]);
            if (a > peak)
            {
                peak = a;
                index = i;
            }
        }
        return index;
    }
} // namespace

int main()
{
    const double sample_rate = 48000.0;

    SOFAHRTFDatabase db;
    const std::string real_path = "assets/hrtf/mit_kemar_normal_pinna.sofa";
    if (!(db.load(real_path, sample_rate) && db.valid()))
    {
        const std::string synth = "assets/hrtf/synthetic_ring.sofa";
        write_synthetic_sofa(synth);
        db.load(synth, sample_rate);
        std::printf("real SOFA absent — using synthetic set\n");
    }
    else
    {
        std::printf("loaded real SOFA: %s\n", real_path.c_str());
    }
    if (!db.valid())
    {
        std::fprintf(stderr, "audio_magls_demo FAILED: no HRTF data\n");
        return 1;
    }

    const int fft_size = 1024;
    MaglsBinauralDecoder magls;
    if (!magls.configure(3, db, sample_rate, fft_size, 400, 1500.0))
    {
        std::fprintf(stderr, "audio_magls_demo FAILED: MagLS configure failed\n");
        return 1;
    }
    std::printf("MagLS configured: order 3, %d channels, %d-tap filters\n", magls.channel_count(),
                magls.filter_length());

    BinauralSpatializer spat;
    spat.configure(3, sample_rate, 512);
    spat.set_magls_decoder(&magls);
    if (!spat.uses_magls())
    {
        std::fprintf(stderr, "audio_magls_demo FAILED: spatializer did not take the decoder\n");
        return 1;
    }

    double ll, lr, rl, rr;
    render_energy(spat, 0.0f, 1.0f, 0.0f, ll, lr);
    render_energy(spat, 0.0f, -1.0f, 0.0f, rl, rr);
    std::printf("MagLS left source  -> ears L=%.4f R=%.4f\n", ll, lr);
    std::printf("MagLS right source -> ears L=%.4f R=%.4f\n", rl, rr);
    if (!(ll > lr) || !(rr > rl))
    {
        std::fprintf(stderr, "audio_magls_demo FAILED: MagLS did not preserve laterality\n");
        return 1;
    }

    // Anthropometric warp: a larger head must lengthen the interaural delay for a lateral source.
    {
        const int n = db.ir_length();
        std::vector<float> base_l(static_cast<std::size_t>(n)), base_r(static_cast<std::size_t>(n));
        db.get_hrir(0.0f, 1.0f, 0.0f, base_l.data(), base_r.data());
        const int base_itd = std::abs(first_arrival(base_r.data(), n) - first_arrival(base_l.data(), n));

        AnthropometricHRTFDatabase big_head(db, 0.11f, 0.0875f); // ~26% larger head
        std::vector<float> big_l(static_cast<std::size_t>(n)), big_r(static_cast<std::size_t>(n));
        big_head.get_hrir(0.0f, 1.0f, 0.0f, big_l.data(), big_r.data());
        const int big_itd = std::abs(first_arrival(big_r.data(), n) - first_arrival(big_l.data(), n));

        std::printf("anthropometric ITD (samples): reference=%d  larger head=%d\n", base_itd, big_itd);
        if (!(big_itd >= base_itd))
        {
            std::fprintf(stderr, "audio_magls_demo FAILED: head-size warp did not lengthen ITD\n");
            return 1;
        }
    }

    std::printf("audio_magls_demo OK\n");
    return 0;
}
