/**************************************************************************/
/* audio_accelerator_demo.cpp                                           */
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
 * @file audio_accelerator_demo.cpp
 * @brief Phase S10 vertical slice: the SushiRuntime SYCL DSP accelerator (§12.2).
 *
 * The only place the runtime enters the audio subsystem: batch DSP offloaded to the SYCL
 * device with k-block lookahead. This is a SYCL translation unit (it includes
 * `accelerator_sycl.hpp`). It:
 *
 *   1. Creates a runtime, builds a @ref SyclDspAccelerator, uploads an FIR, and runs a
 *      pipelined convolution over a test signal — submitting block *t* while collecting
 *      block *t−K* — then checks every collected block against a CPU reference convolution.
 *   2. Reports whether the device path was available; if no device is present it says so
 *      and still exits 0 (the CPU path is what ships everywhere until a GPU is wired).
 *
 * Exits 0 on success (matching output, or a clean no-device fallback).
 */

#include <cmath>
#include <cstdio>
#include <vector>

#include <SushiEngine/audio/accelerator_sycl.hpp>

using namespace SushiEngine::Audio;

int main()
{
    const int block = 256;
    const int ir_len = 64;
    const int lookahead = 3;
    const int blocks = 32;

    // A decaying FIR and a test signal.
    std::vector<float> ir(static_cast<std::size_t>(ir_len));
    for (int i = 0; i < ir_len; ++i)
        ir[static_cast<std::size_t>(i)] = std::exp(-i * 0.08f) * std::cos(i * 0.4f);

    const int total = block * blocks;
    std::vector<float> signal(static_cast<std::size_t>(total));
    for (int i = 0; i < total; ++i)
        signal[static_cast<std::size_t>(i)] = std::sin(i * 0.05f) + 0.3f * std::sin(i * 0.21f);

    // CPU reference convolution of the whole signal (causal FIR).
    std::vector<float> reference(static_cast<std::size_t>(total), 0.0f);
    for (int n = 0; n < total; ++n)
    {
        float acc = 0.0f;
        for (int k = 0; k < ir_len; ++k)
        {
            const int idx = n - k;
            if (idx >= 0)
                acc += signal[static_cast<std::size_t>(idx)] * ir[static_cast<std::size_t>(k)];
        }
        reference[static_cast<std::size_t>(n)] = acc;
    }

    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    SyclDspAccelerator accel(runtime, block, ir_len, lookahead);

    if (!accel.available())
    {
        std::printf("audio_accelerator_demo: no SYCL device available — CPU path is used "
                    "(this is the shipping default). OK\n");
        return 0;
    }
    accel.set_impulse(ir.data(), ir_len);
    std::printf("accelerator available: block=%d ir=%d lookahead=%d history=%d\n", block, ir_len,
                accel.lookahead(), accel.history());

    const int hist = accel.history();
    std::vector<float> padded(static_cast<std::size_t>(hist + block));
    std::vector<float> out(static_cast<std::size_t>(block));

    double max_err = 0.0;
    int checked = 0;
    // Pipeline: submit block t, collect block t−lookahead (already finished on the device).
    for (int t = 0; t < blocks + lookahead; ++t)
    {
        if (t < blocks)
        {
            // Build the padded input: `hist` samples of history then this block.
            const int start = t * block;
            for (int i = 0; i < hist; ++i)
            {
                const int idx = start - hist + i;
                padded[static_cast<std::size_t>(i)] =
                    idx >= 0 ? signal[static_cast<std::size_t>(idx)] : 0.0f;
            }
            for (int i = 0; i < block; ++i)
                padded[static_cast<std::size_t>(hist + i)] =
                    signal[static_cast<std::size_t>(start + i)];
            accel.submit(t, padded.data());
        }

        const int collect_block = t - lookahead;
        if (collect_block >= 0)
        {
            accel.collect(collect_block, out.data());
            for (int i = 0; i < block; ++i)
            {
                const double d = std::fabs(static_cast<double>(out[static_cast<std::size_t>(i)]) -
                                           reference[static_cast<std::size_t>(collect_block * block + i)]);
                if (d > max_err)
                    max_err = d;
                ++checked;
            }
        }
    }

    std::printf("pipelined GPU convolution vs CPU reference: max error=%.3e over %d samples\n",
                max_err, checked);
    if (max_err > 1.0e-3 || checked != total)
    {
        std::fprintf(stderr, "audio_accelerator_demo FAILED: offload result != reference "
                             "(err=%.3e, checked=%d/%d)\n",
                     max_err, checked, total);
        return 1;
    }

    std::printf("audio_accelerator_demo OK\n");
    return 0;
}
