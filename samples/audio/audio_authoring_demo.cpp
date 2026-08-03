/**************************************************************************/
/* audio_authoring_demo.cpp                                             */
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
 * @file audio_authoring_demo.cpp
 * @brief The audio authoring project — the DAW backend — author → flatten → bake → resolve.
 *
 * Builds a project the way the editor's authoring panel would: three media, a weighted Random
 * container, a Blend, and a Layer, rooted by three named events. Then it flattens the mutable
 * tree to a runtime EventDatabase and checks the container semantics survive the flatten (Random
 * stays within its media, Blend endpoints pick the straddled child, Layer plays all children),
 * and finally bakes a real Bank and resolves an event through it. Exits 0 on success.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    // Encodes a constant-valued float buffer as PcmFloat bytes (the codec stores float directly).
    std::vector<std::uint8_t> pcm_float_bytes(float value, int frames)
    {
        std::vector<float> f(static_cast<std::size_t>(frames), value);
        std::vector<std::uint8_t> bytes(f.size() * sizeof(float));
        std::memcpy(bytes.data(), f.data(), bytes.size());
        return bytes;
    }
} // namespace

int main()
{
    AudioAuthoringProject project;

    const std::uint32_t m0 = project.add_media("kick", AudioCodecKind::PcmFloat, 1, 48000, 256);
    const std::uint32_t m1 = project.add_media("snare", AudioCodecKind::PcmFloat, 1, 48000, 256);
    const std::uint32_t m2 = project.add_media("hat", AudioCodecKind::PcmFloat, 1, 48000, 256);

    const int s0 = project.create_sound(m0);
    const int s1 = project.create_sound(m1);
    const int s2 = project.create_sound(m2);

    // A weighted Random of all three.
    const int random = project.create_container(ContainerKind::Random);
    project.add_child(random, s0);
    project.add_child(random, s1);
    project.add_child(random, s2);
    project.set_weight(s0, 1.0f);
    project.set_weight(s1, 2.0f);
    project.set_weight(s2, 1.0f);

    // A Blend between kick (m0) and hat (m2).
    const int blend = project.create_container(ContainerKind::Blend);
    project.add_child(blend, project.create_sound(m0));
    project.add_child(blend, project.create_sound(m2));

    // A Layer of kick + snare (both play).
    const int layer = project.create_container(ContainerKind::Layer);
    project.add_child(layer, project.create_sound(m0));
    project.add_child(layer, project.create_sound(m1));

    const EventId ev_footstep = project.create_event("footstep", random);
    const EventId ev_music = project.create_event("music", blend);
    const EventId ev_stinger = project.create_event("stinger", layer);
    (void)s0;
    (void)s1;
    (void)s2;

    // Flatten the mutable tree into the runtime database.
    EventDatabase db;
    if (!project.flatten(db))
    {
        std::fprintf(stderr, "audio_authoring_demo FAILED: flatten failed\n");
        return 1;
    }
    std::printf("flattened %zu authored nodes -> %zu runtime nodes, %zu events\n",
                project.nodes().size(), db.nodes().size(), project.events().size());

    // Random must resolve to one of the three media.
    {
        bool all_valid = true;
        for (std::uint32_t seed = 0; seed < 50; ++seed)
        {
            ResolveContext context;
            context.seed = seed;
            const std::uint32_t media = db.resolve(ev_footstep, context);
            if (media != m0 && media != m1 && media != m2)
                all_valid = false;
        }
        if (!all_valid)
        {
            std::fprintf(stderr, "audio_authoring_demo FAILED: Random resolved outside its media\n");
            return 1;
        }
        std::printf("Random event resolves within {kick,snare,hat} across 50 seeds\n");
    }

    // Blend endpoints must pick the straddled child.
    {
        ResolveContext lo;
        lo.blend = 0.0f;
        ResolveContext hi;
        hi.blend = 1.0f;
        const std::uint32_t at0 = db.resolve(ev_music, lo);
        const std::uint32_t at1 = db.resolve(ev_music, hi);
        std::printf("Blend at 0.0 -> media %u (kick=%u), at 1.0 -> media %u (hat=%u)\n", at0, m0,
                    at1, m2);
        if (at0 != m0 || at1 != m2)
        {
            std::fprintf(stderr, "audio_authoring_demo FAILED: Blend endpoints wrong\n");
            return 1;
        }
    }

    // Layer must resolve to all children.
    {
        ResolveContext context;
        std::vector<ResolvedSound> sounds;
        db.resolve_all(ev_stinger, context, sounds);
        std::printf("Layer event resolved %zu simultaneous sounds\n", sounds.size());
        if (sounds.size() != 2)
        {
            std::fprintf(stderr, "audio_authoring_demo FAILED: Layer did not play all children\n");
            return 1;
        }
    }

    // Bake a real bank and resolve+decode through it.
    {
        BankBuilder builder;
        const bool ok = project.bake(builder, [&](std::uint32_t id) -> const std::vector<std::uint8_t>& {
            static std::vector<std::uint8_t> storage;
            storage = pcm_float_bytes(0.25f + 0.1f * id, 256);
            return storage;
        });
        if (!ok)
        {
            std::fprintf(stderr, "audio_authoring_demo FAILED: bake failed\n");
            return 1;
        }
        const std::vector<std::uint8_t> blob = builder.build();
        Bank bank;
        if (!bank.load(blob.data(), blob.size()))
        {
            std::fprintf(stderr, "audio_authoring_demo FAILED: bank load failed\n");
            return 1;
        }
        std::vector<float> samples;
        const bool decoded = bank.decode_media(m1, samples);
        std::printf("baked bank: %zu bytes, decode media %u -> %zu samples\n", blob.size(), m1,
                    samples.size());
        if (!decoded || samples.empty())
        {
            std::fprintf(stderr, "audio_authoring_demo FAILED: bank media decode failed\n");
            return 1;
        }
    }

    std::printf("audio_authoring_demo OK\n");
    return 0;
}
