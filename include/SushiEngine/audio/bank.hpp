/**************************************************************************/
/* bank.hpp                                                              */
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

#ifndef SUSHIENGINE_AUDIO_BANK_HPP
#define SUSHIENGINE_AUDIO_BANK_HPP

/**
 * @file bank.hpp
 * @brief The sound bank: authored media + the event/container tree baked into one
 *        compact binary, and the factory that turns a posted event into a live voice.
 *
 * A bank is the unit of authored audio a game loads (§10 of `docs/slop/audio_system.md`):
 * a flat, self-describing binary — a small header, a media table (each sound's codec,
 * format, and where its bytes live), the flattened event/container definitions
 * (`event.hpp`), and one blob of concatenated encoded media. @ref BankBuilder writes one;
 * @ref Bank loads one from a byte buffer (it owns a copy, so the source buffer can go
 * away) and answers media lookups and decodes.
 *
 * @ref BankSourceFactory closes the loop opened at S6: it implements the
 * @ref IEmitterSourceFactory seam the `AudioScene` calls, resolving an emitter's `sound`
 * id — an **event** id — through the bank's @ref EventDatabase to a media id, decoding it
 * (once, cached) and handing back a @ref BufferSource. A resident-decode path today; the
 * streaming path (`streaming.hpp`) plugs in behind the same factory for long assets.
 *
 * The format is little-endian and versioned; all offsets are byte offsets from the blob
 * start. Portable, no SDL and no SushiRuntime — a bank round-trips in a unit test.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include <SushiEngine/audio/audio_scene.hpp>
#include <SushiEngine/audio/codec.hpp>
#include <SushiEngine/audio/event.hpp>
#include <SushiEngine/audio/voice.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief One sound's entry in a bank's media table. */
        struct BankMediaInfo
        {
            std::uint32_t id = 0;
            AudioCodecKind codec = AudioCodecKind::PcmFloat;
            std::uint32_t channels = 1;
            std::uint32_t sample_rate = 48000;
            std::uint32_t frame_count = 0;
            const std::uint8_t* data = nullptr; /**< Encoded bytes (into the bank's blob). */
            std::uint32_t data_size = 0;
        };

        /**
         * @brief Factory for dependency-gated codecs the header-only bank cannot construct itself.
         *
         * Codecs whose implementation pulls a third-party library (e.g. @ref AudioCodecKind::Opus
         * via libopus) live outside `audio.hpp`, so the bank cannot `new` them directly. A consumer
         * that links such a codec registers a factory through @ref set_external_codec_factory; the
         * bank consults it for any kind its built-in switch does not handle. Returns null to decline.
         */
        using ExternalCodecFactory = std::unique_ptr<IAudioCodec> (*)(AudioCodecKind kind,
                                                                      int channels, int sample_rate);

        /** @brief The process-wide registry of external codec factories (empty until registered). */
        inline std::vector<ExternalCodecFactory>& external_codec_factories()
        {
            static std::vector<ExternalCodecFactory> factories;
            return factories;
        }

        /**
         * @brief Adds a factory the bank consults for dependency-gated codecs.
         *
         * Multiple factories coexist — each is tried in turn until one returns non-null — so an
         * Opus TU and a Vorbis TU can both register without clobbering each other.
         *
         * @param factory Callable producing a codec for a given kind/channels/sample rate, or null.
         */
        inline void add_external_codec_factory(ExternalCodecFactory factory)
        {
            if (factory != nullptr)
                external_codec_factories().push_back(factory);
        }

        /**
         * @brief Replaces the whole registry with a single factory (back-compatible setter).
         * @param factory The sole factory, or null to clear the registry.
         */
        inline void set_external_codec_factory(ExternalCodecFactory factory)
        {
            external_codec_factories().clear();
            add_external_codec_factory(factory);
        }

        /**
         * @brief Builds a codec instance for a media entry's wire format.
         * @param kind        The codec kind recorded in the bank.
         * @param channels    Channel count.
         * @param sample_rate Encode sample rate (needed by codecs like Opus).
         * @return A codec, falling back to float PCM if an external kind has no registered factory.
         */
        inline std::unique_ptr<IAudioCodec> make_codec(AudioCodecKind kind, int channels,
                                                       int sample_rate = 48000)
        {
            switch (kind)
            {
                case AudioCodecKind::Pcm16:
                    return std::unique_ptr<IAudioCodec>(new PcmCodec(channels, true));
                case AudioCodecKind::ImaAdpcm:
                    return std::unique_ptr<IAudioCodec>(new ImaAdpcmCodec(channels));
                case AudioCodecKind::PcmFloat:
                    return std::unique_ptr<IAudioCodec>(new PcmCodec(channels, false));
                default:
                    for (ExternalCodecFactory factory : external_codec_factories())
                    {
                        std::unique_ptr<IAudioCodec> codec = factory(kind, channels, sample_rate);
                        if (codec)
                            return codec;
                    }
                    return std::unique_ptr<IAudioCodec>(new PcmCodec(channels, false));
            }
        }

        namespace Detail
        {
            inline void write_u32(std::vector<std::uint8_t>& out, std::uint32_t v)
            {
                out.push_back(static_cast<std::uint8_t>(v & 0xff));
                out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xff));
                out.push_back(static_cast<std::uint8_t>((v >> 16) & 0xff));
                out.push_back(static_cast<std::uint8_t>((v >> 24) & 0xff));
            }

            inline std::uint32_t read_u32(const std::uint8_t* p) noexcept
            {
                return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
                       (static_cast<std::uint32_t>(p[2]) << 16) |
                       (static_cast<std::uint32_t>(p[3]) << 24);
            }
        } // namespace Detail

        /** @brief The bank magic ('SBNK') and format version. */
        constexpr std::uint32_t BANK_MAGIC = 0x4b4e4253u; // 'S','B','N','K' little-endian
        constexpr std::uint32_t BANK_VERSION = 1u;

        /**
         * @brief Accumulates media and events, then serialises one bank binary.
         *
         * Add each sound's already-encoded bytes with @ref add_media (encode with the
         * matching codec — e.g. @ref ImaAdpcmCodec::encode), optionally attach an
         * @ref EventDatabase with @ref set_events, then @ref build the byte vector.
         */
        class BankBuilder
        {
            public:
                /**
                 * @brief Adds one encoded sound to the bank.
                 * @param id          The media id emitters/events reference.
                 * @param codec       The wire format @p encoded is in.
                 * @param channels    Channel count.
                 * @param sample_rate Sample rate in Hz.
                 * @param frame_count Decoded frame count (per channel).
                 * @param encoded     The encoded bytes (copied into the blob).
                 */
                void add_media(std::uint32_t id, AudioCodecKind codec, std::uint32_t channels,
                               std::uint32_t sample_rate, std::uint32_t frame_count,
                               const std::vector<std::uint8_t>& encoded)
                {
                    Entry e;
                    e.id = id;
                    e.codec = codec;
                    e.channels = channels;
                    e.sample_rate = sample_rate;
                    e.frame_count = frame_count;
                    e.offset = static_cast<std::uint32_t>(blob_.size());
                    e.size = static_cast<std::uint32_t>(encoded.size());
                    entries_.push_back(e);
                    blob_.insert(blob_.end(), encoded.begin(), encoded.end());
                }

                /** @brief Bakes an event/container database into the bank. */
                void set_events(const EventDatabase& events)
                {
                    nodes_ = events.nodes();
                    events_ = events.events();
                }

                /** @brief Serialises the accumulated bank to a byte vector. */
                std::vector<std::uint8_t> build() const
                {
                    std::vector<std::uint8_t> out;
                    Detail::write_u32(out, BANK_MAGIC);
                    Detail::write_u32(out, BANK_VERSION);
                    Detail::write_u32(out, static_cast<std::uint32_t>(entries_.size()));
                    Detail::write_u32(out, static_cast<std::uint32_t>(nodes_.size()));
                    Detail::write_u32(out, static_cast<std::uint32_t>(events_.size()));
                    Detail::write_u32(out, static_cast<std::uint32_t>(blob_.size()));

                    for (const Entry& e : entries_)
                    {
                        Detail::write_u32(out, e.id);
                        Detail::write_u32(out, static_cast<std::uint32_t>(e.codec));
                        Detail::write_u32(out, e.channels);
                        Detail::write_u32(out, e.sample_rate);
                        Detail::write_u32(out, e.frame_count);
                        Detail::write_u32(out, e.offset);
                        Detail::write_u32(out, e.size);
                    }
                    for (const ContainerNode& n : nodes_)
                    {
                        Detail::write_u32(out, static_cast<std::uint32_t>(n.kind));
                        Detail::write_u32(out, n.sound);
                        Detail::write_u32(out, n.first_child);
                        Detail::write_u32(out, n.child_count);
                    }
                    for (const EventDef& ev : events_)
                    {
                        Detail::write_u32(out, ev.id);
                        Detail::write_u32(out, ev.root);
                    }
                    out.insert(out.end(), blob_.begin(), blob_.end());
                    return out;
                }

            private:
                struct Entry
                {
                    std::uint32_t id;
                    AudioCodecKind codec;
                    std::uint32_t channels;
                    std::uint32_t sample_rate;
                    std::uint32_t frame_count;
                    std::uint32_t offset;
                    std::uint32_t size;
                };
                std::vector<Entry> entries_;
                std::vector<ContainerNode> nodes_;
                std::vector<EventDef> events_;
                std::vector<std::uint8_t> blob_;
        };

        /**
         * @brief A loaded bank: media lookups, event resolution, and resident decode.
         *
         * @ref load parses a bank binary into an owned copy (so the source buffer may be
         * freed). @ref find_media resolves an id to its @ref BankMediaInfo (its bytes point
         * into the owned blob); @ref decode_media decodes one to interleaved float; and
         * @ref events returns the baked @ref EventDatabase.
         */
        class Bank
        {
            public:
                /**
                 * @brief Parses a bank binary into this object (owning a copy).
                 * @param data The bank bytes.
                 * @param size Byte count.
                 * @return True on a valid, complete bank.
                 */
                bool load(const std::uint8_t* data, std::size_t size)
                {
                    media_.clear();
                    index_.clear();
                    blob_.clear();
                    events_.assign({}, {});

                    if (size < 24 || Detail::read_u32(data) != BANK_MAGIC)
                        return false;
                    if (Detail::read_u32(data + 4) != BANK_VERSION)
                        return false;
                    const std::uint32_t media_count = Detail::read_u32(data + 8);
                    const std::uint32_t node_count = Detail::read_u32(data + 12);
                    const std::uint32_t event_count = Detail::read_u32(data + 16);
                    const std::uint32_t blob_size = Detail::read_u32(data + 20);

                    std::size_t off = 24;
                    const std::size_t media_bytes = static_cast<std::size_t>(media_count) * 28;
                    const std::size_t node_bytes = static_cast<std::size_t>(node_count) * 16;
                    const std::size_t event_bytes = static_cast<std::size_t>(event_count) * 8;
                    if (off + media_bytes + node_bytes + event_bytes + blob_size > size)
                        return false;

                    // Own the blob first so media pointers stay valid for this bank's life.
                    const std::size_t blob_off = off + media_bytes + node_bytes + event_bytes;
                    blob_.assign(data + blob_off, data + blob_off + blob_size);

                    for (std::uint32_t i = 0; i < media_count; ++i)
                    {
                        const std::uint8_t* p = data + off + static_cast<std::size_t>(i) * 28;
                        BankMediaInfo m;
                        m.id = Detail::read_u32(p);
                        m.codec = static_cast<AudioCodecKind>(Detail::read_u32(p + 4));
                        m.channels = Detail::read_u32(p + 8);
                        m.sample_rate = Detail::read_u32(p + 12);
                        m.frame_count = Detail::read_u32(p + 16);
                        const std::uint32_t blob_offset = Detail::read_u32(p + 20);
                        m.data_size = Detail::read_u32(p + 24);
                        if (static_cast<std::size_t>(blob_offset) + m.data_size > blob_.size())
                            return false;
                        m.data = blob_.data() + blob_offset;
                        index_[m.id] = media_.size();
                        media_.push_back(m);
                    }
                    off += media_bytes;

                    std::vector<ContainerNode> nodes(node_count);
                    for (std::uint32_t i = 0; i < node_count; ++i)
                    {
                        const std::uint8_t* p = data + off + static_cast<std::size_t>(i) * 16;
                        nodes[i].kind = static_cast<ContainerKind>(Detail::read_u32(p));
                        nodes[i].sound = Detail::read_u32(p + 4);
                        nodes[i].first_child = Detail::read_u32(p + 8);
                        nodes[i].child_count = Detail::read_u32(p + 12);
                    }
                    off += node_bytes;

                    std::vector<EventDef> events(event_count);
                    for (std::uint32_t i = 0; i < event_count; ++i)
                    {
                        const std::uint8_t* p = data + off + static_cast<std::size_t>(i) * 8;
                        events[i].id = Detail::read_u32(p);
                        events[i].root = Detail::read_u32(p + 4);
                    }
                    events_.assign(std::move(nodes), std::move(events));
                    return true;
                }

                /** @brief Convenience overload loading from a byte vector. */
                bool load(const std::vector<std::uint8_t>& bytes)
                {
                    return load(bytes.data(), bytes.size());
                }

                /** @brief The number of media entries. */
                std::size_t media_count() const noexcept { return media_.size(); }

                /**
                 * @brief Looks up a media entry by id.
                 * @param id  The media id.
                 * @param out Filled on success.
                 * @return True if found.
                 */
                bool find_media(std::uint32_t id, BankMediaInfo& out) const
                {
                    auto it = index_.find(id);
                    if (it == index_.end())
                        return false;
                    out = media_[it->second];
                    return true;
                }

                /**
                 * @brief Decodes a media entry to interleaved float.
                 * @param id  The media id.
                 * @param out Filled with samples (cleared; empty if not found).
                 * @return True if the media was found and decoded.
                 */
                bool decode_media(std::uint32_t id, std::vector<float>& out) const
                {
                    out.clear();
                    BankMediaInfo m;
                    if (!find_media(id, m))
                        return false;
                    std::unique_ptr<IAudioCodec> codec = make_codec(
                        m.codec, static_cast<int>(m.channels), static_cast<int>(m.sample_rate));
                    decode_all(*codec, m.data, static_cast<int>(m.data_size), out);
                    return true;
                }

                /** @brief The baked event/container database. */
                EventDatabase& events() noexcept { return events_; }
                /** @brief The baked event/container database (const). */
                const EventDatabase& events() const noexcept { return events_; }

            private:
                std::vector<BankMediaInfo> media_;
                std::unordered_map<std::uint32_t, std::size_t> index_;
                std::vector<std::uint8_t> blob_;
                EventDatabase events_;
        };

        /**
         * @brief An @ref IEmitterSourceFactory backed by a bank (resident decode).
         *
         * The concrete factory the S6 `AudioScene` was built to accept. `create(id)` treats
         * the id as an **event** (resolving it through the bank's @ref EventDatabase to a
         * media id); if the bank has no events, the id is taken as a direct media id. The
         * decoded samples are cached (decode once, share across every voice), and a
         * non-owning @ref BufferSource plays from the cache — which the factory outlives.
         *
         * Blend/Switch selection reads the factory-level @ref set_blend / @ref set_switch
         * state; every `create` bumps a seed so Random containers vary. Per-emitter RTPC →
         * blend wiring is a later refinement (the seam only carries the id).
         */
        class BankSourceFactory final : public IEmitterSourceFactory
        {
            public:
                /**
                 * @brief Builds the factory over a bank.
                 * @param bank The bank to resolve and decode from (borrowed; must outlive this).
                 * @param loop Whether spawned voices loop their buffer (music vs one-shot SFX).
                 */
                explicit BankSourceFactory(Bank& bank, bool loop = false) noexcept
                    : bank_(bank), loop_(loop)
                {
                }

                /** @brief Sets the Blend-container parameter used by subsequent resolves. */
                void set_blend(float blend) noexcept { context_.blend = blend; }
                /** @brief Sets the Switch-container selector used by subsequent resolves. */
                void set_switch(std::uint32_t value) noexcept { context_.switch_value = value; }

                std::unique_ptr<VoiceSource> create(std::uint32_t sound_id) override
                {
                    if (bank_.events().empty())
                        return make_buffer(sound_id); // no event tree: id is a direct media id

                    context_.seed = seed_++;
                    resolved_.clear();
                    bank_.events().resolve_all(sound_id, context_, resolved_);
                    if (resolved_.empty())
                        return make_buffer(sound_id); // unknown event: fall back to a direct id

                    // A single unity-gain sound plays directly; anything with layers or a
                    // non-unit gain becomes a layered voice that applies the per-sound gains.
                    if (resolved_.size() == 1 && std::fabs(resolved_[0].gain - 1.0f) < 1.0e-3f)
                        return make_buffer(resolved_[0].media_id);

                    std::unique_ptr<LayeredVoiceSource> layered(new LayeredVoiceSource());
                    for (const ResolvedSound& r : resolved_)
                    {
                        std::unique_ptr<VoiceSource> child = make_buffer(r.media_id);
                        if (child)
                            layered->add(std::move(child), r.gain);
                    }
                    if (layered->layer_count() == 0)
                        return nullptr;
                    return std::unique_ptr<VoiceSource>(std::move(layered));
                }

            private:
                /** @brief A non-owning buffer source over a media id's decoded, cached samples. */
                std::unique_ptr<VoiceSource> make_buffer(std::uint32_t media_id)
                {
                    const std::vector<float>* buffer = decoded(media_id);
                    if (buffer == nullptr || buffer->empty())
                        return nullptr;
                    double source_rate = 0.0;
                    BankMediaInfo info;
                    if (bank_.find_media(media_id, info))
                        source_rate = static_cast<double>(info.sample_rate);
                    return std::unique_ptr<VoiceSource>(new BufferSource(
                        buffer->data(), static_cast<int>(buffer->size()), loop_, source_rate));
                }

                const std::vector<float>* decoded(std::uint32_t media_id)
                {
                    auto it = cache_.find(media_id);
                    if (it != cache_.end())
                        return &it->second;
                    std::vector<float> samples;
                    if (!bank_.decode_media(media_id, samples))
                        return nullptr;
                    auto res = cache_.emplace(media_id, std::move(samples));
                    return &res.first->second;
                }

                Bank& bank_;
                bool loop_;
                ResolveContext context_;
                std::uint32_t seed_ = 1;
                std::unordered_map<std::uint32_t, std::vector<float>> cache_;
                std::vector<ResolvedSound> resolved_;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
