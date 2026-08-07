/**************************************************************************/
/* audio_editor_system.cpp                                                */
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

#include "audio_editor_system.hpp"

#include <cmath>

namespace SushiEngine
{
    namespace Editor
    {
        using Audio::AudioVec3;

        namespace
        {
            /** @brief Maps a sound id to a distinct, musically-spaced tone frequency. */
            float tone_frequency(std::uint32_t sound_id) noexcept
            {
                // A pentatonic-ish spread over a couple of octaves so successive ids are
                // clearly distinguishable when auditioning without a real bank.
                static const float semitone[8] = {0, 2, 4, 7, 9, 12, 16, 19};
                const float base = 196.0f; // G3
                const float step = semitone[sound_id % 8] + 12.0f * static_cast<float>((sound_id / 8) % 3);
                return base * std::pow(2.0f, step / 12.0f);
            }

            AudioVec3 to_local(const Vector3& world, const Vector3& eye) noexcept
            {
                return AudioVec3{static_cast<float>(world.x - eye.x),
                                 static_cast<float>(world.y - eye.y),
                                 static_cast<float>(world.z - eye.z)};
            }

            Audio::DistanceModel distance_model(std::uint32_t code) noexcept
            {
                switch (code)
                {
                    case 1: return Audio::DistanceModel::Inverse;
                    case 2: return Audio::DistanceModel::Exponent;
                    default: return Audio::DistanceModel::Linear;
                }
            }
        } // namespace

        std::unique_ptr<Audio::VoiceSource> AudioEditorSystem::ToneFactory::create(std::uint32_t sound_id)
        {
            return std::unique_ptr<Audio::VoiceSource>(new Audio::ToneSource(tone_frequency(sound_id), 0.5f));
        }

        Audio::I3DL2Reverb AudioEditorSystem::to_i3dl2(const Simulation::ReverbZoneParameters& p) noexcept
        {
            Audio::I3DL2Reverb r;
            r.room = p.room;
            r.room_hf = p.room_hf;
            r.decay_time = p.decay_time;
            r.decay_hf_ratio = p.decay_hf_ratio;
            r.reflections = p.reflections;
            r.reverb = p.reverb;
            r.diffusion = p.diffusion;
            r.density = p.density;
            r.wet_dry_mix = p.wet_dry_mix;
            return r;
        }

        AudioEditorSystem::AudioEditorSystem()
            : engine_(64, 24), scene_(engine_.voices(), factory_)
        {
            // A small mixer: three source buses folding into the master, plus a parallel
            // reverb aux bus emitters send into (its wet return also routes to master).
            master_bus_ = engine_.mixer().add_bus(Audio::NO_BUS);
            sfx_bus_ = engine_.mixer().add_bus(master_bus_);
            music_bus_ = engine_.mixer().add_bus(master_bus_);
            reverb_bus_ = engine_.mixer().add_bus(master_bus_);
            engine_.mixer().set_master(master_bus_);

            {
                std::unique_ptr<Audio::FDNReverbEffect> fx(new Audio::FDNReverbEffect());
                Audio::I3DL2Reverb hall = Audio::I3DL2Reverb::concert_hall();
                hall.wet_dry_mix = 100.0f; // aux bus: pure wet, the dry goes direct to master
                fx->set_parameters(hall);
                reverb_ = fx.get();
                engine_.mixer().add_insert(
                    reverb_bus_, std::unique_ptr<Audio::IBusEffect>(new Audio::ReverbBusEffect(std::move(fx))));
            }

            engine_.set_ambisonic_order(3);
            engine_.prepare(sample_rate_, block_);
            engine_.voices().set_max_propagation_distance(500.0f);
            scene_.set_reverb(reverb_);
        }

        AudioEditorSystem::~AudioEditorSystem()
        {
            if (device_)
                device_->close();
        }

        bool AudioEditorSystem::set_enabled(bool enabled)
        {
            if (enabled == enabled_)
                return enabled_ == device_open_;
            enabled_ = enabled;
            if (enabled_)
            {
                device_.reset(new SushiEngine::Audio::SDLAudioDevice());
                Audio::AudioStreamFormat desired;
                desired.sample_rate = 48000;
                desired.channel_count = 2;
                desired.block_frames = block_;
                device_open_ = device_->open(desired, engine_);
                if (!device_open_)
                    device_.reset();
            }
            else
            {
                if (device_)
                    device_->close();
                device_.reset();
                device_open_ = false;
            }
            return enabled_ == device_open_;
        }

        void AudioEditorSystem::update(Simulation::IWorldEditor& world, const Vector3& listener_position,
                                       const Vector3& listener_forward, const Vector3& listener_up)
        {
            if (!device_open_)
                return;

            Audio::SceneSnapshot snapshot;
            snapshot.listener_forward = AudioVec3{static_cast<float>(listener_forward.x),
                                                  static_cast<float>(listener_forward.y),
                                                  static_cast<float>(listener_forward.z)};
            snapshot.listener_up = AudioVec3{static_cast<float>(listener_up.x),
                                             static_cast<float>(listener_up.y),
                                             static_cast<float>(listener_up.z)};

            const std::vector<Simulation::EntityId> ids = world.entities();

            // The highest-priority reverb zone the listener stands inside wins.
            bool have_zone = false;
            std::int32_t best_priority = 0;
            for (Simulation::EntityId id : ids)
            {
                // Same gate as the emitter loop below: a disabled entity is off in every
                // system that touches it, and a zone that still coloured the mix would be
                // the one audible trace of a subtree the author switched off.
                if (!world.has_reverb_zone(id) || !world.enabled_in_hierarchy(id))
                    continue;
                const Simulation::ReverbZoneParameters z = world.reverb_zone_parameters(id);
                const Vector3 c = world.world_transform(id).position;
                const double dx = std::fabs(listener_position.x - c.x);
                const double dy = std::fabs(listener_position.y - c.y);
                const double dz = std::fabs(listener_position.z - c.z);
                const bool inside = dx <= z.half_extents.x && dy <= z.half_extents.y &&
                                    dz <= z.half_extents.z;
                if (inside && (!have_zone || z.priority > best_priority))
                {
                    snapshot.reverb = to_i3dl2(z);
                    snapshot.has_reverb = true;
                    have_zone = true;
                    best_priority = z.priority;
                }
            }

            for (Simulation::EntityId id : ids)
            {
                if (!world.has_audio_emitter(id) || !world.enabled_in_hierarchy(id))
                    continue;
                const Simulation::AudioEmitterParameters e = world.audio_emitter_parameters(id);
                const Vector3 pos = world.world_transform(id).position;

                Audio::EmitterSnapshot es;
                es.key = static_cast<std::uint64_t>(id);
                es.sound = e.sound;
                es.position = to_local(pos, listener_position);
                es.gain = e.gain;
                es.priority = e.priority;
                es.bus = bus_for(e.bus);
                es.spatial = e.spatial;
                es.playing = e.playing;
                es.trigger = e.trigger;
                es.min_distance = e.min_distance;
                es.max_distance = e.max_distance;
                es.model = distance_model(e.distance_model);
                es.rolloff = e.rolloff;
                es.doppler_scale = e.doppler_scale;
                if (e.reverb_send > 0.0f)
                {
                    es.reverb_bus = reverb_bus_;
                    es.reverb_send = e.reverb_send;
                }
                snapshot.emitters.push_back(es);
            }

            factory_.looping = true;
            scene_.apply(snapshot);
        }

        void AudioEditorSystem::preview(std::uint32_t sound_id, float gain)
        {
            if (!device_open_)
                return;
            Audio::VoiceDescriptor d;
            d.base_gain = gain;
            d.priority = 100.0f; // auditions win a real slot
            d.bus = sfx_bus_;
            d.spatial = false;
            engine_.voices().play(
                d, std::unique_ptr<Audio::VoiceSource>(new Audio::ToneSource(tone_frequency(sound_id), 0.5f)));
        }

        void AudioEditorSystem::poll_profile()
        {
            engine_.profiler().latest(profile_);
        }

        void AudioEditorSystem::set_bus_gain(int bus_id, float gain)
        {
            if (bus_id >= 0 && bus_id < engine_.mixer().bus_count())
                engine_.mixer().bus_gain(bus_id).set_target(gain);
        }

        float AudioEditorSystem::bus_gain(int bus_id)
        {
            if (bus_id < 0 || bus_id >= engine_.mixer().bus_count())
                return 0.0f;
            return engine_.mixer().bus_gain(bus_id).target();
        }

        const char* AudioEditorSystem::bus_name(int bus_id) const noexcept
        {
            if (bus_id == master_bus_) return "Master";
            if (bus_id == sfx_bus_) return "SFX";
            if (bus_id == music_bus_) return "Music";
            if (bus_id == reverb_bus_) return "Reverb";
            return "Bus";
        }

        int AudioEditorSystem::bus_for(std::uint32_t code) const noexcept
        {
            switch (code)
            {
                case 0: return master_bus_;
                case 1: return sfx_bus_;
                case 2: return music_bus_;
                case 3: return reverb_bus_;
                default: return sfx_bus_;
            }
        }
    } // namespace Editor
} // namespace SushiEngine
