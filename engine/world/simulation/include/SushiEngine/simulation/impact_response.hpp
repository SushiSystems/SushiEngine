/**************************************************************************/
/* impact_response.hpp                                                    */
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
 * @file impact_response.hpp
 * @brief The engine's own physics listener: a contact becomes a sound and a burst.
 *
 * `ContactEvent` has reached the boundary since P1 carrying the impulse that separates
 * a scrape from a crash, and until this nothing outside the test suite read it. A push
 * interface alone would not have changed that — it would have been a second unread
 * thing on top of the first. This is the reader, and it is data-driven so that an
 * author gets impact audio by filling in a component rather than by writing C++
 * (`docs/design/physics_system.md` §16.48).
 *
 * It touches the world only through `IWorldEditor`, so it names no solver, no audio
 * device and no particle backend — what it does is set two components' fields and let
 * the systems that already read those do their jobs.
 */

#include <cstddef>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/simulation/physics_services.hpp>
#include <SushiEngine/simulation/simulation.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief Turns qualifying contacts into audio and particle bursts.
         *
         * Register it with `IPhysicsEventSource::add_event_sink` using @ref filter, and
         * call @ref update once per tick so its burst timers and cooldowns advance. It
         * does not own the world and must be removed before it is destroyed.
         */
        class ImpactResponseListener final : public IPhysicsEventSink
        {
            public:
                /**
                 * @brief Binds the listener to the world it will write into.
                 * @param world The world editor; must outlive this listener.
                 */
                explicit ImpactResponseListener(IWorldEditor& world) noexcept : world_(world) {}

                /**
                 * @brief The registration filter this listener wants.
                 *
                 * `Begin` only, and no impulse floor. The floor is deliberately left to
                 * the per-entity @ref ImpactResponse::minimum_impulse instead of being
                 * declared here: one number at registration would apply to every entity
                 * in the scene, and a pane of glass and a sandbag do not agree about
                 * what counts as a hit.
                 *
                 * The pair cooldown is left at zero for the same reason — @ref
                 * ImpactResponse::cooldown_seconds is per entity, which is what an
                 * author authored, and a crate bouncing between two walls would pass a
                 * pair cooldown twice over.
                 */
                static PhysicsEventFilter filter() noexcept
                {
                    PhysicsEventFilter wanted;
                    wanted.begin = true;
                    wanted.persist = false;
                    wanted.end = false;
                    wanted.joint_breaks = false;
                    return wanted;
                }

                /** @copydoc IPhysicsEventSink::on_contact */
                void on_contact(const ContactEvent& event) override
                {
                    // Both sides, independently. A crate hitting a bell should ring the
                    // bell and thud the crate, and each has its own thresholds — so this
                    // is two responses to one contact, not one response attributed to
                    // whichever entity the broadphase happened to name first.
                    respond(event.a, event.impulse);
                    respond(event.b, event.impulse);
                }

                /**
                 * @brief Advances the cooldowns and stops any burst whose time is up.
                 *
                 * Call once per tick. Nothing here happens inside `on_contact`, because
                 * a burst has to end on a tick where nothing was hit — which is exactly
                 * the tick `on_contact` is not called on.
                 *
                 * @param delta_time The tick's duration, in seconds.
                 */
                void update(Scalar delta_time)
                {
                    clock_ += delta_time;
                    for (std::size_t i = bursts_.size(); i-- > 0;)
                    {
                        if (clock_ < bursts_[i].until)
                            continue;
                        if (world_.has_particle_emitter(bursts_[i].id))
                        {
                            ParticleEmitterParameters emitter =
                                world_.particle_emitter_parameters(bursts_[i].id);
                            emitter.playing = false;
                            world_.set_particle_emitter_parameters(bursts_[i].id, emitter);
                        }
                        bursts_.erase(bursts_.begin() + std::ptrdiff_t(i));
                    }
                    // Expired cooldowns are dropped here rather than kept, which is the
                    // whole reason the entry stores its expiry: a table of every entity
                    // that has ever been hit would grow for as long as the scene runs.
                    for (std::size_t i = fired_.size(); i-- > 0;)
                    {
                        if (clock_ >= fired_[i].until)
                            fired_.erase(fired_.begin() + std::ptrdiff_t(i));
                    }
                }

                /** @brief Forgets every cooldown and burst, for a scene change. */
                void clear() noexcept
                {
                    bursts_.clear();
                    fired_.clear();
                }

            private:
                /** @brief One entity whose particle emitter is running until @ref until. */
                struct Burst
                {
                    EntityId id = NULL_ENTITY;
                    Scalar until = 0;
                };

                /**
                 * @brief An entity that may not respond again before @ref until.
                 *
                 * The expiry is stored rather than the moment it fired, which is what
                 * makes the table self-pruning: an entry whose time has passed is
                 * indistinguishable from one that never existed, so @ref update can
                 * drop it without consulting the entity's cooldown. Storing the fire
                 * time instead would mean the table only ever grew, since deciding an
                 * entry was stale would need the component that owns the window.
                 */
                struct Fired
                {
                    EntityId id = NULL_ENTITY;
                    Scalar until = 0;
                };

                /**
                 * @brief Fires @p id's response if it has one and the impulse earns it.
                 *
                 * @param id      The entity that was hit; `NULL_ENTITY` for static geometry.
                 * @param impulse The contact's total normal impulse, in newton-seconds.
                 */
                void respond(EntityId id, Scalar impulse)
                {
                    // Static geometry is not an entity and has no components to set.
                    // `ContactEvent` says so with a null id rather than by inventing one,
                    // and this is the reader that would otherwise have to filter a fiction.
                    if (id == NULL_ENTITY || !world_.has_impact_response(id))
                        return;
                    const ImpactResponse response = world_.impact_response(id);
                    if (impulse < response.minimum_impulse)
                        return;

                    for (const Fired& previous : fired_)
                    {
                        if (previous.id == id && clock_ < previous.until)
                            return;
                    }
                    note_fired(id, clock_ + response.cooldown_seconds);

                    // The ramp, clamped. `full_impulse` at or below `minimum_impulse` is
                    // an author saying "everything that qualifies is a full hit" rather
                    // than a mistake to reject, and a division would turn it into one.
                    Scalar strength = 1;
                    if (response.full_impulse > response.minimum_impulse)
                    {
                        strength = (impulse - response.minimum_impulse) /
                                   (response.full_impulse - response.minimum_impulse);
                        strength = strength < Scalar(0) ? Scalar(0)
                                                        : (strength > Scalar(1) ? Scalar(1)
                                                                                : strength);
                    }

                    if (response.plays_audio && world_.has_audio_emitter(id))
                    {
                        AudioEmitterParameters audio = world_.audio_emitter_parameters(id);
                        audio.gain = float(strength);
                        audio.playing = true;
                        // The pulse. Without it a sound already playing would only have
                        // its gain updated, and one that had finished would never start
                        // again — see `audio_scene.hpp`'s one-shot handling.
                        ++audio.trigger;
                        world_.set_audio_emitter_parameters(id, audio);
                    }

                    if (response.emits_particles && world_.has_particle_emitter(id))
                    {
                        ParticleEmitterParameters emitter = world_.particle_emitter_parameters(id);
                        emitter.playing = true;
                        world_.set_particle_emitter_parameters(id, emitter);
                        note_burst(id, clock_ + response.particle_seconds);
                    }
                }

                /** @brief Bars @p id from responding again until @p until. */
                void note_fired(EntityId id, Scalar until)
                {
                    for (Fired& previous : fired_)
                    {
                        if (previous.id != id)
                            continue;
                        previous.until = until;
                        return;
                    }
                    fired_.push_back(Fired{id, until});
                }

                /** @brief Starts or extends @p id's burst so it ends at @p until. */
                void note_burst(EntityId id, Scalar until)
                {
                    for (Burst& burst : bursts_)
                    {
                        if (burst.id != id)
                            continue;
                        // Extended, never shortened: a second impact during a burst
                        // should not cut the first one's sparks off early.
                        burst.until = until > burst.until ? until : burst.until;
                        return;
                    }
                    bursts_.push_back(Burst{id, until});
                }

                IWorldEditor& world_;

                // Linear scans over vectors rather than hash maps, and deliberately: the
                // population is the entities that were hit in the last fraction of a
                // second, which is a handful in any scene where an impact still means
                // something. A map would cost more to hash than this costs to walk.
                std::vector<Burst> bursts_;
                std::vector<Fired> fired_;

                /** @brief Simulated seconds, so a replay suppresses the same responses. */
                Scalar clock_ = 0;
        };
    } // namespace Simulation
} // namespace SushiEngine
