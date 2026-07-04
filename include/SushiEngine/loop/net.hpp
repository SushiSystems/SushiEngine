/**************************************************************************/
/* net.hpp                                                               */
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
 * @file net.hpp
 * @brief SushiLoop's Net layer (M4, docs/slop/SUSHILOOP.md): loopback-only,
 *        server-authoritative reconciliation on top of the M3 rollback machinery.
 *
 * Scope is deliberately narrow: this is a host-side, in-process, synchronous
 * simulation of a client/server exchange — `LoopbackChannel<Command>` — not a real
 * network transport. There are no sockets, no threads, no serialization, and no
 * general P2P/lockstep protocol here; those are explicitly out of scope for this
 * milestone. What this file proves is the *shape* SushiLoop's real network layer
 * will follow: a numbered command stream (reusing `InputHistory<Command>`/
 * `TickId`), a server that is the single source of truth for a tick's command, and
 * a client that predicts locally and reconciles by replaying `Loop::RollbackBuffer`
 * forward whenever the server's authoritative command disagrees with what the
 * client guessed.
 *
 * `NetworkId` is the other half: entities spawned during a networked simulation
 * need to agree on identity between server and client without a matching step.
 * Deriving the id from `(client_id, tick, spawn_sequence)` — a fact both sides
 * already know deterministically, since input and tick order are the only source
 * of divergence per SUSHILOOP.md's "Randomness and identity" — means client and
 * server independently compute the same id for the same spawn without either one
 * being authoritative over the numbering.
 */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/loop/input.hpp>
#include <SushiEngine/loop/rollback.hpp>

namespace SushiEngine
{
    namespace Loop
    {
        namespace Net
        {
            /** @brief A stable id for a connected participant (server is not a client). */
            using ClientId = std::uint32_t;

            /** @brief How many entities a given client has spawned so far this tick. */
            using SpawnSequence = std::uint32_t;

            /**
             * @brief A network-stable entity identity, independent of spawn order.
             *
             * Trivially copyable so it can live as an ordinary component and cross the
             * (simulated, here) wire as data.
             */
            struct NetworkId
            {
                std::uint64_t value = 0;

                /** @brief Equality by packed value. */
                constexpr bool operator==(const NetworkId& other) const noexcept
                {
                    return value == other.value;
                }

                /** @brief Inequality by packed value. */
                constexpr bool operator!=(const NetworkId& other) const noexcept
                {
                    return value != other.value;
                }
            };

            /**
             * @brief Deterministically derives a `NetworkId` from who spawned what, when.
             *
             * Packs `client_id` (top 16 bits), `tick` (next 40 bits), and
             * `spawn_sequence` (bottom 8 bits) into one 64-bit value. Any two
             * participants that agree on which client owns a spawn, which tick it
             * happened on, and that spawn's index within that client's tick — all facts
             * derivable from the numbered command stream itself, not from network
             * arrival order — compute the identical id without a round-trip.
             *
             * @param client_id      The spawning participant's stable id.
             * @param tick           The tick the spawn happened on.
             * @param spawn_sequence The 0-based index of this spawn among @p client_id's
             *                       spawns on @p tick (bottom 8 bits only; wraps past 255).
             * @return The deterministic network id.
             */
            constexpr NetworkId make_network_id(ClientId client_id, TickId tick,
                                                SpawnSequence spawn_sequence) noexcept
            {
                return NetworkId{(std::uint64_t(client_id) << 48) |
                                 ((std::uint64_t(tick) & 0xFFFFFFFFFFULL) << 8) |
                                 (std::uint64_t(spawn_sequence) & 0xFFULL)};
            }

            /** @brief One tick's authoritative command, as returned by the server. */
            template <typename Command>
            struct Ack
            {
                TickId tick;
                Command command;
            };

            /**
             * @brief An in-process, synchronous stand-in for a client-to-server command link.
             *
             * `client_send` records the client's own predicted command for a tick and
             * queues it; `server_process` drains the queue, letting the caller supply the
             * authoritative command for each queued tick (a real server would derive this
             * from its own simulation; here the caller decides, which is what makes the
             * reconciliation tests able to force a disagreement deterministically) and
             * returns one `Ack` per tick. No threads, no serialization: this is loopback
             * only, the scope this milestone commits to.
             *
             * @tparam Command The per-tick command type, matching `InputHistory<Command>`.
             */
            template <typename Command>
            class LoopbackChannel
            {
                public:
                    /**
                     * @brief Records the client's own predicted command and queues it to send.
                     * @param tick    The tick this command applies to.
                     * @param command The client's locally predicted command.
                     */
                    void client_send(TickId tick, Command command)
                    {
                        client_history_.record(tick, command);
                        inflight_.push_back(Packet{tick, command});
                    }

                    /**
                     * @brief Drains queued packets, producing the server's authoritative acks.
                     *
                     * @tparam Corrector A callable `Command(TickId, Command)` returning the
                     * authoritative command for a tick given the client's proposed one — the
                     * identity function if the server simply trusts the client, or a function
                     * that overrides specific ticks to simulate a real authoritative server.
                     * @param corrector Applied to every queued packet, in send order.
                     * @return One `Ack<Command>` per drained packet, in the same order.
                     */
                    template <typename Corrector>
                    std::vector<Ack<Command>> server_process(Corrector corrector)
                    {
                        std::vector<Ack<Command>> acks;
                        acks.reserve(inflight_.size());
                        for (const Packet& packet : inflight_)
                        {
                            Command authoritative = corrector(packet.tick, packet.command);
                            server_history_.record(packet.tick, authoritative);
                            acks.push_back(Ack<Command>{packet.tick, authoritative});
                        }
                        inflight_.clear();
                        return acks;
                    }

                    /** @brief The client's own predicted command history. */
                    InputHistory<Command>& client_history() noexcept { return client_history_; }

                private:
                    struct Packet
                    {
                        TickId tick;
                        Command command;
                    };

                    std::vector<Packet> inflight_;
                    InputHistory<Command> client_history_;
                    InputHistory<Command> server_history_;
            };

            /**
             * @brief Applies server acks to a client, rolling back and replaying on disagreement.
             *
             * Reuses `RollbackBuffer` exactly as M3 built it: it only decides *when* to
             * restore and replay, not how a snapshot is captured or restored. For every
             * ack whose command differs from what the client had locally predicted for
             * that tick, the client's history is corrected to the authoritative value and
             * the earliest such tick is tracked; if any correction happened, the world is
             * restored to that earliest tick and every tick from there through
             * @p current_tick is re-applied via @p apply using the (now-corrected) client
             * history, so ticks the client already got right are simply re-simulated
             * identically and ticks it mispredicted now use the authoritative command.
             *
             * @tparam Command  The per-tick command type; must support `operator==`.
             * @tparam ApplyFn  A callable `void(World&, TickId, const Command&)` applying
             *                  one tick's command to the world during replay.
             * @param world         The client's world to reconcile.
             * @param rollback      The client's rollback buffer; must hold every tick this
             *                      call might need to restore to.
             * @param client_history The client's predicted command history; entries for
             *                      corrected ticks are overwritten in place.
             * @param acks          The server's authoritative commands to reconcile against.
             * @param current_tick  The most recently simulated tick, inclusive replay bound.
             * @param apply         Applies one tick's command to @p world during replay.
             * @return Whether a rollback happened (false if every ack already matched).
             */
            template <typename Command, typename ApplyFn>
            bool reconcile(World& world, RollbackBuffer& rollback,
                           InputHistory<Command>& client_history,
                           const std::vector<Ack<Command>>& acks, TickId current_tick,
                           ApplyFn apply)
            {
                std::optional<TickId> earliest_mismatch;
                for (const Ack<Command>& ack : acks)
                {
                    const Command* predicted = client_history.find(ack.tick);
                    const bool mismatched = predicted == nullptr || !(*predicted == ack.command);
                    if (!mismatched)
                        continue;

                    client_history.correct(ack.tick, ack.command);
                    if (!earliest_mismatch || ack.tick < *earliest_mismatch)
                        earliest_mismatch = ack.tick;
                }

                if (!earliest_mismatch)
                    return false;

                if (!rollback.restore(*earliest_mismatch))
                    return false;

                for (TickId tick = *earliest_mismatch; tick <= current_tick; ++tick)
                {
                    const Command* command = client_history.find(tick);
                    if (command != nullptr)
                        apply(world, tick, *command);
                }

                return true;
            }

            /**
             * @brief The seam a `Loop::App` reconciles against: a numbered command link.
             *
             * The loop never names a concrete transport — it submits each tick's
             * command and polls for the server's authoritative acks through this
             * interface, so single-player (no transport connected), an in-process
             * `LoopbackTransport`, and a future real socket transport are
             * interchangeable without the game's systems or the loop changing
             * (dependency inversion). Making a game multiplayer is therefore one
             * decision — which transport, if any, is connected — not a rewrite.
             *
             * @tparam Command The per-tick command type, matching `InputHistory<Command>`.
             */
            template <typename Command>
            class INetworkTransport
            {
                public:
                    virtual ~INetworkTransport() = default;

                    /**
                     * @brief Submits the client's own predicted command for @p tick.
                     * @param tick    The tick this command applies to.
                     * @param command The locally predicted command about to be simulated.
                     */
                    virtual void submit(TickId tick, const Command& command) = 0;

                    /**
                     * @brief Returns the authoritative acks that have arrived since the last poll.
                     * @return One `Ack<Command>` per newly confirmed tick, in tick order;
                     * empty when nothing new has arrived.
                     */
                    virtual std::vector<Ack<Command>> poll() = 0;
            };

            /**
             * @brief An in-process `INetworkTransport` backed by a `LoopbackChannel`.
             *
             * Wraps the loopback client/server exchange behind the transport seam so a
             * single-process client `App` can be made "networked" against an
             * authoritative command function without a real socket. `submit` queues the
             * client's command; `poll` drains the queue through the authority callback,
             * optionally holding each ack for `latency_ticks` polls so the reconcile
             * path exercises a genuine multi-tick rollback rather than an immediate
             * one-tick correction. It is the concrete transport the tests and
             * `examples/first_game.cpp` use to prove the multiplayer seam end to end.
             *
             * @tparam Command The per-tick command type; must support `operator==`.
             */
            template <typename Command>
            class LoopbackTransport final : public INetworkTransport<Command>
            {
                public:
                    /**
                     * @brief Creates a loopback transport with an authoritative command source.
                     * @param authority     Returns the server's authoritative command for a
                     *                      tick given the client's proposed one — the identity
                     *                      function to trust the client, or an override that
                     *                      forces a mismatch to drive reconciliation.
                     * @param latency_ticks Number of polls to hold each ack before releasing
                     *                      it, modelling round-trip latency; 0 releases at once.
                     */
                    explicit LoopbackTransport(std::function<Command(TickId, Command)> authority,
                                               std::size_t latency_ticks = 0)
                        : authority_(std::move(authority)), latency_ticks_(latency_ticks) {}

                    void submit(TickId tick, const Command& command) override
                    {
                        channel_.client_send(tick, command);
                    }

                    std::vector<Ack<Command>> poll() override
                    {
                        for (const Ack<Command>& ack : channel_.server_process(authority_))
                            delayed_.push_back(Delayed{ack, latency_ticks_});

                        std::vector<Ack<Command>> ready;
                        std::vector<Delayed> still_waiting;
                        for (Delayed& pending : delayed_)
                        {
                            if (pending.remaining == 0)
                            {
                                ready.push_back(pending.ack);
                            }
                            else
                            {
                                --pending.remaining;
                                still_waiting.push_back(pending);
                            }
                        }
                        delayed_ = std::move(still_waiting);
                        return ready;
                    }

                private:
                    /** @brief One ack held back until its latency countdown reaches zero. */
                    struct Delayed
                    {
                        Ack<Command> ack;
                        std::size_t remaining;
                    };

                    LoopbackChannel<Command> channel_;
                    std::function<Command(TickId, Command)> authority_;
                    std::size_t latency_ticks_;
                    std::vector<Delayed> delayed_;
            };
        } // namespace Net
    } // namespace Loop
} // namespace SushiEngine
