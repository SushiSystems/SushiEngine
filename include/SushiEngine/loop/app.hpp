/**************************************************************************/
/* app.hpp                                                               */
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
 * @file app.hpp
 * @brief SushiLoop's authoring surface: the settled API a game is written against.
 *
 * `Loop::App<Command>` is the one object a game constructs. It composes the pieces
 * that already exist — a `World`, a `Schedule`, a `FixedTimestepClock`, a
 * `CommandBuffer`, an `InputHistory`, a `RollbackBuffer`, and a seeded `RngState` —
 * into a single fixed-step deterministic loop with a tidy way to register gameplay.
 * It is deliberately **not** a per-instance object model (no virtual `Update()` per
 * entity): gameplay is written as pure-ECS systems, exactly as `docs/slop/SUSHILOOP.md`
 * commits to, and the App only gives that a settled, ergonomic shape and a
 * well-defined per-tick lifecycle.
 *
 * The loop is **always multiplayer-ready**: every tick's command is captured into a
 * numbered `InputHistory` whether or not a network is attached, and the network is
 * reached only through the `Net::INetworkTransport` abstraction. Turning a
 * single-player game into a networked one is therefore a single decision —
 * `connect()` a transport — not a rewrite of its systems (dependency inversion).
 *
 * The App reads no wall clock. `advance(real_delta_seconds)` takes the host's
 * measured frame time and turns it into whole fixed steps, keeping the simulation
 * free of `chrono::now()` — the same determinism rule `FixedTimestepClock` enforces.
 */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/ecs/command_buffer.hpp>
#include <SushiEngine/ecs/component.hpp>
#include <SushiEngine/ecs/schedule.hpp>
#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/loop/fixed_timestep.hpp>
#include <SushiEngine/loop/input.hpp>
#include <SushiEngine/loop/net.hpp>
#include <SushiEngine/loop/rng.hpp>
#include <SushiEngine/loop/rollback.hpp>

namespace SushiEngine
{
    namespace Loop
    {
        /**
         * @brief The one-time configuration an `App` is constructed with.
         *
         * Everything here is fixed for the App's lifetime, so a value chosen once at
         * construction cannot drift mid-run and silently break replay: the fixed step
         * (which all determinism hinges on), the per-chunk entity capacity, the RNG
         * seed, and how many past ticks the rollback ring retains (0 disables
         * rollback entirely, the right default for a game that never rewinds).
         */
        struct AppConfig
        {
            Scalar fixed_dt_seconds = Scalar(1.0 / 60.0); /**< Duration of one fixed tick, in seconds (> 0). */
            std::size_t chunk_capacity = 1024;            /**< Entities per chunk for every archetype. */
            std::uint64_t rng_seed = 0;                   /**< Seed for the world's deterministic RNG. */
            std::size_t rollback_capacity = 0;            /**< Past ticks the rollback ring keeps; 0 disables rollback. */
        };

        /**
         * @brief The half-built handle returned by `App::system`, completed by `each`.
         *
         * Carries the read/write access set as template parameters (deduced from the
         * `App::system<Access...>` call) and the system's name, then forwards both to
         * `Schedule::each` when `each` supplies the kernel. Splitting registration
         * into `system<...>(name).each(fn)` is purely ergonomic — it reads as one
         * declarative statement — and compiles to exactly the same `Schedule::each`
         * call the ECS already used.
         *
         * @tparam Access The `Read<T>`/`Write<T>` declarations defining the access set.
         */
        template <typename... Access>
        class SystemBuilder
        {
            public:
                /**
                 * @brief Binds a builder to the schedule it will register into.
                 * @param schedule The schedule the completed system is added to.
                 * @param name     The system's name, for diagnostics.
                 */
                SystemBuilder(Schedule& schedule, std::string name)
                    : schedule_(schedule), name_(std::move(name)) {}

                /**
                 * @brief Registers the kernel, completing the system.
                 *
                 * The kernel is called as `fn(i, ptr0, ptr1, ...)` where each pointer
                 * is the column for the matching `Access` in declaration order —
                 * `const T*` for `Read<T>`, `T*` for `Write<T>`.
                 *
                 * @tparam Fn The per-element kernel callable type.
                 * @param fn The per-element kernel.
                 */
                template <typename Fn>
                void each(Fn fn)
                {
                    schedule_.each<Access...>(std::move(name_), std::move(fn));
                }

            private:
                Schedule& schedule_;
                std::string name_;
        };

        /**
         * @brief A SushiLoop game: a world, its systems, and a deterministic fixed-step loop.
         *
         * Construct one, register systems and lifecycle callbacks, then drive it with
         * `advance()` (host frame) or `run_for()` (headless). The App owns its
         * `SushiRuntime`, `World`, and `Schedule`, so the game code never touches the
         * runtime directly — the one-way engine→runtime dependency stays behind this
         * seam. `Command` is the game's own per-tick input type; it is captured,
         * numbered, and (when a transport is connected) reconciled, so the same type
         * that drives local play is exactly what the network layer replays.
         *
         * @tparam Command The game's per-tick command type: trivially copyable and
         * `operator==`-comparable, since it must be numbered, transmitted, and
         * compared during reconciliation.
         */
        template <typename Command>
        class App
        {
            public:
                /**
                 * @brief Creates a game host that owns its own runtime.
                 *
                 * The settled default for a real game: one `App` brings up one
                 * `SushiRuntime`. The prvalue from `Runtime::create()` initialises the
                 * heap object directly (C++17 mandatory copy elision), so the runtime is
                 * never moved or copied.
                 *
                 * @param config The fixed step, chunk capacity, RNG seed, and rollback depth.
                 */
                explicit App(const AppConfig& config = AppConfig{})
                    : owned_runtime_(new SushiRuntime::API::Runtime(
                          SushiRuntime::API::Runtime::create())),
                      runtime_(*owned_runtime_),
                      world_(runtime_, config.chunk_capacity),
                      schedule_(runtime_),
                      clock_(config.fixed_dt_seconds),
                      rng_(seed_rng(config.rng_seed))
                {
                    if (config.rollback_capacity > 0)
                        rollback_.emplace(config.rollback_capacity);
                }

                /**
                 * @brief Creates a game host that borrows an existing runtime.
                 *
                 * For a process that runs more than one world on a single runtime — a
                 * client and a server in one binary, or a test harness — pass a runtime
                 * that outlives the App instead of having it create its own. The App
                 * never destroys a borrowed runtime.
                 *
                 * @param runtime A runtime that outlives this App; borrowed, not owned.
                 * @param config  The fixed step, chunk capacity, RNG seed, and rollback depth.
                 */
                App(SushiRuntime::API::Runtime& runtime, const AppConfig& config)
                    : owned_runtime_(nullptr),
                      runtime_(runtime),
                      world_(runtime_, config.chunk_capacity),
                      schedule_(runtime_),
                      clock_(config.fixed_dt_seconds),
                      rng_(seed_rng(config.rng_seed))
                {
                    if (config.rollback_capacity > 0)
                        rollback_.emplace(config.rollback_capacity);
                }

                App(const App&) = delete;
                App& operator=(const App&) = delete;
                App(App&&) = delete;
                App& operator=(App&&) = delete;

                /** @brief The game world: spawn/destroy entities, reserve archetypes, read components. */
                World& world() noexcept { return world_; }

                /**
                 * @brief The runtime this App drives, whether owned or borrowed.
                 *
                 * The one seam through which an out-of-band, wall-clock consumer that
                 * lives *outside* the deterministic sim island — the audio engine's
                 * optional GPU DSP accelerator — reaches the runtime to allocate USM.
                 * Gameplay never needs this; the loop already hides the runtime behind
                 * `world()`, `commands()`, and `system()`. Exposing it does not weaken
                 * the one-way engine→runtime dependency: the App still owns the
                 * lifetime, and a borrowed runtime is returned, never destroyed.
                 *
                 * @return A reference to the App's runtime.
                 */
                SushiRuntime::API::Runtime& runtime() noexcept { return runtime_; }

                /** @copydoc runtime() */
                const SushiRuntime::API::Runtime& runtime() const noexcept { return runtime_; }

                /** @brief The deferred command buffer, applied at the per-tick barrier. */
                CommandBuffer& commands() noexcept { return commands_; }

                /** @brief The world's deterministic RNG, seeded from the config. */
                RngState& rng() noexcept { return rng_; }

                /** @brief The duration of one fixed simulation step, in seconds. */
                Scalar fixed_dt() const noexcept { return clock_.fixed_dt(); }

                /** @brief The next tick to be simulated (the count already simulated). */
                TickId tick() const noexcept { return tick_; }

                /** @brief The numbered per-tick command stream, the thing rollback replays. */
                const InputHistory<Command>& input_history() const noexcept { return input_history_; }

                /**
                 * @brief Begins declaring a system over the components named by @p Access.
                 *
                 * Usage: `app.system<Read<A>, Write<B>>("name").each(fn)`. Systems run
                 * in registration order where their access conflicts and in parallel
                 * where it does not — the runtime's dependency tracker decides, no
                 * scheduler is written here.
                 *
                 * @tparam Access The `Read<T>`/`Write<T>` declarations for the system.
                 * @param name The system's name, for diagnostics.
                 * @return A builder whose `each(fn)` registers the kernel.
                 */
                template <typename... Access>
                SystemBuilder<Access...> system(std::string name)
                {
                    return SystemBuilder<Access...>(schedule_, std::move(name));
                }

                /**
                 * @brief Sets the one-time world setup, run by `start()`.
                 * @param fn Seeds the initial entities and resources into the world.
                 * @return *this, for chaining.
                 */
                App& on_start(std::function<void(World&)> fn)
                {
                    on_start_ = std::move(fn);
                    return *this;
                }

                /**
                 * @brief Sets how one tick's command mutates the world.
                 *
                 * Applied on the host at the start of every tick, before the systems
                 * run, and re-applied verbatim for each replayed tick during
                 * reconciliation — so this is the single definition of "what an input
                 * does", shared by live play and rollback. Typically it writes the
                 * command onto an input component the systems then read.
                 *
                 * @param fn Applies @p command for @p tick to @p world.
                 * @return *this, for chaining.
                 */
                App& on_command(std::function<void(World&, TickId, const Command&)> fn)
                {
                    on_command_ = std::move(fn);
                    return *this;
                }

                /**
                 * @brief Sets how the local player's command for a tick is sampled.
                 *
                 * Called once per tick to capture the local input as a `Command`, which
                 * is then numbered into the history, applied, and (if connected) sent.
                 * Left unset, every tick's command is a default-constructed `Command`.
                 *
                 * @param fn Returns the local command for the given tick.
                 * @return *this, for chaining.
                 */
                App& sample_command(std::function<Command(TickId)> fn)
                {
                    sample_command_ = std::move(fn);
                    return *this;
                }

                /**
                 * @brief Connects a network transport, making the game multiplayer.
                 *
                 * The one decision that turns a single-player loop into a networked
                 * one: while connected, each tick's command is submitted to @p transport
                 * and the world reconciles against the server's authoritative acks.
                 * Pass `nullptr` to detach and return to single-player. The App does not
                 * own the transport; it must outlive the App.
                 *
                 * @param transport The transport to reconcile against, or `nullptr`.
                 * @return *this, for chaining.
                 */
                App& connect(Net::INetworkTransport<Command>* transport)
                {
                    transport_ = transport;
                    return *this;
                }

                /** @brief Runs the one-time world setup. Call once, before the first tick. */
                void start()
                {
                    if (on_start_)
                        on_start_(world_);
                    commands_.apply(world_);
                }

                /**
                 * @brief Advances the world by whole fixed steps for @p real_delta_seconds.
                 *
                 * Feeds the host's measured frame time into the fixed-step clock and
                 * ticks once per whole step it yields — zero if the host is running
                 * faster than the fixed rate, more than one if a frame hitched. Each
                 * step is identical and deterministic; the App never scales a step by
                 * elapsed time.
                 *
                 * @param real_delta_seconds Wall-clock time since the last call, in
                 * seconds, as measured by the host (never read inside the sim).
                 */
                void advance(Scalar real_delta_seconds)
                {
                    clock_.accumulate(real_delta_seconds);
                    while (clock_.consume_step())
                        step_once();
                }

                /** @brief Advances exactly one fixed step, regardless of elapsed time. */
                void tick_once() { step_once(); }

                /**
                 * @brief Runs exactly @p ticks fixed steps back to back.
                 *
                 * The headless entry point: a deterministic run of a fixed length, for
                 * tests and for a dedicated server that steps at its own rate rather
                 * than off a render clock.
                 *
                 * @param ticks Number of fixed steps to run.
                 */
                void run_for(std::size_t ticks)
                {
                    for (std::size_t i = 0; i < ticks; ++i)
                        step_once();
                }

                /** @brief The interpolation fraction left after the last tick, for render blending. */
                Scalar interpolation() const noexcept { return clock_.interpolation(); }

                /** @brief Times the system graph has been compiled (1 after warm-up). */
                std::size_t compile_count() const noexcept { return schedule_.compile_count(); }

            private:
                /**
                 * @brief Runs one fixed tick: capture, sample, apply, simulate, reconcile.
                 *
                 * The tick's command is captured into the numbered history before it is
                 * applied, and (with rollback enabled) a snapshot is taken first, so a
                 * later authoritative correction can rewind to exactly this tick and
                 * replay it. Reconciliation replays with `simulate` minus the structural
                 * barrier, honouring `RollbackBuffer`'s rule that no entity may spawn or
                 * be destroyed between a capture and its restore (see rollback.hpp) —
                 * spawning inside a rolled-back range is later SushiLoop work.
                 */
                void step_once()
                {
                    const TickId this_tick = tick_;

                    if (rollback_)
                        rollback_->capture(world_, this_tick);

                    const Command command =
                        sample_command_ ? sample_command_(this_tick) : Command{};
                    input_history_.record(this_tick, command);

                    apply_command(this_tick, command);
                    simulate();
                    commands_.apply(world_);

                    if (transport_ != nullptr)
                    {
                        transport_->submit(this_tick, command);
                        const std::vector<Net::Ack<Command>> acks = transport_->poll();
                        if (!acks.empty() && rollback_)
                            Net::reconcile(world_, *rollback_, input_history_, acks, this_tick,
                                           [this](World& world, TickId tick, const Command& corrected)
                                           {
                                               apply_command(tick, corrected);
                                               simulate();
                                               (void)world;
                                           });
                    }

                    ++tick_;
                }

                /** @brief Applies one command to the world via the game's `on_command`. */
                void apply_command(TickId tick, const Command& command)
                {
                    if (on_command_)
                        on_command_(world_, tick, command);
                }

                /** @brief Runs every registered system once over the world. */
                void simulate() { schedule_.run(world_); }

                std::unique_ptr<SushiRuntime::API::Runtime> owned_runtime_;
                SushiRuntime::API::Runtime& runtime_;
                World world_;
                Schedule schedule_;
                CommandBuffer commands_;
                FixedTimestepClock clock_;
                RngState rng_;
                InputHistory<Command> input_history_;
                std::optional<RollbackBuffer> rollback_;
                Net::INetworkTransport<Command>* transport_ = nullptr;
                TickId tick_ = 0;
                std::function<void(World&)> on_start_;
                std::function<void(World&, TickId, const Command&)> on_command_;
                std::function<Command(TickId)> sample_command_;
        };
    } // namespace Loop
} // namespace SushiEngine
