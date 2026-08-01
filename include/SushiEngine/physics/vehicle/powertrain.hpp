/**************************************************************************/
/* powertrain.hpp                                                         */
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
 * @file powertrain.hpp
 * @brief §11.4's chain: crank to contact patch, in one dimension.
 *
 * §10.5 is why this file is not a set of constraints. A drivetrain has mass ratios in
 * the thousands — a crankshaft against a car — and couplings that are exactly rigid, and
 * pushing that through a three-dimensional solver buys stiffness the substep count then
 * has to pay for. Its first escape hatch says what to do instead: *"a powertrain is a
 * chain of rotational inertias, not a spatial mechanism. Simulating it as an independent
 * one-dimensional multibody system and coupling it to the wheels through a torque
 * constraint is both cheaper and more accurate."*
 *
 * So this is that system, and it knows nothing about bodies, handles, or solvers. It is
 * given each driven wheel's spin rate and its inertia about its own axle, and it gives
 * back a torque per wheel. Everything three-dimensional — which body, about which axis,
 * and where the reaction lands — belongs to the caller, which is
 * @ref VehicleInstanceT::begin_tick.
 *
 * ### The state is one number
 *
 * Gearbox and final drive are exact ratios and the differential's outputs are the wheels
 * themselves, so everything downstream of the clutch has its speed *determined* by the
 * wheel speeds the caller measured. The only rotational coordinate that is free is the
 * crankshaft. That is why @ref PowertrainT stores one scalar and not a shaft per stage:
 * the others would be derived values kept in a member, which is a cache and would go
 * stale the first time something else moved a wheel.
 *
 * ### The clutch is solved, not damped
 *
 * A clutch modelled as a stiff spring between two speeds is the explicit integration
 * §0.2 exists to avoid, and it is the classic reason a drivetrain needs a higher rate
 * than the car around it. Instead the torque that would make both sides *equal* at the
 * end of the step is computed in closed form and then clamped to the friction plate's
 * capacity. Below capacity the clutch is locked exactly; above it, it slips at exactly
 * its rated torque. There is no stiffness to tune and nothing to go unstable, which is
 * the same trade XPBD makes everywhere else in this engine.
 *
 * ### The differential is one number, not three cases
 *
 * An open differential splits torque evenly and lets its outputs turn at any speeds. A
 * locked one forces the speeds together. A limited-slip one is the first with a bounded
 * amount of the second. Written as three kinds that would be a branch, a constraint the
 * solver does not have (§10.2 defers `GearJoint` to here), and two of the three untested
 * most of the time. Written as @ref DifferentialSettingsT::lock_torque it is one clamp:
 * zero is open, large is locked, between is a limited-slip. The lock torques are balanced
 * to sum to zero before they leave, because a differential *divides* torque and must
 * never be a source of it.
 */

#include <cstddef>
#include <vector>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief One point on an engine's wide-open-throttle torque curve.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct EngineTorqueSampleT
        {
            /** @brief Crankshaft speed, in radians per second. */
            T rate = T(0);

            /** @brief Torque at full throttle at that speed, in newton-metres. */
            T torque = T(0);
        };

        /**
         * @brief The engine, as an author states it.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct EngineSettingsT
        {
            /**
             * @brief Full-throttle torque against speed, in ascending order of @ref
             *        EngineTorqueSampleT::rate.
             *
             * Interpolated linearly and held flat outside its ends. A curve rather than a
             * peak-torque number because the shape *is* the engine's character: where it
             * peaks decides which gear a corner is taken in, and a single number makes
             * every engine feel like the same electric motor.
             */
            std::vector<EngineTorqueSampleT<T>> curve;

            /**
             * @brief Rotational inertia of the crankshaft and flywheel, in kg·m².
             *
             * Small — this is the number the clutch solve exists to cope with, being
             * three orders below a wheel's reflected inertia in first gear.
             */
            T inertia = T(0.2);

            /**
             * @brief Idle speed, in radians per second.
             *
             * Held by a proportional governor rather than a clamp: below it the throttle
             * floor rises with the shortfall, so the engine settles at idle instead of
             * being pinned there. It also means the engine does not stall, which is a
             * deliberate omission — stalling needs an ignition state and a starter, and
             * those are driver-input surface, not §11.4.
             */
            T idle_rate = T(90);

            /**
             * @brief How far below @ref idle_rate the governor reaches full throttle, in
             *        radians per second.
             *
             * The governor's proportional band, and therefore its droop: a proportional
             * controller settles wherever its output balances the load, so the engine
             * idles a little *under* @ref idle_rate and this number is how much. Narrow it
             * and idle is held tighter; widen it and the engine comes off zero more
             * gently. It is authored rather than hidden because leaving it implicit is how
             * an engine ends up idling ten per cent low with nothing to point at.
             */
            T idle_band = T(20);

            /** @brief Where the limiter cuts the throttle, in radians per second. */
            T limit_rate = T(680);

            /** @brief Speed-independent internal drag, in newton-metres. */
            T friction_torque = T(12);

            /** @brief Speed-proportional drag, in newton-metre-seconds. */
            T viscous_damping = T(0.02);
        };

        /**
         * @brief The gearbox and final drive, as an author states them.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct GearboxSettingsT
        {
            /**
             * @brief Every gear, in selection order, as a signed ratio of input to output.
             *
             * One vector and no separate reverse, because reverse *is* a negative ratio
             * and neutral *is* a ratio of exactly zero. A typical car is
             * `{-3.2, 0, 3.4, 2.1, 1.4, 1.0, 0.8}`, whose neutral is index one. Keeping
             * them in the ordered list a driver moves through means selecting a gear is
             * an index and never a mode plus an index.
             */
            std::vector<T> ratios;

            /** @brief Final-drive ratio, multiplying whichever gear is selected. */
            T final_drive = T(3.9);

            /**
             * @brief Inertia of the shafts between clutch and wheels, in kg·m².
             *
             * Referred to the clutch's *output* — the gearbox input shaft, which turns at
             * engine speed. That is where the clutch solve needs it, and it also means the
             * number is small while its consequence is not: referred the other way, to the
             * wheels, it weighs this times the square of the whole ratio, so in first gear
             * a hundredth of a kilogram-metre-squared is comparable to the wheels
             * themselves. That is the real effect, and it is a large part of why first
             * gear does not accelerate a car as hard as its torque multiplication says.
             */
            T inertia = T(0.01);
        };

        /**
         * @brief The differential, as an author states it.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct DifferentialSettingsT
        {
            /**
             * @brief The most torque the differential may move between its outputs to
             *        bring their speeds together, in newton-metres.
             *
             * Zero is an open differential: torque is split evenly and a lifted wheel
             * spins freely. A large value is a spool. Anything between is a limited-slip,
             * and the number is exactly the one a differential is specified by.
             */
            T lock_torque = T(0);
        };

        /**
         * @brief A whole drivetrain, as an author states it.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct PowertrainSettingsT
        {
            /** @brief The engine. */
            EngineSettingsT<T> engine;

            /** @brief The gearbox and final drive. */
            GearboxSettingsT<T> gearbox;

            /** @brief The differential. */
            DifferentialSettingsT<T> differential;

            /**
             * @brief The most torque the clutch can carry fully engaged, in newton-metres.
             *
             * Scaled by the pedal, so the driver's clutch input is a fraction of this.
             * Should exceed the engine's peak torque or the clutch slips at full throttle
             * in every gear, which is a worn clutch and is a legitimate thing to author.
             */
            T clutch_capacity = T(900);
        };

        /**
         * @brief One driven wheel, across the seam.
         *
         * The caller fills the two measured fields and reads the third. One array of
         * these rather than parallel arrays of rates, inertias and torques, because three
         * arrays are three chances to hand in the wrong length.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct DrivenWheelT
        {
            /** @brief In: how fast the wheel is turning about its axle, in radians per second. */
            T spin_rate = T(0);

            /**
             * @brief In: the wheel's inertia about that axle, in kg·m².
             *
             * Must be positive. A wheel handed in with none takes no drive at all, which
             * is the only answer available: zero inertia and infinite inertia are the same
             * missing number here, and refusing to drive is the harmless one of the two
             * guesses.
             */
            T inertia = T(1);

            /** @brief Out: the torque the drivetrain puts through it, in newton-metres. */
            T drive_torque = T(0);
        };

        /**
         * @brief What the chain did this step.
         *
         * Everything here is a readout — a tachometer, a clutch-slip warning, a
         * diagnostic overlay — and nothing here is state. @ref PowertrainT keeps one
         * number and this is not it.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct PowertrainReportT
        {
            /** @brief Crankshaft speed after the step, in radians per second. */
            T engine_rate = T(0);

            /** @brief Net torque produced at the crank, in newton-metres. */
            T engine_torque = T(0);

            /** @brief Torque carried through the clutch, in newton-metres. */
            T clutch_torque = T(0);

            /** @brief Crank speed minus clutch-output speed, in radians per second. */
            T clutch_slip = T(0);

            /** @brief Total torque delivered to the wheels, in newton-metres. */
            T drive_torque = T(0);

            /** @brief Whether the clutch was at its capacity rather than locked. */
            bool clutch_slipping = false;
        };

        /**
         * @brief The one-dimensional chain of §11.4, stepped once per tick.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class PowertrainT
        {
            public:
                /**
                 * @brief Accepts a drivetrain, or refuses it and keeps the old one.
                 *
                 * Refuses a torque curve that is empty or whose speeds do not ascend,
                 * because @ref EngineSettingsT::curve is interpolated by walking it and a
                 * curve that doubles back would read a torque at a speed no author meant.
                 * A gearbox with no ratios is *not* refused: it is a drivetrain in
                 * permanent neutral, which is what an unpowered trailer is.
                 *
                 * @param settings The drivetrain.
                 * @return False when the curve is unusable; @ref settings is then unchanged.
                 */
                bool configure(const PowertrainSettingsT<T>& settings)
                {
                    if (settings.engine.curve.empty())
                        return false;
                    for (std::size_t i = 1; i < settings.engine.curve.size(); ++i)
                    {
                        if (!(settings.engine.curve[i].rate > settings.engine.curve[i - 1].rate))
                            return false;
                    }

                    settings_ = settings;
                    engine_rate_ = settings_.engine.idle_rate;
                    gear_ = 0;
                    return true;
                }

                /** @brief The drivetrain in force. */
                const PowertrainSettingsT<T>& settings() const noexcept { return settings_; }

                /**
                 * @brief Sets the accelerator, as a fraction; clamped to zero through one.
                 * @param throttle The pedal.
                 */
                void set_throttle(T throttle) noexcept { throttle_ = clamped_fraction(throttle); }

                /** @brief The accelerator last set. */
                T throttle() const noexcept { return throttle_; }

                /**
                 * @brief Sets clutch engagement, as a fraction; clamped to zero through one.
                 *
                 * One is a fully released pedal and a fully clamped plate. Zero carries no
                 * torque at all, which is why it is engagement rather than pedal travel:
                 * the sense a driver's foot has is the caller's to invert once, not this
                 * file's to guess.
                 *
                 * @param engagement How hard the plate is clamped.
                 */
                void set_clutch(T engagement) noexcept { clutch_ = clamped_fraction(engagement); }

                /** @brief The clutch engagement last set. */
                T clutch() const noexcept { return clutch_; }

                /**
                 * @brief Selects a gear by its index in @ref GearboxSettingsT::ratios.
                 * @param index The gear.
                 * @return False when there is no such gear, leaving the selection alone.
                 */
                bool select_gear(std::size_t index) noexcept
                {
                    if (index >= settings_.gearbox.ratios.size())
                        return false;
                    gear_ = index;
                    return true;
                }

                /** @brief The selected gear's index. */
                std::size_t gear() const noexcept { return gear_; }

                /**
                 * @brief The selected gear's ratio times the final drive; zero in neutral.
                 *
                 * Also zero when no gear has been selected at all, so an unconfigured
                 * drivetrain drives nothing rather than driving in first.
                 */
                T total_ratio() const noexcept
                {
                    if (gear_ >= settings_.gearbox.ratios.size())
                        return T(0);
                    return settings_.gearbox.ratios[gear_] * settings_.gearbox.final_drive;
                }

                /** @brief Crankshaft speed, in radians per second. */
                T engine_rate() const noexcept { return engine_rate_; }

                /**
                 * @brief Places the crankshaft speed directly.
                 *
                 * What a replay restoring a snapshot writes, and what a vehicle spawned
                 * rolling in gear needs so its first tick does not start from idle and
                 * shunt the whole car. Negative is clamped away: a crankshaft driven
                 * backwards is a modelling error, not a state.
                 *
                 * @param rate The speed, in radians per second.
                 */
                void set_engine_rate(T rate) noexcept
                {
                    engine_rate_ = rate > T(0) ? rate : T(0);
                }

                /**
                 * @brief Reads full-throttle torque off the curve at a given speed.
                 *
                 * Held flat outside the curve's ends rather than extrapolated, because a
                 * linear extrapolation past the last sample crosses zero and then goes
                 * negative, and an engine that produces reverse torque above its highest
                 * authored speed is a bug that only shows up on a long straight.
                 *
                 * @param rate Crankshaft speed, in radians per second.
                 */
                T curve_torque(T rate) const noexcept
                {
                    const std::vector<EngineTorqueSampleT<T>>& curve = settings_.engine.curve;
                    if (curve.empty())
                        return T(0);
                    if (rate <= curve.front().rate)
                        return curve.front().torque;
                    if (rate >= curve.back().rate)
                        return curve.back().torque;

                    for (std::size_t i = 1; i < curve.size(); ++i)
                    {
                        if (rate > curve[i].rate)
                            continue;
                        const T span = curve[i].rate - curve[i - 1].rate;
                        const T fraction = span > T(0) ? (rate - curve[i - 1].rate) / span : T(0);
                        return curve[i - 1].torque +
                               (curve[i].torque - curve[i - 1].torque) * fraction;
                    }
                    return curve.back().torque;
                }

                /**
                 * @brief Advances the chain one tick and shares its torque out.
                 *
                 * @param wheels Every driven wheel; @ref DrivenWheelT::spin_rate and
                 *               @ref DrivenWheelT::inertia are read and
                 *               @ref DrivenWheelT::drive_torque is written.
                 * @param count  How many.
                 * @param h      The tick, in seconds.
                 * @return What happened, for a gauge to read.
                 */
                PowertrainReportT<T> step(DrivenWheelT<T>* wheels, std::size_t count, T h)
                {
                    PowertrainReportT<T> report;
                    for (std::size_t i = 0; i < count; ++i)
                        wheels[i].drive_torque = T(0);
                    if (!(h > T(0)))
                    {
                        report.engine_rate = engine_rate_;
                        return report;
                    }

                    const T engine_torque = crank_torque();
                    // No driven wheels is the same statement as no gear: there is nothing
                    // on the far side of the clutch for it to pull against.
                    const T ratio = count == 0 ? T(0) : total_ratio();
                    const T mean_rate = mean_spin_rate(wheels, count);
                    const T wheel_inertia = total_wheel_inertia(wheels, count);
                    const T clutch_torque = clutch_torque_for(ratio, mean_rate, wheel_inertia,
                                                              engine_torque, h, report);

                    engine_rate_ += h * (engine_torque - clutch_torque) / engine_inertia();
                    if (engine_rate_ < T(0))
                        engine_rate_ = T(0);

                    const T axle_torque =
                        clutch_torque * ratio * driveline_share(ratio, wheel_inertia);
                    share_drive(wheels, count, axle_torque, mean_rate, h);

                    report.engine_rate = engine_rate_;
                    report.engine_torque = engine_torque;
                    report.clutch_torque = clutch_torque;
                    report.drive_torque = axle_torque;
                    return report;
                }

            private:
                /** @brief Clamps a driver input to the closed unit interval. */
                static T clamped_fraction(T value) noexcept
                {
                    if (value < T(0))
                        return T(0);
                    return value > T(1) ? T(1) : value;
                }

                /** @brief Clamps @p value to plus or minus @p limit. */
                static T clamped_symmetric(T value, T limit) noexcept
                {
                    if (value > limit)
                        return limit;
                    return value < -limit ? -limit : value;
                }

                /** @brief The crank's inertia, floored so the integration cannot divide by zero. */
                T engine_inertia() const noexcept
                {
                    return settings_.engine.inertia > T(0) ? settings_.engine.inertia : T(1);
                }

                /**
                 * @brief Net torque at the crank: the curve, governed and limited, less drag.
                 *
                 * The governor and the limiter are both a throttle floor and a throttle
                 * ceiling rather than added torques, which is what they physically are —
                 * an idle-air valve and an ignition cut both move the *demand*, and
                 * writing them as torques would let a limiter brake an engine that was
                 * already below its limit.
                 */
                T crank_torque() const noexcept
                {
                    const EngineSettingsT<T>& engine = settings_.engine;
                    T demand = engine_rate_ >= engine.limit_rate ? T(0) : throttle_;
                    if (engine.idle_band > T(0) && engine_rate_ < engine.idle_rate)
                    {
                        const T shortfall =
                            (engine.idle_rate - engine_rate_) / engine.idle_band;
                        if (shortfall > demand)
                            demand = clamped_fraction(shortfall);
                    }
                    return demand * curve_torque(engine_rate_) - engine.friction_torque -
                           engine.viscous_damping * engine_rate_;
                }

                /** @brief The mean of every driven wheel's speed; the differential's input. */
                static T mean_spin_rate(const DrivenWheelT<T>* wheels, std::size_t count) noexcept
                {
                    if (count == 0)
                        return T(0);
                    T total = T(0);
                    for (std::size_t i = 0; i < count; ++i)
                        total += wheels[i].spin_rate;
                    return total / T(count);
                }

                /** @brief Every driven wheel's inertia about its own axle, added up. */
                static T total_wheel_inertia(const DrivenWheelT<T>* wheels,
                                             std::size_t count) noexcept
                {
                    T total = T(0);
                    for (std::size_t i = 0; i < count; ++i)
                        total += wheels[i].inertia;
                    return total;
                }

                /**
                 * @brief What fraction of the shaft torque reaches the wheels.
                 *
                 * The rest goes into spinning up the drivetrain's own shafts, which are
                 * geared to the wheels and therefore accelerate with them. Referred to the
                 * wheels, the shafts weigh @ref GearboxSettingsT::inertia times the square
                 * of the ratio — so in a low gear a light gearbox is a heavy load, and it
                 * is a large part of why first gear does not accelerate a car as hard as
                 * its torque multiplication alone predicts.
                 *
                 * Leaving this out is not a small error and it is not a conservative one:
                 * it would deliver torque to the wheels *and* charge the clutch solve for
                 * an inertia nothing ever accelerated, so the two halves of the chain would
                 * disagree about how fast the driveline was turning and the clutch would
                 * never quite lock.
                 */
                T driveline_share(T ratio, T wheel_inertia) const noexcept
                {
                    const T shafts = settings_.gearbox.inertia * ratio * ratio;
                    const T total = shafts + wheel_inertia;
                    return total > T(0) ? wheel_inertia / total : T(0);
                }

                /**
                 * @brief The torque through the clutch: the locking solution, clamped.
                 *
                 * The locking solution is the torque for which the crank and the clutch's
                 * output arrive at the same speed at the end of the step, given the
                 * engine's torque and the two inertias. Solving it rather than damping
                 * toward it is what keeps a drivetrain stable at the car's own rate: a
                 * spring between a flywheel and a car in first gear is exactly the stiff
                 * pair §10.5 says not to build.
                 *
                 * The load the tyres put on the far end is not in it, because the caller
                 * measured the wheel speeds *after* the last tick resolved that load. It
                 * arrives as the speeds, one tick late, which is the same lag every
                 * explicit coupling in this engine already accepts.
                 */
                T clutch_torque_for(T ratio, T mean_rate, T wheel_inertia, T engine_torque, T h,
                                    PowertrainReportT<T>& report) const noexcept
                {
                    if (ratio == T(0))
                        return T(0);

                    const T driveline_inertia =
                        settings_.gearbox.inertia + wheel_inertia / (ratio * ratio);
                    if (!(driveline_inertia > T(0)))
                        return T(0);

                    const T crank_inertia = engine_inertia();
                    const T slip = engine_rate_ - ratio * mean_rate;
                    const T locked =
                        (crank_inertia * driveline_inertia * slip / h +
                         driveline_inertia * engine_torque) /
                        (crank_inertia + driveline_inertia);

                    const T capacity = settings_.clutch_capacity * clutch_;
                    const T carried = clamped_symmetric(locked, capacity);

                    report.clutch_slip = slip;
                    report.clutch_slipping = carried != locked;
                    return carried;
                }

                /**
                 * @brief Splits the axle torque across the wheels through the differential.
                 *
                 * An even split plus a lock torque that pulls each wheel toward the mean,
                 * bounded by @ref DifferentialSettingsT::lock_torque. The lock torques are
                 * then balanced to sum to zero: a differential moves torque between its
                 * outputs and can never be a source of it, and with unequal wheel inertias
                 * or with one wheel's clamp biting the raw values do not cancel on their
                 * own. Balancing can push a wheel past the authored bound by at most that
                 * bound again, which is the right way to be wrong — a differential that
                 * invented torque would accelerate a car with its wheels in the air.
                 */
                void share_drive(DrivenWheelT<T>* wheels, std::size_t count, T axle_torque,
                                 T mean_rate, T h) const noexcept
                {
                    if (count == 0)
                        return;

                    const T even = axle_torque / T(count);
                    const T bound = settings_.differential.lock_torque;
                    T lock_total = T(0);
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        const T lock =
                            bound > T(0)
                                ? clamped_symmetric(
                                      wheels[i].inertia * (mean_rate - wheels[i].spin_rate) / h,
                                      bound)
                                : T(0);
                        wheels[i].drive_torque = lock;
                        lock_total += lock;
                    }

                    const T balance = lock_total / T(count);
                    for (std::size_t i = 0; i < count; ++i)
                        wheels[i].drive_torque += even - balance;
                }

                PowertrainSettingsT<T> settings_;
                T engine_rate_ = T(0);
                T throttle_ = T(0);
                T clutch_ = T(1);
                std::size_t gear_ = 0;
        };

        /** @brief The boundary drivetrain: @ref PowertrainT fixed to `Scalar`. */
        using Powertrain = PowertrainT<Scalar>;

        /** @brief The boundary drivetrain description. */
        using PowertrainSettings = PowertrainSettingsT<Scalar>;

        /** @brief The boundary drivetrain readout. */
        using PowertrainReport = PowertrainReportT<Scalar>;

        /** @brief The boundary driven wheel. */
        using DrivenWheel = DrivenWheelT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
