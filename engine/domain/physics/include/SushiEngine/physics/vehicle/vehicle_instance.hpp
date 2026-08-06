/**************************************************************************/
/* vehicle_instance.hpp                                                   */
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
 * @file vehicle_instance.hpp
 * @brief A whole car in a world: the deformable structure, and the corners it rolls on.
 *
 * This is the composition §11.2 describes and nothing more. It owns a
 * @ref NodeBeamStructureT and a corner per @ref SuspensionSetupT, and its whole job is
 * that the two are created together, destroyed together, and stepped at the same tick
 * boundary. There is no vehicle *physics* here — every row this thing projects belongs
 * to a constraint kind that existed before it.
 *
 * **A corner needs a core to hang from, and that is an asset-level refusal.** §11.2
 * keeps the pure node-beam path open through an empty rigid core, and a pure node-beam
 * vehicle has no rigid body for a strut's chassis end. So an asset with corners and no
 * core is refused rather than silently instanced without its suspension — a car that
 * loaded and then had no wheels would be diagnosed as a suspension bug for as long as
 * it took someone to look at the cook.
 *
 * The driver's surface is steer, brake, throttle, clutch and gear; the tick is
 * @ref VehicleInstanceT::begin_tick before the solver's step and
 * @ref VehicleInstanceT::end_tick after it.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/vehicle/node_beam_structure.hpp>
#include <SushiEngine/physics/vehicle/powertrain.hpp>
#include <SushiEngine/physics/vehicle/suspension.hpp>
#include <SushiEngine/physics/vehicle/tyre_projection.hpp>
#include <SushiEngine/physics/vehicle/vehicle_asset.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief One vehicle, alive in a solver.
         *
         * Non-copyable, for the reason its two members are.
         *
         * @tparam T The scalar element type the solver runs in.
         */
        template <typename T>
        class VehicleInstanceT
        {
            public:
                /** @brief The solver seam this vehicle instances into. */
                using Solver = IConstraintSolver<T>;

                VehicleInstanceT() = default;
                VehicleInstanceT(const VehicleInstanceT&) = delete;
                VehicleInstanceT& operator=(const VehicleInstanceT&) = delete;
                VehicleInstanceT(VehicleInstanceT&&) = default;
                VehicleInstanceT& operator=(VehicleInstanceT&&) = default;

                /**
                 * @brief Instances the structure and every corner, or nothing at all.
                 *
                 * @param solver   The world; borrowed only for the duration of the call.
                 * @param view     The loaded `.sushinodebeam` the asset names.
                 * @param asset    The authored vehicle.
                 * @param settings Where to put it and what its bodies are made of.
                 * @return False when the structure was refused, when the asset asks for
                 *         corners on a vehicle with no rigid core, or when the solver ran
                 *         out of room for either half.
                 */
                bool create(Solver& solver, const Cooking::NodeBeamAssetView& view,
                            const VehicleAssetT<T>& asset,
                            const NodeBeamStructureSettings<T>& settings)
                {
                    destroy(solver);
                    if (!structure_.create(solver, view, settings))
                        return false;
                    if (asset.corners.empty())
                        return true;
                    if (!structure_.has_core())
                    {
                        structure_.destroy(solver);
                        return false;
                    }

                    corners_.resize(asset.corners.size());
                    driven_.clear();
                    for (std::size_t i = 0; i < asset.corners.size(); ++i)
                    {
                        if (!corners_[i].create(solver, structure_.core(), structure_.core_frame(),
                                                structure_.core_center(), asset.corners[i],
                                                settings.position, settings.orientation,
                                                settings.velocity))
                        {
                            destroy(solver);
                            return false;
                        }
                        if (asset.corners[i].driven)
                            driven_.push_back(i);
                    }

                    aerodynamics_ = asset.aerodynamics;
                    apply_body_drag(solver);
                    powertrain_.configure(asset.powertrain);
                    return true;
                }

                /**
                 * @brief Removes every body this vehicle put in the world.
                 * @param solver The world the vehicle was created in.
                 */
                void destroy(Solver& solver)
                {
                    for (SuspensionUnitT<T>& corner : corners_)
                        corner.destroy(solver);
                    corners_.clear();
                    driven_.clear();
                    tyres_.clear();
                    structure_.destroy(solver);
                }

                /**
                 * @brief Steps the drivetrain and puts its torque into the world (§11.4).
                 *
                 * **Call this before the solver's step and @ref end_tick after it.** The
                 * pairing is the whole reason for the names: what this does is add
                 * velocity the step then integrates, and running it afterwards would
                 * leave a tick's worth of drive sitting in the bodies unresolved.
                 *
                 * Three things happen, in the order §11.4 lists them. Each driven corner's
                 * wheel speed and axle inertia are read; the one-dimensional chain is
                 * stepped against them; and the torque it hands back is applied to each
                 * wheel about its own axle, with the sum of the reactions applied to the
                 * chassis core.
                 *
                 * **The reaction goes to the core, not to the carrier.** A driven wheel is
                 * turned by a shaft from the differential, and the differential's casing is
                 * bolted to the chassis — so the torque a driver feels as squat under power
                 * is a chassis reaction and not an unsprung one. Summing the impulses and
                 * negating them on one body also means the vehicle's total angular momentum
                 * is unchanged by its own engine, which is the statement that stops a car
                 * from driving itself around in mid-air.
                 *
                 * The tyres (§11.5) run first, before the drivetrain, because the patch
                 * force is what the wheel's speed is *about* to be resisted by and the
                 * order within one explicit block is otherwise arbitrary — putting the
                 * load-bearing one first keeps the tick readable. Their reports are kept
                 * per corner rather than returned, because a car has four of them and one
                 * return value would have to be a sum of things that do not add.
                 *
                 * @param solver The world the vehicle was created in.
                 * @param h      The tick, in seconds.
                 * @return What the drivetrain did, for a gauge to read.
                 */
                PowertrainReportT<T> begin_tick(Solver& solver, T h)
                {
                    apply_downforce(solver, h);
                    apply_tyres(solver, h);
                    if (driven_.empty())
                        return PowertrainReportT<T>{};

                    wheels_.resize(driven_.size());
                    for (std::size_t i = 0; i < driven_.size(); ++i)
                    {
                        const SuspensionUnitT<T>& corner = corners_[driven_[i]];
                        wheels_[i].spin_rate = corner.spin_rate(solver);
                        wheels_[i].inertia = corner.axle_inertia(solver);
                        wheels_[i].drive_torque = T(0);
                    }

                    const PowertrainReportT<T> report =
                        powertrain_.step(wheels_.data(), wheels_.size(), h);

                    Vector3T<T> reaction{T(0), T(0), T(0)};
                    for (std::size_t i = 0; i < driven_.size(); ++i)
                    {
                        SuspensionUnitT<T>& corner = corners_[driven_[i]];
                        const Vector3T<T> axis = corner.axle_axis(solver);
                        if (!corner.apply_axle_torque(solver, wheels_[i].drive_torque, h))
                            continue;
                        reaction = reaction + axis * (wheels_[i].drive_torque * h);
                    }
                    apply_chassis_reaction(solver, reaction);
                    return report;
                }

                /**
                 * @brief Runs the structure's tick-boundary rules (§11.1).
                 *
                 * The corners have none of their own: a strut does not creep and a
                 * broken mount is a joint the solver's own break rule already removes.
                 * Forwarded rather than reimplemented so a caller has one call per
                 * vehicle per tick and cannot run half of it.
                 *
                 * @param solver The world the vehicle was created in.
                 */
                NodeBeamTickReport end_tick(Solver& solver) { return structure_.end_tick(solver); }

                /**
                 * @brief Steers every corner that steers.
                 *
                 * One angle rather than one per corner, because Ackermann geometry —
                 * the inner wheel turning further than the outer — is a steering *rack*
                 * and belongs with §11.4's rack constraint, not with a per-corner value
                 * a caller would have to compute and keep consistent.
                 *
                 * @param solver The world the vehicle was created in.
                 * @param angle  The steer angle, in radians.
                 * @return How many corners moved.
                 */
                std::size_t set_steer_angle(Solver& solver, T angle)
                {
                    std::size_t steered = 0;
                    for (SuspensionUnitT<T>& corner : corners_)
                    {
                        if (corner.set_steer_angle(solver, angle))
                            ++steered;
                    }
                    return steered;
                }

                /**
                 * @brief Applies the same brake torque at every corner.
                 *
                 * Front-to-rear brake balance is a distribution over this, and a caller
                 * that wants one writes each corner. The one-call form is here because
                 * the common case is the handbrake and the emergency stop, and both are
                 * "all of it".
                 *
                 * @param solver The world the vehicle was created in.
                 * @param torque The most each brake may resist with, in newton-metres.
                 * @return How many corners took it.
                 */
                std::size_t set_brake_torque(Solver& solver, T torque)
                {
                    std::size_t braked = 0;
                    for (SuspensionUnitT<T>& corner : corners_)
                    {
                        if (corner.set_brake_torque(solver, torque))
                            ++braked;
                    }
                    return braked;
                }

                /**
                 * @brief Sets the accelerator, as a fraction of full throttle.
                 * @param throttle The pedal; clamped to zero through one.
                 */
                void set_throttle(T throttle) noexcept { powertrain_.set_throttle(throttle); }

                /**
                 * @brief Sets how hard the clutch is clamped, as a fraction.
                 * @param engagement One is fully engaged; clamped to zero through one.
                 */
                void set_clutch(T engagement) noexcept { powertrain_.set_clutch(engagement); }

                /**
                 * @brief Selects a gear by its index in @ref GearboxSettingsT::ratios.
                 * @param index The gear.
                 * @return False when the drivetrain has no such gear.
                 */
                bool select_gear(std::size_t index) noexcept
                {
                    return powertrain_.select_gear(index);
                }

                /** @brief The drivetrain, for tuning it or reading its crank speed. */
                PowertrainT<T>& powertrain() noexcept { return powertrain_; }

                /** @brief The drivetrain, for reading. */
                const PowertrainT<T>& powertrain() const noexcept { return powertrain_; }

                /** @brief How many corners the differential shares its torque over. */
                std::size_t driven_corner_count() const noexcept { return driven_.size(); }

                /**
                 * @brief What one corner's tyre did on the last @ref begin_tick.
                 *
                 * Empty until the first one, so a caller that reads before stepping gets a
                 * tyre that is not grounded rather than an out-of-range index.
                 *
                 * @param index The corner, in the asset's order.
                 */
                TyreReportT<T> tyre(std::size_t index) const noexcept
                {
                    return index < tyres_.size() ? tyres_[index] : TyreReportT<T>{};
                }

                /**
                 * @brief Visits every body this vehicle put in the world, with its surface.
                 *
                 * The inventory a scene needs in order to fold a gravity field into a car,
                 * write it back, and collide it. It lives here because this is the class
                 * that created the bodies: three scene functions each walking a vehicle's
                 * internals would be three places to forget a corner.
                 *
                 * The order is the order the bodies were added — the core, every node in
                 * asset order, then each corner's carrier and wheel. Fixed rather than
                 * incidental, because a caller that numbers anything from this must number
                 * it the same way on a replay (§0.5).
                 *
                 * The radius is the sphere the body collides as, and **zero means the body
                 * presents no collision surface**: the rigid core's shape is authored
                 * separately (§11.2), and a carrier is instanced coincident with its wheel,
                 * so either one would contribute a contact the car does not have.
                 *
                 * @param visit Called as `visit(BodyHandle, T radius)` once per body.
                 */
                template <typename F>
                void for_each_body(F&& visit) const
                {
                    if (structure_.has_core())
                        visit(structure_.core(), T(0));
                    for (std::size_t i = 0; i < structure_.node_count(); ++i)
                        visit(structure_.node(i), structure_.node_radius(i));
                    for (const SuspensionUnitT<T>& corner : corners_)
                    {
                        visit(corner.carrier(), T(0));
                        visit(corner.wheel(), corner.setup().wheel_radius);
                    }
                }

                /** @brief The deformable half: nodes, beams, the core, the render binding. */
                NodeBeamStructureT<T>& structure() noexcept { return structure_; }

                /** @brief The deformable half, for reading. */
                const NodeBeamStructureT<T>& structure() const noexcept { return structure_; }

                /** @brief How many corners were instanced. */
                std::size_t corner_count() const noexcept { return corners_.size(); }

                /**
                 * @brief One corner.
                 * @param index The corner, in the asset's order; out of range is undefined,
                 *              so callers bound it with @ref corner_count.
                 */
                SuspensionUnitT<T>& corner(std::size_t index) noexcept { return corners_[index]; }

                /** @brief One corner, for reading. */
                const SuspensionUnitT<T>& corner(std::size_t index) const noexcept
                {
                    return corners_[index];
                }

            private:
                /**
                 * @brief Puts the drivetrain's reaction into the chassis core.
                 *
                 * Silent when there is no core, because a pure node-beam vehicle (§11.2)
                 * has no rigid body to react against — and such a vehicle also has no
                 * corners, so this is unreachable rather than merely tolerated.
                 */
                void apply_chassis_reaction(Solver& solver, const Vector3T<T>& impulse)
                {
                    if (!structure_.has_core() || dot(impulse, impulse) <= T(0))
                        return;
                    RigidBodyT<T> core;
                    if (!solver.read_body(structure_.core(), core))
                        return;
                    apply_angular_velocity_impulse(core, impulse, T(-1));
                    solver.write_body(structure_.core(), core);
                }

                /**
                 * @brief Writes the body's drag into the core, once, at create time.
                 *
                 * Drag is a *constant* and not a per-tick force, so it belongs in the body
                 * rather than in the tick: `predict` already spends it every substep, and
                 * the wind field already arrives as the difference against it
                 * (`physics/aero/wind.hpp`). A car therefore meets a gust through exactly
                 * the same path a flag on a pole does, which is what §11.6 asks for and
                 * what a second per-tick drag force here would have quietly broken.
                 */
                void apply_body_drag(Solver& solver)
                {
                    if (!structure_.has_core())
                        return;
                    RigidBodyT<T> core;
                    if (!solver.read_body(structure_.core(), core))
                        return;
                    const T mass = core.inv_mass > T(0) ? T(1) / core.inv_mass : T(0);
                    core.drag_coefficient = quadratic_drag_constant(
                        aerodynamics_.drag_coefficient, aerodynamics_.frontal_area, mass,
                        aerodynamics_.air_density);
                    solver.write_body(structure_.core(), core);
                }

                /**
                 * @brief Presses the car onto the road in proportion to its airspeed.
                 *
                 * At the centre of pressure and along the car's *own* down axis, not the
                 * world's: a car on a banked corner keeps its downforce, and one over a
                 * crest keeps it pointing at the road rather than at the sky. The lever is
                 * the whole point — a wing behind the axle line pitches the car nose-up as
                 * it loads the rear, and that pitch is aerodynamic balance rather than a
                 * side effect.
                 *
                 * Airspeed is measured against the core's own velocity and not against the
                 * wind, because the wind sampler is `sim/`'s and this layer is not allowed
                 * to name it (§4.5). A headwind therefore adds drag through the core's drag
                 * constant, where the sampler does reach, but not downforce — an
                 * understatement rather than an invention, and the alternative is
                 * `physics/vehicle` reaching for the meteorology.
                 */
                void apply_downforce(Solver& solver, T h)
                {
                    if (!structure_.has_core() || !(h > T(0)) ||
                        !(aerodynamics_.downforce_coefficient > T(0)))
                    {
                        return;
                    }

                    RigidBodyT<T> core;
                    if (!solver.read_body(structure_.core(), core))
                        return;

                    const T airspeed = length(core.velocity);
                    const T magnitude = aerodynamic_force(airspeed, aerodynamics_.frontal_area,
                                                          aerodynamics_.downforce_coefficient,
                                                          aerodynamics_.air_density);
                    if (!(magnitude > T(0)))
                        return;

                    const QuaternionT<T>& frame = structure_.core_frame();
                    const Vector3T<T> up_local = rotate(frame, aerodynamics_.up);
                    const T up_length = length(up_local);
                    if (!(up_length > T(0)))
                        return;

                    const Vector3T<T> down =
                        rotate(core.orientation, up_local) * (T(-1) / up_length);
                    const Vector3T<T> lever =
                        rotate(core.orientation,
                               rotate(frame, aerodynamics_.center_of_pressure -
                                                 structure_.core_center()));
                    apply_velocity_impulse(core, down * (magnitude * h), lever, T(1));
                    solver.write_body(structure_.core(), core);
                }

                /**
                 * @brief Runs every corner's tyre against the contacts it is standing in.
                 *
                 * The substep the contact impulses were accumulated in comes from the
                 * solver's own statistics rather than from a parameter: §6.2 derives the
                 * substep count from the scene's state, so it is not a number a caller
                 * knows, and asking for it would be asking a caller to guess at something
                 * the solver had already decided.
                 */
                void apply_tyres(Solver& solver, T h)
                {
                    if (corners_.empty() || !(h > T(0)))
                        return;
                    const std::size_t substeps = solver.statistics().substeps;
                    const T substep = h / T(substeps > 0 ? substeps : 1);

                    tyres_.resize(corners_.size());
                    for (std::size_t i = 0; i < corners_.size(); ++i)
                    {
                        tyres_[i] = apply_tyre_force(solver, corners_[i].wheel(),
                                                     corners_[i].setup().tyre,
                                                     corners_[i].axle_axis(solver), h, substep);
                    }
                }

                NodeBeamStructureT<T> structure_;
                std::vector<SuspensionUnitT<T>> corners_;
                std::vector<TyreReportT<T>> tyres_;
                std::vector<std::size_t> driven_;
                std::vector<DrivenWheelT<T>> wheels_;
                PowertrainT<T> powertrain_;
                typename VehicleAssetT<T>::Aerodynamics aerodynamics_;
        };

        /** @brief The boundary vehicle: @ref VehicleInstanceT fixed to `Scalar`. */
        using VehicleInstance = VehicleInstanceT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
