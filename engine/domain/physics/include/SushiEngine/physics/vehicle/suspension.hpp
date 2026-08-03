/**************************************************************************/
/* suspension.hpp                                                         */
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
 * @file suspension.hpp
 * @brief §11.2's third row: one corner of a car, as a strut and a wheel.
 *
 * "Suspension is joints and drives (§10.1), not beams — a slider joint with a
 * spring-damper drive is more controllable and more stable than a beam network, and
 * it is what every shipping racing game does." This is that sentence, built. There is
 * no new physics in this file: it is two joints from the existing library, two bodies,
 * and the arithmetic that turns four authored numbers into them.
 *
 * ### The strut is a MacPherson, and that is what makes steering free
 *
 * A corner is two joints:
 *
 * | Joint | Between | Holds |
 * |---|---|---|
 * | `Slider` | chassis core ↔ carrier | Travel along the strut axis, bounded by the bump stops, sprung by a position drive at the spring's compliance and damped by that drive's damping. |
 * | `Hinge` | carrier ↔ wheel | The axle. Its motor is the brake. |
 *
 * The carrier is the unsprung, non-rotating half of the corner — the hub and the
 * upright. It exists because the two statements a corner makes are about *different*
 * pairs: the spring acts between the chassis and something that does not spin, and the
 * axle is between that something and something that does. One body cannot be both, and
 * a slider that also let its body spin would be a six-degree-of-freedom joint whose one
 * drive would then have to be the spring *and* the brake.
 *
 * **Steering costs no third joint.** The slider locks all three rotations, which means
 * the carrier's orientation *is* the slider's frame on the chassis side. Rotating that
 * frame about its own primary axis therefore steers the corner — and because the
 * primary axis is the strut axis, the rotation moves nothing else: the travel direction
 * is unchanged, the spring is unchanged, and the axle turns because it is fixed in the
 * carrier. That is a MacPherson strut, where the kingpin and the damper axis coincide,
 * and it is the reason the geometry is worth choosing rather than being incidental.
 *
 * ### What this file does not decide
 *
 * Drive torque is §11.4's, and this file is only where it lands:
 * @ref SuspensionUnitT::apply_axle_torque spins the wheel by a torque somebody else
 * decided, and how much torque there is is @ref PowertrainT's. Tyre forces are §11.5's
 * and @ref tyre_projection.hpp's, which reads the contacts the wheel body is already
 * standing in. This file makes bodies, not colliders: giving the wheel a collision shape
 * is the asset layer's, exactly as it is for the chassis core's `.sushicollision`, and a
 * wheel that was never given one is a wheel the tyre model will correctly report as
 * never touching the ground.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/joint.hpp>
#include <SushiEngine/physics/core/body_flags.hpp>
#include <SushiEngine/physics/core/handle.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/solver/solver_interface.hpp>
#include <SushiEngine/physics/vehicle/tyre.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief One corner, as an author states it.
         *
         * Every direction and position is in the vehicle asset's own space — the same
         * space `NodeBeamNodeRecord::position` is in — because a corner is authored
         * against the car, not against the world.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct SuspensionSetupT
        {
            /** @brief Where the strut's upper pivot sits on the chassis. */
            Vector3T<T> mount{};

            /**
             * @brief The strut axis, pointing from the wheel centre up to @ref mount.
             *
             * Normalized on use. Also the steering axis, which is what makes this a
             * MacPherson strut rather than an assumption.
             */
            Vector3T<T> axis{T(0), T(1), T(0)};

            /**
             * @brief The axle direction, and with it the sign of everything that turns.
             *
             * **Both sides of a car point this the same way.** A wheel is symmetric about
             * its axle, so nothing physical asks it to point outboard, and pointing it
             * outboard would mean the two wheels of one axle spin in opposite senses when
             * the car rolls forward. §11.4's chain reads one signed speed per wheel and
             * writes one signed torque back, so a car built that way would hand its
             * differential a mean of zero and its chassis two reactions that cancel. The
             * axle is therefore a *convention* the whole vehicle shares, not a description
             * of which way the hub cap faces.
             *
             * Orthogonalized against @ref axis on use, because a strut axis that is not
             * exactly vertical and an axle that is exactly lateral are the ordinary
             * authoring case and the two are then not quite perpendicular. Taking the
             * component perpendicular to the strut keeps the wheel's spin axis square to
             * its travel, which is what the author meant.
             */
            Vector3T<T> axle{T(1), T(0), T(0)};

            /** @brief Distance from @ref mount down to the wheel centre at ride height, in metres. */
            T rest_length = T(0.35);

            /** @brief How far the wheel may rise above ride height before the bump stop, in metres. */
            T travel_bump = T(0.12);

            /** @brief How far it may drop below ride height before the rebound stop, in metres. */
            T travel_droop = T(0.12);

            /**
             * @brief Spring rate, in newtons per metre.
             *
             * Becomes the drive's compliance as its reciprocal. At or below zero the
             * strut is rigid at ride height, which is a solid axle rather than an error.
             */
            T spring_rate = T(35000);

            /** @brief Damper rate, in inverse seconds; see @ref JointMotorT::damping. */
            T damping = T(6);

            /**
             * @brief Compliance of the bump and rebound stops, in metres per newton.
             *
             * Zero is a steel stop. A real one is rubber, and a rubber stop is a
             * compliance rather than a second spring (`JointLimitT::compliance`).
             */
            T stop_compliance = T(0);

            /** @brief The hub and upright, in kilograms: the unsprung mass that does not spin. */
            T carrier_mass = T(15);

            /** @brief The wheel and tyre, in kilograms. */
            T wheel_mass = T(20);

            /** @brief Rolling radius, in metres. */
            T wheel_radius = T(0.34);

            /** @brief Tread width, in metres; sets the wheel's transverse inertia. */
            T wheel_width = T(0.22);

            /**
             * @brief Whether @ref SuspensionUnitT::set_steer_angle moves this corner.
             *
             * A rear corner ignores the wheel, and saying so here rather than at the
             * call site means a vehicle steers by one call with no per-corner test in
             * the caller — and means a rear corner cannot be steered by a caller that
             * forgot which corners it had.
             */
            bool steered = false;

            /**
             * @brief Whether §11.4's drivetrain puts torque through this corner.
             *
             * Alongside @ref steered for its reason: which wheels drive is a property of
             * the car and not of the call, so front, rear and four-wheel drive are three
             * assets and not three code paths. A differential is shared out over exactly
             * the corners that say yes here, so this is also how many outputs it has.
             */
            bool driven = false;

            /**
             * @brief The tyre on this corner (§11.5).
             *
             * Per corner rather than per vehicle because front and rear tyres differ on
             * most cars and differ on purpose — the balance between the two ends is a
             * handling decision, not an inconsistency to be tidied away.
             *
             * A `friction` of zero switches the model off and leaves the wheel to the
             * solver's own Coulomb friction, which is what an unpowered castor wants.
             */
            TyreSettingsT<T> tyre;

            /**
             * @brief The material the wheel and carrier collide as.
             *
             * **Point this at a frictionless material when the tyre model is on.** The
             * solver's Coulomb friction runs inside the substep loop on the same contact
             * the tyre model reads, so a gripping wheel gets both and ends up with grip
             * nobody authored and no single wrong number to find. Separate from the
             * vehicle's own material because the wheel is the one part of a car whose
             * surface behaviour is deliberately not the body's.
             */
            std::uint32_t material_index = 0;
        };

        /**
         * @brief One corner, alive: two bodies, two joints, and the three things a driver moves.
         *
         * Non-copyable for @ref NodeBeamStructureT's reason: two copies would name the
         * same solver slots.
         *
         * @tparam T The scalar element type the solver runs in.
         */
        template <typename T>
        class SuspensionUnitT
        {
            public:
                /** @brief The solver seam this corner instances into. */
                using Solver = IConstraintSolver<T>;

                SuspensionUnitT() = default;
                SuspensionUnitT(const SuspensionUnitT&) = delete;
                SuspensionUnitT& operator=(const SuspensionUnitT&) = delete;
                SuspensionUnitT(SuspensionUnitT&&) = default;
                SuspensionUnitT& operator=(SuspensionUnitT&&) = default;

                /**
                 * @brief Builds the corner against an existing chassis body.
                 *
                 * All or nothing, on @ref NodeBeamStructureT::create's reasoning: half a
                 * corner is a wheel held by nothing.
                 *
                 * @param solver      The world; borrowed only for the duration of the call.
                 * @param core        The chassis body the strut hangs from; must be live.
                 * @param core_frame  The rotation taking asset space into the core's body
                 *                    frame — `conjugate` of the core's principal rotation,
                 *                    because `RigidBodyT` stores its inertia as a body-frame
                 *                    diagonal and the core is therefore instanced rotated
                 *                    into its principal frame.
                 *  @param core_center The core's centre of mass, in asset space; joint
                 *                    anchors are measured from it because that is what a
                 *                    joint's local anchor means.
                 * @param setup       The authored corner.
                 * @param position    World position of the asset's origin.
                 * @param orientation World orientation of the asset.
                 * @param velocity    Linear velocity both new bodies start with.
                 * @return False when the solver had no room; nothing is left behind.
                 */
                bool create(Solver& solver, BodyHandle core, const QuaternionT<T>& core_frame,
                            const Vector3T<T>& core_center, const SuspensionSetupT<T>& setup,
                            const Vector3T<T>& position, const QuaternionT<T>& orientation,
                            const Vector3T<T>& velocity)
                {
                    destroy(solver);
                    if (!core.valid())
                        return false;

                    const Vector3T<T> strut = normalized_or(setup.axis, Vector3T<T>{0, 1, 0});
                    const Vector3T<T> axle =
                        orthogonalized(setup.axle, strut, Vector3T<T>{1, 0, 0});
                    const Vector3T<T> hub = setup.mount - strut * setup.rest_length;
                    setup_ = setup;
                    rest_length_ = setup.rest_length;

                    if (!create_bodies(solver, hub, axle, position, orientation, velocity) ||
                        !create_strut(solver, core, core_frame, core_center, strut) ||
                        !create_axle(solver, axle))
                    {
                        destroy(solver);
                        return false;
                    }
                    return true;
                }

                /**
                 * @brief Removes both bodies, and with them both joints.
                 * @param solver The world the corner was created in.
                 */
                void destroy(Solver& solver)
                {
                    if (wheel_.valid())
                        solver.remove_body(wheel_);
                    if (carrier_.valid())
                        solver.remove_body(carrier_);
                    wheel_ = BodyHandle{};
                    carrier_ = BodyHandle{};
                    core_ = BodyHandle{};
                    strut_ = JointHandle{};
                    axle_ = JointHandle{};
                }

                /**
                 * @brief Turns the corner, when it is one that turns.
                 *
                 * Rotates the strut's chassis-side frame about its own primary axis.
                 * Nothing else moves: the travel direction is that axis, so the spring
                 * and the stops are untouched, and the axle turns because it is fixed in
                 * the carrier the slider's rotation lock is holding.
                 *
                 * @param solver The world the corner was created in.
                 * @param angle  The steer angle, in radians; positive by the right-hand
                 *               rule about the strut axis.
                 * @return False when this corner is not steered, or its joint is gone.
                 */
                bool set_steer_angle(Solver& solver, T angle)
                {
                    if (!setup_.steered || !strut_.valid())
                        return false;
                    JointConstraintT<T> joint;
                    if (!solver.read_joint(strut_, joint))
                        return false;
                    joint.local_basis_a =
                        mul(base_basis_, quaternion_axis_angle(Vector3T<T>{1, 0, 0}, angle));
                    steer_angle_ = angle;
                    return solver.write_joint(strut_, joint);
                }

                /**
                 * @brief Applies the brake, as the axle's friction.
                 *
                 * §10.1's own reading of a velocity drive: a target of zero with a force
                 * limit *is* friction, and a brake is friction an author asks for. A
                 * torque at or below zero releases the brake by disabling the drive
                 * rather than by setting its limit to zero — a zero limit means
                 * *unsaturated* on `JointMotorT::max_force`, so releasing a brake that
                 * way would weld the wheel to the hub.
                 *
                 * @param solver The world the corner was created in.
                 * @param torque The most the brake may resist with, in newton-metres.
                 * @return False when the axle joint is gone.
                 */
                bool set_brake_torque(Solver& solver, T torque)
                {
                    if (!axle_.valid())
                        return false;
                    JointConstraintT<T> joint;
                    if (!solver.read_joint(axle_, joint))
                        return false;
                    joint.motor.mode =
                        torque > T(0) ? JointMotorMode::Velocity : JointMotorMode::Disabled;
                    joint.motor.target = T(0);
                    joint.motor.max_force = torque;
                    brake_torque_ = torque > T(0) ? torque : T(0);
                    return solver.write_joint(axle_, joint);
                }

                /**
                 * @brief How far the strut is compressed from ride height, in metres.
                 *
                 * Positive in bump, negative in droop, zero at the height the corner was
                 * authored at. Derived from where the two bodies actually are rather
                 * than from the drive's target, because the target is what the spring is
                 * *asking* for and this is what it got.
                 *
                 * @param solver The world the corner was created in.
                 * @return Zero when either body is gone, which is also what an
                 *         uncompressed strut reads — the honest answer for a corner
                 *         there is nothing to measure.
                 */
                T compression(const Solver& solver) const
                {
                    RigidBodyT<T> carrier;
                    if (!carrier_.valid() || !solver.read_body(carrier_, carrier))
                        return T(0);
                    RigidBodyT<T> chassis;
                    if (!core_.valid() || !solver.read_body(core_, chassis))
                        return T(0);
                    const Vector3T<T> axis =
                        rotate(mul(chassis.orientation, base_basis_), Vector3T<T>{1, 0, 0});
                    const Vector3T<T> anchor =
                        chassis.position + rotate(chassis.orientation, mount_local_);
                    return rest_length_ + dot(carrier.position - anchor, axis);
                }

                /**
                 * @brief The load the strut carried over the last tick, in newtons.
                 *
                 * §10.4's recovery, read off the joint the spring lives on: the mean
                 * force vector its rows carried. What a corner-weight readout is, and
                 * what a suspension inspector shows.
                 *
                 * @param solver The world the corner was created in.
                 */
                Vector3T<T> load(const Solver& solver) const
                {
                    JointConstraintT<T> joint;
                    if (!strut_.valid() || !solver.read_joint(strut_, joint))
                        return Vector3T<T>{T(0), T(0), T(0)};
                    return joint_force(joint);
                }

                /**
                 * @brief How fast the wheel is turning about its axle, in radians per second.
                 * @param solver The world the corner was created in.
                 */
                T spin_rate(const Solver& solver) const
                {
                    RigidBodyT<T> wheel;
                    if (!wheel_.valid() || !solver.read_body(wheel_, wheel))
                        return T(0);
                    return dot(wheel.angular_velocity,
                               rotate(wheel.orientation, Vector3T<T>{1, 0, 0}));
                }

                /**
                 * @brief The wheel's spin axis in world space, as a unit vector.
                 *
                 * The wheel body's own x, because @ref create_bodies built its frame that
                 * way. Asking the corner rather than deriving it from the setup is what
                 * makes the answer follow steering and body roll without a caller having
                 * to compose either.
                 *
                 * Zero for a corner that does not exist, which is a direction no impulse
                 * can be applied along and therefore a safe answer rather than a lie.
                 *
                 * @param solver The world the corner was created in.
                 */
                Vector3T<T> axle_axis(const Solver& solver) const
                {
                    RigidBodyT<T> wheel;
                    if (!wheel_.valid() || !solver.read_body(wheel_, wheel))
                        return Vector3T<T>{T(0), T(0), T(0)};
                    return rotate(wheel.orientation, Vector3T<T>{T(1), T(0), T(0)});
                }

                /**
                 * @brief The wheel's rotational inertia about its own axle, in kg·m².
                 *
                 * What §11.4's chain needs to know about the far end of itself. Read back
                 * from the body rather than recomputed from the setup so that a caller
                 * that retuned the wheel drives against the wheel that is actually there.
                 *
                 * @param solver The world the corner was created in.
                 */
                T axle_inertia(const Solver& solver) const
                {
                    RigidBodyT<T> wheel;
                    if (!wheel_.valid() || !solver.read_body(wheel_, wheel))
                        return T(0);
                    return wheel.inv_inertia.x > T(0) ? T(1) / wheel.inv_inertia.x : T(0);
                }

                /**
                 * @brief Spins the wheel by a torque held over one tick.
                 *
                 * §11.4's coupling, at this end: *"a torque applied to each wheel body."*
                 * A velocity impulse and not a motor, because the drivetrain has already
                 * decided how much torque there is and a motor would decide again — and
                 * because the axle's motor is the brake (§11.2), which must be free to
                 * resist this rather than be overwritten by it.
                 *
                 * @param solver The world the corner was created in.
                 * @param torque The torque, in newton-metres; positive drives the wheel
                 *               the way @ref axle_axis points.
                 * @param h      The tick, in seconds.
                 * @return False when the wheel is gone, so a caller knows not to apply the
                 *         reaction for a torque that never landed.
                 */
                bool apply_axle_torque(Solver& solver, T torque, T h)
                {
                    RigidBodyT<T> wheel;
                    if (!wheel_.valid() || !solver.read_body(wheel_, wheel))
                        return false;
                    const Vector3T<T> axis =
                        rotate(wheel.orientation, Vector3T<T>{T(1), T(0), T(0)});
                    apply_angular_velocity_impulse(wheel, axis * (torque * h), T(1));
                    return solver.write_body(wheel_, wheel);
                }

                /** @brief The spinning body, for a renderer or a tyre model to reach. */
                BodyHandle wheel() const noexcept { return wheel_; }

                /** @brief The hub, which travels but does not spin. */
                BodyHandle carrier() const noexcept { return carrier_; }

                /** @brief The strut joint, for reading its load or writing its spring. */
                JointHandle strut() const noexcept { return strut_; }

                /** @brief The axle joint, where P7-G's drive torque will land. */
                JointHandle axle() const noexcept { return axle_; }

                /** @brief The corner as it was authored, kept so a caller need not carry it. */
                const SuspensionSetupT<T>& setup() const noexcept { return setup_; }

                /** @brief The steer angle last written, in radians; zero for a fixed corner. */
                T steer_angle() const noexcept { return steer_angle_; }

                /** @brief The brake torque last written, in newton-metres. */
                T brake_torque() const noexcept { return brake_torque_; }

                /** @brief Whether both bodies and both joints exist. */
                bool valid() const noexcept
                {
                    return wheel_.valid() && carrier_.valid() && strut_.valid() && axle_.valid();
                }

            private:
                /** @brief Builds the carrier and the wheel, coincident at the hub. */
                bool create_bodies(Solver& solver, const Vector3T<T>& hub,
                                   const Vector3T<T>& axle, const Vector3T<T>& position,
                                   const QuaternionT<T>& orientation,
                                   const Vector3T<T>& velocity)
                {
                    const Vector3T<T> world_hub = position + rotate(orientation, hub);

                    RigidBodyT<T> carrier;
                    carrier.position = world_hub;
                    carrier.previous_position = world_hub;
                    carrier.orientation = orientation;
                    carrier.previous_orientation = orientation;
                    carrier.velocity = velocity;
                    carrier.inv_mass = setup_.carrier_mass > T(0) ? T(1) / setup_.carrier_mass : T(0);
                    // A sphere, because the strut locks the carrier's rotation to the
                    // chassis outright: its inertia is only ever the weight by which an
                    // angular correction is split, and a sphere is the one shape that
                    // makes no claim about which way the hub is facing.
                    const T carrier_inertia =
                        T(0.1) * setup_.carrier_mass * setup_.wheel_radius * setup_.wheel_radius;
                    carrier.inv_inertia = uniform_inverse_inertia(carrier_inertia);
                    carrier.material_index = setup_.material_index;
                    carrier.flags = BodyFlags::dynamic_body;

                    carrier_ = solver.add_body(carrier);
                    if (!carrier_.valid())
                        return false;

                    RigidBodyT<T> wheel = carrier;
                    // The wheel's body frame has the axle as its local x, because that is
                    // the frame its inertia is diagonal in — a cylinder's transverse
                    // moments are equal and its axial one is not, and `RigidBodyT` has
                    // nowhere to say so except by being in that frame.
                    wheel.orientation = mul(orientation, joint_frame_from_axis(axle));
                    wheel.previous_orientation = wheel.orientation;
                    wheel.inv_mass = setup_.wheel_mass > T(0) ? T(1) / setup_.wheel_mass : T(0);
                    wheel.inv_inertia = cylinder_inverse_inertia();
                    wheel_ = solver.add_body(wheel);
                    return wheel_.valid();
                }

                /** @brief Builds the slider: the spring, the damper and the two stops. */
                bool create_strut(Solver& solver, BodyHandle core,
                                  const QuaternionT<T>& core_frame,
                                  const Vector3T<T>& core_center, const Vector3T<T>& strut)
                {
                    const Vector3T<T> strut_local = rotate(core_frame, strut);
                    base_basis_ = joint_frame_from_axis(strut_local);
                    mount_local_ = rotate(core_frame, setup_.mount - core_center);
                    core_ = core;

                    JointConstraintT<T> joint;
                    joint.kind = JointKind::Slider;
                    joint.flags = JointFlags::enabled;
                    joint.a = std::uint32_t(solver.body_slot(core));
                    joint.b = std::uint32_t(solver.body_slot(carrier_));
                    joint.local_anchor_a = mount_local_;
                    joint.local_anchor_b = Vector3T<T>{T(0), T(0), T(0)};
                    joint.local_basis_a = base_basis_;
                    joint.local_basis_b = joint_frame_from_axis(strut);

                    // The travel coordinate is the carrier's offset along the strut axis,
                    // and the carrier hangs *below* the mount — so ride height is a
                    // negative coordinate and bump is the less negative end of the range.
                    joint.linear_limit.lower = -(rest_length_ + setup_.travel_droop);
                    joint.linear_limit.upper = -(rest_length_ - setup_.travel_bump);
                    joint.linear_limit.compliance = setup_.stop_compliance;
                    joint.linear_limit.enabled = true;

                    joint.motor.mode = JointMotorMode::Position;
                    joint.motor.target = -rest_length_;
                    joint.motor.compliance =
                        setup_.spring_rate > T(0) ? T(1) / setup_.spring_rate : T(0);
                    joint.motor.damping = setup_.damping;
                    // Unsaturated: a spring is not a drive that gives up. Its stiffness
                    // is the compliance, and a force limit on it would be a spring that
                    // stops pushing at a load — which is a bump stop, and bump stops are
                    // the limit rows above.
                    joint.motor.max_force = T(0);

                    strut_ = solver.add_joint(joint);
                    return strut_.valid();
                }

                /** @brief Builds the hinge the wheel spins on. */
                bool create_axle(Solver& solver, const Vector3T<T>& axle)
                {
                    JointConstraintT<T> joint;
                    joint.kind = JointKind::Hinge;
                    joint.flags = JointFlags::enabled;
                    joint.a = std::uint32_t(solver.body_slot(carrier_));
                    joint.b = std::uint32_t(solver.body_slot(wheel_));
                    joint.local_anchor_a = Vector3T<T>{T(0), T(0), T(0)};
                    joint.local_anchor_b = Vector3T<T>{T(0), T(0), T(0)};
                    joint.local_basis_a = joint_frame_from_axis(axle);
                    // Identity, because the wheel's body frame was built with the axle as
                    // its local x — the same choice its inertia needed.
                    joint.local_basis_b = QuaternionT<T>{T(0), T(0), T(0), T(1)};
                    // No twist limit: a wheel that could only turn so far is a wheel
                    // that stops the car at the end of the first straight.
                    axle_ = solver.add_joint(joint);
                    return axle_.valid();
                }

                /** @brief The wheel's inverse inertia as a solid cylinder about its axle. */
                Vector3T<T> cylinder_inverse_inertia() const noexcept
                {
                    const T mass = setup_.wheel_mass;
                    const T radius = setup_.wheel_radius;
                    const T axial = T(0.5) * mass * radius * radius;
                    const T transverse =
                        mass * (T(3) * radius * radius + setup_.wheel_width * setup_.wheel_width) /
                        T(12);
                    return Vector3T<T>{axial > T(0) ? T(1) / axial : T(0),
                                       transverse > T(0) ? T(1) / transverse : T(0),
                                       transverse > T(0) ? T(1) / transverse : T(0)};
                }

                /** @brief The same moment about all three axes, inverted. */
                static Vector3T<T> uniform_inverse_inertia(T moment) noexcept
                {
                    const T inverse = moment > T(0) ? T(1) / moment : T(0);
                    return Vector3T<T>{inverse, inverse, inverse};
                }

                /** @brief @p v normalized, or @p fallback when it has no length to speak of. */
                static Vector3T<T> normalized_or(const Vector3T<T>& v,
                                                 const Vector3T<T>& fallback) noexcept
                {
                    const T vector_length = length(v);
                    return vector_length > T(1e-9) ? v * (T(1) / vector_length) : fallback;
                }

                /** @brief The part of @p v perpendicular to @p axis, normalized. */
                static Vector3T<T> orthogonalized(const Vector3T<T>& v, const Vector3T<T>& axis,
                                                  const Vector3T<T>& fallback) noexcept
                {
                    const Vector3T<T> rejected = v - axis * dot(axis, v);
                    const T rejected_length = length(rejected);
                    if (rejected_length > T(1e-9))
                        return rejected * (T(1) / rejected_length);
                    // Parallel to the strut: no axle direction survives, so the fallback
                    // is orthogonalized in turn rather than returned as authored.
                    const Vector3T<T> second = fallback - axis * dot(axis, fallback);
                    const T second_length = length(second);
                    return second_length > T(1e-9) ? second * (T(1) / second_length)
                                                   : Vector3T<T>{T(1), T(0), T(0)};
                }

                SuspensionSetupT<T> setup_{};
                BodyHandle core_;
                BodyHandle carrier_;
                BodyHandle wheel_;
                JointHandle strut_;
                JointHandle axle_;
                /** @brief The strut frame on the chassis before this corner's steer angle. */
                QuaternionT<T> base_basis_{};
                /** @brief The strut's upper pivot in the core's body frame. */
                Vector3T<T> mount_local_{};
                T rest_length_ = 0;
                T steer_angle_ = 0;
                T brake_torque_ = 0;
        };

        /** @brief The boundary corner setup: @ref SuspensionSetupT fixed to `Scalar`. */
        using SuspensionSetup = SuspensionSetupT<Scalar>;

        /** @brief The boundary corner: @ref SuspensionUnitT fixed to `Scalar`. */
        using SuspensionUnit = SuspensionUnitT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
