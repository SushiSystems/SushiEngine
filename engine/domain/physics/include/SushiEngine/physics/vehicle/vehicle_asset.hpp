/**************************************************************************/
/* vehicle_asset.hpp                                                      */
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
 * @file vehicle_asset.hpp
 * @brief What a whole car is, as one authored record.
 *
 * §11.2's closing sentence is the reason this type exists: *"`VehicleInstance` names a
 * `NodeBeamAsset`, and an asset whose rigid core is empty is a pure node-beam vehicle.
 * The architecture does not choose; the asset does."* A vehicle is therefore not a
 * class hierarchy with a hybrid branch and a pure branch — it is this record, and the
 * two branches are two sets of numbers in it.
 *
 * It names five things: the cooked structure, the collision shape the rigid core
 * presents, the corners hung off that core, the drivetrain that turns them, and
 * what the airflow does to the body. **Nothing in `physics/vehicle` dereferences
 * the first two.** They are identifiers the layer that owns assets
 * resolves — the same relationship `sim::Collider::asset` already has to
 * `CollisionAssetId`, and the same width, so a boundary record can assign across
 * without a cast. Putting an asset *store* behind this seam would make `physics/`
 * depend on where files live, which §3.1 spends a whole layer avoiding.
 *
 * The rest, by contrast, is read here and only here: a spring rate is physics, not a
 * filename.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/aero/wind.hpp>
#include <SushiEngine/physics/core/material.hpp>
#include <SushiEngine/physics/vehicle/powertrain.hpp>
#include <SushiEngine/physics/vehicle/suspension.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief One vehicle, as an author states it.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct VehicleAssetT
        {
            /**
             * @brief Identifier of the `.sushinodebeam` this vehicle's structure is cooked into.
             *
             * Resolved outside `physics/vehicle`; `VehicleInstanceT::create` is handed
             * the loaded view rather than reaching for it, so a vehicle can be
             * instanced from bytes that never came from a file — which is what every
             * test in the suite does.
             */
            std::uint32_t structure_asset = 0;

            /**
             * @brief Identifier of the `.sushicollision` the rigid core collides as.
             *
             * The shell collides as its own node cloud's surface (§9.6); the core is a
             * mass and an inertia tensor with no geometry of its own, and this is where
             * its geometry comes from. Zero is a core that presents no shape — correct
             * for a vehicle whose shell covers it entirely, which is most of them.
             */
            std::uint32_t core_collision_asset = 0;

            /** @brief The corners, in whatever order the author lists them. */
            std::vector<SuspensionSetupT<T>> corners;

            /**
             * @brief The body's aerodynamics (§11.6).
             *
             * Drag is written into the core body's own quadratic-drag constant, so the
             * wind field reaches a car through exactly the same path it reaches a flag on
             * a pole. Downforce is not, because it is not drag: it acts along the car's
             * own down axis and at a point that is not the centre of mass, which is why
             * downforce changes a car's *balance* and not merely its grip.
             */
            struct Aerodynamics
            {
                /** @brief Frontal area, in square metres; zero switches the body's aero off. */
                T frontal_area = T(2.2);

                /** @brief Drag coefficient, dimensionless — the number a car is quoted by. */
                T drag_coefficient = T(0.32);

                /**
                 * @brief Downforce coefficient over @ref frontal_area, dimensionless.
                 *
                 * Zero for an ordinary road car, which is the honest default: a saloon
                 * generates a little *lift*, and pretending it makes downforce is how a
                 * road car ends up cornering like a prototype.
                 */
                T downforce_coefficient = T(0);

                /**
                 * @brief Where the aerodynamic force acts, in the vehicle asset's own space.
                 *
                 * Behind the centre of mass on a car with a wing, ahead of it on one with
                 * a splitter, and the difference is the whole of aerodynamic balance. It
                 * is authored in asset space, like every other position in
                 * @ref SuspensionSetupT, because it is a property of the car.
                 */
                Vector3T<T> center_of_pressure{};

                /** @brief Air density, in kg/m³. */
                T air_density = sea_level_air_density<T>;

                /**
                 * @brief The vehicle's own up axis, in asset space.
                 *
                 * Downforce pushes along its negation, so a car does not lose its
                 * downforce by being on a banked corner — which it would if the force
                 * were applied straight down the world.
                 */
                Vector3T<T> up{T(0), T(1), T(0)};
            };

            /** @brief What the airflow does to the body. */
            Aerodynamics aerodynamics;

            /**
             * @brief The drivetrain hung between the engine and whichever corners drive.
             *
             * Read here for the same reason the corners are: a torque curve is physics,
             * not a filename. A default-constructed one has no torque curve and no
             * gears, which @ref PowertrainT::configure refuses and
             * @ref VehicleInstanceT::create then treats as an unpowered vehicle — a
             * trailer, a wreck, a physics-test chassis — rather than as a failure.
             */
            PowertrainSettingsT<T> powertrain;

            /**
             * @brief The surfaces this vehicle's bodies collide as, indexed by material index.
             *
             * The table `SuspensionSetupT::material_index`, @ref node_material_index and
             * @ref core_material_index point into. A car is the one place a body's material
             * has to be an index rather than a value: an ordinary rigid body carries its
             * `PhysicsMaterial` on its own description, but a vehicle is one record standing
             * for four hundred bodies, and a table indexed per body is the only way that
             * record can say the wheels are one surface and the panels another.
             *
             * Empty is the ordinary case: an index that names no entry resolves to
             * `PhysicsMaterialT`'s own defaults, which describe an ordinary solid.
             */
            std::vector<PhysicsMaterialT<T>> materials;

            /** @brief Which of @ref materials the shell's nodes collide as. */
            std::uint32_t node_material_index = 0;

            /**
             * @brief Which of @ref materials the rigid core collides as.
             *
             * Its own, for `NodeBeamStructureSettings`' reason: the shell is the panel that
             * touches the world and the core is a mass whose collision shape is authored
             * separately (§11.2).
             */
            std::uint32_t core_material_index = 0;
        };

        /** @brief The boundary vehicle asset: @ref VehicleAssetT fixed to `Scalar`. */
        using VehicleAsset = VehicleAssetT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
