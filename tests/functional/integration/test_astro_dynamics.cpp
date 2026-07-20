/**************************************************************************/
/* test_astro_dynamics.cpp                                              */
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

// Integration_AstroDynamics: the free-body propagator built from a symplectic integrator
// and a sphere-of-influence rebase. The velocity-Verlet step must conserve orbital energy
// over a full revolution (the whole reason a symplectic scheme is used rather than Euler);
// the injected-field step must agree bit-for-bit with the hard-coded summation it
// generalises; and advancing a body in low Earth orbit through the real field must keep it
// bound and in Earth's frame, deterministically.

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/astro_dynamics.hpp>
#include <SushiEngine/astro/gravity.hpp>
#include <SushiEngine/astro/gravity_field.hpp>
#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/reference_frame.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

namespace
{
    // A stationary inverse-square field about the origin: a clean two-body test bed,
    // independent of date, for the integrator's conservation properties.
    class CentralField final : public IGravityField
    {
        public:
            explicit CentralField(double mu) noexcept : mu_(mu) {}

            WorldVector3 acceleration(const WorldVector3& position, double) const noexcept override
            {
                const double r2 = position.x * position.x + position.y * position.y +
                                  position.z * position.z;
                const double r = std::sqrt(r2);
                const double s = -mu_ / (r2 * r);
                return WorldVector3{s * position.x, s * position.y, s * position.z};
            }

        private:
            double mu_;
    };

    double radius(const WorldVector3& p)
    {
        return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    double speed(const WorldVector3& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    double specific_energy(const StateVector& s, double mu)
    {
        const double v = speed(s.velocity);
        return 0.5 * v * v - mu / radius(s.position);
    }
}

TEST(Integration_AstroDynamics, SymplecticStepConservesEnergyOverAnOrbit)
{
    const double mu = 3.986004418e14; // Earth
    const double r = 1.0e7;
    const double v = std::sqrt(mu / r); // circular speed
    const CentralField field(mu);

    StateVector state{WorldVector3{r, 0.0, 0.0}, WorldVector3{0.0, v, 0.0}};
    const double energy0 = specific_energy(state, mu);

    const double period = 2.0 * 3.141592653589793 * r / v;
    const double dt = 2.0;
    const int steps = static_cast<int>(period / dt);

    double min_r = r;
    double max_r = r;
    for (int i = 0; i < steps; ++i)
    {
        state = integrate_step(state, J2000_JULIAN_DATE, dt, field);
        min_r = std::min(min_r, radius(state.position));
        max_r = std::max(max_r, radius(state.position));
    }

    // The orbit stays circular (radius barely breathes) and energy drifts negligibly.
    EXPECT_NEAR(min_r, r, 0.01 * r);
    EXPECT_NEAR(max_r, r, 0.01 * r);
    EXPECT_NEAR(specific_energy(state, mu), energy0, 1e-4 * std::fabs(energy0));

    // After one full period the body has returned close to where it started.
    EXPECT_NEAR(radius(state.position), r, 0.01 * r);
}

TEST(Integration_AstroDynamics, InjectedFieldStepMatchesTheHardCodedSummation)
{
    const SummedRailsGravityField field;
    const StateVector state{WorldVector3{1.4e11, 3.0e10, -2.0e9},
                            WorldVector3{-6.0e3, 2.7e4, 1.0e2}};

    const StateVector by_free = integrate_step(state, J2000_JULIAN_DATE, 60.0);
    const StateVector by_field = integrate_step(state, J2000_JULIAN_DATE, 60.0, field);

    EXPECT_EQ(by_free.position.x, by_field.position.x);
    EXPECT_EQ(by_free.position.y, by_field.position.y);
    EXPECT_EQ(by_free.position.z, by_field.position.z);
    EXPECT_EQ(by_free.velocity.x, by_field.velocity.x);
    EXPECT_EQ(by_free.velocity.y, by_field.velocity.y);
    EXPECT_EQ(by_free.velocity.z, by_field.velocity.z);
}

TEST(Integration_AstroDynamics, LowOrbitStaysBoundAndInEarthsFrame)
{
    const SummedRailsGravityField field;
    const double r = 1.0e7;
    const double v = std::sqrt(standard_gravitational_parameter(BodyId::Earth) / r);

    // State expressed in Earth's body-centred frame: a near-circular low orbit.
    StateVector local{WorldVector3{r, 0.0, 0.0}, WorldVector3{0.0, v, 0.0}};
    BodyId frame = BodyId::Earth;

    double jd = J2000_JULIAN_DATE;
    const double dt = 5.0;
    for (int i = 0; i < 2000; ++i) // ~1 revolution
    {
        advance_astro_state(local, frame, jd, dt, field);
        jd += dt / SECONDS_PER_DAY;
        // The body stays inside Earth's sphere of influence, so the frame never leaves Earth.
        ASSERT_EQ(frame, BodyId::Earth) << "rebased away from Earth at step " << i;
        // And it stays bound near its orbital radius (the Sun perturbs it only slightly).
        const double body_radius = radius(local.position);
        ASSERT_GT(body_radius, 0.9 * r) << "step " << i;
        ASSERT_LT(body_radius, 1.1 * r) << "step " << i;
    }
}

TEST(Integration_AstroDynamics, AdvanceIsDeterministic)
{
    const SummedRailsGravityField field;
    StateVector a{WorldVector3{1.0e7, 0.0, 0.0}, WorldVector3{0.0, 6.0e3, 0.0}};
    StateVector b = a;
    BodyId fa = BodyId::Earth;
    BodyId fb = BodyId::Earth;

    for (int i = 0; i < 100; ++i)
    {
        advance_astro_state(a, fa, J2000_JULIAN_DATE, 5.0, field);
        advance_astro_state(b, fb, J2000_JULIAN_DATE, 5.0, field);
    }
    EXPECT_EQ(fa, fb);
    EXPECT_EQ(a.position.x, b.position.x);
    EXPECT_EQ(a.position.y, b.position.y);
    EXPECT_EQ(a.velocity.z, b.velocity.z);
}
