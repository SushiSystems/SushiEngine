/**************************************************************************/
/* test_fixed_timestep.cpp                                              */
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

// Unit_FixedTimestep: SushiLoop's Time layer. The clock turns a caller-supplied elapsed
// duration into a whole number of fixed steps plus an interpolation remainder, and never
// reads the wall clock. The determinism-critical property is that the number of steps
// over a run depends only on the total time fed, not on how it was chunked across frames.

#include <gtest/gtest.h>

#include <SushiEngine/loop/fixed_timestep.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Loop;

namespace
{
    constexpr Scalar DT = Scalar(0.016);

    // Runs a full loop: feed each supplied real delta, consume every due step, return
    // the total step count.
    template <std::size_t N>
    int run(const Scalar (&deltas)[N])
    {
        FixedTimestepClock clock(DT);
        int steps = 0;
        for (Scalar d : deltas)
        {
            clock.accumulate(d);
            while (clock.consume_step())
                ++steps;
        }
        return steps;
    }
}

TEST(Unit_FixedTimestep, ExposesItsFixedInterval)
{
    FixedTimestepClock clock(DT);
    EXPECT_DOUBLE_EQ(clock.fixed_dt(), DT);
}

TEST(Unit_FixedTimestep, ConsumesWholeStepsAndReportsTheRemainder)
{
    FixedTimestepClock clock(DT);
    clock.accumulate(Scalar(0.05)); // 0.05 / 0.016 = 3.125 steps
    int steps = 0;
    while (clock.consume_step())
        ++steps;
    EXPECT_EQ(steps, 3);
    // Leftover 0.002 s of a 0.016 s step -> 0.125 interpolation fraction.
    EXPECT_NEAR(clock.interpolation(), 0.125, 1e-9);
}

TEST(Unit_FixedTimestep, YieldsNoStepBelowTheThreshold)
{
    FixedTimestepClock clock(DT);
    clock.accumulate(Scalar(0.010)); // less than one step
    EXPECT_FALSE(clock.consume_step());
    EXPECT_NEAR(clock.interpolation(), 0.010 / 0.016, 1e-9);
}

TEST(Unit_FixedTimestep, StepCountIsIndependentOfChunking)
{
    // Same total elapsed time (1.0 s), fed in different frame sizes, must produce the
    // same number of simulation steps -- the determinism property replay relies on.
    const Scalar coarse[] = {Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1),
                             Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1), Scalar(0.1)};
    Scalar fine[100];
    for (Scalar& f : fine)
        f = Scalar(0.01);

    const int coarse_steps = run(coarse);
    const int fine_steps = run(fine);
    EXPECT_EQ(coarse_steps, fine_steps);
    EXPECT_EQ(coarse_steps, 62); // floor(1.0 / 0.016) = 62
}

TEST(Unit_FixedTimestep, InterpolationStaysInUnitInterval)
{
    FixedTimestepClock clock(DT);
    const Scalar feed[] = {Scalar(0.003), Scalar(0.019), Scalar(0.007), Scalar(0.05),
                           Scalar(0.031)};
    for (Scalar d : feed)
    {
        clock.accumulate(d);
        while (clock.consume_step())
        {
        }
        EXPECT_GE(clock.interpolation(), 0.0);
        EXPECT_LT(clock.interpolation(), 1.0);
    }
}
