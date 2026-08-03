/**************************************************************************/
/* test_soft_body_heat.cpp                                                */
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

// P6-G5's readable half: what a soft-body debug view *means*, with the drawing
// left out. A picture cannot be asserted, but every decision that makes the
// picture worth looking at can be — that full scale is a material threshold and
// not the body's own maximum, that a body at rest is not painted red, and that a
// reading past full scale does not wrap around to looking calm.
//
// Those three are the ones worth a test because each is a mistake that produces a
// *plausible* picture. A view normalized against the body's maximum looks like a
// working heat map right up until somebody trusts it.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <SushiEngine/authoring/soft_body_heat.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Authoring;

namespace
{
    Physics::SoftBodyMaterialT<Scalar> steel_like()
    {
        Physics::SoftBodyMaterialT<Scalar> material;
        material.young_modulus = Scalar(2e11);
        material.poisson_ratio = Scalar(0.3);
        material.density = Scalar(7800);
        material.yield_stress = Scalar(2.5e8);
        material.maximum_plastic_strain = Scalar(0.2);
        return material;
    }

    Simulation::SoftBodyElementSample sample(Scalar stress, Scalar plastic)
    {
        Simulation::SoftBodyElementSample element;
        element.von_mises_stress = stress;
        element.plastic_strain = plastic;
        return element;
    }
} // namespace

TEST(Unit_SoftBodyHeat, TheRampRunsColdToHotAndHitsItsCornersExactly)
{
    // The corner colours are what makes the ramp readable without a legend, so a
    // formula that only approaches them would turn every reading into a guess.
    const Authoring::HeatColour cold = Authoring::heat_colour(0.0f);
    EXPECT_FLOAT_EQ(cold.b, 1.0f);
    EXPECT_FLOAT_EQ(cold.r, 0.0f);
    EXPECT_FLOAT_EQ(cold.g, 0.0f);

    const Authoring::HeatColour green = Authoring::heat_colour(0.5f);
    EXPECT_FLOAT_EQ(green.g, 1.0f);
    EXPECT_FLOAT_EQ(green.r, 0.0f);
    EXPECT_FLOAT_EQ(green.b, 0.0f);

    const Authoring::HeatColour hot = Authoring::heat_colour(1.0f);
    EXPECT_FLOAT_EQ(hot.r, 1.0f);
    EXPECT_FLOAT_EQ(hot.g, 0.0f);
    EXPECT_FLOAT_EQ(hot.b, 0.0f);

    // Monotone in the direction a reader assumes: red never falls as the reading
    // rises, blue never rises.
    float previous_red = -1.0f;
    float previous_blue = 2.0f;
    for (int step = 0; step <= 40; ++step)
    {
        const Authoring::HeatColour colour =
            Authoring::heat_colour(static_cast<float>(step) / 40.0f);
        EXPECT_GE(colour.r, previous_red - 1e-6f) << "red fell at step " << step;
        EXPECT_LE(colour.b, previous_blue + 1e-6f) << "blue rose at step " << step;
        previous_red = colour.r;
        previous_blue = colour.b;
    }
}

TEST(Unit_SoftBodyHeat, AReadingPastFullScaleClampsRatherThanWrapping)
{
    // The failure this rules out is specific and nasty: a wrapped ramp paints the
    // most broken element the same colour as the calmest one.
    const Authoring::HeatColour over = Authoring::heat_colour(4.0f);
    EXPECT_FLOAT_EQ(over.r, 1.0f);
    EXPECT_FLOAT_EQ(over.b, 0.0f);

    const Authoring::HeatColour under = Authoring::heat_colour(-2.0f);
    EXPECT_FLOAT_EQ(under.b, 1.0f);
    EXPECT_FLOAT_EQ(under.r, 0.0f);

    // NaN reads as the coldest rather than as an arbitrary channel value. A stress
    // that went non-finite is a simulation that has already failed, and the view
    // should not be the thing that decides what colour that is.
    const Authoring::HeatColour nan_colour =
        Authoring::heat_colour(std::numeric_limits<float>::quiet_NaN());
    EXPECT_FLOAT_EQ(nan_colour.b, 1.0f);
    EXPECT_FLOAT_EQ(nan_colour.r, 0.0f);
}

TEST(Unit_SoftBodyHeat, FullScaleIsTheMaterialsYieldAndNotTheBodysWorstElement)
{
    // The design decision the whole view rests on. An element at a tenth of yield
    // reads as a tenth — not as "the worst in this body", which is what a
    // maximum-relative scale would say and which would be red on a body at rest.
    const Physics::SoftBodyMaterialT<Scalar> material = steel_like();

    EXPECT_NEAR(Authoring::soft_body_element_intensity(Authoring::SoftBodyDebugView::Stress,
                                            sample(Scalar(2.5e7), Scalar(0)), material),
                0.1f, 1e-5f);
    EXPECT_NEAR(Authoring::soft_body_element_intensity(Authoring::SoftBodyDebugView::Stress,
                                            sample(material.yield_stress, Scalar(0)), material),
                1.0f, 1e-5f);
    // Past yield still reads as full scale rather than beyond it.
    EXPECT_NEAR(Authoring::soft_body_element_intensity(Authoring::SoftBodyDebugView::Stress,
                                            sample(Scalar(1e10), Scalar(0)), material),
                1.0f, 1e-5f);
}

TEST(Unit_SoftBodyHeat, ABodyAtRestIsNotPaintedRed)
{
    // Stated as its own case because it is the observable consequence of the
    // decision above, and the one a reviewer would actually notice was wrong.
    const Physics::SoftBodyMaterialT<Scalar> material = steel_like();
    const float intensity = Authoring::soft_body_element_intensity(
        Authoring::SoftBodyDebugView::Stress, sample(Scalar(0), Scalar(0)), material);
    EXPECT_FLOAT_EQ(intensity, 0.0f);
    EXPECT_FLOAT_EQ(Authoring::heat_colour(intensity).r, 0.0f);
}

TEST(Unit_SoftBodyHeat, PlasticStrainScalesAgainstTheHardeningCeiling)
{
    const Physics::SoftBodyMaterialT<Scalar> material = steel_like();
    EXPECT_NEAR(Authoring::soft_body_element_intensity(Authoring::SoftBodyDebugView::PlasticStrain,
                                            sample(Scalar(0), Scalar(0.05)), material),
                0.25f, 1e-5f);
    EXPECT_NEAR(Authoring::soft_body_element_intensity(
                    Authoring::SoftBodyDebugView::PlasticStrain,
                    sample(Scalar(0), material.maximum_plastic_strain), material),
                1.0f, 1e-5f);
}

TEST(Unit_SoftBodyHeat, APurelyElasticMaterialHasNoScaleAndSaysSo)
{
    // No yield stress means no threshold, so there is no honest normalization —
    // and inventing one (against the body's own maximum, say) would paint an
    // elastic body that can never deform permanently as if it were failing.
    Physics::SoftBodyMaterialT<Scalar> elastic = steel_like();
    elastic.yield_stress = Scalar(0);
    elastic.maximum_plastic_strain = Scalar(0);

    EXPECT_FLOAT_EQ(Authoring::soft_body_element_intensity(Authoring::SoftBodyDebugView::Stress,
                                                sample(Scalar(1e9), Scalar(0)), elastic),
                    0.0f);
    EXPECT_FLOAT_EQ(Authoring::soft_body_element_intensity(
                        Authoring::SoftBodyDebugView::PlasticStrain,
                        sample(Scalar(0), Scalar(0.5)), elastic),
                    0.0f);
}

TEST(Unit_SoftBodyHeat, TheStructuralViewsCarryNoReading)
{
    // Off and Wireframe are not heat maps, and answering anything but zero for them
    // would mean a wireframe silently picked up a colour from whichever field
    // happened to be non-zero.
    const Physics::SoftBodyMaterialT<Scalar> material = steel_like();
    const Simulation::SoftBodyElementSample hot = sample(Scalar(1e10), Scalar(1));
    EXPECT_FLOAT_EQ(
        Authoring::soft_body_element_intensity(Authoring::SoftBodyDebugView::Off, hot,
                                               material),
        0.0f);
    EXPECT_FLOAT_EQ(
        Authoring::soft_body_element_intensity(Authoring::SoftBodyDebugView::Wireframe, hot,
                                               material),
        0.0f);
}
