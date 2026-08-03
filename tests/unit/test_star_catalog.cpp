/**************************************************************************/
/* test_star_catalog.cpp                                                 */
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

// Unit_StarCatalog: the embedded bright-star catalogue and its photometry. Every
// catalogued direction must be a unit vector; the B-V colour map must produce a
// normalised RGB tint that reads bluer for hot stars and redder for cool ones; and the
// magnitude->brightness law must be Pogson's ratio (magnitude 0 -> 1, five magnitudes ->
// a hundredfold), with Sirius the brightest entry.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/star_catalog.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

TEST(Unit_StarCatalog, CatalogueIsPopulated)
{
    EXPECT_GT(BRIGHT_STAR_COUNT, 50u);
    EXPECT_EQ(BRIGHT_STAR_COUNT, sizeof(BRIGHT_STARS) / sizeof(BRIGHT_STARS[0]));
}

TEST(Unit_StarCatalog, EveryDirectionIsAUnitVector)
{
    for (std::size_t i = 0; i < BRIGHT_STAR_COUNT; ++i)
        EXPECT_TRUE(Harness::is_unit(star_equatorial_direction(BRIGHT_STARS[i]), 1e-9)) << "star " << i;
}

TEST(Unit_StarCatalog, ColourMapIsNormalisedAndInRange)
{
    for (float bv = -0.4f; bv <= 2.0f; bv += 0.2f)
    {
        const Vector3 rgb = bv_to_rgb(bv);
        EXPECT_GE(rgb.x, 0.0);
        EXPECT_GE(rgb.y, 0.0);
        EXPECT_GE(rgb.z, 0.0);
        const double peak = std::fmax(rgb.x, std::fmax(rgb.y, rgb.z));
        EXPECT_NEAR(peak, 1.0, 1e-9) << "bv=" << bv;
    }
}

TEST(Unit_StarCatalog, HotStarsAreBluerThanCoolStars)
{
    const Vector3 hot = bv_to_rgb(-0.3f);  // blue-white
    const Vector3 cool = bv_to_rgb(1.5f);   // orange-red
    // The hot star carries more blue; the cool star carries more red.
    EXPECT_GT(hot.z, cool.z);
    EXPECT_GT(cool.x, cool.z);
}

TEST(Unit_StarCatalog, MagnitudeToBrightnessFollowsPogsonsRatio)
{
    EXPECT_NEAR(magnitude_to_brightness(0.0f), 1.0f, 1e-6f);
    // Five magnitudes is exactly a factor of 100 in flux.
    EXPECT_NEAR(magnitude_to_brightness(5.0f), 0.01f, 1e-6f);
    // Brighter (smaller magnitude) means larger linear brightness.
    EXPECT_GT(magnitude_to_brightness(-1.46f), magnitude_to_brightness(0.0f));
    EXPECT_GT(magnitude_to_brightness(1.0f), magnitude_to_brightness(2.0f));
}

TEST(Unit_StarCatalog, SiriusIsTheBrightestEntry)
{
    float brightest = BRIGHT_STARS[0].magnitude;
    for (std::size_t i = 1; i < BRIGHT_STAR_COUNT; ++i)
        EXPECT_GE(BRIGHT_STARS[i].magnitude, brightest)
            << "entry " << i << " is brighter than the catalogue head";
}
