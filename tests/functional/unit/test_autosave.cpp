/**************************************************************************/
/* test_autosave.cpp                                                      */
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

// The autosave policy, pinned: the clock runs only while a save would be
// meaningful, ineligibility resets it (the interval measures *continuous* dirty
// time), and a fire restarts the clock so saves are one interval apart.

#include <gtest/gtest.h>

#include "core/autosave.hpp"

using SushiEngine::Editor::AutosaveTimer;

TEST(Unit_Autosave, FiresAfterContinuousEligibleInterval)
{
    AutosaveTimer timer;
    for (int i = 0; i < 9; ++i)
        EXPECT_FALSE(timer.tick(true, 1.0, 10.0f));
    EXPECT_TRUE(timer.tick(true, 1.0, 10.0f));
    // The clock restarted: the next fire is one full interval later, not immediate.
    EXPECT_FALSE(timer.tick(true, 1.0, 10.0f));
}

TEST(Unit_Autosave, IneligibilityResetsTheClock)
{
    AutosaveTimer timer;
    for (int i = 0; i < 9; ++i)
        EXPECT_FALSE(timer.tick(true, 1.0, 10.0f));
    // One clean (or disabled, or pathless) frame discards the accumulated time…
    EXPECT_FALSE(timer.tick(false, 1.0, 10.0f));
    // …so eligibility must again hold for the whole interval before a fire.
    for (int i = 0; i < 9; ++i)
        EXPECT_FALSE(timer.tick(true, 1.0, 10.0f));
    EXPECT_TRUE(timer.tick(true, 1.0, 10.0f));
}

TEST(Unit_Autosave, NeverFiresWhileIneligible)
{
    AutosaveTimer timer;
    for (int i = 0; i < 100; ++i)
        EXPECT_FALSE(timer.tick(false, 60.0, 10.0f));
}
