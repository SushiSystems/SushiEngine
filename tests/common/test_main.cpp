/**************************************************************************/
/* test_main.cpp                                                          */
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

#include <gtest/gtest.h>

/**
 * @brief Entry point for the SushiEngine test suite.
 *
 * Initializes GoogleTest and runs every registered case. The engine ships no
 * device code of its own, so each test translation unit instantiates its SYCL
 * kernels against the real runtime — there are no mocks.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return 0 on success, non-zero if any test fails.
 */
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
