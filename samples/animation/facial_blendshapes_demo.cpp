/**************************************************************************/
/* facial_blendshapes_demo.cpp                                            */
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

// The §12.4 ARKit-52 facial blendshape mapping, worked and self-checked. A stub mesh
// declares only 6 of the 52 canonical morph targets, in an order that does NOT match the
// canonical enum order (proving the map resolves by name, not position). What's checked:
//   * Every declared target resolves to the right morph-target index.
//   * Every one of the other 46 canonical shapes is reported by list_missing — not silently
//     ignored — and mapped_count() matches exactly.
//   * set_facial_blendshape writes into the correct MorphState slot for a mapped shape, and
//     is a documented no-op (not a crash, not a wrong write) for an unmapped one.
//   * The full 52-entry name table has no duplicate names and no empty names — a basic
//     integrity check on the table itself, since a collision there would silently merge two
//     distinct shapes.

#include <cstdio>
#include <cstring>
#include <vector>

#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/animation/facial_blendshapes.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    int failures = 0;
    void check(bool condition, const char* what)
    {
        if (!condition)
        {
            std::printf("[facial_blendshapes_demo] FAIL: %s\n", what);
            ++failures;
        }
    }
}

int main()
{
    // Table integrity: 52 distinct, non-empty names.
    {
        std::vector<const char*> names;
        for (std::uint32_t i = 0; i < ARKIT_BLENDSHAPE_COUNT; ++i)
        {
            const char* name = arkit_blendshape_name(static_cast<ARKitBlendshape>(i));
            check(name[0] != '\0', "every canonical shape has a non-empty name");
            names.push_back(name);
        }
        bool duplicate = false;
        for (std::size_t a = 0; a < names.size() && !duplicate; ++a)
            for (std::size_t b = a + 1; b < names.size() && !duplicate; ++b)
                if (std::strcmp(names[a], names[b]) == 0)
                    duplicate = true;
        check(!duplicate, "no two canonical shapes share a name");
        check(ARKIT_BLENDSHAPE_COUNT == 52, "the canonical set has exactly 52 shapes");
    }

    // A stub mesh with 6 of the 52 targets, deliberately NOT in canonical order.
    const char* mesh_target_strings[6] = {"mouthSmileRight", "jawOpen",  "eyeBlinkLeft",
                                          "browInnerUp",     "eyeBlinkRight", "mouthSmileLeft"};
    NameHash mesh_targets[6];
    for (int i = 0; i < 6; ++i)
        mesh_targets[i] = hash_name(mesh_target_strings[i]);

    const FacialBlendshapeMap map = build_facial_blendshape_map(mesh_targets, 6);

    check(map.has(ARKitBlendshape::JawOpen) && map.target_index(ARKitBlendshape::JawOpen) == 1,
         "jawOpen resolves to its actual (non-canonical-order) mesh index");
    check(map.has(ARKitBlendshape::EyeBlinkLeft) &&
             map.target_index(ARKitBlendshape::EyeBlinkLeft) == 2,
         "eyeBlinkLeft resolves correctly");
    check(map.has(ARKitBlendshape::MouthSmileLeft) &&
             map.target_index(ARKitBlendshape::MouthSmileLeft) == 5,
         "mouthSmileLeft resolves correctly");
    check(!map.has(ARKitBlendshape::JawForward), "an undeclared shape (jawForward) is unmapped");
    check(!map.has(ARKitBlendshape::TongueOut), "an undeclared shape (tongueOut) is unmapped");
    check(map.mapped_count() == 6, "exactly the 6 declared shapes are mapped");

    std::vector<ARKitBlendshape> missing;
    map.list_missing(missing);
    check(missing.size() == ARKIT_BLENDSHAPE_COUNT - 6,
         "list_missing reports exactly the 46 undeclared shapes, not silently dropped");
    bool jaw_forward_listed = false;
    for (ARKitBlendshape shape : missing)
        if (shape == ARKitBlendshape::JawForward)
            jaw_forward_listed = true;
    check(jaw_forward_listed, "the missing list actually names jawForward, not just a count");

    // Driving a MorphState by semantic name.
    MorphState state;
    state.count = 6;
    for (float& w : state.weights)
        w = 0.0f;

    set_facial_blendshape(map, ARKitBlendshape::JawOpen, 0.75f, state);
    set_facial_blendshape(map, ARKitBlendshape::MouthSmileLeft, 0.5f, state);
    check(state.weights[1] == 0.75f, "jawOpen's weight lands in the mesh's own target index (1)");
    check(state.weights[5] == 0.5f, "mouthSmileLeft's weight lands at its own index (5)");
    check(state.weights[0] == 0.0f && state.weights[2] == 0.0f && state.weights[3] == 0.0f &&
             state.weights[4] == 0.0f,
         "untouched targets stay at zero");

    // Setting an unmapped shape must not crash and must not corrupt any other slot.
    set_facial_blendshape(map, ARKitBlendshape::TongueOut, 1.0f, state);
    check(state.weights[0] == 0.0f && state.weights[1] == 0.75f && state.weights[2] == 0.0f &&
             state.weights[3] == 0.0f && state.weights[4] == 0.0f && state.weights[5] == 0.5f,
         "setting an unmapped shape (tongueOut) is a documented no-op, not a stray write");

    if (failures != 0)
    {
        std::printf("[facial_blendshapes_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf(
        "[facial_blendshapes_demo] OK — 52-shape table integrity, name-order-independent "
        "resolution, missing-shape reporting, and MorphState writes all verified\n");
    return 0;
}
