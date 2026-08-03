/**************************************************************************/
/* main.cpp                                                               */
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

// Desktop entry point: a thin `while(!app.should_quit())` outside `PlayerApp`, which
// assumes no particular loop shape itself — a future mobile host drives `frame()`
// from its own OS-owned callback instead of a loop like this one. `--headless` picks a
// different, fixed-frame-count loop instead (PLATFORM0 S6): the CI-shaped one, driven
// with a deterministic tick rather than wall-clock, since nothing displays this run
// and reproducibility across machines matters more than realtime pacing.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <string>

#include "boot_manifest.hpp"
#include "player_app.hpp"

namespace
{
    // What a shipped, double-click-launched build reads with no CLI arguments at all:
    // this file beside wherever the process's current directory ends up, which is the
    // executable's own directory for a double-clicked .exe on Windows.
    constexpr const char* DEFAULT_MANIFEST_NAME = "boot.json";

    // A fixed tick for --headless rather than wall-clock: a CI run's whole point is to
    // reproduce the same result on whatever machine runs it, and wall-clock delta makes
    // the world's fixed-step accumulator (ISimulation::tick's own FixedTimestepClock)
    // advance by a different number of steps depending on how fast that machine is.
    constexpr double HEADLESS_FIXED_DELTA_SECONDS = 1.0 / 60.0;
    constexpr int DEFAULT_HEADLESS_FRAME_COUNT = 60;
} // namespace

int main(int argc, char** argv)
{
    try
    {
        SushiEngine::Player::BootManifest manifest;

        // First pass: an explicit --manifest has to be found before anything else is
        // applied, since everything that follows is allowed to override what it loads.
        std::string manifest_path;
        for (int i = 1; i < argc; ++i)
            if (std::string(argv[i]) == "--manifest" && i + 1 < argc)
                manifest_path = argv[++i];
        if (manifest_path.empty() && std::filesystem::exists(DEFAULT_MANIFEST_NAME))
            manifest_path = DEFAULT_MANIFEST_NAME;
        if (!manifest_path.empty())
            SushiEngine::Player::load_boot_manifest(manifest_path, manifest);

        // Second pass: CLI arguments layered on top of the manifest (or its defaults,
        // if none was found) — the developer's terminal always wins over the shipped
        // config, which is what makes local testing possible without editing the file.
        bool headless = false;
        int frame_count = DEFAULT_HEADLESS_FRAME_COUNT;
        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--manifest")
                ++i; // path already consumed above
            else if (arg == "--validation")
                manifest.enable_validation = true;
            else if (arg == "--scene" && i + 1 < argc)
                manifest.scene_path = argv[++i];
            else if (arg == "--headless")
                headless = true;
            else if (arg == "--frames" && i + 1 < argc)
                frame_count = std::atoi(argv[++i]);
            else if (arg.rfind("--", 0) != 0)
                manifest.scene_path = arg; // a bare positional path
        }

        SushiEngine::Player::PlayerApp::Description desc;
        desc.scene_path = manifest.scene_path;
        desc.window_title = manifest.window_title;
        desc.width = manifest.width;
        desc.height = manifest.height;
        desc.enable_validation = manifest.enable_validation;
        desc.organization = manifest.organization;
        desc.application = manifest.application;
        desc.headless = headless;

        SushiEngine::Player::PlayerApp app;
        app.start(desc);

        if (headless)
        {
            int frames_run = 0;
            for (; frames_run < frame_count && !app.should_quit(); ++frames_run)
                app.frame(HEADLESS_FIXED_DELTA_SECONDS);
            std::fprintf(stdout, "SushiEngine player: %d headless frame(s) completed.\n",
                        frames_run);
        }
        else
        {
            std::chrono::steady_clock::time_point last_frame_time =
                std::chrono::steady_clock::now();
            while (!app.should_quit())
            {
                const std::chrono::steady_clock::time_point frame_time =
                    std::chrono::steady_clock::now();
                const double real_delta_seconds =
                    std::chrono::duration<double>(frame_time - last_frame_time).count();
                last_frame_time = frame_time;
                app.frame(real_delta_seconds);
            }
        }

        app.shutdown();
        return 0;
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "SushiEngine player: %s\n", error.what());
        return 1;
    }
}
