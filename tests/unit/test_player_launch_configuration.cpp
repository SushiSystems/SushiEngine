/**************************************************************************/
/* test_player_launch_configuration.cpp                                   */
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

// How a shipped player decides what to launch: a `boot.json` beside the executable,
// overlaid with whatever the terminal said. Both halves are checked here because both
// are silent when they go wrong — a manifest field dropped on the floor and a `--scene`
// that loses to the file it was meant to override look identical from the outside (the
// player comes up showing the wrong world) and neither raises anything.
//
// The refusals carry as much weight as the round trip. A launch configuration is read
// from a file a user can edit and hand-write, so the cases worth having are the ones a
// hand-written file actually produces: a truncated document, a JSON array where an
// object belongs, and a field spelled with the wrong type. None of those may cost the
// launch, and none may leave the manifest half-overwritten.

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "boot_manifest.hpp"
#include "launch_configuration.hpp"

namespace
{
    namespace fs = std::filesystem;
    using SushiEngine::Player::BootManifest;
    using SushiEngine::Player::LaunchConfiguration;
    using SushiEngine::Player::load_boot_manifest;
    using SushiEngine::Player::resolve_launch_configuration;

    /** @brief A scratch manifest in the system temp dir, written on construction and
     *         removed on destruction. */
    class ScratchManifest
    {
        public:
            ScratchManifest(const char* name, const std::string& contents)
                : path_((fs::temp_directory_path() / name).string())
            {
                std::ofstream output(path_, std::ios::binary | std::ios::trunc);
                output << contents;
            }

            ~ScratchManifest()
            {
                std::error_code error;
                fs::remove(path_, error);
            }

            ScratchManifest(const ScratchManifest&) = delete;
            ScratchManifest& operator=(const ScratchManifest&) = delete;

            const std::string& path() const noexcept { return path_; }

        private:
            std::string path_;
    };

    /** @brief A path in the temp dir that is guaranteed not to exist. */
    std::string missing_path(const char* name)
    {
        const std::string path = (fs::temp_directory_path() / name).string();
        std::error_code error;
        fs::remove(path, error);
        return path;
    }

    /**
     * @brief Resolves a launch from @p arguments the way `main` does, program name and all.
     * @param arguments The arguments after the program name.
     * @param default_manifest_path What `main` passes as `boot.json`; empty reads no default.
     * @return The resolved configuration.
     */
    LaunchConfiguration resolve(const std::vector<std::string>& arguments,
                                const std::string& default_manifest_path = std::string())
    {
        std::vector<const char*> raw;
        raw.push_back("se_player");
        for (const std::string& argument : arguments)
            raw.push_back(argument.c_str());
        return resolve_launch_configuration(static_cast<int>(raw.size()), raw.data(),
                                            default_manifest_path);
    }

    /** @brief Every field of a manifest, spelled with values no default shares. */
    const char* const FULL_MANIFEST = R"({
        "scene_path": "worlds/arena.sushiscene",
        "window_title": "Arena",
        "width": 1920,
        "height": 1080,
        "organization": "Publisher",
        "application": "Arena",
        "enable_validation": true
    })";
} // namespace

TEST(Unit_PlayerLaunchConfiguration, NoArgumentsAndNoManifestGiveTheCompiledInDefaults)
{
    const LaunchConfiguration launch = resolve({});

    EXPECT_TRUE(launch.manifest.scene_path.empty());
    EXPECT_EQ(launch.manifest.window_title, "SushiEngine Player");
    EXPECT_EQ(launch.manifest.width, 1280u);
    EXPECT_EQ(launch.manifest.height, 720u);
    EXPECT_EQ(launch.manifest.organization, "SushiSystems");
    EXPECT_EQ(launch.manifest.application, "SushiEnginePlayer");
    EXPECT_FALSE(launch.manifest.enable_validation);
    EXPECT_FALSE(launch.headless);
    EXPECT_EQ(launch.frame_count, 60);
}

TEST(Unit_PlayerLaunchConfiguration, ANamedManifestSuppliesEveryField)
{
    const ScratchManifest manifest("se_player_full.json", FULL_MANIFEST);

    const LaunchConfiguration launch = resolve({"--manifest", manifest.path()});

    EXPECT_EQ(launch.manifest.scene_path, "worlds/arena.sushiscene");
    EXPECT_EQ(launch.manifest.window_title, "Arena");
    EXPECT_EQ(launch.manifest.width, 1920u);
    EXPECT_EQ(launch.manifest.height, 1080u);
    EXPECT_EQ(launch.manifest.organization, "Publisher");
    EXPECT_EQ(launch.manifest.application, "Arena");
    EXPECT_TRUE(launch.manifest.enable_validation);
}

TEST(Unit_PlayerLaunchConfiguration, TheDefaultManifestIsReadWhenNoneIsNamed)
{
    const ScratchManifest manifest("se_player_default.json", FULL_MANIFEST);

    const LaunchConfiguration launch = resolve({}, manifest.path());

    EXPECT_EQ(launch.manifest.scene_path, "worlds/arena.sushiscene");
    EXPECT_EQ(launch.manifest.window_title, "Arena");
}

TEST(Unit_PlayerLaunchConfiguration, AnExplicitManifestDisplacesTheDefaultOne)
{
    const ScratchManifest named("se_player_named.json", R"({"window_title": "Named"})");
    const ScratchManifest fallback("se_player_fallback.json", FULL_MANIFEST);

    const LaunchConfiguration launch = resolve({"--manifest", named.path()}, fallback.path());

    EXPECT_EQ(launch.manifest.window_title, "Named");
    // The default manifest was not read at all, so its fields are absent rather than merged.
    EXPECT_TRUE(launch.manifest.scene_path.empty());
    EXPECT_EQ(launch.manifest.width, 1280u);
}

TEST(Unit_PlayerLaunchConfiguration, TheCommandLineOverridesTheManifestItLoaded)
{
    const ScratchManifest manifest("se_player_override.json", FULL_MANIFEST);

    // --scene appears before --manifest to prove the order of the arguments does not
    // decide the precedence: the manifest is always loaded first and overridden second.
    const LaunchConfiguration launch =
        resolve({"--scene", "worlds/lobby.sushiscene", "--manifest", manifest.path()});

    EXPECT_EQ(launch.manifest.scene_path, "worlds/lobby.sushiscene");
    // Everything the command line did not name still comes from the manifest.
    EXPECT_EQ(launch.manifest.window_title, "Arena");
    EXPECT_EQ(launch.manifest.width, 1920u);
}

TEST(Unit_PlayerLaunchConfiguration, ABarePositionalPathIsTheScene)
{
    const LaunchConfiguration launch = resolve({"worlds/lobby.sushiscene"});

    EXPECT_EQ(launch.manifest.scene_path, "worlds/lobby.sushiscene");
}

TEST(Unit_PlayerLaunchConfiguration, ValidationIsAFlagRatherThanAValue)
{
    const LaunchConfiguration launch = resolve({"--validation"});

    EXPECT_TRUE(launch.manifest.enable_validation);
    // The flag consumes no operand, so a path after it is still the scene.
    const LaunchConfiguration with_scene = resolve({"--validation", "worlds/lobby.sushiscene"});
    EXPECT_TRUE(with_scene.manifest.enable_validation);
    EXPECT_EQ(with_scene.manifest.scene_path, "worlds/lobby.sushiscene");
}

TEST(Unit_PlayerLaunchConfiguration, HeadlessAndFrameCountComeOnlyFromTheCommandLine)
{
    // A manifest may not turn a product's window off: a broken install that came up
    // headless would be indistinguishable from a broken machine.
    const ScratchManifest manifest("se_player_headless.json",
                                   R"({"headless": true, "frames": 5})");

    const LaunchConfiguration from_file = resolve({"--manifest", manifest.path()});
    EXPECT_FALSE(from_file.headless);
    EXPECT_EQ(from_file.frame_count, 60);

    const LaunchConfiguration from_arguments = resolve({"--headless", "--frames", "12"});
    EXPECT_TRUE(from_arguments.headless);
    EXPECT_EQ(from_arguments.frame_count, 12);
}

TEST(Unit_PlayerLaunchConfiguration, AnOptionThatConsumesItsOperandNeverReadsItAsAScene)
{
    const ScratchManifest manifest("se_player_operand.json", R"({"window_title": "Arena"})");

    // Each of these operands would be a bare positional path if the option before it did
    // not claim it — the mistake that makes a player load a file named "12".
    const LaunchConfiguration launch =
        resolve({"--manifest", manifest.path(), "--frames", "12"});

    EXPECT_EQ(launch.manifest.window_title, "Arena");
    EXPECT_TRUE(launch.manifest.scene_path.empty());
    EXPECT_EQ(launch.frame_count, 12);
}

TEST(Unit_PlayerLaunchConfiguration, AnOptionMissingItsOperandIsIgnoredRatherThanFatal)
{
    const LaunchConfiguration launch = resolve({"--scene"});

    EXPECT_TRUE(launch.manifest.scene_path.empty());
    EXPECT_FALSE(launch.headless);
}

TEST(Unit_PlayerLaunchConfiguration, AMissingManifestCostsTheLaunchNothing)
{
    const std::string absent = missing_path("se_player_absent.json");

    const LaunchConfiguration named = resolve({"--manifest", absent});
    EXPECT_EQ(named.manifest.window_title, "SushiEngine Player");

    const LaunchConfiguration defaulted = resolve({}, absent);
    EXPECT_EQ(defaulted.manifest.window_title, "SushiEngine Player");
}

TEST(Unit_PlayerBootManifest, AWellFormedObjectIsAccepted)
{
    const ScratchManifest manifest("se_boot_accepted.json", FULL_MANIFEST);

    BootManifest fields;
    EXPECT_TRUE(load_boot_manifest(manifest.path(), fields));
    EXPECT_EQ(fields.scene_path, "worlds/arena.sushiscene");
}

TEST(Unit_PlayerBootManifest, OnlyTheFieldsTheDocumentNamesAreReplaced)
{
    const ScratchManifest manifest("se_boot_partial.json", R"({"width": 800})");

    BootManifest fields;
    fields.window_title = "Pre-seeded";
    ASSERT_TRUE(load_boot_manifest(manifest.path(), fields));

    EXPECT_EQ(fields.width, 800u);
    EXPECT_EQ(fields.window_title, "Pre-seeded");
    EXPECT_EQ(fields.height, 720u);
}

TEST(Unit_PlayerBootManifest, AMalformedDocumentIsRefusedWholeAndChangesNothing)
{
    // Truncated mid-object, the shape a half-written file on a full disk leaves behind.
    const ScratchManifest manifest("se_boot_truncated.json", R"({"width": 800,)");

    BootManifest fields;
    EXPECT_FALSE(load_boot_manifest(manifest.path(), fields));
    EXPECT_EQ(fields.width, 1280u);
    EXPECT_EQ(fields.height, 720u);
}

TEST(Unit_PlayerBootManifest, ADocumentThatIsNotAnObjectIsRefused)
{
    const ScratchManifest array("se_boot_array.json", R"([{"width": 800}])");

    BootManifest fields;
    EXPECT_FALSE(load_boot_manifest(array.path(), fields));
    EXPECT_EQ(fields.width, 1280u);
}

TEST(Unit_PlayerBootManifest, AMistypedFieldCostsOnlyItself)
{
    // A hand-written file's most common error: the number quoted, the flag spelled as a
    // string. Neither may throw out of the read, and neither may take the fields around
    // it down with it.
    const ScratchManifest manifest(
        "se_boot_mistyped.json",
        R"({"width": "1920", "enable_validation": "yes", "window_title": "Arena"})");

    BootManifest fields;
    ASSERT_TRUE(load_boot_manifest(manifest.path(), fields));

    EXPECT_EQ(fields.width, 1280u);
    EXPECT_FALSE(fields.enable_validation);
    EXPECT_EQ(fields.window_title, "Arena");
}

TEST(Unit_PlayerBootManifest, AnEmptyFileIsRefused)
{
    const ScratchManifest manifest("se_boot_empty.json", "");

    BootManifest fields;
    EXPECT_FALSE(load_boot_manifest(manifest.path(), fields));
    EXPECT_EQ(fields.window_title, "SushiEngine Player");
}
