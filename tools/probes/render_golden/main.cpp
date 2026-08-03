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

// RHI0's golden-image harness: render a fixed scene headlessly for a fixed number of
// frames, read the output image back, and compare it against a recorded reference.
// This is the safety net the whole RHI programme runs on
// (docs/slop/cross_platform_engineering_plan.md §5.6) — it is what can tell a
// behaviour-preserving refactor from a silent regression, which nothing in this
// repository could do before it.
//
// **What a golden pins, and what it cannot.** An exact hash is a statement about one
// GPU and one driver. A driver update changes rounding in the last bit of a handful of
// pixels and every hash in the set goes red at once, which is indistinguishable from a
// real regression if a hash is all you kept. So each golden also carries a coarse
// *thumbprint* — the frame averaged down to a 32x18 grid, in plain hexadecimal — and a
// mismatch is reported against it: a driver's rounding moves the thumbprint by a
// channel level or two, while a pass that stopped running moves it by tens. The hash
// answers "did anything change"; the thumbprint answers "is this the kind of change
// worth waking someone for".
//
// **What is deliberately not in the set yet.** Nothing that draws sky or cloud. The
// cloudscape and the atmosphere nest are under active development, and a golden that
// goes red every day is a golden people learn to ignore — the cases below switch both
// off at the Environment and are the mesh-shading half of the frame: depth, shadows,
// opaque, tonemap, TAA. They are added when that work settles, not before.

#include <cstdint>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/scene_view.hpp>

#include "material/asset_library.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "rhi/vulkan/vulkan_scene_view.hpp"

namespace
{
    using namespace SushiEngine;
    using namespace SushiEngine::Render;

    constexpr std::uint32_t WIDTH = 512;
    constexpr std::uint32_t HEIGHT = 288;

    /**
     * @brief Frames rendered before the image is read.
     *
     * Enough for the temporal resolve to have accumulated a full jitter cycle, because
     * a golden taken mid-convergence would pin the convergence rate rather than the
     * image — and the first thing any temporal change alters is the rate.
     */
    constexpr std::uint32_t FRAMES = 12;

    constexpr std::uint32_t THUMB_COLUMNS = 32;
    constexpr std::uint32_t THUMB_ROWS = 18;

    /** @brief A frame reduced to what a reviewer can read in a text diff. */
    struct Thumbprint
    {
        /** @brief THUMB_ROWS × THUMB_COLUMNS cells, three channels each, averaged. */
        std::uint8_t cells[THUMB_ROWS * THUMB_COLUMNS * 3] = {};
    };

    /** @brief The golden file format this build reads and writes. */
    constexpr unsigned GOLDEN_VERSION = 2;

    /** @brief One recorded reference: what the frame hashed to, and roughly looked like. */
    struct Golden
    {
        std::uint64_t hash = 0;
        Thumbprint thumbprint;
        /** @brief Per-pass hashes, in the order the recorded frame produced them. */
        std::vector<PassOutputHash> passes;
        bool present = false;
    };

    /**
     * @brief The key a pass hash is matched on across two runs.
     *
     * Pass name and resource name, plus which occurrence of that pair this is — the
     * shadow pass writes one cascade per invocation and they are not interchangeable.
     * Matching by key rather than by position is what lets a *reordered* frame still
     * compare, while a *renamed* pass correctly reads as a difference.
     */
    std::string pass_key(const PassOutputHash& entry, int occurrence)
    {
        return entry.pass + "/" + entry.resource + "#" + std::to_string(occurrence);
    }

    /**
     * @brief Whether a pass's hash is stable enough to belong in a golden.
     *
     * The atmosphere and cloudscape are being rewritten, and their passes register and
     * write whether or not the `Environment` switched their *effect* off. A golden that
     * goes red every day is one people learn to ignore, so their outputs are left out by
     * name until that work settles.
     *
     * This filter is the harness's judgement, not the engine's: `PassCapture` captures
     * everything it can reach, and which of that is worth pinning is a question about
     * this set of references rather than about the renderer.
     */
    bool stable_pass(const std::string& name)
    {
        static const char* const moving[] = {"cloud", "sky", "weather", "atmosphere"};
        for (const char* fragment : moving)
            if (name.find(fragment) != std::string::npos)
                return false;
        return true;
    }

    /**
     * @brief The one spelling of a name, used in the file and in every comparison.
     *
     * A name reaches here with spaces in it ("shadow cascades"), and the file format is
     * whitespace-separated, so it has to be folded to a single token somewhere. Doing
     * that only on the way out was a bug: the recorded name then never equalled the
     * live one, and every pass reported itself as simultaneously gone and new.
     */
    std::string token_of(const std::string& name)
    {
        std::string token = name.empty() ? std::string("unnamed") : name;
        for (char& character : token)
            if (character == ' ' || character == '\t' || character == '\n')
                character = '_';
        return token;
    }

    /**
     * @brief The passes this golden pins: normalised to their file spelling, filtered.
     *
     * Normalising here rather than at the point of writing is what keeps the recorded
     * and the live side of a comparison in the same vocabulary.
     */
    std::vector<PassOutputHash> golden_passes(const std::vector<PassOutputHash>& passes)
    {
        std::vector<PassOutputHash> kept;
        kept.reserve(passes.size());
        for (const PassOutputHash& entry : passes)
        {
            if (!stable_pass(entry.pass))
                continue;
            PassOutputHash normalised = entry;
            normalised.pass = token_of(entry.pass);
            normalised.resource = token_of(entry.resource);
            kept.push_back(std::move(normalised));
        }
        return kept;
    }

    /** @brief Keys @p passes in record order, numbering repeats of the same pair. */
    std::vector<std::string> pass_keys(const std::vector<PassOutputHash>& passes)
    {
        std::vector<std::string> keys;
        keys.reserve(passes.size());
        for (std::size_t i = 0; i < passes.size(); ++i)
        {
            int occurrence = 0;
            for (std::size_t j = 0; j < i; ++j)
                if (passes[j].pass == passes[i].pass &&
                    passes[j].resource == passes[i].resource)
                    ++occurrence;
            keys.push_back(pass_key(passes[i], occurrence));
        }
        return keys;
    }

    /**
     * @brief FNV-1a over the whole image.
     *
     * A cryptographic digest would say nothing more here: the adversary is a compiler
     * or a driver, not a forger, and every bit of the image is in the input.
     */
    std::uint64_t hash_image(const FrameImage& image) noexcept
    {
        std::uint64_t hash = 1469598103934665603ull;
        for (const std::uint8_t byte : image.rgba)
        {
            hash ^= byte;
            hash *= 1099511628211ull;
        }
        return hash;
    }

    /** @brief Averages @p image down to the reviewable grid. */
    Thumbprint thumbprint_of(const FrameImage& image) noexcept
    {
        Thumbprint print;
        if (image.width == 0 || image.height == 0)
            return print;

        for (std::uint32_t row = 0; row < THUMB_ROWS; ++row)
        {
            const std::uint32_t y0 = row * image.height / THUMB_ROWS;
            const std::uint32_t y1 = (row + 1) * image.height / THUMB_ROWS;
            for (std::uint32_t column = 0; column < THUMB_COLUMNS; ++column)
            {
                const std::uint32_t x0 = column * image.width / THUMB_COLUMNS;
                const std::uint32_t x1 = (column + 1) * image.width / THUMB_COLUMNS;
                std::uint64_t sum[3] = {0, 0, 0};
                std::uint64_t count = 0;
                for (std::uint32_t y = y0; y < y1; ++y)
                {
                    for (std::uint32_t x = x0; x < x1; ++x)
                    {
                        const std::size_t offset =
                            (static_cast<std::size_t>(y) * image.width + x) * 4;
                        sum[0] += image.rgba[offset + 0];
                        sum[1] += image.rgba[offset + 1];
                        sum[2] += image.rgba[offset + 2];
                        ++count;
                    }
                }
                if (count == 0)
                    continue;
                const std::size_t cell =
                    (static_cast<std::size_t>(row) * THUMB_COLUMNS + column) * 3;
                for (int channel = 0; channel < 3; ++channel)
                    print.cells[cell + channel] =
                        static_cast<std::uint8_t>(sum[channel] / count);
            }
        }
        return print;
    }

    /** @brief The largest and mean per-channel distance between two thumbprints. */
    void compare_thumbprints(const Thumbprint& a, const Thumbprint& b,
                             int& largest, double& mean) noexcept
    {
        largest = 0;
        std::uint64_t total = 0;
        constexpr std::size_t COUNT = sizeof(a.cells);
        for (std::size_t i = 0; i < COUNT; ++i)
        {
            const int delta = int(a.cells[i]) - int(b.cells[i]);
            const int magnitude = delta < 0 ? -delta : delta;
            if (magnitude > largest)
                largest = magnitude;
            total += static_cast<std::uint64_t>(magnitude);
        }
        mean = double(total) / double(COUNT);
    }

    std::string golden_path(const char* directory, const char* name)
    {
        return std::string(directory) + "/" + name + ".golden";
    }

    /** @brief Reads a recorded golden, or returns one marked absent. */
    Golden read_golden(const std::string& path)
    {
        Golden golden;
        std::ifstream file(path);
        if (!file)
            return golden;

        std::string magic;
        unsigned version = 0;
        unsigned width = 0;
        unsigned height = 0;
        unsigned frames = 0;
        unsigned long long hash = 0;
        file >> magic >> version >> width >> height >> frames >> hash;
        // A golden recorded at a different size or frame count is not this golden.
        // Refusing it rather than adapting is the point: the extent and the frame count
        // are inputs to the image, so a reference taken under others answers a different
        // question, and silently comparing against it would be the one failure mode a
        // regression harness cannot have.
        if (!file || magic != "sushi-golden" || version != GOLDEN_VERSION ||
            width != WIDTH || height != HEIGHT || frames != FRAMES)
            return golden;

        golden.hash = hash;
        for (std::size_t i = 0; i < sizeof(golden.thumbprint.cells); i += 3)
        {
            std::string cell;
            file >> cell;
            if (!file || cell.size() != 6)
                return Golden{};
            for (int channel = 0; channel < 3; ++channel)
                golden.thumbprint.cells[i + std::size_t(channel)] =
                    static_cast<std::uint8_t>(
                        std::stoul(cell.substr(std::size_t(channel) * 2, 2), nullptr, 16));
        }
        std::string marker;
        std::size_t pass_count = 0;
        file >> marker >> pass_count;
        // A golden with no pass section is not an older golden to be tolerated — the
        // version field already refused those. It is a truncated file.
        if (!file || marker != "passes")
            return Golden{};
        golden.passes.reserve(pass_count);
        for (std::size_t i = 0; i < pass_count; ++i)
        {
            std::string digits;
            PassOutputHash entry;
            file >> digits >> entry.pass >> entry.resource;
            if (!file)
                return Golden{};
            entry.hash = std::stoull(digits, nullptr, 16);
            golden.passes.push_back(std::move(entry));
        }

        golden.present = true;
        return golden;
    }

    /** @brief Writes a golden in the text form a review can read. */
    bool write_golden(const std::string& path, std::uint64_t hash, const Thumbprint& print,
                      const std::vector<PassOutputHash>& passes)
    {
        std::ofstream file(path);
        if (!file)
            return false;
        file << "sushi-golden " << GOLDEN_VERSION << " " << WIDTH << " " << HEIGHT << " "
             << FRAMES << " " << hash << "\n";
        file << std::hex << std::setfill('0');
        for (std::uint32_t row = 0; row < THUMB_ROWS; ++row)
        {
            for (std::uint32_t column = 0; column < THUMB_COLUMNS; ++column)
            {
                const std::size_t cell =
                    (static_cast<std::size_t>(row) * THUMB_COLUMNS + column) * 3;
                for (int channel = 0; channel < 3; ++channel)
                    file << std::setw(2)
                         << unsigned(print.cells[cell + std::size_t(channel)]);
                file << (column + 1 == THUMB_COLUMNS ? '\n' : ' ');
            }
        }
        file << std::dec << "passes " << passes.size() << "\n";
        file << std::hex << std::setfill('0');
        for (const PassOutputHash& entry : passes)
            // Already in file spelling: golden_passes() normalised them, which is what
            // keeps a recorded name equal to the live one it will be compared against.
            file << std::setw(16) << entry.hash << " " << entry.pass << " "
                 << entry.resource << "\n";
        return bool(file);
    }

    /**
     * @brief Describes how two runs' per-pass hashes differ, one line per difference.
     *
     * Returns the lines rather than printing them so the caller can decide what to say
     * first — and so the count is known before the header that announces it.
     *
     * Three findings, kept apart because they mean different things: a pass that stopped
     * producing an output was culled or removed, a pass that started producing one is
     * new, and a pass whose hash moved is the one a bisect is actually looking for.
     *
     * @param recorded The golden's passes.
     * @param actual   This run's passes.
     * @return One human-readable line per difference; empty when the two agree.
     */
    std::vector<std::string> pass_differences(const std::vector<PassOutputHash>& recorded,
                                              const std::vector<PassOutputHash>& actual)
    {
        const std::vector<std::string> recorded_keys = pass_keys(recorded);
        const std::vector<std::string> actual_keys = pass_keys(actual);
        std::vector<std::string> lines;

        const auto index_of = [](const std::vector<std::string>& keys,
                                 const std::string& key)
        {
            for (std::size_t i = 0; i < keys.size(); ++i)
                if (keys[i] == key)
                    return i;
            return keys.size();
        };

        char buffer[256];
        for (std::size_t i = 0; i < recorded_keys.size(); ++i)
        {
            const std::size_t match = index_of(actual_keys, recorded_keys[i]);
            if (match == actual_keys.size())
            {
                std::snprintf(buffer, sizeof(buffer), "gone     %s",
                              recorded_keys[i].c_str());
                lines.push_back(buffer);
                continue;
            }
            if (actual[match].hash == recorded[i].hash)
                continue;
            std::snprintf(buffer, sizeof(buffer), "changed  %-40s %016llx != %016llx",
                          recorded_keys[i].c_str(),
                          static_cast<unsigned long long>(actual[match].hash),
                          static_cast<unsigned long long>(recorded[i].hash));
            lines.push_back(buffer);
        }

        for (std::size_t j = 0; j < actual_keys.size(); ++j)
        {
            if (index_of(recorded_keys, actual_keys[j]) != recorded_keys.size())
                continue;
            std::snprintf(buffer, sizeof(buffer), "new      %s", actual_keys[j].c_str());
            lines.push_back(buffer);
        }
        return lines;
    }

    /** @brief Prints difference lines under a case, indented. */
    void print_differences(const std::vector<std::string>& lines)
    {
        for (const std::string& line : lines)
            std::printf("%-20s   %s\n", "", line.c_str());
    }

    /** @brief Dumps the frame as a binary PPM, so a mismatch can be looked at. */
    bool write_ppm(const std::string& path, const FrameImage& image)
    {
        std::ofstream file(path, std::ios::binary);
        if (!file)
            return false;
        file << "P6\n" << image.width << " " << image.height << "\n255\n";
        std::vector<char> row(static_cast<std::size_t>(image.width) * 3);
        for (std::uint32_t y = 0; y < image.height; ++y)
        {
            for (std::uint32_t x = 0; x < image.width; ++x)
            {
                const std::size_t source =
                    (static_cast<std::size_t>(y) * image.width + x) * 4;
                for (int channel = 0; channel < 3; ++channel)
                    row[static_cast<std::size_t>(x) * 3 + std::size_t(channel)] =
                        static_cast<char>(image.rgba[source + std::size_t(channel)]);
            }
            file.write(row.data(), std::streamsize(row.size()));
        }
        return bool(file);
    }

    /** @brief One golden case: a name, and how it differs from the base scene. */
    struct Case
    {
        const char* name;
        bool shadows;
    };

    /**
     * @brief The scene every case renders, built once and identically every run.
     *
     * Hand-built rather than loaded from an asset: an asset file is a second thing that
     * can change under the golden, and the built-in primitives need no file at all.
     */
    std::vector<MeshInstance> build_scene()
    {
        std::vector<MeshInstance> instances;

        const auto place = [&](MeshKind kind, Vector3 position, Vector3 shape,
                               Vector3 color, Scalar metallic, Scalar roughness)
        {
            MeshInstance instance;
            instance.model = translation(position);
            instance.kind = kind;
            instance.shape_params = shape;
            instance.color = color;
            instance.id = static_cast<std::uint32_t>(instances.size() + 1);
            instance.material.albedo = color;
            instance.material.metallic = float(metallic);
            instance.material.roughness = float(roughness);
            instances.push_back(instance);
        };

        // A ground slab, so the shadow pass has something to receive on.
        place(MeshKind::Box, Vector3{0.0, -0.5, 0.0}, Vector3{6.0, 0.5, 6.0},
              Vector3{0.55, 0.55, 0.58}, 0.0, 0.9);
        // Three casters spanning the metallic/rough corners of the BRDF, so a change to
        // shading shows up somewhere rather than possibly nowhere.
        place(MeshKind::Sphere, Vector3{-1.6, 0.8, 0.0}, Vector3{0.8, 0.0, 0.0},
              Vector3{0.85, 0.25, 0.2}, 0.0, 0.35);
        place(MeshKind::Box, Vector3{0.4, 0.7, -0.6}, Vector3{0.7, 0.7, 0.7},
              Vector3{0.2, 0.55, 0.85}, 1.0, 0.2);
        place(MeshKind::Cylinder, Vector3{2.2, 0.9, 0.5}, Vector3{0.5, 0.9, 0.0},
              Vector3{0.9, 0.8, 0.3}, 0.0, 0.65);
        return instances;
    }

    /** @brief The environment every case renders under: no sky, no cloud, one sun. */
    Environment build_environment()
    {
        Environment environment;
        // Both off deliberately — see this file's header. With them off the frame is the
        // mesh-shading half, which is the half that is not being rewritten this week.
        environment.atmosphere.enabled = false;
        environment.clouds.enabled = false;
        environment.stars.enabled = false;
        environment.night.enabled = false;
        environment.fog.enabled = false;
        environment.gi.enabled = false;

        // Toward the sun, not the way the light travels — a sun below the ground plane
        // is the easiest way to record a golden of a black frame and call it a baseline.
        environment.sun.direction = normalize(Vector3{-0.4, 0.8, 0.45});
        environment.sun.color = Vector3{1.0, 0.96, 0.9};
        environment.sun.intensity = 3.0f;
        environment.ambient = Vector3{0.05, 0.06, 0.08};
        environment.exposure = 0.18f;
        return environment;
    }

    /** @brief A camera looking at the scene from a fixed, unremarkable angle. */
    CameraView build_camera()
    {
        CameraView camera;
        const Vector3 eye{3.2, 2.6, 5.5};
        const Vector3 target{0.0, 0.6, 0.0};
        camera.view = look_at(eye, target, Vector3{0.0, 1.0, 0.0});
        camera.near_plane = 0.1f;
        camera.far_plane = 200.0f;
        camera.projection = perspective(Scalar(0.9), Scalar(WIDTH) / Scalar(HEIGHT),
                                        Scalar(camera.near_plane),
                                        Scalar(camera.far_plane));
        camera.world_position = WorldVector3{eye.x, eye.y, eye.z};
        return camera;
    }

    /** @brief Settings pinned so nothing in the frame depends on how fast it ran. */
    RenderSettings build_settings(const Case& scenario)
    {
        RenderSettings settings;
        settings.quality = RenderQuality::High;
        settings.anti_aliasing = AntiAliasingMode::Temporal;
        settings.render_scale = 1.0f;
        // The one setting that would make the image a function of the machine's speed.
        settings.dynamic_resolution.enabled = false;
        settings.shadows.enabled = scenario.shadows;
        // Pinned rather than left to the quality tier, for two reasons. A tier that
        // clamps these differently on another machine would change the image without
        // anything in the renderer having changed; and the cascade atlas is the frame's
        // largest single output by a wide margin, so at the authored 2048 it alone spends
        // most of a capture's staging budget and truncates the frame.
        settings.shadows.cascade_count = 4;
        settings.shadows.resolution = 1024;
        settings.lights.shadow_atlas_size = 1024;
        return settings;
    }
} // namespace

int main(int argc, char** argv)
{
    bool update = false;
    bool dump = false;
    // Capture is on by default, and the goldens are recorded with it on, so the two
    // sides of a comparison always allocate their transients the same way. Turning it
    // off is not merely "skip the per-pass part": it re-renders the frame the way a
    // shipping build allocates it, so a whole-frame hash that only matches with capture
    // on is a pass reading a transient it never wrote. That is worth being able to ask.
    bool capture = true;
    // The build points this at the checked-in references, so the harness works from any
    // working directory — which matters, because the one thing it must never do is
    // silently record a fresh baseline because it could not find the old one.
#if defined(SE_GOLDEN_DIR)
    std::string directory = SE_GOLDEN_DIR;
#else
    std::string directory = "goldens";
#endif
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument = argv[i];
        if (argument == "--update")
            update = true;
        else if (argument == "--dump")
            dump = true;
        else if (argument == "--no-capture")
            capture = false;
        else if (argument == "--goldens" && i + 1 < argc)
            directory = argv[++i];
        else
        {
            std::printf("usage: render_golden [--update] [--dump] [--no-capture] "
                        "[--goldens DIR]\n");
            return 2;
        }
    }

    // Recording without capture would write a reference whose pass section is empty,
    // and every later run would then report the whole frame as new. Refused rather than
    // half-honoured: a harness that can quietly weaken its own baseline is not one.
    if (update && !capture)
    {
        std::printf("render_golden: --update cannot be combined with --no-capture; a "
                    "golden without per-pass hashes is not a golden\n");
        return 2;
    }

    const Case cases[] = {
        {"opaque_lit", true},
        // The same scene with the shadow pass switched off, so a shadow regression is
        // distinguishable from a shading one rather than reddening a single case.
        {"opaque_unshadowed", false},
    };

    try
    {
        RenderDeviceDescription desc;
        Vulkan::VulkanDevice device(desc);
        std::printf("device: %s\n", device.info().name.c_str());

        Assets::AssetLibrary assets(device);
        Vulkan::VulkanSceneView view(device, assets);
        view.resize(WIDTH, HEIGHT);

        if (capture && !view.enable_pass_capture(true))
        {
            // Not a soft failure. Every golden on disk carries per-pass hashes, so a
            // backend that cannot produce them cannot answer the question this harness
            // asks — and pretending otherwise would report the whole frame as changed.
            std::printf("render_golden: this backend cannot capture per-pass output; "
                        "re-run with --no-capture to compare whole frames only\n");
            return 2;
        }

        const std::vector<MeshInstance> instances = build_scene();
        const Environment environment = build_environment();
        const CameraView camera = build_camera();

        int failures = 0;
        int written = 0;
        for (const Case& scenario : cases)
        {
            view.set_settings(build_settings(scenario));
            for (std::uint32_t frame = 0; frame < FRAMES; ++frame)
                view.render(camera, environment, instances.data(), instances.size(), 0);

            FrameImage image;
            if (!view.read_output(view.current_slot(), image))
            {
                std::printf("%-20s READBACK FAILED\n", scenario.name);
                ++failures;
                continue;
            }

            PassCaptureReport report;
            if (capture && !view.read_pass_hashes(view.current_slot(), report))
            {
                std::printf("%-20s PASS CAPTURE FAILED\n", scenario.name);
                ++failures;
                continue;
            }
            // An incomplete capture is not a smaller golden, it is a golden that cannot
            // notice the passes it never saw. Refused outright rather than recorded with
            // a warning nobody reads on the day it matters.
            if (report.dropped_by_budget > 0)
            {
                std::printf("%-20s CAPTURE TRUNCATED  %u output(s) dropped, %llu of %llu "
                            "MiB spent -- raise PassCapture::DEFAULT_BUDGET\n",
                            scenario.name, report.dropped_by_budget,
                            static_cast<unsigned long long>(report.bytes_used >> 20),
                            static_cast<unsigned long long>(report.bytes_budget >> 20));
                ++failures;
                continue;
            }
            const std::vector<PassOutputHash> passes = golden_passes(report.passes);

            const std::uint64_t hash = hash_image(image);
            const Thumbprint print = thumbprint_of(image);
            const std::string path = golden_path(directory.c_str(), scenario.name);
            const Golden golden = read_golden(path);

            if (update || !golden.present)
            {
                if (passes.empty())
                {
                    // Capture was on and produced nothing: the frame recorded no
                    // capturable output at all. Writing that would record a baseline
                    // that can never fail.
                    std::printf("%-20s NO PASSES CAPTURED -- refusing to record\n",
                                scenario.name);
                    ++failures;
                    continue;
                }
                if (!write_golden(path, hash, print, passes))
                {
                    std::printf("%-20s CANNOT WRITE %s\n", scenario.name, path.c_str());
                    ++failures;
                    continue;
                }
                std::printf("%-20s %s (hash %016llx, %zu of %zu captured outputs kept, "
                            "%u un-copyable, %llu MiB)\n",
                            scenario.name, golden.present ? "UPDATED" : "RECORDED",
                            static_cast<unsigned long long>(hash), passes.size(),
                            report.passes.size(), report.dropped_by_format,
                            static_cast<unsigned long long>(report.bytes_used >> 20));
                ++written;
                continue;
            }

            if (hash == golden.hash)
            {
                // The whole frame is identical. Per-pass hashes can still differ, and
                // when they do it is worth saying out loud rather than passing quietly:
                // an intermediate changed without reaching the screen, which is either a
                // refactor doing exactly what it promised or a bug the final image hid.
                const std::vector<std::string> internal =
                    capture ? pass_differences(golden.passes, passes)
                            : std::vector<std::string>{};
                if (internal.empty())
                {
                    std::printf("%-20s OK\n", scenario.name);
                    continue;
                }
                std::printf("%-20s MISMATCH  frame identical, %zu pass output(s) differ\n",
                            scenario.name, internal.size());
                print_differences(internal);
                ++failures;
                continue;
            }

            int largest = 0;
            double mean = 0.0;
            compare_thumbprints(print, golden.thumbprint, largest, mean);
            std::printf("%-20s MISMATCH  hash %016llx != %016llx  "
                        "thumbprint max %d mean %.2f levels\n",
                        scenario.name, static_cast<unsigned long long>(hash),
                        static_cast<unsigned long long>(golden.hash), largest, mean);
            // The line that decides whether this is worth waking someone for.
            std::printf("%-20s %s\n", "",
                        largest <= 2
                            ? "looks like driver/compiler rounding, not a visible change"
                            : "a visible change -- inspect the dump before re-recording");
            // The half of the answer the whole-frame hash cannot give: which pass.
            if (capture)
            {
                const std::vector<std::string> which =
                    pass_differences(golden.passes, passes);
                if (which.empty())
                    std::printf("%-20s   every captured pass output is unchanged -- the "
                                "difference is in a pass whose output was not captured\n",
                                "");
                else
                    print_differences(which);
            }
            if (dump)
            {
                const std::string ppm = std::string(scenario.name) + "_actual.ppm";
                if (write_ppm(ppm, image))
                    std::printf("%-20s wrote %s\n", "", ppm.c_str());
            }
            ++failures;
        }

        std::printf("RESULT: %s (%d recorded, %d failed)\n",
                    failures == 0 ? "OK" : "FAILED", written, failures);
        return failures == 0 ? 0 : 1;
    }
    catch (const std::exception& error)
    {
        std::printf("render_golden: %s\n", error.what());
        return 1;
    }
}
