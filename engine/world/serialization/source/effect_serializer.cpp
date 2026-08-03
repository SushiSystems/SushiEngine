/**************************************************************************/
/* effect_serializer.cpp                                                  */
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

#include "effect_serializer.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

#include <SushiEngine/material/material.hpp>
#include <SushiEngine/render/asset_library_interface.hpp>

namespace SushiEngine
{
    namespace Scene
    {
        namespace
        {
            using nlohmann::json;

            json vector3_to_json(const Vector3& v) { return json{{"x", v.x}, {"y", v.y}, {"z", v.z}}; }

            Vector3 vector3_from_json(const json& node, const Vector3& fallback)
            {
                if (!node.is_object())
                    return fallback;
                return Vector3{node.value("x", fallback.x), node.value("y", fallback.y),
                               node.value("z", fallback.z)};
            }

            /**
             * @brief Reads a member as an enum, keeping @p fallback when it is absent or out of
             *        range.
             *
             * Enums are written as their numeric value, so an unknown one from a newer build must
             * not become an out-of-range enum here — that would be undefined behaviour in the
             * switch every consumer eventually does on it.
             */
            template <typename Enum>
            Enum enum_from_json(const json& node, const char* key, Enum fallback,
                                std::uint32_t count)
            {
                const std::uint32_t value =
                    node.value(key, static_cast<std::uint32_t>(fallback));
                return value < count ? static_cast<Enum>(value) : fallback;
            }

            json curve_to_json(const VFX::AnimationCurve& curve)
            {
                json keys = json::array();
                for (const VFX::CurveKey& key : curve.keys())
                {
                    keys.push_back(json{{"time", key.time},
                                        {"value", key.value},
                                        {"in_tangent", key.in_tangent},
                                        {"out_tangent", key.out_tangent}});
                }
                return keys;
            }

            void curve_from_json(const json& node, VFX::AnimationCurve& curve)
            {
                curve.keys().clear();
                if (!node.is_array())
                    return;
                for (const json& key : node)
                {
                    VFX::CurveKey out;
                    out.time = key.value("time", 0.0f);
                    out.value = key.value("value", 0.0f);
                    out.in_tangent = key.value("in_tangent", 0.0f);
                    out.out_tangent = key.value("out_tangent", 0.0f);
                    curve.add_key(out);
                }
            }

            json gradient_to_json(const VFX::ColorGradient& gradient)
            {
                json colors = json::array();
                for (const VFX::ColorKey& key : gradient.color_keys())
                    colors.push_back(json{{"time", key.time}, {"color", vector3_to_json(key.color)}});
                json alphas = json::array();
                for (const VFX::AlphaKey& key : gradient.alpha_keys())
                    alphas.push_back(json{{"time", key.time}, {"alpha", key.alpha}});
                return json{{"colors", colors}, {"alphas", alphas}};
            }

            void gradient_from_json(const json& node, VFX::ColorGradient& gradient)
            {
                gradient = VFX::ColorGradient{};
                if (!node.is_object())
                    return;
                const json& colors = node.value("colors", json::array());
                for (const json& key : colors)
                {
                    VFX::ColorKey out;
                    out.time = key.value("time", 0.0f);
                    out.color = vector3_from_json(key.value("color", json::object()),
                                                  Vector3{1, 1, 1});
                    gradient.add_color_key(out);
                }
                const json& alphas = node.value("alphas", json::array());
                for (const json& key : alphas)
                {
                    VFX::AlphaKey out;
                    out.time = key.value("time", 0.0f);
                    out.alpha = key.value("alpha", 1.0f);
                    gradient.add_alpha_key(out);
                }
            }

            json force_field_to_json(const VFX::ForceFieldModule& field)
            {
                return json{{"enabled", field.enabled},
                            {"kind", static_cast<std::uint32_t>(field.kind)},
                            {"position", vector3_to_json(field.position)},
                            {"axis", vector3_to_json(field.axis)},
                            {"strength", field.strength},
                            {"radius", field.radius},
                            {"falloff", field.falloff}};
            }

            VFX::ForceFieldModule force_field_from_json(const json& node)
            {
                VFX::ForceFieldModule field;
                field.enabled = node.value("enabled", field.enabled);
                field.kind = enum_from_json(node, "kind", field.kind, VFX::FORCE_FIELD_KIND_COUNT);
                field.position = vector3_from_json(node.value("position", json::object()),
                                                   field.position);
                field.axis = vector3_from_json(node.value("axis", json::object()), field.axis);
                field.strength = node.value("strength", field.strength);
                field.radius = node.value("radius", field.radius);
                field.falloff = node.value("falloff", field.falloff);
                return field;
            }

            json emitter_to_json(const VFX::EmitterDescriptor& e)
            {
                json bursts = json::array();
                for (const VFX::ParticleBurst& burst : e.spawn.bursts)
                    bursts.push_back(json{{"time", burst.time}, {"count", burst.count}});

                json fields = json::array();
                for (const VFX::ForceFieldModule& field : e.force_fields)
                    fields.push_back(force_field_to_json(field));

                return json{
                    {"name", e.name},
                    {"domain", static_cast<std::uint32_t>(e.domain)},
                    {"capacity", e.capacity},
                    {"duration", e.duration},
                    {"looping", e.looping},
                    {"prewarm", e.prewarm},
                    {"spawn",
                     json{{"enabled", e.spawn.enabled},
                          {"rate_per_second", e.spawn.rate_per_second},
                          {"bursts", bursts}}},
                    {"shape",
                     json{{"shape", static_cast<std::uint32_t>(e.shape.shape)},
                          {"radius", e.shape.radius},
                          {"cone_angle_radians", e.shape.cone_angle_radians},
                          {"arc_radians", e.shape.arc_radians},
                          {"box_half_extents", vector3_to_json(e.shape.box_half_extents)},
                          {"emit_from_shell", e.shape.emit_from_shell}}},
                    {"init",
                     json{{"lifetime_min", e.init.lifetime_min},
                          {"lifetime_max", e.init.lifetime_max},
                          {"speed_min", e.init.speed_min},
                          {"speed_max", e.init.speed_max},
                          {"size_min", e.init.size_min},
                          {"size_max", e.init.size_max},
                          {"rotation_min", e.init.rotation_min},
                          {"rotation_max", e.init.rotation_max},
                          {"angular_velocity_min", e.init.angular_velocity_min},
                          {"angular_velocity_max", e.init.angular_velocity_max},
                          {"color", vector3_to_json(e.init.color)}}},
                    {"gravity",
                     json{{"enabled", e.gravity.enabled},
                          {"acceleration", vector3_to_json(e.gravity.acceleration)}}},
                    {"drag", json{{"enabled", e.drag.enabled}, {"coefficient", e.drag.coefficient}}},
                    {"turbulence",
                     json{{"enabled", e.turbulence.enabled},
                          {"frequency", e.turbulence.frequency},
                          {"amplitude", e.turbulence.amplitude}}},
                    {"force_fields", fields},
                    {"collision",
                     json{{"enabled", e.collision.enabled},
                          {"use_distance_field", e.collision.use_distance_field},
                          {"restitution", e.collision.restitution},
                          {"friction", e.collision.friction},
                          {"thickness", e.collision.thickness}}},
                    {"size_over_life",
                     json{{"enabled", e.size_over_life.enabled},
                          {"curve", curve_to_json(e.size_over_life.curve)}}},
                    {"color_over_life",
                     json{{"enabled", e.color_over_life.enabled},
                          {"gradient", gradient_to_json(e.color_over_life.gradient)}}},
                    {"render",
                     json{{"blend", static_cast<std::uint32_t>(e.render.blend)},
                          {"sort", static_cast<std::uint32_t>(e.render.sort)},
                          {"alignment", static_cast<std::uint32_t>(e.render.alignment)},
                          {"velocity_stretch", e.render.velocity_stretch},
                          {"soft_particles", e.render.soft_particles},
                          {"soft_fade_distance", e.render.soft_fade_distance},
                          // The path is what a file is read back by; the handle rides along
                          // because the same capture serves the in-memory snapshots undo/redo
                          // and play-mode take, where the id is still live and re-reading every
                          // texture off disk to restore it would be absurd. A load from disk
                          // overwrites the handle from the path (@ref resolve_effect_textures),
                          // so a stale one never survives into another session.
                          {"texture_path", e.render.texture_path},
                          {"texture", e.render.texture},
                          {"mesh", e.render.mesh},
                          {"flipbook_rows", e.render.flipbook_rows},
                          {"flipbook_columns", e.render.flipbook_columns},
                          {"lit", e.render.lit}}},
                    {"beam",
                     json{{"enabled", e.beam.enabled},
                          {"start", vector3_to_json(e.beam.start)},
                          {"end", vector3_to_json(e.beam.end)},
                          {"width", e.beam.width},
                          {"sag", e.beam.sag},
                          {"noise_amplitude", e.beam.noise_amplitude},
                          {"noise_frequency", e.beam.noise_frequency}}}};
            }

            VFX::EmitterDescriptor emitter_from_json(const json& node)
            {
                VFX::EmitterDescriptor e;
                if (!node.is_object())
                    return e;

                e.name = node.value("name", e.name);
                e.domain = enum_from_json(node, "domain", e.domain, VFX::SIMULATION_DOMAIN_COUNT);
                e.capacity = node.value("capacity", e.capacity);
                e.duration = node.value("duration", e.duration);
                e.looping = node.value("looping", e.looping);
                e.prewarm = node.value("prewarm", e.prewarm);

                const json spawn = node.value("spawn", json::object());
                e.spawn.enabled = spawn.value("enabled", e.spawn.enabled);
                e.spawn.rate_per_second = spawn.value("rate_per_second", e.spawn.rate_per_second);
                for (const json& burst : spawn.value("bursts", json::array()))
                {
                    VFX::ParticleBurst out;
                    out.time = burst.value("time", 0.0f);
                    out.count = burst.value("count", 0u);
                    e.spawn.bursts.push_back(out);
                }

                const json shape = node.value("shape", json::object());
                e.shape.shape =
                    enum_from_json(shape, "shape", e.shape.shape, VFX::EMITTER_SHAPE_COUNT);
                e.shape.radius = shape.value("radius", e.shape.radius);
                e.shape.cone_angle_radians =
                    shape.value("cone_angle_radians", e.shape.cone_angle_radians);
                e.shape.arc_radians = shape.value("arc_radians", e.shape.arc_radians);
                e.shape.box_half_extents = vector3_from_json(
                    shape.value("box_half_extents", json::object()), e.shape.box_half_extents);
                e.shape.emit_from_shell = shape.value("emit_from_shell", e.shape.emit_from_shell);

                const json init = node.value("init", json::object());
                e.init.lifetime_min = init.value("lifetime_min", e.init.lifetime_min);
                e.init.lifetime_max = init.value("lifetime_max", e.init.lifetime_max);
                e.init.speed_min = init.value("speed_min", e.init.speed_min);
                e.init.speed_max = init.value("speed_max", e.init.speed_max);
                e.init.size_min = init.value("size_min", e.init.size_min);
                e.init.size_max = init.value("size_max", e.init.size_max);
                e.init.rotation_min = init.value("rotation_min", e.init.rotation_min);
                e.init.rotation_max = init.value("rotation_max", e.init.rotation_max);
                e.init.angular_velocity_min =
                    init.value("angular_velocity_min", e.init.angular_velocity_min);
                e.init.angular_velocity_max =
                    init.value("angular_velocity_max", e.init.angular_velocity_max);
                e.init.color = vector3_from_json(init.value("color", json::object()), e.init.color);

                const json gravity = node.value("gravity", json::object());
                e.gravity.enabled = gravity.value("enabled", e.gravity.enabled);
                e.gravity.acceleration = vector3_from_json(
                    gravity.value("acceleration", json::object()), e.gravity.acceleration);

                const json drag = node.value("drag", json::object());
                e.drag.enabled = drag.value("enabled", e.drag.enabled);
                e.drag.coefficient = drag.value("coefficient", e.drag.coefficient);

                const json turbulence = node.value("turbulence", json::object());
                e.turbulence.enabled = turbulence.value("enabled", e.turbulence.enabled);
                e.turbulence.frequency = turbulence.value("frequency", e.turbulence.frequency);
                e.turbulence.amplitude = turbulence.value("amplitude", e.turbulence.amplitude);

                for (const json& field : node.value("force_fields", json::array()))
                    e.force_fields.push_back(force_field_from_json(field));

                const json collision = node.value("collision", json::object());
                e.collision.enabled = collision.value("enabled", e.collision.enabled);
                e.collision.use_distance_field =
                    collision.value("use_distance_field", e.collision.use_distance_field);
                e.collision.restitution = collision.value("restitution", e.collision.restitution);
                e.collision.friction = collision.value("friction", e.collision.friction);
                e.collision.thickness = collision.value("thickness", e.collision.thickness);

                const json size_over_life = node.value("size_over_life", json::object());
                e.size_over_life.enabled = size_over_life.value("enabled", e.size_over_life.enabled);
                curve_from_json(size_over_life.value("curve", json::array()),
                                e.size_over_life.curve);

                const json color_over_life = node.value("color_over_life", json::object());
                e.color_over_life.enabled =
                    color_over_life.value("enabled", e.color_over_life.enabled);
                gradient_from_json(color_over_life.value("gradient", json::object()),
                                   e.color_over_life.gradient);

                const json render = node.value("render", json::object());
                e.render.blend =
                    enum_from_json(render, "blend", e.render.blend, VFX::BLEND_MODE_COUNT);
                e.render.sort = enum_from_json(render, "sort", e.render.sort, VFX::SORT_MODE_COUNT);
                e.render.alignment = enum_from_json(render, "alignment", e.render.alignment,
                                                    VFX::RENDER_ALIGNMENT_COUNT);
                e.render.velocity_stretch = render.value("velocity_stretch", e.render.velocity_stretch);
                e.render.soft_particles = render.value("soft_particles", e.render.soft_particles);
                e.render.soft_fade_distance =
                    render.value("soft_fade_distance", e.render.soft_fade_distance);
                e.render.texture_path = render.value("texture_path", e.render.texture_path);
                e.render.texture = render.value("texture", e.render.texture);
                e.render.mesh = render.value("mesh", e.render.mesh);
                e.render.flipbook_rows = render.value("flipbook_rows", e.render.flipbook_rows);
                e.render.flipbook_columns =
                    render.value("flipbook_columns", e.render.flipbook_columns);
                e.render.lit = render.value("lit", e.render.lit);

                const json beam = node.value("beam", json::object());
                e.beam.enabled = beam.value("enabled", e.beam.enabled);
                e.beam.start = vector3_from_json(beam.value("start", json::object()), e.beam.start);
                e.beam.end = vector3_from_json(beam.value("end", json::object()), e.beam.end);
                e.beam.width = beam.value("width", e.beam.width);
                e.beam.sag = beam.value("sag", e.beam.sag);
                e.beam.noise_amplitude = beam.value("noise_amplitude", e.beam.noise_amplitude);
                e.beam.noise_frequency = beam.value("noise_frequency", e.beam.noise_frequency);
                return e;
            }
        } // namespace

        const char* const EFFECT_FILE_EXTENSION = ".sushieffect";

        nlohmann::json capture_effect(const VFX::ParticleEffect& effect)
        {
            json emitters = json::array();
            for (const VFX::EmitterDescriptor& emitter : effect.emitters)
                emitters.push_back(emitter_to_json(emitter));
            return json{{"name", effect.name}, {"emitters", emitters}};
        }

        bool apply_effect(const nlohmann::json& root, VFX::ParticleEffect& effect)
        {
            if (!root.is_object() || !root.contains("emitters"))
                return false;
            const json& emitters = root["emitters"];
            if (!emitters.is_array())
                return false;

            VFX::ParticleEffect loaded;
            loaded.name = root.value("name", std::string{"Effect"});
            for (const json& emitter : emitters)
                loaded.emitters.push_back(emitter_from_json(emitter));
            effect = std::move(loaded);
            return true;
        }

        bool save_effect(const VFX::ParticleEffect& effect, const std::string& path)
        {
            std::error_code error;
            const std::filesystem::path parent = std::filesystem::path(path).parent_path();
            if (!parent.empty())
                std::filesystem::create_directories(parent, error);

            std::ofstream file(path);
            if (!file.is_open())
                return false;
            file << capture_effect(effect).dump(2);
            return file.good();
        }

        bool load_effect(const std::string& path, VFX::ParticleEffect& effect)
        {
            std::ifstream file(path);
            if (!file.is_open())
                return false;
            json root = json::parse(file, nullptr, false);
            if (root.is_discarded())
                return false;
            return apply_effect(root, effect);
        }

        void resolve_effect_textures(VFX::ParticleEffect& effect, Render::IAssetLibrary& assets)
        {
            for (VFX::EmitterDescriptor& emitter : effect.emitters)
            {
                if (emitter.render.texture_path.empty())
                {
                    emitter.render.texture = VFX::NO_PARTICLE_TEXTURE;
                    continue;
                }
                const Render::TextureId loaded = assets.load_texture(
                    emitter.render.texture_path.c_str(), Render::TextureColorSpace::SRGB);
                emitter.render.texture = loaded == Render::INVALID_TEXTURE
                                             ? VFX::NO_PARTICLE_TEXTURE
                                             : static_cast<std::uint32_t>(loaded);
            }
        }

        std::vector<std::string> list_effect_files(const std::string& directory)
        {
            std::vector<std::string> files;
            std::error_code error;
            if (!std::filesystem::is_directory(directory, error))
                return files;
            for (const std::filesystem::directory_entry& entry :
                 std::filesystem::directory_iterator(directory, error))
            {
                if (!entry.is_regular_file(error))
                    continue;
                if (entry.path().extension() == EFFECT_FILE_EXTENSION)
                    files.push_back(entry.path().string());
            }
            std::sort(files.begin(), files.end());
            return files;
        }
    } // namespace Scene
} // namespace SushiEngine
