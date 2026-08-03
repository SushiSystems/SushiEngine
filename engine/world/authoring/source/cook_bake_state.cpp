/**************************************************************************/
/* cook_bake_state.cpp                                                    */
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

#include "cook_bake_state.hpp"

#include <algorithm>
#include <fstream>
#include <utility>

#include <nlohmann/json.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            using nlohmann::json;
            using Physics::Cooking::CookingParameters;
            using Physics::Cooking::CookingThresholds;
            using Physics::Cooking::ImportProfile;
            using Physics::Cooking::ImportProfileLibrary;
            using Physics::Cooking::ImportProfileOverride;
            using Physics::Cooking::NodeBeamCookerSettings;

            // §16.45.3's storage format: everything the Bake panel and the Cooking Override
            // modal can set, and nothing they cannot — `force_recook` is deliberately absent
            // (it describes one press of a button, not a saved property, per its own doc
            // comment on `ImportProfile`) and `DerivedCookingParameters` is never stored at
            // all, since it is recomputed from `CookingParameters` on every read.

            json cooking_parameters_to_json(const CookingParameters& p)
            {
                return json{{"fidelity", p.fidelity},
                           {"voxel_resolution", p.voxel_resolution},
                           {"target_tetrahedron_count", p.target_tetrahedron_count},
                           {"simulation_level_count", p.simulation_level_count},
                           {"convex_piece_count", p.convex_piece_count},
                           {"distance_field_resolution", p.distance_field_resolution},
                           {"surface_conforming_passes", p.surface_conforming_passes},
                           {"suggested_substep_count", p.suggested_substep_count},
                           {"hull_vertex_budget", p.hull_vertex_budget},
                           {"weld_tolerance", p.weld_tolerance},
                           {"density", p.density},
                           {"accuracy_lattice_order", p.accuracy_lattice_order},
                           {"cook_collision", p.cook_collision},
                           {"cook_soft_body", p.cook_soft_body},
                           {"cook_node_beam", p.cook_node_beam},
                           {"static_geometry", p.static_geometry}};
            }

            CookingParameters cooking_parameters_from_json(const json& j)
            {
                CookingParameters p;
                p.fidelity = j.value("fidelity", p.fidelity);
                p.voxel_resolution = j.value("voxel_resolution", p.voxel_resolution);
                p.target_tetrahedron_count =
                    j.value("target_tetrahedron_count", p.target_tetrahedron_count);
                p.simulation_level_count =
                    j.value("simulation_level_count", p.simulation_level_count);
                p.convex_piece_count = j.value("convex_piece_count", p.convex_piece_count);
                p.distance_field_resolution =
                    j.value("distance_field_resolution", p.distance_field_resolution);
                p.surface_conforming_passes =
                    j.value("surface_conforming_passes", p.surface_conforming_passes);
                p.suggested_substep_count =
                    j.value("suggested_substep_count", p.suggested_substep_count);
                p.hull_vertex_budget = j.value("hull_vertex_budget", p.hull_vertex_budget);
                p.weld_tolerance = j.value("weld_tolerance", p.weld_tolerance);
                p.density = j.value("density", p.density);
                p.accuracy_lattice_order =
                    j.value("accuracy_lattice_order", p.accuracy_lattice_order);
                p.cook_collision = j.value("cook_collision", p.cook_collision);
                p.cook_soft_body = j.value("cook_soft_body", p.cook_soft_body);
                p.cook_node_beam = j.value("cook_node_beam", p.cook_node_beam);
                p.static_geometry = j.value("static_geometry", p.static_geometry);
                return p;
            }

            json cooking_thresholds_to_json(const CookingThresholds& t)
            {
                return json{{"max_unembedded_vertices", t.max_unembedded_vertices},
                           {"max_inverted_elements", t.max_inverted_elements},
                           {"min_element_quality", t.min_element_quality},
                           {"max_hausdorff_error", t.max_hausdorff_error},
                           {"require_watertight_source", t.require_watertight_source}};
            }

            CookingThresholds cooking_thresholds_from_json(const json& j)
            {
                CookingThresholds t;
                t.max_unembedded_vertices =
                    j.value("max_unembedded_vertices", t.max_unembedded_vertices);
                t.max_inverted_elements =
                    j.value("max_inverted_elements", t.max_inverted_elements);
                t.min_element_quality = j.value("min_element_quality", t.min_element_quality);
                t.max_hausdorff_error = j.value("max_hausdorff_error", t.max_hausdorff_error);
                t.require_watertight_source =
                    j.value("require_watertight_source", t.require_watertight_source);
                return t;
            }

            json node_beam_settings_to_json(const NodeBeamCookerSettings& s)
            {
                return json{
                    {"material",
                     json{{"young_modulus", double(s.material.young_modulus)},
                          {"poisson_ratio", double(s.material.poisson_ratio)},
                          {"density", double(s.material.density)},
                          {"damping", double(s.material.damping)},
                          {"yield_stress", double(s.material.yield_stress)},
                          {"plastic_creep", double(s.material.plastic_creep)},
                          {"maximum_plastic_strain", double(s.material.maximum_plastic_strain)},
                          {"fracture_stress", double(s.material.fracture_stress)}}},
                    {"core_mass_fraction", s.core_mass_fraction},
                    {"structural_length_ratio", s.structural_length_ratio},
                    {"skin_search_ratio", s.skin_search_ratio},
                    {"attach_shell_to_core", s.attach_shell_to_core}};
            }

            NodeBeamCookerSettings node_beam_settings_from_json(const json& j)
            {
                NodeBeamCookerSettings s;
                if (j.contains("material"))
                {
                    const json& m = j["material"];
                    s.material.young_modulus =
                        Scalar(m.value("young_modulus", double(s.material.young_modulus)));
                    s.material.poisson_ratio =
                        Scalar(m.value("poisson_ratio", double(s.material.poisson_ratio)));
                    s.material.density = Scalar(m.value("density", double(s.material.density)));
                    s.material.damping = Scalar(m.value("damping", double(s.material.damping)));
                    s.material.yield_stress =
                        Scalar(m.value("yield_stress", double(s.material.yield_stress)));
                    s.material.plastic_creep =
                        Scalar(m.value("plastic_creep", double(s.material.plastic_creep)));
                    s.material.maximum_plastic_strain = Scalar(
                        m.value("maximum_plastic_strain", double(s.material.maximum_plastic_strain)));
                    s.material.fracture_stress =
                        Scalar(m.value("fracture_stress", double(s.material.fracture_stress)));
                }
                s.core_mass_fraction = j.value("core_mass_fraction", s.core_mass_fraction);
                s.structural_length_ratio =
                    j.value("structural_length_ratio", s.structural_length_ratio);
                s.skin_search_ratio = j.value("skin_search_ratio", s.skin_search_ratio);
                s.attach_shell_to_core = j.value("attach_shell_to_core", s.attach_shell_to_core);
                return s;
            }

            json import_profile_to_json(const ImportProfile& profile)
            {
                return json{{"parameters", cooking_parameters_to_json(profile.parameters)},
                           {"thresholds", cooking_thresholds_to_json(profile.thresholds)},
                           {"node_beam_settings",
                            node_beam_settings_to_json(profile.node_beam_settings)}};
            }

            ImportProfile import_profile_from_json(const json& j)
            {
                ImportProfile profile;
                if (j.contains("parameters"))
                    profile.parameters = cooking_parameters_from_json(j["parameters"]);
                if (j.contains("thresholds"))
                    profile.thresholds = cooking_thresholds_from_json(j["thresholds"]);
                if (j.contains("node_beam_settings"))
                    profile.node_beam_settings =
                        node_beam_settings_from_json(j["node_beam_settings"]);
                return profile;
            }

            json import_profile_override_to_json(const ImportProfileOverride& o)
            {
                json j = json::object();
                if (o.fidelity.has_value())
                    j["fidelity"] = *o.fidelity;
                if (o.cook_collision.has_value())
                    j["cook_collision"] = *o.cook_collision;
                if (o.cook_soft_body.has_value())
                    j["cook_soft_body"] = *o.cook_soft_body;
                if (o.cook_node_beam.has_value())
                    j["cook_node_beam"] = *o.cook_node_beam;
                if (o.static_geometry.has_value())
                    j["static_geometry"] = *o.static_geometry;
                return j;
            }

            ImportProfileOverride import_profile_override_from_json(const json& j)
            {
                ImportProfileOverride o;
                if (j.contains("fidelity"))
                    o.fidelity = j["fidelity"].get<float>();
                if (j.contains("cook_collision"))
                    o.cook_collision = j["cook_collision"].get<bool>();
                if (j.contains("cook_soft_body"))
                    o.cook_soft_body = j["cook_soft_body"].get<bool>();
                if (j.contains("cook_node_beam"))
                    o.cook_node_beam = j["cook_node_beam"].get<bool>();
                if (j.contains("static_geometry"))
                    o.static_geometry = j["static_geometry"].get<bool>();
                return o;
            }
        } // namespace

        using nlohmann::json;

        CookBakeState::CookBakeState(Physics::Cooking::MeshLoader loader,
                                     const std::string& cache_directory)
            : chain_(Physics::Cooking::MeshPostProcessorChain::with_shipped_processors())
        {
            if (cache_directory.empty())
            {
                store_ = std::make_unique<Physics::Cooking::MemoryCookedAssetStore>();
            }
            else
            {
                auto filesystem_store =
                    std::make_unique<Physics::Cooking::FilesystemCookedAssetStore>(
                        cache_directory);
                // A cache directory that cannot be created makes cooking slow, not impossible,
                // so the memory store stands in rather than the panel refusing to work.
                if (filesystem_store->usable())
                    store_ = std::move(filesystem_store);
                else
                    store_ = std::make_unique<Physics::Cooking::MemoryCookedAssetStore>();
            }

            // The loader is moved into the service, which owns the worker that calls it.
            service_ = std::make_unique<Physics::Cooking::CookingService>(std::move(loader),
                                                                         chain_, store_.get());
        }

        bool CookBakeState::load_profiles()
        {
            if (profile_storage_path_.empty())
                return true;
            std::ifstream stream(profile_storage_path_, std::ios::binary);
            if (!stream)
                return true; // Nothing to load yet; not an error.

            json document;
            try
            {
                stream >> document;
            }
            catch (const json::parse_error&)
            {
                return false;
            }

            if (document.contains("project_default"))
                profiles_.set_project_default(import_profile_from_json(document["project_default"]));
            if (document.contains("overrides") && document["overrides"].is_object())
            {
                for (const auto& [asset_path, override_json] : document["overrides"].items())
                    profiles_.set_override(asset_path,
                                          import_profile_override_from_json(override_json));
            }
            return true;
        }

        bool CookBakeState::save_profiles() const
        {
            if (profile_storage_path_.empty())
                return true;

            json overrides = json::object();
            for (const auto& [asset_path, override_values] : profiles_.overrides())
                overrides[asset_path] = import_profile_override_to_json(override_values);

            json document{{"project_default", import_profile_to_json(profiles_.project_default())},
                         {"overrides", overrides}};

            std::ofstream stream(profile_storage_path_, std::ios::binary | std::ios::trunc);
            if (!stream)
                return false;
            stream << document.dump(2);
            return static_cast<bool>(stream);
        }

        CookBakeState::~CookBakeState()
        {
            // Explicit and ordered: the service has a thread that touches the store and the
            // chain, so it has to be gone before either. Declaration order would give this for
            // free, and saying it here is cheaper than finding out it did not.
            service_.reset();
        }

        void CookBakeState::bake(const std::string& asset_path)
        {
            if (asset_path.empty())
                return;
            service_->submit(asset_path, profiles_.resolve(asset_path));
        }

        void CookBakeState::rebake(const std::string& asset_path)
        {
            if (asset_path.empty())
                return;

            // The eviction itself happens inside the processor, which is the first place the
            // mesh and the cooker that owns the key are both in hand — the key needs the
            // source's content hash, and the source is behind the loader on the worker thread.
            Physics::Cooking::ImportProfile profile = profiles_.resolve(asset_path);
            profile.force_recook = true;
            service_->submit(asset_path, std::move(profile));
        }

        std::size_t CookBakeState::poll()
        {
            const std::vector<Physics::Cooking::CookedImport> imports =
                service_->take_completed();
            for (const Physics::Cooking::CookedImport& imported : imports)
            {
                BakedAssetEntry entry_value;
                entry_value.asset_path = imported.asset_path;
                entry_value.loaded = imported.loaded;
                entry_value.source_triangle_count = imported.source_triangle_count;

                const Physics::Cooking::MeshPostProcessResult* collision =
                    imported.product(Physics::Cooking::CookedAssetKind::Collision);
                if (collision != nullptr)
                {
                    entry_value.collision_bytes = collision->bytes;
                    entry_value.collision_report = collision->report;
                }
                const Physics::Cooking::MeshPostProcessResult* soft =
                    imported.product(Physics::Cooking::CookedAssetKind::SoftBody);
                if (soft != nullptr)
                {
                    entry_value.soft_body_bytes = soft->bytes;
                    entry_value.soft_body_report = soft->report;
                }
                const Physics::Cooking::MeshPostProcessResult* node_beam =
                    imported.product(Physics::Cooking::CookedAssetKind::NodeBeam);
                if (node_beam != nullptr)
                {
                    entry_value.node_beam_bytes = node_beam->bytes;
                    entry_value.node_beam_report = node_beam->report;
                }

                // One entry per asset, replaced in place: a re-cook of a crate should update
                // the crate's row rather than add a second one the artist has to tell apart.
                const auto existing =
                    std::find_if(entries_.begin(), entries_.end(),
                                 [&imported](const BakedAssetEntry& candidate)
                                 { return candidate.asset_path == imported.asset_path; });
                if (existing != entries_.end())
                    *existing = std::move(entry_value);
                else
                    entries_.insert(entries_.begin(), std::move(entry_value));

                if (selected_.empty() || selected_ == imported.asset_path)
                {
                    selected_ = imported.asset_path;
                    refresh_wireframe();
                }
            }
            return imports.size();
        }

        const BakedAssetEntry* CookBakeState::entry(const std::string& asset_path) const
        {
            for (const BakedAssetEntry& candidate : entries_)
            {
                if (candidate.asset_path == asset_path)
                    return &candidate;
            }
            return nullptr;
        }

        void CookBakeState::select(const std::string& asset_path)
        {
            if (selected_ == asset_path)
                return;
            selected_ = asset_path;
            refresh_wireframe();
        }

        bool CookBakeState::busy() const
        {
            const Physics::Cooking::CookingServiceStatus current = service_->status();
            return current.busy || current.queued > 0;
        }

        void CookBakeState::refresh_wireframe()
        {
            wireframe_.clear();
            const BakedAssetEntry* current = entry(selected_);
            if (current == nullptr || !current->has_collision())
                return;

            const Physics::Cooking::CollisionAssetView view =
                Physics::Cooking::load_collision_blob(current->collision_bytes.data(),
                                                      current->collision_bytes.size());
            if (!view.valid)
                return;
            Physics::Cooking::collision_asset_wireframe(view, wireframe_);
        }
    } // namespace Editor
} // namespace SushiEngine
