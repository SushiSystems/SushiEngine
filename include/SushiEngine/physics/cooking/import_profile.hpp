/**************************************************************************/
/* import_profile.hpp                                                     */
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

#pragma once

/**
 * @file import_profile.hpp
 * @brief What happens to a mesh when it lands in the project, and who decides.
 *
 * §8.1's import profile: *"a per-project default plus a per-asset override decides which
 * cookers run — this is how 'drop a mesh in and it is soft-body ready' happens without
 * every rock in the level paying for a tetrahedral mesh."*
 *
 * That sentence is the whole design constraint. The default has to be generous enough that
 * dropping a mesh in produces something usable with no action at all, and the override has
 * to be cheap enough that the one crate in a hundred that wants to be deformable can say so
 * without a project-wide setting changing.
 *
 * **The override is deliberately partial, and deliberately small.** Four fields, and they
 * are the four an artist actually touches per asset: how accurately, and which of the three
 * kinds of thing this mesh is. Everything else in @ref CookingParameters — the vertex
 * budget, the weld tolerance, the sampling order — is engineering policy that belongs to the
 * project, and letting it be overridden per asset would produce a project where no two
 * assets were cooked comparably and no measurement meant anything.
 */

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include <SushiEngine/physics/cooking/cooking_parameters.hpp>
#include <SushiEngine/physics/cooking/cooking_report.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /**
             * @brief Everything an import needs to decide, resolved.
             *
             * What a post-processor reads. Produced either as the project default or by
             * @ref resolve_import_profile folding an override into it.
             */
            struct ImportProfile
            {
                /** @brief The dial and the pinned overrides every cooker reads. */
                CookingParameters parameters;

                /** @brief The limits a produced asset is judged against (§8.5). */
                CookingThresholds thresholds;

                /**
                 * @brief Drop any cached asset before cooking, this once.
                 *
                 * §14's "Re-cook" button. On the *profile* and deliberately not in
                 * @ref CookingParameters, because it must not reach
                 * @ref cooking_parameters_hash: a flag that changed the key would not bypass
                 * the stale entry, it would file the result under a different one and leave the
                 * next ordinary cook reading the stale entry again.
                 *
                 * Not persisted and not part of an override — it describes one press of a
                 * button, not a property of an asset.
                 */
                bool force_recook = false;
            };

            /**
             * @brief The per-asset half: only what this asset says differs.
             *
             * Optional-valued rather than a second full profile, so "this crate is
             * deformable" is one field and not a copy of the project's settings that goes
             * stale the moment the project's change.
             */
            struct ImportProfileOverride
            {
                /** @brief How accurately to cook this asset, in [0, 1]. */
                std::optional<float> fidelity;

                /** @brief Whether to produce a `.sushicollision` for it. */
                std::optional<bool> cook_collision;

                /** @brief Whether to produce a `.sushisoft` for it. */
                std::optional<bool> cook_soft_body;

                /** @brief Whether this is authored-static geometry (§8.4 item 4). */
                std::optional<bool> static_geometry;

                /** @brief Whether any field is set; an empty override resolves to the default. */
                bool empty() const noexcept
                {
                    return !fidelity.has_value() && !cook_collision.has_value() &&
                           !cook_soft_body.has_value() && !static_geometry.has_value();
                }
            };

            /**
             * @brief Folds @p asset over @p project_default.
             *
             * @param project_default The project's settings.
             * @param asset           What this asset says differs; may be empty.
             * @return The resolved profile.
             */
            inline ImportProfile resolve_import_profile(const ImportProfile& project_default,
                                                        const ImportProfileOverride& asset)
            {
                ImportProfile resolved = project_default;
                if (asset.fidelity.has_value())
                    resolved.parameters.fidelity = *asset.fidelity;
                if (asset.cook_collision.has_value())
                    resolved.parameters.cook_collision = *asset.cook_collision;
                if (asset.cook_soft_body.has_value())
                    resolved.parameters.cook_soft_body = *asset.cook_soft_body;
                if (asset.static_geometry.has_value())
                    resolved.parameters.static_geometry = *asset.static_geometry;
                return resolved;
            }

            /**
             * @brief The project's default profile, plus whatever individual assets say.
             *
             * Keyed by the asset's path as the project spells it. A plain map and not a
             * pattern matcher: a glob rule that quietly makes half a directory deformable is
             * the kind of setting nobody can find the source of when a level's cook time
             * triples, and per-asset overrides are what §8.1 actually specifies.
             */
            class ImportProfileLibrary
            {
            public:
                /** @brief The default every asset without an override is cooked at. */
                const ImportProfile& project_default() const noexcept { return default_; }

                /** @brief Replaces the project default. */
                void set_project_default(const ImportProfile& profile) { default_ = profile; }

                /**
                 * @brief Records what one asset says differs.
                 *
                 * An empty override *removes* the entry rather than storing a no-op, so
                 * "reset to project default" and "was never set" are the same state — which
                 * they have to be, or a project accumulates entries that do nothing and a
                 * changed default silently fails to reach them.
                 *
                 * @param asset_path The asset's path, as the project spells it.
                 * @param override_values What differs; empty removes any entry.
                 */
                void set_override(const std::string& asset_path,
                                  const ImportProfileOverride& override_values)
                {
                    if (override_values.empty())
                        overrides_.erase(asset_path);
                    else
                        overrides_[asset_path] = override_values;
                }

                /** @brief Whether @p asset_path has an override recorded. */
                bool has_override(const std::string& asset_path) const
                {
                    return overrides_.find(asset_path) != overrides_.end();
                }

                /** @brief How many assets carry an override. */
                std::size_t override_count() const noexcept { return overrides_.size(); }

                /**
                 * @brief The profile @p asset_path is cooked at.
                 *
                 * @param asset_path The asset's path.
                 * @return The project default with this asset's override folded in.
                 */
                ImportProfile resolve(const std::string& asset_path) const
                {
                    const auto found = overrides_.find(asset_path);
                    if (found == overrides_.end())
                        return default_;
                    return resolve_import_profile(default_, found->second);
                }

            private:
                ImportProfile default_;
                std::unordered_map<std::string, ImportProfileOverride> overrides_;
            };
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
