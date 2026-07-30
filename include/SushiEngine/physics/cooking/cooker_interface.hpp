/**************************************************************************/
/* cooker_interface.hpp                                                   */
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
 * @file cooker_interface.hpp
 * @brief The three seams the cooking pipeline is assembled from.
 *
 * §3.3's entry for `physics/cooking`, and §4.2's open/closed rule applied to it: a new
 * stage is an object inserted into a list, not a case added to a function, and a new
 * output kind is a new @ref IMeshCooker rather than a branch inside an existing one.
 *
 * @ref ICookedAssetStore is the one that earns its keep least obviously and matters
 * most. The cooker never writes a file. It hands bytes to a store, and the store
 * decides whether those bytes land on a disk, in a memory map, or in a test's vector —
 * which is what makes the whole pipeline runnable on a build machine, in a unit test,
 * and inside the editor without three code paths. It is also where the content-hash
 * cache lives, because "has this already been cooked" is a question about storage.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/cooking/cooking_parameters.hpp>
#include <SushiEngine/physics/cooking/cooking_report.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /** @brief Which cooked asset family a blob belongs to. */
            enum class CookedAssetKind : std::uint32_t
            {
                /** @brief A `.sushicollision`: hulls or a static hierarchy, plus a field. */
                Collision = 0,

                /** @brief A `.sushisoft`: a tetrahedral mesh and its embedding. */
                SoftBody,
            };

            /**
             * @brief What identifies a cooked asset: its input, its cooker, its settings.
             *
             * The three-part key §8.1 specifies. All three are needed and each catches a
             * different mistake: the content hash catches an edited mesh, the parameters
             * hash catches a moved dial, and the cooker version catches *the cooker
             * itself changing* — which is the one a pipeline usually forgets, and its
             * symptom is a bug fixed in the cooker that never reaches the assets cooked
             * before the fix.
             */
            struct CookedAssetKey
            {
                /** @brief @ref mesh_content_hash of the source geometry. */
                std::uint64_t source_hash = 0;

                /** @brief @ref cooking_parameters_hash of the resolved parameters. */
                std::uint64_t parameters_hash = 0;

                /** @brief The producing cooker's @ref IMeshCooker::version. */
                std::uint32_t cooker_version = 0;

                /** @brief Which family the blob belongs to. */
                CookedAssetKind kind = CookedAssetKind::Collision;

                /** @brief Whether two keys name the same cooked asset. */
                bool operator==(const CookedAssetKey& other) const noexcept
                {
                    return source_hash == other.source_hash &&
                           parameters_hash == other.parameters_hash &&
                           cooker_version == other.cooker_version && kind == other.kind;
                }

                /** @brief Whether two keys name different cooked assets. */
                bool operator!=(const CookedAssetKey& other) const noexcept
                {
                    return !(*this == other);
                }
            };

            /**
             * @brief The three hashes folded into one, for a store that wants a filename.
             *
             * @param key The asset's identity.
             * @return A hash that differs whenever any part of @p key differs.
             */
            inline std::uint64_t cooked_asset_key_hash(const CookedAssetKey& key) noexcept
            {
                std::uint64_t hash = 1469598103934665603ull;
                hash = hash_bytes(hash, key.source_hash);
                hash = hash_bytes(hash, key.parameters_hash);
                hash = hash_bytes(hash, key.cooker_version);
                hash = hash_bytes(hash, key.kind);
                return hash;
            }

            /**
             * @brief Where cooked blobs live, and the only thing that touches storage.
             *
             * Owned by the consumer (§3.3), so the editor's store, the build machine's
             * store and a test's store are the same code path with different objects
             * behind them.
             *
             * Implementations must be safe to @ref load from while a cook is running on
             * another thread, because §8.1 puts cooking off the main thread and the
             * editor keeps drawing. They are not required to tolerate two concurrent
             * @ref store calls for the same key; the pipeline serializes those.
             */
            class ICookedAssetStore
            {
            public:
                virtual ~ICookedAssetStore() = default;

                /**
                 * @brief Whether an asset for @p key is already stored.
                 *
                 * The whole cache: a true answer here is what makes an unchanged mesh
                 * with unchanged parameters never re-cook.
                 *
                 * @param key The asset's identity.
                 * @return True when @ref load would succeed.
                 */
                virtual bool contains(const CookedAssetKey& key) const = 0;

                /**
                 * @brief Reads a stored asset's bytes.
                 *
                 * @param key The asset's identity.
                 * @param out Receives the blob; cleared first, left empty on failure.
                 * @return False when nothing is stored for @p key, or the read failed.
                 */
                virtual bool load(const CookedAssetKey& key, std::vector<std::byte>& out) const = 0;

                /**
                 * @brief Writes an asset's bytes, replacing any previous ones.
                 *
                 * @param key   The asset's identity.
                 * @param bytes The blob to store.
                 * @return False when the write failed; a caller may still use @p bytes,
                 *         since a store that cannot cache is slow rather than broken.
                 */
                virtual bool store(const CookedAssetKey& key,
                                   const std::vector<std::byte>& bytes) = 0;

                /**
                 * @brief Forgets a stored asset.
                 *
                 * What the editor's "Re-cook" button needs: the point of pressing it is
                 * to get past a cache entry whose key has not changed, usually because
                 * the cooker is being worked on.
                 *
                 * @param key The asset's identity.
                 * @return True when something was removed.
                 */
                virtual bool evict(const CookedAssetKey& key) = 0;
            };

            /**
             * @brief One ordered step of a cook.
             *
             * §4.2: `Repair -> Voxelize -> Tetrahedralize -> Optimize -> Embed ->
             * Decompose -> BakeDistanceField -> BuildLevelsOfDetail -> Serialize` is a
             * list of these, so adding a stage inserts an object.
             *
             * A stage reports progress rather than being asked for it, because the only
             * code that knows a voxelizer is a third of the way through a flood fill is
             * the flood fill.
             *
             * @tparam Context The cook state a family of stages passes between them. A
             *                 template parameter and not a base class: a rigid cook's
             *                 state and a soft-body cook's state have nothing in common,
             *                 and a shared base would exist only to be downcast.
             */
            template <typename Context>
            class ICookingStage
            {
            public:
                virtual ~ICookingStage() = default;

                /** @brief A short stable name, for the report and the progress display. */
                virtual const char* name() const noexcept = 0;

                /**
                 * @brief Runs the stage against @p context.
                 *
                 * @param context The cook state, read and advanced in place.
                 * @return False to abort the cook; the pipeline records
                 *         @ref CookingStatus::StageFailed and names this stage.
                 */
                virtual bool run(Context& context) = 0;
            };

            /** @brief How far along a cook is, for a caller drawing a progress bar. */
            struct CookingProgress
            {
                /** @brief The running stage's @ref ICookingStage::name, or nullptr. */
                const char* stage = nullptr;

                /** @brief Stages completed so far. */
                std::uint32_t completed_stages = 0;

                /** @brief Stages in the pipeline. */
                std::uint32_t total_stages = 0;
            };

            /**
             * @brief What a caller passes to be told how a cook is going.
             *
             * A pure-virtual sink rather than a `std::function`, so the pipeline can be
             * compiled without a heap allocation per cook and a caller that wants no
             * progress passes nothing.
             */
            class ICookingProgressSink
            {
            public:
                virtual ~ICookingProgressSink() = default;

                /**
                 * @brief Called as each stage begins, on the cooking thread.
                 *
                 * @param progress Where the cook has got to.
                 */
                virtual void on_progress(const CookingProgress& progress) = 0;
            };

            /**
             * @brief A mesh in, one cooked asset out.
             *
             * §3.3: `CollisionCooker | SoftBodyCooker | NodeBeamCooker`. The interface
             * carries the cache key's third component, because the version belongs to the
             * cooker and asking the caller to supply it is asking it to be wrong.
             */
            class IMeshCooker
            {
            public:
                virtual ~IMeshCooker() = default;

                /** @brief A short stable name, for reports and logs. */
                virtual const char* name() const noexcept = 0;

                /**
                 * @brief This cooker's output-format version.
                 *
                 * Bumped whenever the produced bytes would differ for the same input,
                 * which is what invalidates every cached asset it ever wrote. Forgetting
                 * to bump it is the one mistake this key cannot catch on the caller's
                 * behalf.
                 */
                virtual std::uint32_t version() const noexcept = 0;

                /** @brief Which asset family this cooker produces. */
                virtual CookedAssetKind kind() const noexcept = 0;

                /**
                 * @brief The cache key @p mesh at @p parameters would be stored under.
                 *
                 * On the cooker because the cooker owns its version, and the version is the one
                 * part of the key nothing else can derive. Published because two callers need
                 * it and neither can compute it: an editor showing where an asset is cached,
                 * and a "Re-cook" that has to *evict* — which is the whole point of that
                 * button, since it exists for the case where the key has not changed and the
                 * cooker has.
                 *
                 * @param mesh       The source geometry.
                 * @param parameters The authored parameters.
                 * @return The key @ref cook would look under and write to.
                 */
                virtual CookedAssetKey cache_key(const Geometry::TriangleMeshView& mesh,
                                                 const CookingParameters& parameters)
                    const noexcept = 0;

                /**
                 * @brief Cooks @p mesh into @p out, or serves it from @p store.
                 *
                 * The cache check is the cooker's rather than the caller's, so that "an
                 * unchanged mesh is never re-cooked" is a property of the pipeline and
                 * not of every call site remembering to ask.
                 *
                 * @param mesh       The source geometry.
                 * @param parameters The dial and any pinned overrides.
                 * @param store      Where to look before cooking and to write after; may
                 *                   be null, which cooks unconditionally and caches
                 *                   nothing.
                 * @param progress   Told as each stage begins; may be null.
                 * @param out        Receives the asset bytes; cleared first.
                 * @return The report. Its @c status says whether @p out holds an asset,
                 *         and @c served_from_cache says whether anything was cooked.
                 */
                virtual CookingReport cook(const Geometry::TriangleMeshView& mesh,
                                           const CookingParameters& parameters,
                                           ICookedAssetStore* store,
                                           ICookingProgressSink* progress,
                                           std::vector<std::byte>& out) = 0;
            };
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
