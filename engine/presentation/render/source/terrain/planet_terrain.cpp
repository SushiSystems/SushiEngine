/**************************************************************************/
/* planet_terrain.cpp                                                     */
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

#include "terrain/planet_terrain.hpp"

#include <algorithm>
#include <cstring>

#include <SushiEngine/terrain/height_function.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Terrain
        {
            namespace
            {
                namespace Field = SushiEngine::Terrain;

                /** @brief The grid extent of a node at a depth: two over two to the depth. */
                float grid_span(std::uint8_t depth) noexcept
                {
                    return 2.0f / static_cast<float>(std::uint64_t(1) << depth);
                }

                /**
                 * @brief The asset stem a body's pack is written under.
                 *
                 * Only bodies with a baker recipe appear; the rest have nothing to name.
                 * Kept as the ephemeris index rather than @c Astro::BodyId because this
                 * layer is deliberately astro-free — the index arrives on the environment.
                 */
                const char* pack_stem(int body) noexcept
                {
                    switch (body)
                    {
                        case 4: return "moon";
                        default: return nullptr;
                    }
                }
            } // namespace

            std::string default_pack_path(int body)
            {
                const char* stem = body >= 0 ? pack_stem(body) : nullptr;
                if (stem == nullptr)
                    return std::string{};
                return std::string("assets/planet/") + stem + ".compact.planet";
            }

            PlanetTerrain::PlanetTerrain(Vulkan::VulkanDevice& device,
                                         const PlanetTerrainDescription& description)
                : device_(device), source_(pack_), description_(description)
            {
                records_.reserve(description.maximum_nodes);
                nodes_.reserve(description.maximum_nodes);
                misses_.reserve(description.maximum_nodes);
                scratch_.resize(Field::TILE_SAMPLE_COUNT);
            }

            void PlanetTerrain::set_body(int body, const std::string& pack_path)
            {
                body_index_ = body;
                records_.clear();
                nodes_.clear();
                // The edits go with the body they were authored against: a crater is a
                // direction on *this* sphere, and carrying the stack across would reshape
                // the next world at whatever point that direction happens to hit. The
                // pending work goes with them — it names tiles of the world being left.
                layers_.clear();
                dirty_footprints_.clear();
                recompile_.clear();
                pack_ = pack_path.empty() ? Field::PlanetPack{}
                                          : Field::load_planet_pack(pack_path);
                if (!pack_.loaded())
                    return;

                // The pool is created with the first body that has one and outlives every
                // body after it: its slots carry no identity beyond what the residency says
                // they hold, so a new body needs the index cleared and nothing else.
                if (!cache_)
                    cache_ = std::make_unique<TileCache>(device_, description_.slot_count);
                else
                    cache_->forget_all();
            }

            void PlanetTerrain::prepare(const TerrainFrameView& view)
            {
                records_.clear();
                misses_.clear();
                uploads_ = 0;
                inherited_ = 0;
                statistics_ = Field::QuadtreeStatistics{};

                std::memcpy(body_.body_to_scene, view.body_to_scene,
                            sizeof(body_.body_to_scene));
                // Kept for the authoring surface, which has no other way to say "here":
                // the camera's own direction from the body centre is the ground under it.
                // A camera exactly at the centre has no direction, so the axis stands.
                const double reach = length(view.camera_body_fixed);
                if (reach > 0.0)
                    view_direction_ = normalize(view.camera_body_fixed);
                const Field::Ellipsoid& ellipsoid = pack_.ellipsoid();
                body_.semi_axes[0] = static_cast<float>(ellipsoid.semi_axis_x);
                body_.semi_axes[1] = static_cast<float>(ellipsoid.semi_axis_y);
                body_.semi_axes[2] = static_cast<float>(ellipsoid.semi_axis_z);
                body_.semi_axes[3] = 0.0f;

                if (!pack_.loaded() || !cache_)
                    return;

                // Built here rather than held as a member: it carries a copy of the
                // ellipsoid, and a member would go stale the moment set_body pointed the
                // pack at a different world. It is three references wide; there is nothing
                // to save by keeping one.
                const Field::HeightFunction height{source_, layers_, ellipsoid};

                cache_->begin_frame(view.frame_index, view.frame_slot);
                collect_recompilations();

                Field::QuadtreeParameters parameters;
                parameters.screen_error_pixels = description_.screen_error_pixels;
                parameters.viewport_height_pixels = view.viewport_height_pixels;
                parameters.vertical_field_of_view_radians = view.vertical_field_of_view_radians;
                parameters.maximum_depth = description_.maximum_depth;
                parameters.maximum_nodes = description_.maximum_nodes;
                parameters.frustum = view.frustum;
                statistics_ = Field::select_terrain_nodes(ellipsoid, source_,
                                                          view.camera_body_fixed, parameters,
                                                          nodes_);

                // Bind first, stage second. A node that already has its own tile costs a
                // lookup; one that does not is recorded and dealt with in nearest-first
                // order, because the tiles worth the frame's upload budget are the ones
                // closest to the camera rather than the ones the selection happened to
                // visit first.
                records_.resize(nodes_.size());
                for (std::size_t index = 0; index < nodes_.size(); ++index)
                {
                    Field::TileBinding binding;
                    if (!cache_->bind(nodes_[index].address, binding) || !binding.exact)
                        misses_.push_back(Miss{index, nodes_[index].distance_metres});
                }

                std::sort(misses_.begin(), misses_.end(),
                          [](const Miss& a, const Miss& b) { return a.distance < b.distance; });

                // Ahead of the misses in the frame's budget: a queued recompile is ground
                // already on screen carrying the shape it had before an edit, while a miss
                // is ground drawn coarser than asked for. Wrong beats coarse.
                drain_recompilations(height);

                for (const Miss& miss : misses_)
                {
                    if (!cache_->can_stage())
                        break;
                    const Field::TileAddress& address = nodes_[miss.node].address;
                    Field::TileStatistics ignored;
                    if (!height.evaluate_tile(address, scratch_.data(), ignored))
                        continue; // the source does not cover it; the ancestor still draws
                    if (cache_->stage(address, scratch_.data()) == Field::INVALID_TILE_SLOT)
                        break;
                    ++uploads_;
                }

                // Pack after staging, so a tile that arrived this frame is drawn this frame
                // rather than a frame late.
                std::size_t written = 0;
                for (std::size_t index = 0; index < nodes_.size(); ++index)
                {
                    const Field::TerrainNode& node = nodes_[index];
                    Field::TileBinding binding;
                    if (!cache_->bind(node.address, binding))
                        continue; // nothing covers it at all; drawing it would sample noise
                    if (!binding.exact)
                        ++inherited_;

                    TerrainNodeRecord& record = records_[written++];
                    record.origin[0] = static_cast<float>(node.origin_camera_relative.x);
                    record.origin[1] = static_cast<float>(node.origin_camera_relative.y);
                    record.origin[2] = static_cast<float>(node.origin_camera_relative.z);
                    record.origin[3] = grid_span(node.address.depth);

                    record.centre[0] = static_cast<float>(node.centre_cube.x);
                    record.centre[1] = static_cast<float>(node.centre_cube.y);
                    record.centre[2] = static_cast<float>(node.centre_cube.z);
                    record.centre[3] = static_cast<float>(node.address.face);

                    const Field::TileGridRect rect = Field::tile_grid_rect(node.address);
                    record.grid_morph[0] = static_cast<float>(rect.s_minimum);
                    record.grid_morph[1] = static_cast<float>(rect.t_minimum);
                    record.grid_morph[2] = static_cast<float>(node.morph_start_metres);
                    record.grid_morph[3] = static_cast<float>(node.morph_end_metres);

                    record.decode[0] = binding.minimum_metres;
                    record.decode[1] = binding.maximum_metres;
                    record.decode[2] = static_cast<float>(binding.slot);
                    record.decode[3] = 0.0f;

                    record.uv_rect[0] = binding.rect.offset_s;
                    record.uv_rect[1] = binding.rect.offset_t;
                    record.uv_rect[2] = binding.rect.scale_s;
                    record.uv_rect[3] = binding.rect.scale_t;
                }
                records_.resize(written);
            }

            double PlanetTerrain::mean_radius_metres() const noexcept
            {
                if (!pack_.loaded())
                    return 0.0;
                const Field::Ellipsoid& ellipsoid = pack_.ellipsoid();
                return (ellipsoid.semi_axis_x + ellipsoid.semi_axis_y + ellipsoid.semi_axis_z) /
                       3.0;
            }

            Field::TerrainLayer PlanetTerrain::layer(std::size_t index) const
            {
                if (index >= layers_.size())
                    return Field::TerrainLayer{};
                return layers_.at(index);
            }

            bool PlanetTerrain::insert_layer(const Field::TerrainLayer& layer)
            {
                if (!layers_.insert(layer))
                    return false;
                mark_dirty(layer.footprint);
                return true;
            }

            bool PlanetTerrain::update_layer(std::uint32_t order,
                                             const Field::TerrainLayer& layer)
            {
                const Field::TerrainLayer* existing = layers_.find(order);
                if (existing == nullptr)
                    return false;
                if (layer.order != order && layers_.find(layer.order) != nullptr)
                    return false;
                // Copied before the removal invalidates the pointer, and needed after it:
                // the ground the layer stops covering is as wrong as the ground it starts
                // covering, and only the old record knows where that was.
                const Field::LayerFootprint vacated = existing->footprint;
                layers_.remove(order);
                layers_.insert(layer);
                mark_dirty(vacated);
                mark_dirty(layer.footprint);
                return true;
            }

            bool PlanetTerrain::remove_layer(std::uint32_t order)
            {
                const Field::TerrainLayer* existing = layers_.find(order);
                if (existing == nullptr)
                    return false;
                const Field::LayerFootprint vacated = existing->footprint;
                layers_.remove(order);
                mark_dirty(vacated);
                return true;
            }

            bool PlanetTerrain::swap_layer_order(std::uint32_t first, std::uint32_t second)
            {
                if (first == second)
                    return false;
                const Field::TerrainLayer* lower = layers_.find(first);
                const Field::TerrainLayer* upper = layers_.find(second);
                if (lower == nullptr || upper == nullptr)
                    return false;
                Field::TerrainLayer moved_lower = *lower;
                Field::TerrainLayer moved_upper = *upper;
                moved_lower.order = second;
                moved_upper.order = first;
                layers_.remove(first);
                layers_.remove(second);
                layers_.insert(moved_lower);
                layers_.insert(moved_upper);
                mark_dirty(moved_lower.footprint);
                mark_dirty(moved_upper.footprint);
                return true;
            }

            void PlanetTerrain::clear_layers()
            {
                for (std::size_t index = 0; index < layers_.size(); ++index)
                    mark_dirty(layers_.at(index).footprint);
                layers_.clear();
            }

            void PlanetTerrain::mark_dirty(const Field::LayerFootprint& footprint)
            {
                // With no pool there is nothing compiled to be stale, and remembering the
                // region anyway would grow a list nothing ever drains.
                if (!cache_)
                    return;
                dirty_footprints_.push_back(footprint);
            }

            void PlanetTerrain::collect_recompilations()
            {
                if (dirty_footprints_.empty())
                    return;

                const std::uint32_t slots = cache_->slot_count();
                for (std::uint32_t slot = 0; slot < slots; ++slot)
                {
                    if (!cache_->slot_occupied(slot))
                        continue;
                    const Field::TileAddress address = cache_->slot_address(slot);
                    const SushiEngine::Vector3 centre = Field::tile_sample_direction(
                        address, Field::TILE_STRIDE / 2u, Field::TILE_STRIDE / 2u);
                    const double radius = Field::tile_angular_radius(address);
                    for (const Field::LayerFootprint& footprint : dirty_footprints_)
                    {
                        if (!Field::footprint_overlaps(footprint, centre, radius))
                            continue;
                        recompile_.push_back(address);
                        break;
                    }
                }
                dirty_footprints_.clear();

                std::sort(recompile_.begin(), recompile_.end(),
                          [](const Field::TileAddress& a, const Field::TileAddress& b)
                          {
                              return Field::tile_address_key(a) < Field::tile_address_key(b);
                          });
                recompile_.erase(
                    std::unique(recompile_.begin(), recompile_.end(),
                                [](const Field::TileAddress& a, const Field::TileAddress& b)
                                {
                                    return Field::tile_address_key(a) ==
                                           Field::tile_address_key(b);
                                }),
                    recompile_.end());
            }

            void PlanetTerrain::drain_recompilations(const Field::HeightFunction& height)
            {
                while (!recompile_.empty() && cache_->can_stage())
                {
                    const Field::TileAddress address = recompile_.back();
                    // A tile evicted since the edit holds nothing stale: whatever is staged
                    // into its slot next is compiled from the stack as it is then.
                    if (cache_->find(address) != Field::INVALID_TILE_SLOT)
                    {
                        Field::TileStatistics statistics;
                        if (height.evaluate_tile(address, scratch_.data(), statistics))
                        {
                            if (cache_->stage(address, scratch_.data()) ==
                                Field::INVALID_TILE_SLOT)
                                return; // no slot to take this frame; it stays queued
                            ++uploads_;
                        }
                    }
                    recompile_.pop_back();
                }
            }
        } // namespace Terrain
    } // namespace Render
} // namespace SushiEngine
