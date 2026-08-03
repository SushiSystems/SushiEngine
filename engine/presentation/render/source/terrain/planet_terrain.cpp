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
                                         const PlanetTerrainDescription& desc)
                : device_(device), source_(pack_), desc_(desc)
            {
                records_.reserve(desc.maximum_nodes);
                nodes_.reserve(desc.maximum_nodes);
                misses_.reserve(desc.maximum_nodes);
                scratch_.resize(Field::TILE_SAMPLE_COUNT);
            }

            void PlanetTerrain::set_body(int body, const std::string& pack_path)
            {
                body_index_ = body;
                records_.clear();
                nodes_.clear();
                pack_ = pack_path.empty() ? Field::PlanetPack{}
                                          : Field::load_planet_pack(pack_path);
                if (!pack_.loaded())
                    return;

                // The pool is created with the first body that has one and outlives every
                // body after it: its slots carry no identity beyond what the residency says
                // they hold, so a new body needs the index cleared and nothing else.
                if (!cache_)
                    cache_ = std::make_unique<TileCache>(device_, desc_.slot_count);
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

                Field::QuadtreeParameters parameters;
                parameters.screen_error_pixels = desc_.screen_error_pixels;
                parameters.viewport_height_pixels = view.viewport_height_pixels;
                parameters.vertical_field_of_view_radians = view.vertical_field_of_view_radians;
                parameters.maximum_depth = desc_.maximum_depth;
                parameters.maximum_nodes = desc_.maximum_nodes;
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
        } // namespace Terrain
    } // namespace Render
} // namespace SushiEngine
