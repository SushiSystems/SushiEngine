/**************************************************************************/
/* node_beam_asset.cpp                                                    */
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

#include <SushiEngine/physics/cooking/node_beam_asset.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            namespace
            {
                /**
                 * @brief Hands out aligned section offsets, one call per section.
                 *
                 * The same reservation loop the soft-body blob is laid out by, for the same
                 * reason: the arithmetic is identical every time, and a cursor written out by
                 * hand per section acquires exactly one transposed line, whose symptom is a
                 * section reading its neighbour's bytes and producing a plausible vehicle
                 * rather than a crash.
                 */
                class BlobLayout
                {
                public:
                    explicit BlobLayout(std::size_t header_bytes)
                        : cursor_(align_up(header_bytes, ALIGNMENT))
                    {
                    }

                    /** @brief Reserves @p bytes and returns the offset they start at. */
                    std::uint32_t reserve(std::size_t bytes) noexcept
                    {
                        const std::uint32_t offset = std::uint32_t(cursor_);
                        cursor_ = align_up(cursor_ + bytes, ALIGNMENT);
                        return offset;
                    }

                    /** @brief The blob's total size, with the last section's padding included. */
                    std::size_t total() const noexcept { return cursor_; }

                private:
                    static constexpr std::size_t ALIGNMENT = 16;

                    static std::size_t align_up(std::size_t value, std::size_t alignment) noexcept
                    {
                        return (value + alignment - 1) & ~(alignment - 1);
                    }

                    std::size_t cursor_;
                };

                /** @brief Copies a vector's bytes into the blob at @p offset. */
                template <typename T>
                void write_section(std::byte* base, std::uint32_t offset,
                                   const std::vector<T>& values) noexcept
                {
                    if (!values.empty())
                        std::memcpy(base + offset, values.data(), values.size() * sizeof(T));
                }

                /** @brief Whether a section of @p bytes fits inside a blob of @p total. */
                bool section_fits(std::uint32_t total, std::uint32_t offset,
                                  std::size_t bytes) noexcept
                {
                    if (bytes == 0)
                        return true;
                    if (offset > total)
                        return false;
                    return std::size_t(total) - offset >= bytes;
                }

                /**
                 * @brief One past the highest part index any record names.
                 *
                 * Derived rather than authored, so @ref NodeBeamSummary::part_count cannot
                 * disagree with the records it counts. A structure whose parts are numbered
                 * with a gap still reports the range, because the range is what an array
                 * indexed by part has to be sized to.
                 */
                std::uint32_t derive_part_count(const NodeBeamAsset& asset) noexcept
                {
                    std::uint32_t count = 0;
                    const auto widen = [&count](std::uint32_t part) noexcept
                    {
                        if (part + 1 > count)
                            count = part + 1;
                    };
                    for (const NodeBeamNodeRecord& node : asset.nodes)
                        widen(node.part);
                    for (const NodeBeamBeamRecord& beam : asset.beams)
                        widen(beam.part);
                    for (const NodeBeamAttachmentRecord& attachment : asset.attachments)
                        widen(attachment.part);
                    return count;
                }
            } // namespace

            bool build_node_beam_blob(const NodeBeamAsset& asset, std::vector<std::byte>& out)
            {
                out.clear();
                if (asset.nodes.empty())
                    return false;
                if (asset.surface_indices.size() % 3 != 0)
                    return false;

                const std::size_t nodes = asset.nodes.size();

                // Cross-references first, because those are what a format this wide gets
                // wrong and each one unchecked is a read into a neighbouring section.
                for (const NodeBeamBeamRecord& beam : asset.beams)
                {
                    if (std::size_t(beam.a) >= nodes || std::size_t(beam.b) >= nodes)
                        return false;
                    // A beam onto itself has no axis, projects nothing, and would sit in the
                    // structure reporting zero load while the panel it was meant to hold
                    // flaps. Rejected at the cook rather than explained later.
                    if (beam.a == beam.b)
                        return false;
                }
                for (const std::uint32_t index : asset.surface_indices)
                {
                    if (std::size_t(index) >= nodes)
                        return false;
                }
                for (const NodeBeamAttachmentRecord& attachment : asset.attachments)
                {
                    if (std::size_t(attachment.node) >= nodes)
                        return false;
                }
                for (const NodeBeamSkinRecord& record : asset.skin)
                {
                    for (std::uint32_t i = 0; i < NODE_BEAM_SKIN_INFLUENCES; ++i)
                    {
                        // Every slot, whatever its weight: the format's promise is that a
                        // reader never has to test a weight before trusting an index.
                        if (std::size_t(record.nodes[i]) >= nodes)
                            return false;
                    }
                }

                // An asset whose summary disagrees with its records is an asset that reports
                // one structure and simulates another, so the derived count is checked here
                // rather than trusted.
                if (asset.summary.part_count != derive_part_count(asset))
                    return false;

                BlobLayout layout(sizeof(NodeBeamBlobHeader));
                NodeBeamBlobHeader header{};
                header.parameters_offset = layout.reserve(sizeof(CookingParameters));
                header.summary_offset = layout.reserve(sizeof(NodeBeamSummary));
                header.core_offset = layout.reserve(sizeof(NodeBeamCoreRecord));
                header.nodes_offset = layout.reserve(nodes * sizeof(NodeBeamNodeRecord));
                header.beams_offset =
                    layout.reserve(asset.beams.size() * sizeof(NodeBeamBeamRecord));
                header.surface_indices_offset =
                    layout.reserve(asset.surface_indices.size() * sizeof(std::uint32_t));
                header.attachments_offset =
                    layout.reserve(asset.attachments.size() * sizeof(NodeBeamAttachmentRecord));
                header.skin_offset = layout.reserve(asset.skin.size() * sizeof(NodeBeamSkinRecord));

                const std::size_t total = layout.total();
                out.assign(total, std::byte{0});
                std::byte* base = out.data();

                std::memcpy(header.magic, NODE_BEAM_BLOB_MAGIC, sizeof(header.magic));
                header.version = NODE_BEAM_BLOB_VERSION;
                header.total_size = std::uint32_t(total);
                header.node_count = std::uint32_t(nodes);
                header.beam_count = std::uint32_t(asset.beams.size());
                header.surface_index_count = std::uint32_t(asset.surface_indices.size());
                header.attachment_count = std::uint32_t(asset.attachments.size());
                header.skin_count = std::uint32_t(asset.skin.size());
                std::memcpy(base, &header, sizeof(header));

                // The parameters travel inside the asset (§8.3 stage 10), so a re-cook is
                // reproducible and a mismatch is detectable without the project file.
                std::memcpy(base + header.parameters_offset, &asset.parameters,
                            sizeof(CookingParameters));
                std::memcpy(base + header.summary_offset, &asset.summary, sizeof(NodeBeamSummary));
                std::memcpy(base + header.core_offset, &asset.core, sizeof(NodeBeamCoreRecord));

                write_section(base, header.nodes_offset, asset.nodes);
                write_section(base, header.beams_offset, asset.beams);
                write_section(base, header.surface_indices_offset, asset.surface_indices);
                write_section(base, header.attachments_offset, asset.attachments);
                write_section(base, header.skin_offset, asset.skin);
                return true;
            }

            bool validate_node_beam_blob(const std::byte* data, std::size_t size) noexcept
            {
                if (data == nullptr || size < sizeof(NodeBeamBlobHeader))
                    return false;

                NodeBeamBlobHeader header{};
                std::memcpy(&header, data, sizeof(header));
                if (std::memcmp(header.magic, NODE_BEAM_BLOB_MAGIC, sizeof(header.magic)) != 0)
                    return false;
                if (header.version != NODE_BEAM_BLOB_VERSION)
                    return false;
                if (header.total_size > size)
                    return false;
                if (header.node_count == 0)
                    return false;
                if (header.surface_index_count % 3 != 0)
                    return false;

                const std::uint32_t total = header.total_size;
                if (!section_fits(total, header.parameters_offset, sizeof(CookingParameters)))
                    return false;
                if (!section_fits(total, header.summary_offset, sizeof(NodeBeamSummary)))
                    return false;
                if (!section_fits(total, header.core_offset, sizeof(NodeBeamCoreRecord)))
                    return false;
                if (!section_fits(total, header.nodes_offset,
                                  std::size_t(header.node_count) * sizeof(NodeBeamNodeRecord)))
                    return false;
                if (!section_fits(total, header.beams_offset,
                                  std::size_t(header.beam_count) * sizeof(NodeBeamBeamRecord)))
                    return false;
                if (!section_fits(total, header.surface_indices_offset,
                                  std::size_t(header.surface_index_count) * sizeof(std::uint32_t)))
                    return false;
                if (!section_fits(total, header.attachments_offset,
                                  std::size_t(header.attachment_count) *
                                      sizeof(NodeBeamAttachmentRecord)))
                    return false;
                if (!section_fits(total, header.skin_offset,
                                  std::size_t(header.skin_count) * sizeof(NodeBeamSkinRecord)))
                    return false;

                // Checked here and not only at write time, because a blob may have been
                // produced by an older writer or edited by hand.
                const NodeBeamBeamRecord* beams =
                    reinterpret_cast<const NodeBeamBeamRecord*>(data + header.beams_offset);
                for (std::uint32_t i = 0; i < header.beam_count; ++i)
                {
                    if (beams[i].a >= header.node_count || beams[i].b >= header.node_count)
                        return false;
                    if (beams[i].a == beams[i].b)
                        return false;
                }
                const std::uint32_t* surface =
                    reinterpret_cast<const std::uint32_t*>(data + header.surface_indices_offset);
                for (std::uint32_t i = 0; i < header.surface_index_count; ++i)
                {
                    if (surface[i] >= header.node_count)
                        return false;
                }
                const NodeBeamAttachmentRecord* attachments =
                    reinterpret_cast<const NodeBeamAttachmentRecord*>(data +
                                                                      header.attachments_offset);
                for (std::uint32_t i = 0; i < header.attachment_count; ++i)
                {
                    if (attachments[i].node >= header.node_count)
                        return false;
                }
                const NodeBeamSkinRecord* skin =
                    reinterpret_cast<const NodeBeamSkinRecord*>(data + header.skin_offset);
                for (std::uint32_t i = 0; i < header.skin_count; ++i)
                {
                    for (std::uint32_t influence = 0; influence < NODE_BEAM_SKIN_INFLUENCES;
                         ++influence)
                    {
                        if (skin[i].nodes[influence] >= header.node_count)
                            return false;
                    }
                }
                return true;
            }

            NodeBeamAssetView load_node_beam_blob(const std::byte* data, std::size_t size) noexcept
            {
                NodeBeamAssetView view;
                if (!validate_node_beam_blob(data, size))
                    return view;

                NodeBeamBlobHeader header{};
                std::memcpy(&header, data, sizeof(header));
                std::memcpy(&view.parameters, data + header.parameters_offset,
                            sizeof(CookingParameters));
                std::memcpy(&view.summary, data + header.summary_offset, sizeof(NodeBeamSummary));
                std::memcpy(&view.core, data + header.core_offset, sizeof(NodeBeamCoreRecord));

                const auto pointer = [data](std::uint32_t offset, std::uint32_t count) -> const void*
                {
                    return count == 0 ? nullptr : static_cast<const void*>(data + offset);
                };

                view.node_count = header.node_count;
                view.nodes = static_cast<const NodeBeamNodeRecord*>(
                    pointer(header.nodes_offset, header.node_count));
                view.beam_count = header.beam_count;
                view.beams = static_cast<const NodeBeamBeamRecord*>(
                    pointer(header.beams_offset, header.beam_count));
                view.surface_index_count = header.surface_index_count;
                view.surface_indices = static_cast<const std::uint32_t*>(
                    pointer(header.surface_indices_offset, header.surface_index_count));
                view.attachment_count = header.attachment_count;
                view.attachments = static_cast<const NodeBeamAttachmentRecord*>(
                    pointer(header.attachments_offset, header.attachment_count));
                view.skin_count = header.skin_count;
                view.skin = static_cast<const NodeBeamSkinRecord*>(
                    pointer(header.skin_offset, header.skin_count));
                view.valid = true;
                return view;
            }

            namespace
            {
                /**
                 * @brief The centroid and frame @p record's nodes imply.
                 *
                 * A template on the position lookup rather than a `std::function`, so the
                 * cook side and the runtime side share one definition without either paying
                 * an indirect call — and, more to the point, so neither can drift from the
                 * other, which would store a vertex in one frame and read it in another.
                 */
                template <typename PositionOf>
                void skin_basis(const NodeBeamSkinRecord& record, const PositionOf& position_of,
                                Vector3& centroid, Vector3 axes[3]) noexcept
                {
                    Scalar weights[NODE_BEAM_SKIN_INFLUENCES];
                    read_node_beam_skin_weights<Scalar>(record, weights);

                    centroid = Vector3{0, 0, 0};
                    for (std::uint32_t i = 0; i < NODE_BEAM_SKIN_INFLUENCES; ++i)
                        centroid = centroid + position_of(record.nodes[i]) * weights[i];

                    node_beam_skin_frame(position_of(record.nodes[0]),
                                         position_of(record.nodes[1]),
                                         position_of(record.nodes[2]), axes);
                }
            } // namespace

            Vector3 evaluate_node_beam_skin(const NodeBeamAssetView& view,
                                            const NodeBeamSkinRecord& record,
                                            const Vector3* positions) noexcept
            {
                if (!view.valid)
                    return Vector3{0, 0, 0};
                for (std::uint32_t i = 0; i < NODE_BEAM_SKIN_INFLUENCES; ++i)
                {
                    if (record.nodes[i] >= view.node_count)
                        return Vector3{0, 0, 0};
                }

                const NodeBeamNodeRecord* nodes = view.nodes;
                const auto position_of = [nodes, positions](std::uint32_t index) -> Vector3
                {
                    return positions != nullptr ? positions[index] : nodes[index].position;
                };

                Vector3 centroid{0, 0, 0};
                Vector3 axes[3];
                skin_basis(record, position_of, centroid, axes);
                return centroid + axes[0] * Scalar(record.offset[0]) +
                       axes[1] * Scalar(record.offset[1]) + axes[2] * Scalar(record.offset[2]);
            }

            void build_node_beam_skin_offset(const NodeBeamNodeRecord* nodes,
                                             const NodeBeamSkinRecord& record,
                                             const Vector3& point, float out[3]) noexcept
            {
                out[0] = 0.0f;
                out[1] = 0.0f;
                out[2] = 0.0f;
                if (nodes == nullptr)
                    return;

                const auto position_of = [nodes](std::uint32_t index) -> Vector3
                {
                    return nodes[index].position;
                };

                Vector3 centroid{0, 0, 0};
                Vector3 axes[3];
                skin_basis(record, position_of, centroid, axes);

                const Vector3 difference = point - centroid;
                for (int axis = 0; axis < 3; ++axis)
                    out[axis] = float(dot(axes[axis], difference));
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
