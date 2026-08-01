/**************************************************************************/
/* test_planet_pack.cpp                                                   */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_PlanetPack: the baked terrain asset's format
// (docs/slop/solar_system_overhaul.md §5.2), and the height source over it.
//
// Almost everything here is driven by *synthesized* bytes rather than by the shipped
// asset, for the reason the climatology test gives for the same arrangement: a test that
// can only fail when a checked-out asset is present is a test that will be skipped on the
// machine where it mattered. The refusals especially -- a reader is judged by what it
// declines, and a malformed pack has to be produced deliberately to be declined.
//
// The last few cases do read the baked lunar asset, and skip when it is absent. What they
// check is what only real data can answer: that the elevations the engine reads back are
// the Moon's.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/terrain/height_function.hpp>
#include <SushiEngine/terrain/layer_stack.hpp>
#include <SushiEngine/terrain/pack_format.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Terrain;

namespace
{
    constexpr double MOON_RADIUS_METRES = 1737400.0;

    void append_u16(std::vector<std::uint8_t>& out, std::uint16_t value)
    {
        out.push_back(static_cast<std::uint8_t>(value & 0xFFu));
        out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFu));
    }

    void append_u32(std::vector<std::uint8_t>& out, std::uint32_t value)
    {
        for (int shift = 0; shift < 32; shift += 8)
            out.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFu));
    }

    void append_u64(std::vector<std::uint8_t>& out, std::uint64_t value)
    {
        for (int shift = 0; shift < 64; shift += 8)
            out.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFu));
    }

    void append_f32(std::vector<std::uint8_t>& out, float value)
    {
        std::uint32_t word = 0;
        std::memcpy(&word, &value, sizeof(word));
        append_u32(out, word);
    }

    void append_f64(std::vector<std::uint8_t>& out, double value)
    {
        std::uint64_t word = 0;
        std::memcpy(&word, &value, sizeof(word));
        append_u64(out, word);
    }

    void write_u32(std::vector<std::uint8_t>& blob, std::size_t offset, std::uint32_t value)
    {
        for (int shift = 0; shift < 32; shift += 8)
            blob[offset + static_cast<std::size_t>(shift / 8)] =
                static_cast<std::uint8_t>((value >> shift) & 0xFFu);
    }

    void write_u64(std::vector<std::uint8_t>& blob, std::size_t offset, std::uint64_t value)
    {
        for (int shift = 0; shift < 64; shift += 8)
            blob[offset + static_cast<std::size_t>(shift / 8)] =
                static_cast<std::uint8_t>((value >> shift) & 0xFFu);
    }

    /** One tile a builder will write: its address and the elevations it carries. */
    struct BuilderTile
    {
        TileAddress address;
        std::vector<float> heights;
    };

    /**
     * Builds packs the way `pack.py` does, so a refusal test can start from something
     * valid and break exactly one thing.
     */
    struct PackBuilder
    {
        std::string provenance = "test provenance";
        std::uint32_t body_id = 4;
        double semi_axis_x = MOON_RADIUS_METRES;
        double semi_axis_y = MOON_RADIUS_METRES;
        double semi_axis_z = MOON_RADIUS_METRES;
        std::uint32_t grid_size = TILE_GRID_SIZE;
        std::uint32_t apron = TILE_APRON;
        std::uint8_t data_depth = 3;
        std::vector<BuilderTile> tiles;

        /**
         * Whether to sort the index by address key, as `PackWriter::close` does. Off only
         * for the test that checks the reader refuses an unsorted index.
         */
        bool sort_index = true;

        std::size_t index_offset() const
        {
            return PLANET_PACK_HEADER_BYTES + provenance.size();
        }

        std::size_t payload_offset() const
        {
            return index_offset() + tiles.size() * PLANET_PACK_RECORD_BYTES;
        }

        std::vector<std::uint8_t> build() const
        {
            std::vector<BuilderTile> ordered = tiles;
            if (sort_index)
            {
                std::sort(ordered.begin(), ordered.end(),
                          [](const BuilderTile& a, const BuilderTile& b)
                          {
                              return tile_address_key(a.address) < tile_address_key(b.address);
                          });
            }

            std::vector<std::uint8_t> blob;
            blob.insert(blob.end(), PLANET_PACK_MAGIC,
                        PLANET_PACK_MAGIC + sizeof(PLANET_PACK_MAGIC));
            append_u32(blob, PLANET_PACK_VERSION);
            append_u32(blob, body_id);
            append_f64(blob, semi_axis_x);
            append_f64(blob, semi_axis_y);
            append_f64(blob, semi_axis_z);
            append_u32(blob, grid_size);
            append_u32(blob, apron);
            blob.push_back(data_depth);
            blob.insert(blob.end(), 3, 0);
            append_u32(blob, static_cast<std::uint32_t>(ordered.size()));
            append_u32(blob, static_cast<std::uint32_t>(provenance.size()));
            blob.insert(blob.end(), 4, 0);
            blob.insert(blob.end(), provenance.begin(), provenance.end());

            // The payloads are all one size, so every offset is known before any is written.
            std::vector<std::vector<std::uint8_t>> payloads;
            std::vector<float> lows;
            std::vector<float> highs;
            for (const BuilderTile& tile : ordered)
            {
                float low = tile.heights[0];
                float high = tile.heights[0];
                for (float height : tile.heights)
                {
                    low = height < low ? height : low;
                    high = height > high ? height : high;
                }
                low = std::floor(low);
                high = std::ceil(high);
                lows.push_back(low);
                highs.push_back(high);

                std::vector<std::uint8_t> payload;
                payload.reserve(TILE_SAMPLE_COUNT * 2u);
                const double span = static_cast<double>(high) - static_cast<double>(low);
                for (float height : tile.heights)
                {
                    std::uint16_t value = 0;
                    if (span > 0.0)
                    {
                        const double scaled =
                            (static_cast<double>(height) - static_cast<double>(low)) *
                            (65535.0 / span);
                        const double clamped = scaled < 0.0 ? 0.0 : (scaled > 65535.0 ? 65535.0
                                                                                      : scaled);
                        value = static_cast<std::uint16_t>(clamped + 0.5);
                    }
                    append_u16(payload, value);
                }
                payloads.push_back(std::move(payload));
            }

            for (std::size_t entry = 0; entry < ordered.size(); ++entry)
            {
                append_u64(blob, tile_address_key(ordered[entry].address));
                append_u64(blob, payload_offset() + entry * TILE_SAMPLE_COUNT * 2u);
                append_u32(blob, TILE_SAMPLE_COUNT * 2u);
                append_u16(blob, PLANET_PACK_CODEC_QUANTISED);
                append_u16(blob, 0);
                append_f32(blob, lows[entry]);
                append_f32(blob, highs[entry]);
                append_f32(blob, lows[entry]);
                append_f32(blob, highs[entry]);
            }
            for (const std::vector<std::uint8_t>& payload : payloads)
                blob.insert(blob.end(), payload.begin(), payload.end());
            return blob;
        }
    };

    /** A tile whose elevation is a plane in grid coordinates; bilinear resampling is exact. */
    BuilderTile planar_tile(const TileAddress& address, double along_s, double along_t)
    {
        BuilderTile tile;
        tile.address = address;
        tile.heights.resize(TILE_SAMPLE_COUNT, 0.0f);
        for (std::uint32_t row = 0; row < TILE_STRIDE; ++row)
        {
            for (std::uint32_t column = 0; column < TILE_STRIDE; ++column)
            {
                double grid_s = 0.0;
                double grid_t = 0.0;
                tile_sample_grid_coordinate(address, column, row, grid_s, grid_t);
                tile.heights[tile_sample_index(column, row)] =
                    static_cast<float>(along_s * grid_s + along_t * grid_t);
            }
        }
        return tile;
    }

    PackBuilder one_tile_builder()
    {
        PackBuilder builder;
        builder.tiles.push_back(planar_tile(TileAddress{CubeFace::PositiveX, 0, 0, 0},
                                            1000.0, 500.0));
        return builder;
    }

    /** Walks up from the working directory to find the baked lunar asset, if any. */
    std::string locate_shipped_asset()
    {
        const char* prefixes[] = {"", "../", "../../", "../../../", "../../../../"};
        for (const char* prefix : prefixes)
        {
            const std::string candidate =
                std::string(prefix) + "assets/planet/moon.compact.planet";
            std::ifstream probe(candidate, std::ios::binary);
            if (probe)
                return candidate;
        }
        return std::string();
    }
} // namespace

TEST(Unit_PlanetPack, AdoptsAWellFormedPack)
{
    PlanetPack pack;
    ASSERT_TRUE(pack.adopt(one_tile_builder().build()));
    EXPECT_TRUE(pack.loaded());
    EXPECT_EQ(pack.body_id(), 4u);
    EXPECT_EQ(static_cast<int>(pack.height_data_depth()), 3);
    EXPECT_EQ(pack.tile_count(), 1u);
    EXPECT_EQ(pack.provenance(), "test provenance");
    EXPECT_DOUBLE_EQ(pack.ellipsoid().semi_axis_z, MOON_RADIUS_METRES);
}

TEST(Unit_PlanetPack, AnUnloadedPackAnswersNothing)
{
    PlanetPack pack;
    EXPECT_FALSE(pack.loaded());
    EXPECT_EQ(pack.tile_count(), 0u);
    EXPECT_EQ(pack.find(TileAddress{CubeFace::PositiveX, 0, 0, 0}), nullptr);

    std::vector<float> heights(TILE_SAMPLE_COUNT, 0.0f);
    TileStatistics statistics;
    EXPECT_FALSE(pack.read_tile(TileAddress{CubeFace::PositiveX, 0, 0, 0}, heights.data(),
                                statistics));
}

TEST(Unit_PlanetPack, RefusesEveryMalformedHeader)
{
    // Each case breaks exactly one thing in an otherwise valid pack, so a refusal can only
    // be attributed to the thing that was broken.
    {
        std::vector<std::uint8_t> blob = one_tile_builder().build();
        blob[0] = 'X';
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "bad magic";
    }
    {
        std::vector<std::uint8_t> blob = one_tile_builder().build();
        write_u32(blob, 8, PLANET_PACK_VERSION + 1u);
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "future version";
    }
    {
        std::vector<std::uint8_t> blob = one_tile_builder().build();
        blob.resize(PLANET_PACK_HEADER_BYTES - 1u);
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "truncated header";
    }
    {
        // A pack baked for a different tile grid would otherwise be read as garbage of
        // exactly the right length, which is the failure this field exists to prevent.
        std::vector<std::uint8_t> blob = one_tile_builder().build();
        write_u32(blob, 40, TILE_GRID_SIZE - 1u);
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "wrong grid size";
    }
    {
        std::vector<std::uint8_t> blob = one_tile_builder().build();
        write_u32(blob, 44, TILE_APRON + 1u);
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "wrong apron";
    }
    {
        std::vector<std::uint8_t> blob = one_tile_builder().build();
        blob[48] = MAX_TILE_DEPTH + 1u;
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "unaddressable data depth";
    }
    {
        std::vector<std::uint8_t> blob = one_tile_builder().build();
        write_u32(blob, 56, PLANET_PACK_MAX_PROVENANCE_BYTES + 1u);
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "oversized provenance";
    }
    {
        std::vector<std::uint8_t> blob = one_tile_builder().build();
        write_u32(blob, 52, 100000u);
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "an index that does not fit";
    }
    {
        std::vector<std::uint8_t> blob = one_tile_builder().build();
        for (std::size_t byte = 16; byte < 24; ++byte)
            blob[byte] = 0;
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "degenerate ellipsoid";
    }
}

TEST(Unit_PlanetPack, RefusesEveryMalformedRecord)
{
    const PackBuilder builder = one_tile_builder();
    const std::size_t record = builder.index_offset();
    {
        std::vector<std::uint8_t> blob = builder.build();
        blob[record + 20] = 7; // codec
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "unknown codec";
    }
    {
        std::vector<std::uint8_t> blob = builder.build();
        write_u32(blob, record + 16, 10u); // length
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "wrong payload length";
    }
    {
        std::vector<std::uint8_t> blob = builder.build();
        write_u64(blob, record + 8, 0u); // offset, into the header
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "a payload aliasing the index";
    }
    {
        std::vector<std::uint8_t> blob = builder.build();
        write_u64(blob, record + 8, blob.size());
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "a payload past the end";
    }
    {
        std::vector<std::uint8_t> blob = builder.build();
        // An inverted quantisation range would decode every sample as a descending ramp.
        std::uint32_t low = 0;
        std::uint32_t high = 0;
        std::memcpy(&low, blob.data() + record + 24, 4);
        std::memcpy(&high, blob.data() + record + 28, 4);
        std::memcpy(blob.data() + record + 24, &high, 4);
        std::memcpy(blob.data() + record + 28, &low, 4);
        PlanetPack pack;
        EXPECT_FALSE(pack.adopt(blob)) << "inverted quantisation range";
    }
}

TEST(Unit_PlanetPack, RefusesAnIndexThatDoesNotAscend)
{
    // The lookup is a binary search, so a descending index is not merely untidy: it is a
    // lookup that silently misses tiles that are present.
    PackBuilder builder;
    builder.sort_index = false;
    builder.tiles.push_back(planar_tile(TileAddress{CubeFace::PositiveX, 1, 1, 1}, 10.0, 0.0));
    builder.tiles.push_back(planar_tile(TileAddress{CubeFace::PositiveX, 1, 0, 0}, 20.0, 0.0));
    PlanetPack pack;
    EXPECT_FALSE(pack.adopt(builder.build()));

    // The same two tiles in order are accepted.
    PackBuilder ordered;
    ordered.tiles.push_back(planar_tile(TileAddress{CubeFace::PositiveX, 1, 0, 0}, 20.0, 0.0));
    ordered.tiles.push_back(planar_tile(TileAddress{CubeFace::PositiveX, 1, 1, 1}, 10.0, 0.0));
    PlanetPack accepted;
    EXPECT_TRUE(accepted.adopt(ordered.build()));
}

TEST(Unit_PlanetPack, FindsStoredTilesAndOnlyThose)
{
    PackBuilder builder;
    for (std::uint32_t index = 0; index < 4u; ++index)
    {
        builder.tiles.push_back(planar_tile(
            TileAddress{CubeFace::PositiveX, 1, index & 1u, (index >> 1) & 1u}, 100.0, 0.0));
    }
    PlanetPack pack;
    ASSERT_TRUE(pack.adopt(builder.build()));
    EXPECT_EQ(pack.tile_count(), 4u);

    for (std::uint32_t index = 0; index < 4u; ++index)
    {
        const TileAddress address{CubeFace::PositiveX, 1, index & 1u, (index >> 1) & 1u};
        EXPECT_NE(pack.find(address), nullptr);
    }
    EXPECT_EQ(pack.find(TileAddress{CubeFace::NegativeZ, 1, 0, 0}), nullptr);
    EXPECT_EQ(pack.find(TileAddress{CubeFace::PositiveX, 2, 0, 0}), nullptr);
}

TEST(Unit_PlanetPack, DecodesWithinTheQuantisationStep)
{
    const BuilderTile original = planar_tile(TileAddress{CubeFace::PositiveX, 0, 0, 0},
                                             1000.0, 500.0);
    PackBuilder builder;
    builder.tiles.push_back(original);
    PlanetPack pack;
    ASSERT_TRUE(pack.adopt(builder.build()));

    const PlanetPackRecord* record = pack.find(original.address);
    ASSERT_NE(record, nullptr);
    const double step = static_cast<double>(record->quantised_maximum_metres -
                                            record->quantised_minimum_metres) /
                        65535.0;

    std::vector<float> decoded(TILE_SAMPLE_COUNT, 0.0f);
    TileStatistics statistics;
    ASSERT_TRUE(pack.read_tile(original.address, decoded.data(), statistics));

    double worst = 0.0;
    for (std::uint32_t index = 0; index < TILE_SAMPLE_COUNT; ++index)
    {
        const double error = std::fabs(static_cast<double>(decoded[index]) -
                                       static_cast<double>(original.heights[index]));
        worst = error > worst ? error : worst;
    }
    // Half a step is what round-to-nearest gives; anything more is a layout error.
    EXPECT_LE(worst, step * 0.51) << "worst " << worst << " m against a step of " << step;
    EXPECT_GT(step, 0.0);
}

TEST(Unit_PlanetPack, SourceReportsCoverageAndDataDepth)
{
    PlanetPack pack;
    ASSERT_TRUE(pack.adopt(one_tile_builder().build()));
    const PackHeightSource source(pack);

    const TileAddress root{CubeFace::PositiveX, 0, 0, 0};
    EXPECT_EQ(static_cast<int>(source.data_depth(root)), 3);

    std::vector<float> heights(TILE_SAMPLE_COUNT, -1.0f);
    TileStatistics statistics;
    EXPECT_TRUE(source.sample_tile(root, heights.data(), statistics));

    // A face the pack does not carry at all has no ancestor to fall back to.
    EXPECT_FALSE(source.sample_tile(TileAddress{CubeFace::NegativeZ, 2, 0, 0}, heights.data(),
                                    statistics));
}

TEST(Unit_PlanetPack, SourceResamplesAnAncestorPastTheStoredDepth)
{
    // The pack holds only the root of one face; the quadtree still has to be able to
    // descend, and what it gets back must be the ancestor's surface rather than noise.
    // The stored tile is a plane in grid coordinates, and bilinear resampling of a plane is
    // exact, so the expected value at every child sample is known in closed form.
    PlanetPack pack;
    ASSERT_TRUE(pack.adopt(one_tile_builder().build()));
    const PackHeightSource source(pack);

    const TileAddress child{CubeFace::PositiveX, 2, 3, 1};
    std::vector<float> heights(TILE_SAMPLE_COUNT, 0.0f);
    TileStatistics statistics;
    ASSERT_TRUE(source.sample_tile(child, heights.data(), statistics));

    double worst = 0.0;
    for (std::uint32_t row = 0; row < TILE_STRIDE; row += 16u)
    {
        for (std::uint32_t column = 0; column < TILE_STRIDE; column += 16u)
        {
            double grid_s = 0.0;
            double grid_t = 0.0;
            tile_sample_grid_coordinate(child, column, row, grid_s, grid_t);
            const double expected = 1000.0 * grid_s + 500.0 * grid_t;
            const double error = std::fabs(
                static_cast<double>(heights[tile_sample_index(column, row)]) - expected);
            worst = error > worst ? error : worst;
        }
    }
    EXPECT_LT(worst, 0.1) << "resampling a plane must reproduce it; worst " << worst << " m";
    EXPECT_LT(statistics.minimum_metres, statistics.maximum_metres);
}

TEST(Unit_PlanetPack, SourceComposesWithTheHeightFunction)
{
    PlanetPack pack;
    ASSERT_TRUE(pack.adopt(one_tile_builder().build()));
    const PackHeightSource source(pack);

    LayerStack layers;
    TerrainLayer crater;
    crater.order = 1;
    crater.operation = LayerOperation::Crater;
    crater.footprint.direction = Vector3{1.0, 0.0, 0.0};
    crater.footprint.inner_radians = 0.05;
    crater.footprint.outer_radians = 0.12;
    crater.profile.depth_metres = 2000.0;
    crater.profile.rim_metres = 300.0;
    ASSERT_TRUE(layers.insert(crater));

    const HeightFunction height(source, layers,
                                ellipsoid_of_revolution(MOON_RADIUS_METRES, 0.0));

    const TileAddress root{CubeFace::PositiveX, 0, 0, 0};
    std::vector<float> without(TILE_SAMPLE_COUNT, 0.0f);
    std::vector<float> with(TILE_SAMPLE_COUNT, 0.0f);
    TileStatistics plain;
    TileStatistics edited;
    ASSERT_TRUE(source.sample_tile(root, without.data(), plain));
    ASSERT_TRUE(height.evaluate_tile(root, with.data(), edited));

    // The face centre is the crater's own centre, so it drops by the full depth.
    const std::uint32_t centre = TILE_STRIDE / 2u;
    const std::uint32_t index = tile_sample_index(centre, centre);
    EXPECT_NEAR(static_cast<double>(with[index]),
                static_cast<double>(without[index]) - 2000.0, 1.0);
    EXPECT_LT(edited.minimum_metres, plain.minimum_metres);
}

// The data: what the baked asset claims about the actual Moon.

TEST(Unit_PlanetPack, ShippedLunarAssetLoadsAndCarriesItsProvenance)
{
    const std::string path = locate_shipped_asset();
    if (path.empty())
        GTEST_SKIP() << "no baked lunar terrain checked out; run `se planet bake`";

    const PlanetPack pack = load_planet_pack(path);
    ASSERT_TRUE(pack.loaded());
    EXPECT_EQ(pack.body_id(), 4u) << "Astro::BodyId::Moon";
    EXPECT_NEAR(pack.ellipsoid().semi_axis_x, MOON_RADIUS_METRES, 1.0);
    EXPECT_NEAR(pack.ellipsoid().semi_axis_z, MOON_RADIUS_METRES, 1.0)
        << "the Moon is modelled as a sphere";
    EXPECT_GT(pack.tile_count(), 0u);
    EXPECT_FALSE(pack.provenance().empty()) << "attribution must travel in the asset";
    EXPECT_NE(pack.provenance().find("LOLA"), std::string::npos);
}

TEST(Unit_PlanetPack, ShippedLunarAssetHasTheMoonsRelief)
{
    const std::string path = locate_shipped_asset();
    if (path.empty())
        GTEST_SKIP() << "no baked lunar terrain checked out; run `se planet bake`";

    const PlanetPack pack = load_planet_pack(path);
    ASSERT_TRUE(pack.loaded());

    // LOLA's global extremes are about -9.1 km and +10.8 km. A pack sampled onto a coarse
    // quadtree will not reach either exactly, but it must be recognisably that body: a
    // mirrored or misscaled bake lands nowhere near this.
    const PackHeightSource source(pack);
    for (std::uint32_t face = 0; face < CUBE_FACE_COUNT; ++face)
    {
        const TileAddress root{static_cast<CubeFace>(face), 0, 0, 0};
        EXPECT_NE(pack.find(root), nullptr) << "every face must carry its root tile";
    }

    float lowest = 0.0f;
    float highest = 0.0f;
    for (const PlanetPackRecord& record : pack.records())
    {
        lowest = record.grid_minimum_metres < lowest ? record.grid_minimum_metres : lowest;
        highest = record.grid_maximum_metres > highest ? record.grid_maximum_metres : highest;
    }
    EXPECT_LT(lowest, -6000.0f) << "the South Pole-Aitken floor is nowhere near this shallow";
    EXPECT_GT(highest, 7000.0f) << "the far-side highlands are nowhere near this low";
    EXPECT_GT(lowest, -12000.0f);
    EXPECT_LT(highest, 14000.0f);

    // And the deepest stored level is reachable through the source.
    const TileAddress deep{CubeFace::PositiveZ, pack.height_data_depth(), 0, 0};
    std::vector<float> heights(TILE_SAMPLE_COUNT, 0.0f);
    TileStatistics statistics;
    EXPECT_TRUE(source.sample_tile(deep, heights.data(), statistics));
}
