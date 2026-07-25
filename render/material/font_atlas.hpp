/**************************************************************************/
/* font_atlas.hpp                                                         */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
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
 * @file font_atlas.hpp
 * @brief The UI overlay's glyph atlas: one baked font, one texture, per-glyph metrics.
 *
 * The renderer needs to turn a string into quads, which needs a rasterized font. This
 * bakes the printable ASCII range once at bring-up into a single alpha texture and keeps
 * the per-glyph source rectangle and advance beside it, so laying out a run at draw time
 * is arithmetic and no glyph is ever rasterized again.
 *
 * Deliberately one font at one size. Text is scaled by sampling the baked bitmap, which
 * is soft when magnified far past the bake height — the honest trade for an overlay that
 * costs one texture and no runtime rasterizer. A UI that needs crisp text at every size
 * wants signed-distance-field glyphs, which is a different atlas, not a bigger one.
 */

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace SushiEngine
{
    namespace Render
    {
        namespace Assets
        {
            class TextureLibrary;

            /** @brief First and last code points baked; the printable ASCII range. */
            constexpr char FONT_FIRST_CODEPOINT = 32;
            constexpr char FONT_LAST_CODEPOINT = 126;

            /** @brief One baked glyph: where it sits in the atlas and how it is placed. */
            struct FontGlyph
            {
                float u0 = 0.0f; /**< Atlas texture coordinates, normalized. */
                float v0 = 0.0f;
                float u1 = 0.0f;
                float v1 = 0.0f;
                float offset_x = 0.0f; /**< Pen-relative top-left of the quad, in bake pixels. */
                float offset_y = 0.0f;
                float width = 0.0f; /**< Quad size in bake pixels. */
                float height = 0.0f;
                float advance = 0.0f; /**< Pen movement to the next glyph, in bake pixels. */
            };

            /**
             * @brief A baked font: its atlas texture and the metrics to lay text out with.
             *
             * Invalid until @ref build succeeds. An invalid atlas is not an error the caller
             * has to handle specially — the overlay simply draws its rectangles and skips its
             * text, which is a legible degradation rather than a blank screen.
             */
            class FontAtlas
            {
                public:
                    FontAtlas() = default;

                    /**
                     * @brief Bakes a TrueType font into the texture library.
                     *
                     * @param textures     Where the baked atlas is registered (and uploaded).
                     * @param ttf          The whole font file's bytes.
                     * @param byte_count   Length of @p ttf.
                     * @param pixel_height Cap height to bake at, in pixels.
                     * @return Whether the font parsed and every glyph fit the atlas.
                     */
                    bool build(TextureLibrary& textures, const std::uint8_t* ttf,
                               std::size_t byte_count, float pixel_height);

                    /**
                     * @brief Bakes whichever conventional system font is present.
                     *
                     * A convenience for a host that has no opinion about typeface: it tries a
                     * short list of fonts the platform is overwhelmingly likely to ship. It is
                     * explicitly a fallback, not a font-management policy — a shipped game
                     * passes its own bytes to @ref build.
                     *
                     * @param textures     Where the baked atlas is registered.
                     * @param pixel_height Cap height to bake at, in pixels.
                     * @return Whether a font was found and baked.
                     */
                    bool build_from_system_font(TextureLibrary& textures, float pixel_height);

                    /** @brief Whether a font baked successfully and text can be drawn. */
                    bool valid() const noexcept { return valid_; }

                    /** @brief The atlas texture's id in the library, for its heap index. */
                    std::uint32_t texture() const noexcept { return texture_; }

                    /** @brief The pixel height the glyphs were baked at; the scaling reference. */
                    float bake_height() const noexcept { return bake_height_; }

                    /** @brief Distance from the baseline to the top of the line, in bake pixels. */
                    float ascent() const noexcept { return ascent_; }

                    /**
                     * @brief Texture coordinate of a texel guaranteed to be opaque white.
                     *
                     * Untextured geometry (a panel, a button's fill) shares the one overlay draw
                     * with the glyphs, so it still samples the atlas — it just samples here,
                     * where the value multiplies its colour by one. Without this a solid
                     * rectangle would have to be a second pipeline or a second draw.
                     *
                     * When no font baked these are zero, which lands on the texture library's
                     * opaque-white default the overlay falls back to — so rectangles keep
                     * drawing correctly with no font at all.
                     */
                    float white_u() const noexcept { return white_u_; }
                    float white_v() const noexcept { return white_v_; }

                    /** @brief Baseline-to-baseline distance, in bake pixels. */
                    float line_height() const noexcept { return line_height_; }

                    /**
                     * @brief The metrics for one code point.
                     * @param codepoint The character to look up.
                     * @return Its glyph, or a zero-width glyph when it is outside the baked range.
                     */
                    const FontGlyph& glyph(char codepoint) const noexcept;

                    /**
                     * @brief The width a run would occupy, in pixels, at @p font_size.
                     * @param text      The characters to measure.
                     * @param length    How many of them.
                     * @param font_size The size the run will be drawn at, in pixels.
                     * @return The advance sum, scaled from the bake height to @p font_size.
                     */
                    float measure(const char* text, std::size_t length, float font_size) const noexcept;

                private:
                    static constexpr std::size_t GLYPH_COUNT =
                        static_cast<std::size_t>(FONT_LAST_CODEPOINT - FONT_FIRST_CODEPOINT + 1);

                    bool valid_ = false;
                    std::uint32_t texture_ = 0;
                    float bake_height_ = 0.0f;
                    float ascent_ = 0.0f;
                    float line_height_ = 0.0f;
                    float white_u_ = 0.0f;
                    float white_v_ = 0.0f;
                    FontGlyph glyphs_[GLYPH_COUNT]{};
                    FontGlyph missing_{};
            };

            /**
             * @brief Reads a whole file into memory.
             * @param path  The file to read.
             * @param bytes Receives the contents; cleared first, empty when the read fails.
             * @return Whether the file was opened and read.
             */
            bool read_file(const std::string& path, std::vector<std::uint8_t>& bytes);
        } // namespace Assets
    } // namespace Render
} // namespace SushiEngine
