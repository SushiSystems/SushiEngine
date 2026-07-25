/**************************************************************************/
/* font_atlas.cpp                                                         */
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

#include "material/font_atlas.hpp"

#include <fstream>
#include <ios>

#include <stb_truetype.h>

#include "material/texture_library.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Assets
        {
            namespace
            {
                /**
                 * @brief Atlas edge length in texels.
                 *
                 * 512 holds the 95 printable ASCII glyphs comfortably at the sizes a UI bakes
                 * at, and stays small enough that the upload is unremarkable. The bake reports
                 * failure rather than silently dropping glyphs if a very large pixel height
                 * ever overflows it.
                 */
                constexpr int ATLAS_EXTENT = 512;

                /** @brief System fonts tried, in order, by @ref build_from_system_font. */
                const char* const SYSTEM_FONT_PATHS[] = {
#if defined(_WIN32)
                    "C:/Windows/Fonts/segoeui.ttf",
                    "C:/Windows/Fonts/arial.ttf",
                    "C:/Windows/Fonts/tahoma.ttf",
#else
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
                    "/usr/share/fonts/TTF/DejaVuSans.ttf",
                    "/System/Library/Fonts/Helvetica.ttc",
#endif
                };
            } // namespace

            bool read_file(const std::string& path, std::vector<std::uint8_t>& bytes)
            {
                bytes.clear();
                std::ifstream file(path, std::ios::binary | std::ios::ate);
                if (!file)
                    return false;

                const std::streampos size = file.tellg();
                if (size <= 0)
                    return false;
                file.seekg(0, std::ios::beg);

                bytes.resize(static_cast<std::size_t>(size));
                if (!file.read(reinterpret_cast<char*>(bytes.data()),
                               static_cast<std::streamsize>(bytes.size())))
                {
                    bytes.clear();
                    return false;
                }
                return true;
            }

            bool FontAtlas::build(TextureLibrary& textures, const std::uint8_t* ttf,
                                  std::size_t byte_count, float pixel_height)
            {
                valid_ = false;
                white_u_ = 0.0f;
                white_v_ = 0.0f;
                if (ttf == nullptr || byte_count == 0 || pixel_height <= 0.0f)
                    return false;

                stbtt_fontinfo info{};
                if (stbtt_InitFont(&info, ttf, stbtt_GetFontOffsetForIndex(ttf, 0)) == 0)
                    return false;

                std::vector<std::uint8_t> coverage(
                    static_cast<std::size_t>(ATLAS_EXTENT) * ATLAS_EXTENT, 0);
                std::vector<stbtt_bakedchar> baked(GLYPH_COUNT);

                // A negative return means the glyphs ran out of atlas; a positive one is the
                // number of rows actually used. Either way every glyph up to the return fit,
                // but a partial bake would draw some characters as blanks, so it is a failure.
                const int result = stbtt_BakeFontBitmap(
                    ttf, 0, pixel_height, coverage.data(), ATLAS_EXTENT, ATLAS_EXTENT,
                    FONT_FIRST_CODEPOINT, static_cast<int>(GLYPH_COUNT), baked.data());
                if (result <= 0)
                    return false;

                // Claim texel (0, 0) as the opaque-white sample untextured overlay geometry
                // reads. The baker lays glyphs out from (1, 1) with a one-texel border, so the
                // first row and column are always empty — but this writes it rather than
                // assuming it, since everything solid in the UI depends on it.
                coverage[0] = 255;
                const float half_texel = 0.5f / static_cast<float>(ATLAS_EXTENT);
                white_u_ = half_texel;
                white_v_ = half_texel;

                // The library uploads RGBA8, and the shader wants coverage in alpha with white
                // rgb so a tint multiplies cleanly. Expanding here costs one atlas-sized buffer
                // once at bring-up and saves the fragment shader a swizzle every pixel.
                std::vector<std::uint8_t> rgba(coverage.size() * 4);
                for (std::size_t i = 0; i < coverage.size(); ++i)
                {
                    rgba[i * 4 + 0] = 255;
                    rgba[i * 4 + 1] = 255;
                    rgba[i * 4 + 2] = 255;
                    rgba[i * 4 + 3] = coverage[i];
                }

                // Linear, not sRGB: the values are coverage, not colour, and gamma-decoding
                // them would thin every glyph.
                texture_ = static_cast<std::uint32_t>(
                    textures.add("sushi::ui_font_atlas", rgba.data(), ATLAS_EXTENT, ATLAS_EXTENT,
                                 TextureColorSpace::Linear));

                const float inverse_extent = 1.0f / static_cast<float>(ATLAS_EXTENT);
                for (std::size_t i = 0; i < GLYPH_COUNT; ++i)
                {
                    const stbtt_bakedchar& source = baked[i];
                    FontGlyph& glyph = glyphs_[i];
                    glyph.u0 = static_cast<float>(source.x0) * inverse_extent;
                    glyph.v0 = static_cast<float>(source.y0) * inverse_extent;
                    glyph.u1 = static_cast<float>(source.x1) * inverse_extent;
                    glyph.v1 = static_cast<float>(source.y1) * inverse_extent;
                    glyph.offset_x = source.xoff;
                    glyph.offset_y = source.yoff;
                    glyph.width = static_cast<float>(source.x1 - source.x0);
                    glyph.height = static_cast<float>(source.y1 - source.y0);
                    glyph.advance = source.xadvance;
                }

                int ascent = 0;
                int descent = 0;
                int line_gap = 0;
                stbtt_GetFontVMetrics(&info, &ascent, &descent, &line_gap);
                const float scale = stbtt_ScaleForPixelHeight(&info, pixel_height);
                ascent_ = static_cast<float>(ascent) * scale;
                line_height_ = static_cast<float>(ascent - descent + line_gap) * scale;

                bake_height_ = pixel_height;
                valid_ = true;
                return true;
            }

            bool FontAtlas::build_from_system_font(TextureLibrary& textures, float pixel_height)
            {
                std::vector<std::uint8_t> bytes;
                for (const char* path : SYSTEM_FONT_PATHS)
                {
                    if (!read_file(path, bytes))
                        continue;
                    if (build(textures, bytes.data(), bytes.size(), pixel_height))
                        return true;
                }
                return false;
            }

            const FontGlyph& FontAtlas::glyph(char codepoint) const noexcept
            {
                if (codepoint < FONT_FIRST_CODEPOINT || codepoint > FONT_LAST_CODEPOINT)
                    return missing_;
                return glyphs_[static_cast<std::size_t>(codepoint - FONT_FIRST_CODEPOINT)];
            }

            float FontAtlas::measure(const char* text, std::size_t length,
                                     float font_size) const noexcept
            {
                if (!valid_ || text == nullptr || bake_height_ <= 0.0f)
                    return 0.0f;
                float advance = 0.0f;
                for (std::size_t i = 0; i < length; ++i)
                    advance += glyph(text[i]).advance;
                return advance * (font_size / bake_height_);
            }
        } // namespace Assets
    } // namespace Render
} // namespace SushiEngine
