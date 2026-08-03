/**************************************************************************/
/* metar_parser.hpp                                                      */
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
 * @file metar_parser.hpp
 * @brief A real, working METAR text parser, and its translation into a `WeatherColumn`.
 *
 * `docs/slop/weather_and_clouds.md` §5.4/§7 W6: `IngestedWeather` blends GRIB toward METARs
 * near airfields. Full GRIB2 binary decoding is a large, specialized, external-data-format task
 * genuinely out of scope for this phase (see `ingested_weather.hpp`'s file docs for where that
 * seam is left as a named stub); METAR, by contrast, is a compact, well-documented, purely
 * textual space-delimited format, and this file is a real parser for it, not a stub -- the W6
 * task brief calls this out explicitly as "genuinely achievable... and has real value".
 *
 * The parser is a tolerant token scanner, not a strict grammar: it walks whitespace-delimited
 * groups, recognizes the ones this bridge can use (wind, cloud layers, temperature/dewpoint,
 * present weather), and silently skips every group it does not recognize (visibility, altimeter,
 * runway visual range, remarks, station id, observation time). A METAR missing a recognized
 * group simply leaves that part of the report at its default rather than failing the whole
 * parse -- real METARs vary in which groups they include (CAVOK reports skip cloud groups
 * entirely, for instance), so a strict all-or-nothing parse would reject valid reports.
 *
 * Only what `WeatherColumn` can actually use is extracted: this is deliberately not a general
 * METAR decoder (no visibility, runway state, or remarks decoding) -- the same "narrow to the
 * bridge's own contract" discipline `weather_provider.hpp`'s `StaticWeather::decompose` and
 * `regional_weather_grid.hpp`'s `sample_column` already follow.
 */

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include <SushiEngine/simulation/weather_flight_hazards.hpp>
#include <SushiEngine/simulation/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief One decoded cloud layer group (e.g. `BKN008`, `OVC002CB`), feet-native like METAR itself.
         */
        struct MetarCloudLayer
        {
            char cover = 'C';        /**< 'F'ew, 'S'catterered, 'B'roken, 'O'vercast, 'V'ertical-visibility. */
            double base_meters = 0.0; /**< Layer base above the surface, metres (METAR reports hundreds of feet). */
        };

        /** @brief Cloud layers a single report tracks; METAR practice rarely exceeds four. */
        constexpr int METAR_MAX_CLOUD_LAYERS = 4;

        /**
         * @brief The subset of a decoded METAR report this bridge can use.
         *
         * Plain data; @ref parse_metar fills it, @ref metar_to_weather_column consumes it. Not a
         * full aviation-weather decode (see the file docs) -- exactly the fields
         * `metar_to_weather_column` needs, and nothing else.
         */
        struct MetarReport
        {
            /** @brief Whether the string parsed as a recognizable METAR at all (station-less input, empty string, etc. leave this false). */
            bool valid = false;

            bool wind_calm = false;              /**< `00000KT`-style: no usable direction. */
            bool wind_variable_direction = false; /**< `VRB`-style: direction not meaningful. */
            double wind_direction_degrees = 0.0;  /**< True bearing the wind blows FROM, [0, 360). */
            double wind_speed_mps = 0.0;          /**< Sustained speed, metres/second. */
            double wind_gust_mps = 0.0;           /**< Gust speed if reported, else 0. */

            MetarCloudLayer layers[METAR_MAX_CLOUD_LAYERS]{}; /**< Decoded cloud groups, base-ascending as reported. */
            int layer_count = 0;                              /**< Populated entries in @ref layers. */
            bool sky_clear = false;                           /**< An explicit `CLR`/`SKC`/`NSC`/`NCD` group. */

            bool has_temperature = false; /**< Whether the `TT/DD` group parsed. */
            double temperature_c = 0.0;   /**< Surface air temperature, Celsius. */
            double dewpoint_c = 0.0;      /**< Surface dewpoint, Celsius. */

            /** @brief Derived from present-weather groups (`-RA`, `+TSRA`, `VCSH`, ...), [0, 1]. */
            float precipitation_intensity = 0.0f;
            bool thunderstorm = false; /**< Whether any present-weather group named `TS`. */
        };

        namespace MetarInternal
        {
            inline std::vector<std::string> tokenize(const std::string& raw)
            {
                std::istringstream stream(raw);
                std::vector<std::string> tokens;
                std::string token;
                while (stream >> token)
                    tokens.push_back(token);
                return tokens;
            }

            inline bool all_digits(const std::string& s, std::size_t begin, std::size_t end)
            {
                if (begin >= end || end > s.size())
                    return false;
                for (std::size_t i = begin; i < end; ++i)
                    if (!std::isdigit(static_cast<unsigned char>(s[i])))
                        return false;
                return true;
            }

            inline bool ends_with(const std::string& s, const char* suffix)
            {
                const std::size_t n = std::strlen(suffix);
                return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
            }

            inline bool try_parse_wind(const std::string& token, MetarReport& report)
            {
                std::string body;
                double unit_to_mps = 0.0;
                if (ends_with(token, "KT"))
                {
                    body = token.substr(0, token.size() - 2);
                    unit_to_mps = 0.514444;
                }
                else if (ends_with(token, "MPS"))
                {
                    body = token.substr(0, token.size() - 3);
                    unit_to_mps = 1.0;
                }
                else if (ends_with(token, "KMH"))
                {
                    body = token.substr(0, token.size() - 3);
                    unit_to_mps = 1.0 / 3.6;
                }
                else
                {
                    return false;
                }
                if (body.size() < 5)
                    return false;

                bool variable = body.compare(0, 3, "VRB") == 0;
                if (!variable && !all_digits(body, 0, 3))
                    return false;

                const double direction = variable ? 0.0 : std::atof(body.substr(0, 3).c_str());
                std::string remainder = body.substr(3);

                const std::size_t gust_pos = remainder.find('G');
                std::string speed_str = gust_pos == std::string::npos ? remainder : remainder.substr(0, gust_pos);
                std::string gust_str = gust_pos == std::string::npos ? std::string() : remainder.substr(gust_pos + 1);
                if (speed_str.empty() || !all_digits(speed_str, 0, speed_str.size()))
                    return false;
                if (!gust_str.empty() && !all_digits(gust_str, 0, gust_str.size()))
                    return false;

                const double speed = std::atof(speed_str.c_str()) * unit_to_mps;
                const double gust = gust_str.empty() ? 0.0 : std::atof(gust_str.c_str()) * unit_to_mps;

                report.wind_variable_direction = variable;
                report.wind_direction_degrees = direction;
                report.wind_speed_mps = speed;
                report.wind_gust_mps = gust;
                report.wind_calm = !variable && direction == 0.0 && speed == 0.0;
                return true;
            }

            inline bool try_parse_cloud(const std::string& token, MetarReport& report)
            {
                if (token == "CLR" || token == "SKC" || token == "NSC" || token == "NCD")
                {
                    report.sky_clear = true;
                    return true;
                }

                struct Prefix { const char* code; std::size_t length; char cover; };
                static const Prefix PREFIXES[] = {
                    {"FEW", 3, 'F'}, {"SCT", 3, 'S'}, {"BKN", 3, 'B'}, {"OVC", 3, 'O'}, {"VV", 2, 'V'},
                };
                for (const Prefix& prefix : PREFIXES)
                {
                    if (token.compare(0, prefix.length, prefix.code) != 0)
                        continue;
                    const std::size_t digits_begin = prefix.length;
                    if (!all_digits(token, digits_begin, digits_begin + 3))
                        return false;
                    if (report.layer_count >= METAR_MAX_CLOUD_LAYERS)
                        return true; // recognized, just no room left to record it.
                    const double hundreds_of_feet =
                        std::atof(token.substr(digits_begin, 3).c_str());
                    MetarCloudLayer& layer = report.layers[report.layer_count++];
                    layer.cover = prefix.cover;
                    layer.base_meters = hundreds_of_feet * 100.0 * 0.3048;
                    return true;
                }
                return false;
            }

            inline bool try_parse_temperature(const std::string& token, MetarReport& report)
            {
                const std::size_t slash = token.find('/');
                if (slash == std::string::npos || slash == 0 || slash + 1 >= token.size())
                    return false;
                const std::string temp_part = token.substr(0, slash);
                const std::string dew_part = token.substr(slash + 1);

                const auto parse_side = [](const std::string& side, double& out) -> bool
                {
                    if (side.empty())
                        return false;
                    const bool negative = side[0] == 'M';
                    const std::string digits = negative ? side.substr(1) : side;
                    if (digits.empty() || digits.size() > 2 || !all_digits(digits, 0, digits.size()))
                        return false;
                    out = std::atof(digits.c_str()) * (negative ? -1.0 : 1.0);
                    return true;
                };

                double temperature = 0.0;
                double dewpoint = 0.0;
                if (!parse_side(temp_part, temperature) || !parse_side(dew_part, dewpoint))
                    return false;

                report.has_temperature = true;
                report.temperature_c = temperature;
                report.dewpoint_c = dewpoint;
                return true;
            }

            inline bool known_weather_code(const std::string& code) noexcept
            {
                static const char* const CODES[] = {
                    "MI", "PR", "BC", "DR", "BL", "SH", "TS", "FZ",             // descriptors
                    "DZ", "RA", "SN", "SG", "IC", "PL", "GR", "GS", "UP",       // precipitation
                    "BR", "FG", "FU", "VA", "DU", "SA", "HZ", "PY",             // obscuration
                    "PO", "SQ", "FC", "SS", "DS",                              // other
                };
                for (const char* known : CODES)
                    if (code == known)
                        return true;
                return false;
            }

            inline bool is_precipitation_code(const std::string& code) noexcept
            {
                static const char* const CODES[] = {"DZ", "RA", "SN", "SG", "IC", "PL", "GR", "GS", "UP"};
                for (const char* known : CODES)
                    if (code == known)
                        return true;
                return false;
            }

            inline bool try_parse_present_weather(const std::string& token, MetarReport& report)
            {
                std::string body = token;
                float severity = 0.55f; // moderate, METAR's unmarked default intensity.
                float vicinity_scale = 1.0f;

                if (!body.empty() && (body[0] == '-' || body[0] == '+'))
                {
                    severity = body[0] == '-' ? 0.25f : 0.9f;
                    body = body.substr(1);
                }
                if (body.compare(0, 2, "VC") == 0)
                {
                    vicinity_scale = 0.4f; // reported near the station, not overhead.
                    body = body.substr(2);
                }
                if (body.empty() || body.size() % 2 != 0)
                    return false;

                bool has_precipitation = false;
                bool has_thunderstorm = false;
                bool has_shower = false;
                for (std::size_t i = 0; i < body.size(); i += 2)
                {
                    const std::string code = body.substr(i, 2);
                    if (!known_weather_code(code))
                        return false;
                    has_precipitation = has_precipitation || is_precipitation_code(code);
                    has_thunderstorm = has_thunderstorm || code == "TS";
                    has_shower = has_shower || code == "SH";
                }

                if (has_thunderstorm)
                    report.thunderstorm = true;
                // TS and bare SH ("shower[s]") both imply falling precipitation by definition
                // even when no explicit precip-type code accompanies them (a real, if terse,
                // METAR convention -- "VCSH" alone is a common report).
                if (has_precipitation || has_thunderstorm || has_shower)
                {
                    const float contribution = severity * vicinity_scale;
                    report.precipitation_intensity = std::max(report.precipitation_intensity, contribution);
                }
                return true;
            }
        } // namespace MetarInternal

        /**
         * @brief Parses a raw METAR report string into the fields `metar_to_weather_column` needs.
         *
         * @param raw The report text (e.g. `"METAR KJFK 261851Z 27015G25KT 10SM FEW250 24/12 A3005"`),
         *            with or without the leading `METAR`/`SPECI` type indicator.
         * @return The decoded fields; `MetarReport::valid` is true whenever at least one
         *         recognized group was found (an empty or garbage string leaves it false).
         */
        inline MetarReport parse_metar(const std::string& raw)
        {
            using namespace MetarInternal;
            MetarReport report;
            for (const std::string& token : tokenize(raw))
            {
                if (token == "METAR" || token == "SPECI" || token == "AUTO" || token == "COR")
                    continue;
                if (token == "RMK")
                    break; // remarks section: free text, not this bridge's concern.

                if (try_parse_wind(token, report) || try_parse_cloud(token, report) ||
                    try_parse_temperature(token, report) || try_parse_present_weather(token, report))
                {
                    report.valid = true;
                }
                // Every other group (station id, observation time, visibility, RVR, altimeter,
                // wind variable-direction) is intentionally left unrecognized and skipped -- see
                // the file docs.
            }
            return report;
        }

        /**
         * @brief Translates a decoded METAR into the bridge's own `WeatherColumn` shape.
         *
         * The design doc's §5.4 seam point: "the column representation is already identical;
         * nothing downstream changes". An invalid report (nothing recognized) returns a default,
         * all-clear column rather than fabricating weather from a report that carried none.
         *
         * @param report A report from @ref parse_metar.
         * @return A `WeatherColumn` this station's observation implies.
         */
        inline WeatherColumn metar_to_weather_column(const MetarReport& report)
        {
            WeatherColumn column{};
            if (!report.valid)
                return column;

            for (int i = 0; i < report.layer_count; ++i)
            {
                const MetarCloudLayer& layer = report.layers[i];
                float coverage = 0.0f;
                switch (layer.cover)
                {
                case 'F': coverage = 0.1875f; break; // few: 1-2 oktas.
                case 'S': coverage = 0.4375f; break; // scattered: 3-4 oktas.
                case 'B': coverage = 0.75f; break;   // broken: 5-7 oktas.
                case 'O': coverage = 1.0f; break;    // overcast: 8 oktas.
                case 'V': coverage = 1.0f; break;    // vertical visibility: obscured, treated as full.
                default: break;
                }
                WeatherLevelState& state = column.levels[static_cast<int>(cloud_level_for_altitude(layer.base_meters))];
                state.coverage = std::max(state.coverage, coverage);
                state.density_scale = std::max(state.density_scale, std::clamp(coverage * 1.4f, 0.0f, 2.0f));
                state.convective_fraction =
                    std::max(state.convective_fraction, report.thunderstorm ? 0.8f : 0.3f);
            }

            if (report.has_temperature)
            {
                // A single surface reading can only honestly inform the Low band; METAR carries
                // no upper-air sounding, so Mid/High keep the neutral ISA default rather than
                // having a surface reading propagated to altitudes it says nothing about.
                column.levels[static_cast<int>(CloudLevel::Low)].temperature_offset_c =
                    float(report.temperature_c - ISA_SEA_LEVEL_TEMPERATURE_C);
            }

            column.precipitation = std::clamp(report.precipitation_intensity, 0.0f, 1.0f);

            if (!report.wind_calm && !report.wind_variable_direction)
            {
                constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;
                // METAR direction is where the wind blows FROM; the vector it blows TOWARD is
                // rotated 180 degrees from that.
                const double toward_radians =
                    (report.wind_direction_degrees + 180.0) * DEGREES_TO_RADIANS;
                column.wind_u_mps = float(report.wind_speed_mps * std::sin(toward_radians));
                column.wind_v_mps = float(report.wind_speed_mps * std::cos(toward_radians));
            }
            return column;
        }
    } // namespace Simulation
} // namespace SushiEngine
