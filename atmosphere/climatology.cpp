/**************************************************************************/
/* climatology.cpp                                                        */
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

#include <SushiEngine/atmosphere/climatology.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace SushiEngine
{
    namespace Atmosphere
    {
        namespace
        {
            constexpr double PI = 3.14159265358979323846;

            /** @brief Identifies a baked climatology; an engine asset is otherwise unlabelled bytes. */
            constexpr char ASSET_MAGIC[4] = {'S', 'E', 'T', '0'};

            /** @brief Layout version. A reader that does not know it declines rather than guesses. */
            constexpr std::uint32_t ASSET_VERSION = 1;

            /** @brief Sea surface temperature the analytic bands imply at the equator, K. */
            constexpr double ANALYTIC_EQUATORIAL_SURFACE_KELVIN = 300.0;

            /** @brief Reads @p value from @p blob at @p cursor, advancing it. False if short. */
            template <typename T>
            bool take(const std::vector<std::uint8_t>& blob, std::size_t& cursor, T& value)
            {
                if (cursor + sizeof(T) > blob.size())
                    return false;
                std::memcpy(&value, blob.data() + cursor, sizeof(T));
                cursor += sizeof(T);
                return true;
            }

            /** @brief Reads @p count floats into @p out. False if the blob is short. */
            bool take_floats(const std::vector<std::uint8_t>& blob, std::size_t& cursor,
                             std::size_t count, std::vector<float>& out)
            {
                if (cursor + count * sizeof(float) > blob.size())
                    return false;
                out.resize(count);
                if (count != 0)
                    std::memcpy(out.data(), blob.data() + cursor, count * sizeof(float));
                cursor += count * sizeof(float);
                return true;
            }

            /** @brief Wraps @p value into [0, 1). */
            double wrap_unit(double value) noexcept
            {
                const double wrapped = value - std::floor(value);
                return wrapped >= 1.0 ? 0.0 : wrapped;
            }
        } // namespace

        bool Climatology::adopt(const std::vector<std::uint8_t>& blob)
        {
            if (blob.size() < 4 || std::memcmp(blob.data(), ASSET_MAGIC, 4) != 0)
                return false;

            std::size_t cursor = 4;
            std::uint32_t version = 0;
            std::int32_t bands = 0;
            std::int32_t months = 0;
            std::int32_t surface_longitudes = 0;
            std::int32_t surface_latitudes = 0;
            std::uint32_t provenance_length = 0;
            if (!take(blob, cursor, version) || version != ASSET_VERSION)
                return false;
            if (!take(blob, cursor, bands) || !take(blob, cursor, months))
                return false;
            if (!take(blob, cursor, surface_longitudes) || !take(blob, cursor, surface_latitudes))
                return false;
            if (!take(blob, cursor, provenance_length))
                return false;
            // Every dimension is used as a divisor and as an array bound below, so a zero or a
            // negative one is refused here rather than discovered as a division by zero in a
            // query three phases later. Twelve is not required -- a body with a different year
            // may bake a different number of intervals -- but at least one is.
            if (bands < 2 || months < 1 || surface_longitudes < 0 || surface_latitudes < 0)
                return false;
            if (provenance_length > 4096)
                return false;

            // Read into locals, not into the members: a blob that turns out to be short partway
            // through must leave a working climatology behind, and the one already loaded is a
            // working climatology.
            const std::size_t profile_count = std::size_t(bands) * std::size_t(months);
            const std::size_t surface_count =
                std::size_t(surface_longitudes) * std::size_t(surface_latitudes) * std::size_t(months);
            std::vector<float> upper;
            std::vector<float> lower;
            std::vector<float> saturation;
            std::vector<float> land;
            std::vector<float> sst;
            if (!take_floats(blob, cursor, profile_count, upper) ||
                !take_floats(blob, cursor, profile_count, lower) ||
                !take_floats(blob, cursor, profile_count, saturation))
                return false;
            // The land mask is one field, not one per month -- coastlines do not have a season.
            if (!take_floats(blob, cursor, std::size_t(surface_longitudes) *
                                               std::size_t(surface_latitudes), land) ||
                !take_floats(blob, cursor, surface_count, sst))
                return false;
            if (cursor + provenance_length != blob.size())
                return false;

            provenance_.assign(reinterpret_cast<const char*>(blob.data() + cursor),
                               provenance_length);
            profile_grid_.latitude_bands = bands;
            profile_grid_.months = months;
            surface_longitude_cells_ = surface_longitudes;
            surface_latitude_cells_ = surface_latitudes;
            upper_wind_ = std::move(upper);
            lower_wind_ = std::move(lower);
            saturation_ = std::move(saturation);
            land_fraction_ = std::move(land);
            sea_surface_temperature_ = std::move(sst);
            baked_ = true;
            return true;
        }

        double Climatology::sample_profile(const std::vector<float>& profile,
                                           double latitude_radians, double year_fraction) const
        {
            const int bands = profile_grid_.latitude_bands;
            const int months = profile_grid_.months;

            // Latitude: cell-centred bands from pole to pole, clamped rather than wrapped. There
            // is nothing past a pole to interpolate toward, and folding across one would blend
            // the band with itself.
            const double clamped = std::min(std::max(latitude_radians, -0.5 * PI), 0.5 * PI);
            const double position = (clamped + 0.5 * PI) / PI * double(bands) - 0.5;
            const int low_band = std::min(std::max(int(std::floor(position)), 0), bands - 1);
            const int high_band = std::min(low_band + 1, bands - 1);
            const double band_blend = std::min(std::max(position - double(low_band), 0.0), 1.0);

            // Month: centred on the middle of each interval and wrapping, so a query on the
            // thirty-first of December interpolates into January rather than clamping onto a
            // December that is about to end.
            const double month_position = wrap_unit(year_fraction) * double(months) - 0.5;
            const double month_floor = std::floor(month_position);
            const double month_blend = month_position - month_floor;
            const int low_month = ((int(month_floor) % months) + months) % months;
            const int high_month = (low_month + 1) % months;

            const auto at = [&](int month, int band) {
                return double(profile[std::size_t(month) * std::size_t(bands) + std::size_t(band)]);
            };
            const double early = at(low_month, low_band) * (1.0 - band_blend) +
                                 at(low_month, high_band) * band_blend;
            const double late = at(high_month, low_band) * (1.0 - band_blend) +
                                at(high_month, high_band) * band_blend;
            return early * (1.0 - month_blend) + late * month_blend;
        }

        /**
         * @brief The analytic jet: one Gaussian per hemisphere, at the same latitude and width.
         *
         * Shared by both layers because the *only* difference between them is the peak speed,
         * and the difference between those two peaks is the vertical shear that decides whether
         * the mean state makes storms at all.
         */
        static double analytic_jet(const AnalyticClimatology& bands, double latitude_radians,
                                   double speed)
        {
            const double north =
                (latitude_radians - bands.jet_latitude_radians) / bands.jet_width_radians;
            const double south =
                (latitude_radians + bands.jet_latitude_radians) / bands.jet_width_radians;
            return speed * (std::exp(-north * north) + std::exp(-south * south));
        }

        double Climatology::upper_zonal_wind_mps(double latitude_radians,
                                                 double year_fraction) const
        {
            return baked_ ? sample_profile(upper_wind_, latitude_radians, year_fraction)
                          : analytic_jet(bands_, latitude_radians, bands_.upper_jet_speed_mps);
        }

        double Climatology::lower_zonal_wind_mps(double latitude_radians,
                                                 double year_fraction) const
        {
            return baked_ ? sample_profile(lower_wind_, latitude_radians, year_fraction)
                          : analytic_jet(bands_, latitude_radians, bands_.lower_jet_speed_mps);
        }

        double Climatology::saturation_kg_per_m2(double latitude_radians,
                                                 double year_fraction) const
        {
            if (baked_)
                return sample_profile(saturation_, latitude_radians, year_fraction);

            // A latitudinal surface temperature, then Clausius-Clapeyron as a single e-folding.
            // This is the analytic latitude band §4 says T0 degrades to, and it is bit-for-bit
            // what the core computed inline before T0 existed -- so a core with no asset
            // reproduces every number C2 and C3 measured.
            const double sine = std::sin(latitude_radians);
            const double cooling = bands_.equator_to_pole_kelvin * sine * sine;
            return bands_.equatorial_saturation_kg_per_m2 *
                   std::exp(-cooling / bands_.saturation_lapse_kelvin);
        }

        double Climatology::land_fraction(const GeographicPosition& position) const
        {
            if (!baked_ || surface_longitude_cells_ <= 0 || surface_latitude_cells_ <= 0)
                return 0.0; // an analytic latitude band has no continents, and says so

            const double longitude = wrap_unit(position.longitude_radians / (2.0 * PI));
            const double latitude =
                std::min(std::max(position.latitude_radians, -0.5 * PI), 0.5 * PI);
            const int x = std::min(int(longitude * double(surface_longitude_cells_)),
                                   surface_longitude_cells_ - 1);
            const int y = std::min(
                std::max(int((latitude + 0.5 * PI) / PI * double(surface_latitude_cells_)), 0),
                surface_latitude_cells_ - 1);
            return double(land_fraction_[std::size_t(y) * std::size_t(surface_longitude_cells_) +
                                         std::size_t(x)]);
        }

        double Climatology::sea_surface_temperature_kelvin(const GeographicPosition& position,
                                                           double year_fraction) const
        {
            if (baked_ && surface_longitude_cells_ > 0 && surface_latitude_cells_ > 0)
            {
                const double longitude = wrap_unit(position.longitude_radians / (2.0 * PI));
                const double latitude =
                    std::min(std::max(position.latitude_radians, -0.5 * PI), 0.5 * PI);
                const int x = std::min(int(longitude * double(surface_longitude_cells_)),
                                       surface_longitude_cells_ - 1);
                const int y = std::min(
                    std::max(int((latitude + 0.5 * PI) / PI * double(surface_latitude_cells_)), 0),
                    surface_latitude_cells_ - 1);
                const int months = std::max(profile_grid_.months, 1);
                const int month = std::min(int(wrap_unit(year_fraction) * double(months)),
                                           months - 1);
                const std::size_t plane =
                    std::size_t(surface_longitude_cells_) * std::size_t(surface_latitude_cells_);
                return double(sea_surface_temperature_[std::size_t(month) * plane +
                                                       std::size_t(y) *
                                                           std::size_t(surface_longitude_cells_) +
                                                       std::size_t(x)]);
            }

            // The same latitudinal temperature the saturation profile is built on, so a consumer
            // never has to branch on whether an asset was loaded -- it gets a colder pole and a
            // warm equator either way, just without the ocean's own structure.
            const double sine = std::sin(position.latitude_radians);
            return ANALYTIC_EQUATORIAL_SURFACE_KELVIN - bands_.equator_to_pole_kelvin * sine * sine;
        }
    } // namespace Atmosphere
} // namespace SushiEngine
