/**************************************************************************/
/* sofa_hrtf.hpp                                                         */
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
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#ifndef SUSHIENGINE_AUDIO_SOFA_HRTF_HPP
#define SUSHIENGINE_AUDIO_SOFA_HRTF_HPP

/**
 * @file sofa_hrtf.hpp
 * @brief The SOFA/HDF5 measured-HRTF loader — an `IHRTFDatabase` from a real dataset file.
 *
 * SOFA (Spatially Oriented Format for Acoustics, AES69) is the standard container for
 * measured HRIR sets; it is an HDF5 file whose `SimpleFreeFieldHRIR` convention stores three
 * datasets this loader reads: `Data.IR` `[M measurements][R receivers][N taps]` (the two-ear
 * impulse responses), `SourcePosition` `[M][3]` (azimuth°, elevation°, radius, spherical), and
 * `Data.SamplingRate`. @ref SofaHRTFDatabase reads them through the HDF5 C API, converts each
 * source position to a head-relative unit vector, optionally resamples the taps to the stream
 * rate, and serves the nearest pair to the spatializer through the @ref IHRTFDatabase seam.
 *
 * @ref write_sofa bakes the same three datasets, so a set can be authored/round-tripped
 * without an external artifact. This header pulls in HDF5, so — like `opus_codec.hpp` — it is
 * not on the `audio.hpp` umbrella and is compiled only where `hdf5::hdf5-static`/`hdf5` links.
 */

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <hdf5.h>

#include <SushiEngine/audio/hrtf.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief A measured-HRTF database loaded from a SOFA (HDF5) file. */
        class SofaHRTFDatabase final : public IHRTFDatabase
        {
            public:
                int ir_length() const noexcept override { return ir_length_; }
                double sample_rate() const noexcept override { return sample_rate_; }

                /** @brief The number of measured directions loaded. */
                int measurement_count() const noexcept { return measurement_count_; }

                /** @brief Whether a dataset is loaded and usable. */
                bool valid() const noexcept { return measurement_count_ > 0 && ir_length_ > 0; }

                /**
                 * @brief Loads a SOFA HRIR set from an HDF5 file.
                 *
                 * Reads `Data.IR`, `SourcePosition`, and `Data.SamplingRate`, precomputes each
                 * measurement's head-relative unit direction, and (when @p target_sample_rate is
                 * non-zero and differs) linearly resamples every impulse response to that rate.
                 *
                 * @param path               Path to the `.sofa` file.
                 * @param target_sample_rate Rate to resample taps to, or 0 to keep the native rate.
                 * @return True on success; false if the file or a required dataset is missing.
                 */
                bool load(const std::string& path, double target_sample_rate = 0.0)
                {
                    measurement_count_ = 0;
                    ir_length_ = 0;

                    const hid_t file = H5Fopen(path.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
                    if (file < 0)
                        return false;

                    hsize_t ir_dims[3] = {0, 0, 0};
                    std::vector<double> ir;
                    hsize_t pos_dims[2] = {0, 0};
                    std::vector<double> positions;
                    std::vector<double> rate;

                    bool ok = read_dataset(file, "Data.IR", ir, ir_dims, 3);
                    ok = ok && read_dataset(file, "SourcePosition", positions, pos_dims, 2);
                    hsize_t rate_dims[2] = {0, 0};
                    ok = ok && read_dataset_flat(file, "Data.SamplingRate", rate, rate_dims, 2);
                    H5Fclose(file);
                    if (!ok || ir_dims[1] < 2 || pos_dims[0] != ir_dims[0] || rate.empty())
                        return false;

                    const int measurements = static_cast<int>(ir_dims[0]);
                    const int receivers = static_cast<int>(ir_dims[1]);
                    const int taps = static_cast<int>(ir_dims[2]);
                    const double native_rate = rate[0];

                    directions_.assign(static_cast<std::size_t>(measurements * 3), 0.0f);
                    for (int m = 0; m < measurements; ++m)
                    {
                        const double az_deg = positions[static_cast<std::size_t>(m * 3 + 0)];
                        const double el_deg = positions[static_cast<std::size_t>(m * 3 + 1)];
                        const double az = az_deg * 3.14159265358979323846 / 180.0;
                        const double el = el_deg * 3.14159265358979323846 / 180.0;
                        // Azimuth is counterclockwise from front toward the left ear.
                        directions_[static_cast<std::size_t>(m * 3 + 0)] =
                            static_cast<float>(std::cos(el) * std::cos(az)); // front
                        directions_[static_cast<std::size_t>(m * 3 + 1)] =
                            static_cast<float>(std::cos(el) * std::sin(az)); // left
                        directions_[static_cast<std::size_t>(m * 3 + 2)] =
                            static_cast<float>(std::sin(el)); // up
                    }

                    const bool resample =
                        target_sample_rate > 0.0 &&
                        std::fabs(target_sample_rate - native_rate) > 1e-6 && native_rate > 0.0;
                    const int out_taps =
                        resample ? static_cast<int>(std::lround(taps * target_sample_rate /
                                                                native_rate))
                                 : taps;

                    ir_.assign(static_cast<std::size_t>(measurements * 2 * out_taps), 0.0f);
                    for (int m = 0; m < measurements; ++m)
                    {
                        for (int ear = 0; ear < 2; ++ear)
                        {
                            const double* source =
                                &ir[static_cast<std::size_t>((m * receivers + ear) * taps)];
                            float* destination =
                                &ir_[static_cast<std::size_t>((m * 2 + ear) * out_taps)];
                            if (!resample)
                            {
                                for (int t = 0; t < taps; ++t)
                                    destination[t] = static_cast<float>(source[t]);
                            }
                            else
                            {
                                const double step = static_cast<double>(taps) / out_taps;
                                for (int t = 0; t < out_taps; ++t)
                                {
                                    const double sp = t * step;
                                    const int i0 = static_cast<int>(sp);
                                    const int i1 = (i0 + 1 < taps) ? i0 + 1 : taps - 1;
                                    const double frac = sp - i0;
                                    destination[t] = static_cast<float>(source[i0] * (1.0 - frac) +
                                                                        source[i1] * frac);
                                }
                            }
                        }
                    }

                    measurement_count_ = measurements;
                    ir_length_ = out_taps;
                    sample_rate_ = resample ? target_sample_rate : native_rate;
                    return true;
                }

                void get_hrir(float front, float left, float up, float* left_ir,
                              float* right_ir) const noexcept override
                {
                    for (int t = 0; t < ir_length_; ++t)
                    {
                        left_ir[t] = 0.0f;
                        right_ir[t] = 0.0f;
                    }
                    if (measurement_count_ == 0)
                        return;

                    float length = std::sqrt(front * front + left * left + up * up);
                    if (length < 1e-8f)
                        length = 1.0f;
                    const float qx = front / length, qy = left / length, qz = up / length;

                    int best = 0;
                    float best_dot = -2.0f;
                    for (int m = 0; m < measurement_count_; ++m)
                    {
                        const float dot =
                            directions_[static_cast<std::size_t>(m * 3 + 0)] * qx +
                            directions_[static_cast<std::size_t>(m * 3 + 1)] * qy +
                            directions_[static_cast<std::size_t>(m * 3 + 2)] * qz;
                        if (dot > best_dot)
                        {
                            best_dot = dot;
                            best = m;
                        }
                    }

                    const float* l = &ir_[static_cast<std::size_t>((best * 2 + 0) * ir_length_)];
                    const float* r = &ir_[static_cast<std::size_t>((best * 2 + 1) * ir_length_)];
                    for (int t = 0; t < ir_length_; ++t)
                    {
                        left_ir[t] = l[t];
                        right_ir[t] = r[t];
                    }
                }

            private:
                static bool read_dataset(hid_t file, const char* name, std::vector<double>& out,
                                         hsize_t* dims, int expected_rank)
                {
                    const hid_t dset = H5Dopen2(file, name, H5P_DEFAULT);
                    if (dset < 0)
                        return false;
                    const hid_t space = H5Dget_space(dset);
                    const int rank = H5Sget_simple_extent_ndims(space);
                    if (rank != expected_rank)
                    {
                        H5Sclose(space);
                        H5Dclose(dset);
                        return false;
                    }
                    H5Sget_simple_extent_dims(space, dims, nullptr);
                    hsize_t total = 1;
                    for (int i = 0; i < rank; ++i)
                        total *= dims[i];
                    out.assign(static_cast<std::size_t>(total), 0.0);
                    const herr_t err = H5Dread(dset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL,
                                               H5P_DEFAULT, out.data());
                    H5Sclose(space);
                    H5Dclose(dset);
                    return err >= 0;
                }

                // Reads a dataset of any rank up to 2 into a flat buffer (for the scalar-ish rate).
                static bool read_dataset_flat(hid_t file, const char* name, std::vector<double>& out,
                                              hsize_t* dims, int max_rank)
                {
                    const hid_t dset = H5Dopen2(file, name, H5P_DEFAULT);
                    if (dset < 0)
                        return false;
                    const hid_t space = H5Dget_space(dset);
                    const int rank = H5Sget_simple_extent_ndims(space);
                    if (rank > max_rank)
                    {
                        H5Sclose(space);
                        H5Dclose(dset);
                        return false;
                    }
                    hsize_t total = 1;
                    if (rank == 0)
                    {
                        dims[0] = 1;
                    }
                    else
                    {
                        H5Sget_simple_extent_dims(space, dims, nullptr);
                        for (int i = 0; i < rank; ++i)
                            total *= dims[i];
                    }
                    out.assign(static_cast<std::size_t>(total), 0.0);
                    const herr_t err = H5Dread(dset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL,
                                               H5P_DEFAULT, out.data());
                    H5Sclose(space);
                    H5Dclose(dset);
                    return err >= 0;
                }

                std::vector<float> ir_;         // [m][ear][tap], flattened
                std::vector<float> directions_; // [m][3] head-relative unit (front, left, up)
                int measurement_count_ = 0;
                int ir_length_ = 0;
                double sample_rate_ = 48000.0;
        };

        /** @brief Creates an HDF5 double dataset (helper for @ref write_sofa). */
        inline bool write_double_dataset(hid_t file, const char* name, int rank,
                                         const hsize_t* dims, const double* data);

        /**
         * @brief Bakes an HRIR set to a minimal SOFA (HDF5) file the loader round-trips.
         *
         * Writes the three datasets @ref SofaHRTFDatabase reads — `Data.IR` `[M][2][N]`,
         * `SourcePosition` `[M][3]`, and `Data.SamplingRate` — as HDF5 doubles.
         *
         * @param path        Output `.sofa` path.
         * @param measurements Number of directions M.
         * @param taps        Impulse-response length N.
         * @param sample_rate Sampling rate written to `Data.SamplingRate`.
         * @param ir          `[M][2][N]` interleaved impulse responses (left ear, then right).
         * @param az_el_r     `[M][3]` source positions (azimuth°, elevation°, radius).
         * @return True on success.
         */
        inline bool write_sofa(const std::string& path, int measurements, int taps,
                               double sample_rate, const float* ir, const float* az_el_r)
        {
            const hid_t file =
                H5Fcreate(path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
            if (file < 0)
                return false;

            bool ok = true;
            {
                const hsize_t dims[3] = {static_cast<hsize_t>(measurements), 2,
                                         static_cast<hsize_t>(taps)};
                std::vector<double> buffer(static_cast<std::size_t>(measurements * 2 * taps));
                for (std::size_t i = 0; i < buffer.size(); ++i)
                    buffer[i] = ir[i];
                ok = ok && write_double_dataset(file, "Data.IR", 3, dims, buffer.data());
            }
            {
                const hsize_t dims[2] = {static_cast<hsize_t>(measurements), 3};
                std::vector<double> buffer(static_cast<std::size_t>(measurements * 3));
                for (std::size_t i = 0; i < buffer.size(); ++i)
                    buffer[i] = az_el_r[i];
                ok = ok && write_double_dataset(file, "SourcePosition", 2, dims, buffer.data());
            }
            {
                const hsize_t dims[1] = {1};
                ok = ok && write_double_dataset(file, "Data.SamplingRate", 1, dims, &sample_rate);
            }

            H5Fclose(file);
            return ok;
        }

        namespace Detail
        {
            inline bool write_double_dataset_impl(hid_t file, const char* name, int rank,
                                                  const hsize_t* dims, const double* data)
            {
                const hid_t space = H5Screate_simple(rank, dims, nullptr);
                const hid_t dset = H5Dcreate2(file, name, H5T_IEEE_F64LE, space, H5P_DEFAULT,
                                              H5P_DEFAULT, H5P_DEFAULT);
                bool ok = dset >= 0;
                if (ok)
                {
                    const herr_t err = H5Dwrite(dset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL,
                                                H5P_DEFAULT, data);
                    ok = err >= 0;
                    H5Dclose(dset);
                }
                H5Sclose(space);
                return ok;
            }
        } // namespace Detail

        /** @brief Creates an HDF5 double dataset (helper for @ref write_sofa). */
        inline bool write_double_dataset(hid_t file, const char* name, int rank,
                                         const hsize_t* dims, const double* data)
        {
            return Detail::write_double_dataset_impl(file, name, rank, dims, data);
        }
    } // namespace Audio
} // namespace SushiEngine

#endif
