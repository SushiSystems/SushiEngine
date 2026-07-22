/**************************************************************************/
/* probe_volume.cpp                                                       */
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

#include "gi/probe_volume.hpp"

#include <cmath>

namespace SushiEngine
{
    namespace Render
    {
        namespace Gi
        {
            void configure_probe_volume(const double eye[3], bool enabled, float intensity,
                                        float normal_bias, ProbeVolumeConfig& out) noexcept
            {
                const std::int32_t counts[3] = {PROBE_COUNT_HORIZONTAL, PROBE_COUNT_VERTICAL,
                                                PROBE_COUNT_HORIZONTAL};

                out.params[0] = enabled ? 1.0f : 0.0f;
                out.params[1] = intensity;
                out.params[2] = normal_bias;
                out.params[3] = static_cast<float>(GI_NUM_CASCADES);

                out.counts[0] = counts[0];
                out.counts[1] = counts[1];
                out.counts[2] = counts[2];
                out.counts[3] = PROBE_COUNT_TOTAL;

                for (std::int32_t cascade = 0; cascade < GI_NUM_CASCADES; ++cascade)
                {
                    const double spacing = static_cast<double>(probe_cascade_spacing(cascade));
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        // Anchor probe (0,0,0) half a grid below the nearest multiple of this
                        // cascade's own spacing to the eye, so the cascade stays centred on the
                        // camera while its probes sit at fixed world positions. Each cascade
                        // snaps to its own lattice, so the coarse grids shift far less often.
                        const double center = std::floor(eye[axis] / spacing + 0.5) * spacing;
                        const double origin_world =
                            center - 0.5 * static_cast<double>(counts[axis] - 1) * spacing;
                        out.cascade_origin[cascade][axis] =
                            static_cast<float>(origin_world - eye[axis]);
                    }
                    out.cascade_origin[cascade][3] = static_cast<float>(spacing);
                }
            }
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
