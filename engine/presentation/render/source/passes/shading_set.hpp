/**************************************************************************/
/* shading_set.hpp                                                        */
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
 * @file shading_set.hpp
 * @brief What every `pbr.frag` draw needs in set 0, in one place.
 *
 * `pbr.frag` reads twenty-six descriptors — the IBL trio and its SH buffer, four shadow
 * sources, the material and motion arrays, the clustered light and decal grids with their
 * index lists, the punctual shadow atlas, ambient occlusion, and the GI probe volume — and
 * set 0 is a **push descriptor** set, so a pass that writes only some of them leaves the
 * rest undefined. That is not a mild bug: the fragment shader samples an undefined
 * descriptor and the device is lost.
 *
 * Three passes shade with `pbr.frag` (opaque, transparent, terrain) and each needs exactly
 * this set and exactly the graph reads that make it safe to read. Keeping one copy is
 * therefore not tidiness, it is the difference between adding a fourth such pass being a
 * one-line job and being a debugging session — and between a new binding reaching every
 * shading pass and reaching two of the three.
 *
 * The two halves must stay paired. @ref declare_shading_set is what makes the graph derive
 * the barriers; @ref write_shading_set is what points the descriptors at the results.
 * Writing without declaring reads stale data; declaring without writing is the device loss
 * above.
 */

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "scene/scene_layout.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Assets
        {
            class MaterialSystem;
        }

        namespace Lighting
        {
            class LightSystem;
        }

        namespace Scene
        {
            class MotionSystem;
        }

        namespace Passes
        {
            class CloudShadowMapPass;
            class IBLPass;
            class IrradianceVolumePass;

            /**
             * @brief The producers a shading draw reads through set 0.
             *
             * References, gathered at the call site rather than stored, because every one of
             * them is already a member of the pass doing the shading — this is the argument
             * list of the write, given a name.
             */
            struct ShadingSetSources
            {
                IBLPass& ibl;
                CloudShadowMapPass& cloud_shadow;
                IrradianceVolumePass& gi;
                Assets::MaterialSystem& materials;
                SushiEngine::Render::Scene::MotionSystem& motion;
                Lighting::LightSystem& lights;
            };

            /**
             * @brief Declares every frame resource the shading set reads.
             *
             * Call from a pass's build callback, alongside its own attachments and reads.
             * Covers only what @ref write_shading_set binds; a pass's own inputs — its
             * vertex data, its instance buffers — remain its own to declare.
             *
             * @param builder The pass being built.
             * @param frame   This frame's context, for its target handles.
             */
            void declare_shading_set(Graph::RenderPassBuilder& builder,
                                     const Frame::FrameContext& frame);

            /**
             * @brief Queues all twenty-six of set 0's descriptors.
             *
             * Does not commit: the caller owns the writer, so a pass that pushes the same
             * set against two pipeline layouts in a row (the GPU-driven path does) writes
             * once and commits twice.
             *
             * @param writer  The batch to fill.
             * @param sources The producers to point at.
             * @param frame   This frame's context.
             * @param context The executing pass's context, for resolved handles.
             */
            void write_shading_set(SushiEngine::Render::Scene::SceneSetWriter& writer,
                                   const ShadingSetSources& sources,
                                   const Frame::FrameContext& frame,
                                   const Graph::PassContext& context);
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
