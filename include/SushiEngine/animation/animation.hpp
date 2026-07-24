/**************************************************************************/
/* animation.hpp                                                          */
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
 * @file animation.hpp
 * @brief Umbrella for the skeletal-animation asset + evaluation layer (phases A0, A1).
 *
 * The foundations the rest of the animation stack builds on: the name-identity hash,
 * the immutable skeleton view, the relocatable `.sushiskel`/`.sushianim` cook/load, the
 * @ref SushiEngine::Animation::IAnimationDatabase seam that hands skeletons and clips out
 * by id, the single-clip @ref SushiEngine::Animation::AnimationPlayer component, and the
 * @ref SushiEngine::Animation::ClipEvaluator that samples a clip into an object-space skin
 * palette. Controller and IK layers arrive in later phases and extend these types;
 * nothing here depends on the renderer or the ECS.
 */

#include <SushiEngine/animation/asset_id.hpp>
#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp>
#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/clip_compressed.hpp>
#include <SushiEngine/animation/clip_compress.hpp>
#include <SushiEngine/animation/additive.hpp>
#include <SushiEngine/animation/morph.hpp>
#include <SushiEngine/animation/generic_track.hpp>
#include <SushiEngine/animation/keyframe.hpp>
#include <SushiEngine/animation/evaluator.hpp>
#include <SushiEngine/animation/batch_evaluator.hpp>
#include <SushiEngine/animation/animation_player.hpp>
#include <SushiEngine/animation/skin_vertex.hpp>
#include <SushiEngine/animation/blend_tree.hpp>
#include <SushiEngine/animation/avatar_mask.hpp>
#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/animator_components.hpp>
#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/animator_step.hpp>
#include <SushiEngine/animation/animator_evaluator.hpp>
#include <SushiEngine/animation/pose_modifier.hpp>
#include <SushiEngine/animation/ik_two_bone.hpp>
#include <SushiEngine/animation/ik_look_at.hpp>
#include <SushiEngine/animation/ik_chain.hpp>
#include <SushiEngine/animation/ik_foot_placement.hpp>
#include <SushiEngine/animation/humanoid.hpp>
#include <SushiEngine/animation/retarget.hpp>
#include <SushiEngine/animation/edit_preview.hpp>
