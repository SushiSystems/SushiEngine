/**************************************************************************/
/* animator_preview_panel.cpp                                            */
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

#include "animator_preview_panel.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

#include "animated_mesh_preview.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            /** @brief The sentinel "no layer selected for removal" index. */
            constexpr std::size_t NO_LAYER = static_cast<std::size_t>(-1);

            // The Add Layer form's transient authoring state — not part of the compiled
            // controller until "Add Layer" is pressed, so it lives here rather than on
            // AnimatedMeshPreview, like the other panels' file-static authoring buffers.
            int add_layer_clip_index = 0;
            std::vector<bool> add_layer_mask; // per joint; resized to the loaded skeleton below
            float add_layer_mask_default_weight = 0.0f;
            float add_layer_weight = 1.0f;
            bool add_layer_additive = false;

            // The IK section's transient joint/pole/weight fields — applied to
            // AnimatedMeshPreview::set_two_bone_ik only on "Apply IK", so a half-picked chain
            // never reaches the live pose-modifier stack mid-edit.
            int ik_upper = 0;
            int ik_mid = 0;
            int ik_tip = 0;
            float ik_pole[3] = {0.0f, 0.0f, 1.0f};
            float ik_weight = 0.0f;

            /** @brief A combo box over every joint in @p skeleton, by name. */
            void joint_combo(const char* label, const Animation::SkeletonView& skeleton,
                             int& joint_index)
            {
                if (joint_index < 0 || joint_index >= static_cast<int>(skeleton.joint_count))
                    joint_index = 0;
                const char* current_name =
                    skeleton.joint_count > 0
                        ? skeleton.joint_name(static_cast<std::uint32_t>(joint_index))
                        : "";
                if (ImGui::BeginCombo(label, current_name))
                {
                    for (std::uint32_t j = 0; j < skeleton.joint_count; ++j)
                    {
                        const bool selected = static_cast<int>(j) == joint_index;
                        if (ImGui::Selectable(skeleton.joint_name(j), selected))
                            joint_index = static_cast<int>(j);
                        if (selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            }

            void draw_layer_list(AnimatedMeshPreview& preview)
            {
                ImGui::Text("Layers (%zu)", preview.layer_count());
                std::size_t pending_remove = NO_LAYER;
                for (std::size_t i = 0; i < preview.layer_count(); ++i)
                {
                    ImGui::PushID(static_cast<int>(i));
                    ImGui::Text("%zu: %s%s", i, preview.layer_name(i).c_str(),
                               preview.layer_additive(i) ? " (additive)" : " (override)");
                    if (i == 0)
                    {
                        ImGui::TextDisabled("  base layer, always full weight");
                    }
                    else
                    {
                        float weight = preview.layer_weight(i);
                        ImGui::SetNextItemWidth(180.0f);
                        if (ImGui::SliderFloat("Weight", &weight, 0.0f, 1.0f))
                            preview.set_layer_weight(i, weight);
                        ImGui::SameLine();
                        if (ImGui::Button("Remove"))
                            pending_remove = i;
                    }
                    ImGui::PopID();
                }
                // Applied after the loop: remove_layer() shifts every later layer's index, so
                // acting on it mid-iteration would skip or double-visit a layer.
                if (pending_remove != NO_LAYER)
                    preview.remove_layer(pending_remove);
            }

            void draw_add_layer_form(AnimatedMeshPreview& preview,
                                     const Animation::SkeletonView& skeleton)
            {
                const std::vector<Animation::GltfClip>& clips = preview.available_clips();
                if (clips.empty())
                {
                    ImGui::TextDisabled("No other clips in the source file");
                    return;
                }
                if (add_layer_clip_index >= static_cast<int>(clips.size()))
                    add_layer_clip_index = 0;

                if (ImGui::BeginCombo("Clip", clips[static_cast<std::size_t>(add_layer_clip_index)]
                                                  .name.c_str()))
                {
                    for (int i = 0; i < static_cast<int>(clips.size()); ++i)
                    {
                        const bool selected = i == add_layer_clip_index;
                        if (ImGui::Selectable(clips[static_cast<std::size_t>(i)].name.c_str(),
                                              selected))
                            add_layer_clip_index = i;
                        if (selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
                ImGui::SliderFloat("Initial Weight", &add_layer_weight, 0.0f, 1.0f);
                ImGui::Checkbox("Additive", &add_layer_additive);

                if (add_layer_mask.size() != skeleton.joint_count)
                    add_layer_mask.assign(skeleton.joint_count, false);

                if (ImGui::TreeNode("Mask"))
                {
                    ImGui::TextWrapped(
                        "Checked joints admit this layer at full weight; unchecked joints admit "
                        "the default weight below (an avatar mask).");
                    ImGui::SliderFloat("Default Weight", &add_layer_mask_default_weight, 0.0f,
                                       1.0f);
                    for (std::uint32_t j = 0; j < skeleton.joint_count; ++j)
                    {
                        bool checked = add_layer_mask[j];
                        if (ImGui::Checkbox(skeleton.joint_name(j), &checked))
                            add_layer_mask[j] = checked;
                    }
                    ImGui::TreePop();
                }

                if (ImGui::Button("Add Layer"))
                {
                    // A mask with no entries and default_weight 1.0 admits every joint at full
                    // weight — identical in effect to no mask at all — so this is always safe
                    // to build and pass, no "did the user actually touch the mask" branch needed.
                    Animation::MaskDesc mask_desc;
                    mask_desc.default_weight = add_layer_mask_default_weight;
                    for (std::uint32_t j = 0; j < skeleton.joint_count; ++j)
                        if (add_layer_mask[j])
                        {
                            Animation::MaskDesc::Entry entry;
                            entry.joint = skeleton.joint_name(j);
                            entry.weight = 1.0f;
                            mask_desc.entries.push_back(entry);
                        }
                    preview.add_layer(
                        clips[static_cast<std::size_t>(add_layer_clip_index)].name.c_str(),
                        &mask_desc, add_layer_weight, add_layer_additive);
                }
            }

            void draw_ik_section(AnimatedMeshPreview& preview,
                                 const Animation::SkeletonView& skeleton)
            {
                joint_combo("Upper", skeleton, ik_upper);
                joint_combo("Mid", skeleton, ik_mid);
                joint_combo("Tip", skeleton, ik_tip);
                ImGui::DragFloat3("Pole", ik_pole, 0.01f);
                ImGui::SliderFloat("IK Weight", &ik_weight, 0.0f, 1.0f);

                const Animation::TwoBoneIk& current = preview.two_bone_ik();
                ImGui::Text("Target (drag in Scene viewport): %.3f, %.3f, %.3f",
                           static_cast<float>(current.target.x),
                           static_cast<float>(current.target.y),
                           static_cast<float>(current.target.z));

                if (ImGui::Button("Apply IK"))
                    preview.set_two_bone_ik(
                        static_cast<std::uint32_t>(ik_upper), static_cast<std::uint32_t>(ik_mid),
                        static_cast<std::uint32_t>(ik_tip),
                        current.target, // preserve whatever the viewport gizmo last dragged it to
                        Vector3{ik_pole[0], ik_pole[1], ik_pole[2]}, ik_weight);
            }
        } // namespace

        void draw_animator_preview_panel(EditorContext& context)
        {
            if (!context.panels.animator_preview)
                return;
            // "Animator", not "Animator Preview": this is authoring — layers, masks, IK — over the
            // character the Preview surface shows. The preview itself is one screen for everything
            // being authored, so nothing else calls itself a preview window.
            if (!ImGui::Begin("Animator", &context.panels.animator_preview))
            {
                ImGui::End();
                return;
            }

            AnimatedMeshPreview* preview = context.animated_mesh_preview;
            if (preview == nullptr)
            {
                ImGui::TextDisabled("No preview available");
                ImGui::End();
                return;
            }

            // The character loader: type or paste a rigged .gltf/.glb path — or
            // double-click one in the Project panel, which routes here. This replaced the
            // hard-coded demo asset as the only way a character ever reached the preview.
            std::string& character_path = context.panel_state.character_path;
            ImGui::SetNextItemWidth(340.0f);
            ImGui::InputText("##character_path", &character_path);
            ImGui::SameLine();
            ImGui::BeginDisabled(context.assets == nullptr);
            if (ImGui::Button("Load Character"))
            {
                if (preview->load_gltf(character_path.c_str(), *context.assets))
                    context.panels.preview = true;
            }
            ImGui::EndDisabled();

            if (!preview->loaded())
            {
                ImGui::TextDisabled("No character loaded");
                ImGui::End();
                return;
            }

            bool playing = preview->playing();
            if (ImGui::Checkbox("Playing", &playing))
                preview->set_playing(playing);
            ImGui::SameLine();
            if (ImGui::Button("Restart"))
                preview->restart();

            ImGui::Separator();
            draw_layer_list(*preview);

            ImGui::Separator();
            ImGui::Text("Add Layer");
            draw_add_layer_form(*preview, preview->skeleton());

            ImGui::Separator();
            ImGui::Text("Two-Bone IK");
            draw_ik_section(*preview, preview->skeleton());

            ImGui::Separator();
            ImGui::Text("Blend Shapes");
            const std::uint32_t morph_count = preview->morph_target_count();
            if (morph_count == 0)
            {
                ImGui::TextDisabled("Loaded mesh has no morph targets.");
            }
            else
            {
                const std::uint32_t morph_tracks = preview->clip_morph_track_count();
                bool clip_driven = preview->clip_driven_morphs();
                if (ImGui::Checkbox("Driven by clip", &clip_driven))
                    preview->set_clip_driven_morphs(clip_driven);
                const bool sliders_live = !clip_driven || morph_tracks == 0;
                for (std::uint32_t i = 0; i < morph_count; ++i)
                {
                    ImGui::PushID(static_cast<int>(i));
                    float weight = preview->morph_weight(i);
                    ImGui::BeginDisabled(!sliders_live);
                    if (ImGui::SliderFloat("##weight", &weight, 0.0f, 1.0f))
                        preview->set_morph_weight(i, weight);
                    ImGui::EndDisabled();
                    ImGui::SameLine();
                    ImGui::TextUnformatted(preview->morph_target_name(i).c_str());
                    ImGui::PopID();
                }
                if (morph_tracks == 0)
                    ImGui::TextDisabled(
                        "The clip has no morph tracks — these weights are held as set.");
                else if (clip_driven)
                    ImGui::TextDisabled("%u clip track(s) driving these weights.", morph_tracks);
            }

            ImGui::Separator();
            bool use_dqs = preview->dual_quaternion_skinning();
            if (ImGui::Checkbox("Dual-Quaternion Skinning", &use_dqs))
                preview->set_dual_quaternion_skinning(use_dqs);
            ImGui::TextDisabled(
                "Fixes candy-wrapper pinch under large bends; most visible on a "
                "sharply bent limb.");

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
