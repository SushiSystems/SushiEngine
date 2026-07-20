/**************************************************************************/
/* material_inspector.cpp                                                 */
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

#include "material_inspector.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>

#include <imgui.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            using SushiEngine::Render::BlendMode;
            using SushiEngine::Render::IAssetLibrary;
            using SushiEngine::Render::INVALID_TEXTURE;
            using SushiEngine::Render::MaterialCullMode;
            using SushiEngine::Render::SurfaceType;
            using SushiEngine::Render::TextureColorSpace;
            using SushiEngine::Render::TextureId;
            using SushiEngine::Render::TextureWrap;

            /**
             * @brief The path last typed into each texture slot.
             *
             * The material stores an opaque id, not a path, so the text the user typed
             * has to live somewhere for the field to keep showing it. Keyed by the
             * widget's ImGui id, which is unique per slot per inspected entity.
             */
            std::unordered_map<ImGuiID, std::string>& slot_paths()
            {
                static std::unordered_map<ImGuiID, std::string> paths;
                return paths;
            }

            /**
             * @brief Draws one texture slot: a path field, a load button, and a clear.
             * @param label       Slot name shown to the user.
             * @param texture     The slot's texture id, edited in place.
             * @param assets      Library the path is loaded through.
             * @param color_space How the file's values are to be interpreted.
             * @return true when the slot changed.
             */
            bool texture_slot(const char* label, TextureId& texture, IAssetLibrary& assets,
                              TextureColorSpace color_space)
            {
                ImGui::PushID(label);
                bool changed = false;

                const ImGuiID id = ImGui::GetID(label);
                std::string& path = slot_paths()[id];
                char buffer[512];
                std::snprintf(buffer, sizeof(buffer), "%s", path.c_str());

                ImGui::SetNextItemWidth(-140.0f);
                if (ImGui::InputText(label, buffer, sizeof(buffer),
                                     ImGuiInputTextFlags_EnterReturnsTrue))
                {
                    path = buffer;
                    const TextureId loaded = path.empty()
                                                 ? INVALID_TEXTURE
                                                 : assets.load_texture(path.c_str(), color_space);
                    if (texture != INVALID_TEXTURE)
                        assets.release_texture(texture);
                    texture = loaded;
                    changed = true;
                }
                else
                {
                    path = buffer;
                }

                ImGui::SameLine();
                if (ImGui::SmallButton("Load"))
                {
                    const TextureId loaded = path.empty()
                                                 ? INVALID_TEXTURE
                                                 : assets.load_texture(path.c_str(), color_space);
                    if (texture != INVALID_TEXTURE)
                        assets.release_texture(texture);
                    texture = loaded;
                    changed = true;
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("Clear") && texture != INVALID_TEXTURE)
                {
                    assets.release_texture(texture);
                    texture = INVALID_TEXTURE;
                    path.clear();
                    changed = true;
                }
                ImGui::SameLine();
                ImGui::TextDisabled("%s", texture != INVALID_TEXTURE ? "set" : "none");

                ImGui::PopID();
                return changed;
            }

            /**
             * @brief Draws a tiling/offset pair — the Unity ST vector.
             * @param label     Prefix shown on both rows.
             * @param transform The transform, edited in place.
             * @return true when either row changed.
             */
            bool transform_rows(const char* label,
                                SushiEngine::Render::TextureTransform& transform)
            {
                ImGui::PushID(label);
                bool changed = false;
                float tiling[2] = {transform.tiling_x, transform.tiling_y};
                if (ImGui::DragFloat2("Tiling", tiling, 0.01f))
                {
                    transform.tiling_x = tiling[0];
                    transform.tiling_y = tiling[1];
                    changed = true;
                }
                float offset[2] = {transform.offset_x, transform.offset_y};
                if (ImGui::DragFloat2("Offset", offset, 0.005f))
                {
                    transform.offset_x = offset[0];
                    transform.offset_y = offset[1];
                    changed = true;
                }
                ImGui::PopID();
                return changed;
            }

            /**
             * @brief Edits a Vector3 as an HDR colour.
             * @param label Row label.
             * @param value The colour, edited in place.
             * @param hdr   Allow values above 1.
             * @return true when the colour changed.
             */
            bool color_row(const char* label, SushiEngine::Vector3& value, bool hdr)
            {
                float rgb[3] = {static_cast<float>(value.x), static_cast<float>(value.y),
                                static_cast<float>(value.z)};
                const ImGuiColorEditFlags flags =
                    hdr ? (ImGuiColorEditFlags_HDR | ImGuiColorEditFlags_Float) : 0;
                if (!ImGui::ColorEdit3(label, rgb, flags))
                    return false;
                value = SushiEngine::Vector3{rgb[0], rgb[1], rgb[2]};
                return true;
            }
        } // namespace

        bool draw_material_editor(SushiEngine::Render::Material& material, IAssetLibrary& assets)
        {
            bool changed = false;

            // Tiling and offset lead, as they do in Unity's Standard shader: they govern
            // every map in the main set, so they belong above the maps they affect.
            changed |= transform_rows("Main Maps", material.main_transform);
            ImGui::Separator();

            changed |= texture_slot("Albedo", material.albedo_map, assets,
                                    TextureColorSpace::Srgb);
            changed |= color_row("Base Color", material.albedo, false);
            changed |= ImGui::SliderFloat("Alpha", &material.base_alpha, 0.0f, 1.0f);

            changed |= texture_slot("Metallic-Roughness", material.metallic_roughness_map, assets,
                                    TextureColorSpace::Linear);
            changed |= ImGui::SliderFloat("Metallic", &material.metallic, 0.0f, 1.0f);
            changed |= ImGui::SliderFloat("Roughness", &material.roughness, 0.045f, 1.0f);
            changed |= ImGui::Checkbox("Packed Occlusion (ORM)", &material.packed_occlusion);

            changed |= texture_slot("Normal", material.normal_map, assets,
                                    TextureColorSpace::Linear);
            changed |= ImGui::SliderFloat("Normal Scale", &material.normal_scale, 0.0f, 4.0f);

            changed |= texture_slot("Height", material.height_map, assets,
                                    TextureColorSpace::Linear);
            changed |= ImGui::SliderFloat("Height Scale", &material.height_scale, 0.0f, 0.2f,
                                          "%.4f");
            int parallax_steps = static_cast<int>(material.parallax_steps);
            if (ImGui::SliderInt("Parallax Steps", &parallax_steps, 0, 64))
            {
                material.parallax_steps = static_cast<std::uint32_t>(parallax_steps);
                changed = true;
            }
            changed |= ImGui::Checkbox("Parallax Self-Shadow", &material.parallax_shadows);
            ImGui::SameLine();
            changed |= ImGui::Checkbox("Silhouette Clip", &material.parallax_silhouette_clip);

            changed |= texture_slot("Occlusion", material.occlusion_map, assets,
                                    TextureColorSpace::Linear);
            changed |= ImGui::SliderFloat("Occlusion Strength", &material.occlusion_strength, 0.0f,
                                          1.0f);

            changed |= ImGui::Checkbox("Emission", &material.emissive_enabled);
            if (material.emissive_enabled)
            {
                changed |= texture_slot("Emissive", material.emissive_map, assets,
                                        TextureColorSpace::Srgb);
                changed |= color_row("Emissive Color", material.emissive, true);
                changed |= ImGui::DragFloat("Emissive Intensity", &material.emissive_intensity,
                                            0.05f, 0.0f, 100.0f);
            }

            if (ImGui::TreeNode("Detail"))
            {
                changed |= transform_rows("Detail Maps", material.detail_transform);
                changed |= texture_slot("Detail Albedo", material.detail_albedo_map, assets,
                                        TextureColorSpace::Srgb);
                changed |= texture_slot("Detail Normal", material.detail_normal_map, assets,
                                        TextureColorSpace::Linear);
                changed |= texture_slot("Detail Mask", material.detail_mask_map, assets,
                                        TextureColorSpace::Linear);
                changed |= ImGui::SliderFloat("Detail Normal Scale",
                                              &material.detail_normal_scale, 0.0f, 4.0f);
                ImGui::TreePop();
            }

            if (ImGui::TreeNode("Advanced"))
            {
                ImGui::TextDisabled("Each lobe below costs shading time on every pixel");
                changed |= ImGui::SliderFloat("Anisotropy", &material.anisotropy, -1.0f, 1.0f);
                changed |= ImGui::SliderFloat("Anisotropy Rotation",
                                              &material.anisotropy_rotation, 0.0f, 1.0f);
                changed |= ImGui::SliderFloat("Clearcoat", &material.clearcoat, 0.0f, 1.0f);
                changed |= ImGui::SliderFloat("Clearcoat Roughness",
                                              &material.clearcoat_roughness, 0.02f, 1.0f);
                changed |= color_row("Sheen Color", material.sheen_color, false);
                changed |= ImGui::SliderFloat("Sheen Roughness", &material.sheen_roughness, 0.05f,
                                              1.0f);
                changed |= ImGui::SliderFloat("Transmission", &material.transmission, 0.0f, 1.0f);
                changed |= ImGui::DragFloat("Thickness", &material.thickness, 0.005f, 0.0f, 10.0f);
                changed |= color_row("Subsurface Color", material.subsurface_color, false);
                changed |= ImGui::SliderFloat("IOR", &material.ior, 1.0f, 3.0f);
                ImGui::TreePop();
            }

            if (ImGui::TreeNode("Rendering"))
            {
                const char* surface_names[] = {"Opaque", "Cutout", "Transparent", "Fade"};
                int surface = static_cast<int>(material.surface_type);
                if (ImGui::Combo("Surface Type", &surface, surface_names, 4))
                {
                    material.surface_type = static_cast<SurfaceType>(surface);
                    changed = true;
                }
                if (material.surface_type == SurfaceType::Cutout)
                    changed |= ImGui::SliderFloat("Alpha Cutoff", &material.alpha_cutoff, 0.0f,
                                                  1.0f);

                const char* cull_names[] = {"Off (double-sided)", "Front", "Back"};
                int cull = static_cast<int>(material.cull_mode);
                if (ImGui::Combo("Cull Mode", &cull, cull_names, 3))
                {
                    material.cull_mode = static_cast<MaterialCullMode>(cull);
                    changed = true;
                }

                const char* blend_names[] = {"Alpha", "Premultiplied", "Additive", "Multiply"};
                int blend = static_cast<int>(material.blend_mode);
                if (ImGui::Combo("Blend Mode", &blend, blend_names, 4))
                {
                    material.blend_mode = static_cast<BlendMode>(blend);
                    changed = true;
                }

                int queue = material.render_queue;
                if (ImGui::DragInt("Render Queue", &queue, 1.0f, 0, 5000))
                {
                    material.render_queue = queue;
                    changed = true;
                }

                changed |= ImGui::Checkbox("Cast Shadows", &material.cast_shadows);
                ImGui::SameLine();
                changed |= ImGui::Checkbox("Receive Shadows", &material.receive_shadows);
                changed |= ImGui::Checkbox("GPU Instancing", &material.gpu_instancing);

                const char* wrap_names[] = {"Repeat", "Clamp", "Mirror"};
                int wrap = static_cast<int>(material.wrap_mode);
                if (ImGui::Combo("Wrap Mode", &wrap, wrap_names, 3))
                {
                    material.wrap_mode = static_cast<TextureWrap>(wrap);
                    changed = true;
                }
                changed |= ImGui::SliderFloat("Anisotropic Filtering",
                                              &material.anisotropic_filtering, 1.0f, 16.0f, "%.0f");
                ImGui::TreePop();
            }

            return changed;
        }
    } // namespace Editor
} // namespace SushiEngine
