/**************************************************************************/
/* player_app.cpp                                                         */
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

#include "player_app.hpp"

#include <stdexcept>

#include <SDL.h>

#include <SushiEngine/core/types.hpp>

#include "scene_serializer.hpp"
#include "user_data_directory.hpp"

namespace SushiEngine
{
    namespace Player
    {
        namespace
        {
            /**
             * @brief Builds a view/projection from a world camera, exactly as
             *        `applications/editor/source/camera/scene_camera.hpp`'s `WorldCameraSource::view()` does.
             *
             * Not shared with that class: it lives in `editor/`, which this target does not
             * (and must not) link, and the computation itself is four lines — small enough
             * that duplicating it is cheaper and more honest than inventing a shared library
             * for one function two very different hosts happen to need identically.
             */
            Render::CameraView camera_view_from_state(const Simulation::CameraState& state,
                                                       float aspect_ratio)
            {
                Render::CameraView view;
                view.view = look_at(state.position, state.target, state.up);
                view.projection =
                    perspective(state.vertical_fov_radians, aspect_ratio, state.near_plane,
                                state.far_plane);
                view.world_position =
                    WorldVector3{state.position.x, state.position.y, state.position.z};
                view.near_plane = static_cast<float>(state.near_plane);
                view.far_plane = static_cast<float>(state.far_plane);
                return view;
            }

            /**
             * @brief The camera a scene with no active camera is shown from.
             *
             * `RenderScene::has_camera` false means "nothing should be drawn as the game"
             * per its own doc comment; the editor's Game view honours that by showing a
             * placeholder instead of a frame. This skeleton has no placeholder surface to
             * show instead, so it falls back to a fixed default view rather than presenting
             * whatever the swapchain image happened to hold — the same eye/target/up a
             * fresh `WorldCameraSource` defaults to.
             */
            const Simulation::CameraState DEFAULT_CAMERA{
                Vector3{0, 7, 12}, Vector3{0, 0, 0}, Vector3{0, 1, 0}};
        } // namespace

        PlayerApp::PlayerApp() = default;

        PlayerApp::~PlayerApp()
        {
            shutdown();
        }

        void PlayerApp::start(const Description& description)
        {
            description_ = description;

            // Headless (PLATFORM0 S6): no window, no input manager, nothing SDL-owned at
            // all — `window_` staying null is what every other method reads as "headless"
            // from here on, rather than re-testing `description_.headless` at each call site.
            if (!description_.headless)
            {
                window_.reset(new Platform::SDLWindow(description_.window_title.c_str(),
                                                      static_cast<int>(description_.width),
                                                      static_cast<int>(description_.height)));
                window_->add_event_handler([this](const void* event)
                                           { handle_window_event(event); });

                input_.reset(new Input::InputManager());
                input_translator_.reset(new Input::SDLInputTranslator(*input_));
                window_->add_event_handler([this](const void* event)
                                           { input_translator_->handle_native_event(event); });
            }

            simulation_ = Simulation::create_simulation();
            if (!description_.scene_path.empty() &&
                !Scene::load_scene(simulation_->world(), description_.scene_path))
                throw std::runtime_error("SushiEngine::Player::PlayerApp: failed to load scene \"" +
                                         description_.scene_path + "\"");

            create_render_resources();
        }

        void PlayerApp::create_render_resources()
        {
            Render::WindowRendererDescription render_description;
            render_description.enable_validation = description_.enable_validation;
            render_description.width = description_.width;
            render_description.height = description_.height;

            if (window_)
            {
                std::uint32_t width = 0;
                std::uint32_t height = 0;
                window_->drawable_size(width, height);
                render_description.required_instance_extensions =
                    window_->vulkan_instance_extensions();
                render_description.surface_factory = [this](std::uint64_t instance)
                { return window_->create_vulkan_surface(instance); };
                render_description.width = width != 0 ? width : description_.width;
                render_description.height = height != 0 ? height : description_.height;
            }
            // Else headless: render_description.surface_factory stays empty, which is exactly
            // what VulkanWindowRenderer reads (PLATFORM0 S6) as "build no swapchain at all" —
            // the same construction path render_probe/render_golden use.

            // A shipped player's pipeline cache must not write into its own install
            // directory, so a per-user path here is the one thing this skeleton differs
            // from the editor's own (source-tree-relative, correct for hot-reload) default.
            // Left empty (AssetLibrary's own compiled-in-default fallback) if SDL could
            // not resolve a per-user directory, rather than writing a bare relative
            // filename into whatever the process's current directory happens to be.
            const std::string user_data = Platform::user_data_directory(
                description_.organization.c_str(), description_.application.c_str());
            if (!user_data.empty())
                render_description.pipeline_cache_path = user_data + "pipeline_cache.bin";

            renderer_ = Render::create_window_renderer(render_description);
            scene_view_ = renderer_->create_scene_view();
            simulation_->set_atmosphere_mirror(&renderer_->assets());
        }

        void PlayerApp::handle_window_event(const void* native_event)
        {
            const SDL_Event* event = static_cast<const SDL_Event*>(native_event);
            if (event->type != SDL_WINDOWEVENT)
                return;
            if (event->window.event == SDL_WINDOWEVENT_MINIMIZED)
                suspend();
            else if (event->window.event == SDL_WINDOWEVENT_RESTORED)
                resume();
        }

        void PlayerApp::frame(double real_delta_seconds)
        {
            if (window_)
            {
                // Pumped unconditionally, suspended or not: a RESTORED event is how
                // resume() gets called at all, so a suspended host that stopped pumping
                // would never hear about being un-suspended.
                if (!window_->pump_events())
                    quit_requested_ = true;
            }

            if (suspended_)
                return; // no renderer/scene view to draw into until resume() rebuilds one

            if (window_)
            {
                input_->set_gate(Input::InputGate{}); // nothing here ever withholds input
                input_->begin_frame();
            }

            simulation_->tick(static_cast<Scalar>(real_delta_seconds));
            const Simulation::RenderScene& scene = simulation_->render_scene();

            // RenderInstance -> MeshInstance: the one channel that always needs a shape
            // change (Simulation:: and Render:: intentionally do not share this type, the
            // same separation the editor's identical loop is built against). Lights,
            // decals, skinned instances, and cosmetic emitters are already Render::-typed
            // inside RenderScene and pass straight through below with no conversion.
            instances_.clear();
            instances_.reserve(scene.instances.size());
            for (const Simulation::RenderInstance& source : scene.instances)
            {
                Render::MeshInstance instance;
                instance.model = source.model;
                instance.color = source.color;
                instance.id = static_cast<std::uint32_t>(source.id);
                instance.kind = static_cast<Render::MeshKind>(source.shape_kind);
                instance.shape_parameters = source.shape_parameters;
                instance.mesh = source.mesh;
                instance.material = source.material;
                instances_.push_back(instance);
            }

            // Deformable surfaces: pointers into this frame's concatenated vertex/index
            // arrays, offset per surface exactly as the editor's identical loop does.
            deformable_.clear();
            deformable_.reserve(scene.deformable_instances.size());
            for (const Simulation::DeformableInstance& surface : scene.deformable_instances)
            {
                Render::DeformableMeshView view;
                view.vertices = scene.deformable_vertices.data() + surface.first_vertex;
                view.vertex_count = surface.vertex_count;
                view.indices = scene.deformable_indices.data() + surface.first_index;
                view.index_count = surface.index_count;
                view.topology_revision = surface.topology_revision;
                view.color = surface.color;
                view.id = static_cast<std::uint32_t>(surface.id);
                deformable_.push_back(view);
            }

            // Deterministic particles: the CPU-simulated path's already-placed billboards.
            particle_billboards_.clear();
            particle_billboards_.reserve(scene.particle_billboards.size());
            for (const Simulation::ParticleBillboard& particle : scene.particle_billboards)
            {
                Render::ParticleBillboard billboard;
                billboard.position = particle.position;
                billboard.color = particle.color;
                billboard.size = particle.size;
                billboard.alpha = particle.alpha;
                billboard.rotation = particle.rotation;
                particle_billboards_.push_back(billboard);
            }

            std::uint32_t width = description_.width;
            std::uint32_t height = description_.height;
            if (window_)
            {
                window_->drawable_size(width, height);
                if (width == 0 || height == 0)
                    return; // minimized (and no MINIMIZED event fired yet this tick)
            }

            scene_view_->resize(width, height);
            const float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);
            const Render::CameraView camera =
                camera_view_from_state(scene.has_camera ? scene.camera : DEFAULT_CAMERA,
                                       aspect_ratio);

            const Render::ParticleEmitterView* emitters =
                scene.particle_emitters.empty() ? nullptr : scene.particle_emitters.data();
            const Render::SkinnedInstance* skinned =
                scene.skinned_instances.empty() ? nullptr : scene.skinned_instances.data();

            scene_view_->render(camera, scene.environment, instances_.data(), instances_.size(),
                                Render::NO_PICK, deformable_.data(), deformable_.size(),
                                scene.lights.data(), scene.lights.size(), scene.decals.data(),
                                scene.decals.size(), /*show_grid=*/false, skinned,
                                scene.skinned_instances.size(), emitters,
                                scene.particle_emitters.size(), particle_billboards_.data(),
                                particle_billboards_.size());

            // Headless never presents: there is no swapchain image to blit into, and
            // present_scene_view()/end_frame() are both well-defined no-ops on a headless
            // renderer regardless — but calling them from here would still cost the
            // present_source() lookup and a virtual call for nothing, so this skips it
            // outright rather than relying on the callee's own no-op.
            if (window_)
            {
                renderer_->present_scene_view(*scene_view_, scene_view_->current_slot(), width,
                                              height);
                renderer_->end_frame();
            }
        }

        void PlayerApp::suspend()
        {
            if (suspended_)
                return;
            if (renderer_)
                renderer_->wait_idle();
            scene_view_.reset();
            renderer_.reset();
            suspended_ = true;
        }

        void PlayerApp::resume()
        {
            if (!suspended_)
                return;
            create_render_resources();
            suspended_ = false;
        }

        void PlayerApp::shutdown()
        {
            if (renderer_)
                renderer_->wait_idle();
            scene_view_.reset();
            renderer_.reset();
            simulation_.reset();
            input_translator_.reset();
            input_.reset();
            window_.reset();
        }
    } // namespace Player
} // namespace SushiEngine
