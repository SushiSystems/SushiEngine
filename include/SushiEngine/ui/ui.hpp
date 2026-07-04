/**************************************************************************/
/* ui.hpp                                                                */
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
 * @file ui.hpp
 * @brief The `UI` façade: build a canvas of buttons and labels, lay it out, drive it.
 *
 * `UI` is the ergonomic surface a game (or the editor) writes UI against. It owns no
 * world of its own — it operates on an existing `World`, spawning UI elements as
 * ordinary entities carrying the `ui/components.hpp` component set — so UI lives in
 * the same ECS the rest of the game does (the authored choice: UI is part of the
 * world, in the hierarchy, serializable and snapshot-able). Alongside the ECS
 * components the façade keeps a light ordered index (`Element`) of what it created,
 * the same host-side-record pattern the editor uses for names and visibility, giving
 * a deterministic paint and hit-test order without needing the ECS to answer "does
 * this entity have a button".
 *
 * Each frame the host calls `update(screen_size, pointer)`: it resolves every
 * element's `ComputedRect` (parents before children), runs the button state machine
 * and click detection off the pointer, tints button graphics, and fires the
 * registered `on_click` callbacks. `build_draw_list()` then yields a
 * renderer-agnostic `UIDrawList` of coloured rects and text runs — the seam a Vulkan
 * (or any) 2D overlay pass consumes.
 */

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include <SushiEngine/ecs/entity.hpp>
#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/ui/components.hpp>
#include <SushiEngine/ui/interaction.hpp>
#include <SushiEngine/ui/layout.hpp>
#include <SushiEngine/ui/rect.hpp>

namespace SushiEngine
{
    namespace UI
    {
        /** @brief One coloured rectangle to draw, in paint order. */
        struct UIDrawRect
        {
            Rect rect;
            Color color;
        };

        /** @brief One text run to draw: its rectangle, string, size, and colour. */
        struct UITextRun
        {
            Rect rect;
            char text[UI_TEXT_CAPACITY] = {0};
            std::uint32_t length = 0;
            Scalar font_size = 18;
            Color color;
        };

        /**
         * @brief A frame's UI geometry, back to front, for a 2D overlay pass to draw.
         *
         * Renderer-agnostic on purpose: it names no graphics API, so a Vulkan overlay,
         * a test, or a headless tool consume the same list. Rects and texts are each in
         * creation (paint) order, so a later element draws over an earlier one.
         */
        struct UIDrawList
        {
            std::vector<UIDrawRect> rects;
            std::vector<UITextRun> texts;
        };

        /**
         * @brief Builds and drives a retained UI over an existing ECS world.
         *
         * Not copyable: it holds a reference to the world and an index of the entities
         * it created there.
         */
        class UI
        {
            public:
                /**
                 * @brief Binds the façade to the world it will build UI entities in.
                 * @param world The world that stores the UI entities and their components.
                 */
                explicit UI(World& world) noexcept : world_(world) {}

                UI(const UI&) = delete;
                UI& operator=(const UI&) = delete;

                /**
                 * @brief Creates a Canvas root; its rect fills the screen at `update` time.
                 * @param reference_size The resolution the UI is authored at (for future scaling).
                 * @return The canvas entity, to parent top-level elements under.
                 */
                Entity canvas(Vector2 reference_size = Vector2{Scalar(1280), Scalar(720)})
                {
                    const Entity entity = world_.spawn(Canvas{reference_size}, RectTransform{},
                                                       ComputedRect{});
                    Element element;
                    element.entity = entity;
                    element.is_canvas = true;
                    elements_.push_back(element);
                    return entity;
                }

                /**
                 * @brief Creates a solid-colour panel (a background image) under @p parent.
                 * @param parent    The parent element (a canvas or another element).
                 * @param transform The panel's layout.
                 * @param color     Its fill colour.
                 * @return The new panel entity.
                 */
                Entity panel(Entity parent, const RectTransform& transform, const Color& color)
                {
                    return image(parent, transform, color);
                }

                /**
                 * @brief Creates a solid-colour image under @p parent.
                 * @param parent    The parent element.
                 * @param transform The image's layout.
                 * @param color     Its fill colour.
                 * @return The new image entity.
                 */
                Entity image(Entity parent, const RectTransform& transform, const Color& color)
                {
                    const Entity entity = world_.spawn(transform, ComputedRect{}, UIParent{parent},
                                                       UIImage{color});
                    Element element;
                    element.entity = entity;
                    element.parent = parent;
                    element.has_image = true;
                    elements_.push_back(element);
                    return entity;
                }

                /**
                 * @brief Creates a text label under @p parent.
                 * @param parent    The parent element.
                 * @param transform The label's layout.
                 * @param text      The (inline-stored, truncated) string to show.
                 * @param color     The text colour.
                 * @return The new label entity.
                 */
                Entity label(Entity parent, const RectTransform& transform, const char* text,
                             const Color& color = Color{0, 0, 0, 1})
                {
                    UIText label_text;
                    set_text(label_text, text);
                    label_text.color = color;
                    const Entity entity = world_.spawn(transform, ComputedRect{}, UIParent{parent},
                                                       label_text);
                    Element element;
                    element.entity = entity;
                    element.parent = parent;
                    element.has_text = true;
                    elements_.push_back(element);
                    return entity;
                }

                /**
                 * @brief Creates a clickable button (an image, a button, and a centred label).
                 *
                 * The button's own image is the graphic it tints per state; a child label
                 * stretched to fill it shows @p text. @p on_click fires on a completed
                 * click (press and release both inside the button) during `update`.
                 *
                 * @param parent    The parent element.
                 * @param transform The button's layout.
                 * @param text      The button caption.
                 * @param on_click  Called when the button is clicked; may be empty.
                 * @return The new button entity.
                 */
                Entity button(Entity parent, const RectTransform& transform, const char* text,
                              std::function<void()> on_click)
                {
                    UIButton button_component;
                    const Entity entity = world_.spawn(transform, ComputedRect{}, UIParent{parent},
                                                       UIImage{button_component.normal_color},
                                                       button_component);
                    world_.get<UIButton>(entity).target_graphic = entity;

                    Element element;
                    element.entity = entity;
                    element.parent = parent;
                    element.has_image = true;
                    element.has_button = true;
                    elements_.push_back(element);

                    if (on_click)
                        callbacks_.emplace(entity.index, std::move(on_click));

                    if (text != nullptr && text[0] != '\0')
                    {
                        RectTransform label_transform;
                        label_transform.anchor_min = Vector2{0, 0};
                        label_transform.anchor_max = Vector2{1, 1};
                        label_transform.size_delta = Vector2{0, 0};
                        label(entity, label_transform, text, Color{Scalar(0.1), Scalar(0.1),
                                                                    Scalar(0.1), 1});
                    }
                    return entity;
                }

                /**
                 * @brief Resolves layout, runs interaction, and fires click callbacks for a frame.
                 *
                 * @param screen_size The current screen (or panel) size the canvas fills.
                 * @param pointer     The pointer state for this frame.
                 */
                void update(const Vector2& screen_size, const PointerInput& pointer)
                {
                    solve_layout(screen_size);
                    run_interaction(pointer);
                }

                /** @brief The clicks emitted by the most recent `update`. */
                const std::vector<UIClickEvent>& clicks() const noexcept { return clicks_; }

                /** @brief Builds this frame's draw list from the resolved rects and graphics. */
                UIDrawList build_draw_list() const
                {
                    UIDrawList list;
                    for (const Element& element : elements_)
                    {
                        if (!world_.alive(element.entity))
                            continue;
                        const Rect rect = world_.get<ComputedRect>(element.entity).rect;
                        if (element.has_image)
                            list.rects.push_back(
                                UIDrawRect{rect, world_.get<UIImage>(element.entity).color});
                        if (element.has_text)
                        {
                            const UIText& source = world_.get<UIText>(element.entity);
                            UITextRun run;
                            run.rect = rect;
                            run.length = source.length;
                            run.font_size = source.font_size;
                            run.color = source.color;
                            for (std::uint32_t i = 0; i < UI_TEXT_CAPACITY; ++i)
                                run.text[i] = source.text[i];
                            list.texts.push_back(run);
                        }
                    }
                    return list;
                }

                /** @brief The world the UI entities live in. */
                World& world() noexcept { return world_; }

            private:
                /** @brief One created UI entity, with the flags layout/interaction/draw need. */
                struct Element
                {
                    Entity entity;
                    Entity parent;
                    bool is_canvas = false;
                    bool has_image = false;
                    bool has_button = false;
                    bool has_text = false;
                };

                /** @brief Resolves every element's ComputedRect, parents before children. */
                void solve_layout(const Vector2& screen_size)
                {
                    for (const Element& element : elements_)
                    {
                        if (!world_.alive(element.entity))
                            continue;
                        Rect rect;
                        if (element.is_canvas)
                        {
                            rect = resolve_canvas_rect(screen_size, world_.get<Canvas>(element.entity));
                        }
                        else
                        {
                            const Rect parent_rect =
                                world_.get<ComputedRect>(element.parent).rect;
                            rect = resolve_rect(parent_rect,
                                                world_.get<RectTransform>(element.entity));
                        }
                        world_.get<ComputedRect>(element.entity).rect = rect;
                    }
                }

                /** @brief Runs the button state machine and click detection for one pointer frame. */
                void run_interaction(const PointerInput& pointer)
                {
                    clicks_.clear();
                    const bool down_edge = pointer.down && !previous_.down;
                    const bool up_edge = !pointer.down && previous_.down;

                    // The topmost hovered interactable button: later-created elements paint
                    // over earlier ones, so the last match in creation order wins.
                    Entity hovered;
                    for (const Element& element : elements_)
                    {
                        if (!element.has_button || !world_.alive(element.entity))
                            continue;
                        const UIButton& button_component = world_.get<UIButton>(element.entity);
                        if (!button_component.interactable)
                            continue;
                        if (world_.get<ComputedRect>(element.entity).rect.contains(pointer.position))
                            hovered = element.entity;
                    }

                    if (down_edge && !hovered.is_null())
                        pressed_ = hovered;

                    for (const Element& element : elements_)
                    {
                        if (!element.has_button || !world_.alive(element.entity))
                            continue;
                        UIButton& button_component = world_.get<UIButton>(element.entity);
                        const bool is_hovered =
                            !hovered.is_null() && element.entity == hovered;
                        if (!button_component.interactable)
                            button_component.state = ButtonState::Disabled;
                        else if (!pressed_.is_null() && element.entity == pressed_ &&
                                 pointer.down && is_hovered)
                            button_component.state = ButtonState::Pressed;
                        else if (is_hovered)
                            button_component.state = ButtonState::Highlighted;
                        else
                            button_component.state = ButtonState::Normal;

                        if (!button_component.target_graphic.is_null() &&
                            world_.alive(button_component.target_graphic))
                            world_.get<UIImage>(button_component.target_graphic).color =
                                button_state_color(button_component);
                    }

                    if (up_edge)
                    {
                        if (!pressed_.is_null() && !hovered.is_null() && pressed_ == hovered)
                        {
                            clicks_.push_back(UIClickEvent{pressed_});
                            const auto it = callbacks_.find(pressed_.index);
                            if (it != callbacks_.end() && it->second)
                                it->second();
                        }
                        pressed_ = Entity{};
                    }

                    previous_ = pointer;
                }

                World& world_;
                std::vector<Element> elements_;
                std::unordered_map<std::uint32_t, std::function<void()>> callbacks_;
                PointerInput previous_;
                Entity pressed_;
                std::vector<UIClickEvent> clicks_;
        };
    } // namespace UI
} // namespace SushiEngine
