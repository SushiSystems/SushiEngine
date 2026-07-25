/**************************************************************************/
/* authoring.hpp                                                         */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#ifndef SUSHIENGINE_AUDIO_AUTHORING_HPP
#define SUSHIENGINE_AUDIO_AUTHORING_HPP

/**
 * @file authoring.hpp
 * @brief The audio authoring project — the mutable side of the sound-designer's DAW.
 *
 * The runtime @ref EventDatabase is a *baked*, flattened, immutable structure: children live
 * in contiguous ranges with no pointers, ideal to load and walk but impossible to edit. This is
 * its authoring-time counterpart: a **mutable tree** of media, container nodes (Sound / Random /
 * Sequence / Blend / Switch / Layer), and named events that a tool (or the editor's authoring
 * panel) grows, reorders, and re-parents freely. @ref AudioAuthoringProject::flatten lays the
 * tree back into the runtime's contiguous form (a breadth-first pass that places every node's
 * children as one block), and @ref bake writes a complete @ref Bank — the whole author→ship
 * pipeline the engine previously lacked.
 *
 * Dependency-free (only the engine's own bank/event headers), so it rides the `audio.hpp`
 * umbrella and is testable headlessly; the ImGui authoring panel is a thin view over it.
 */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <SushiEngine/audio/bank.hpp>
#include <SushiEngine/audio/event.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief A media asset in the authoring project (one encoded sound). */
        struct AuthoredMedia
        {
            std::uint32_t id = 0;
            std::string name;
            AudioCodecKind codec = AudioCodecKind::PcmFloat;
            std::uint32_t channels = 1;
            std::uint32_t sample_rate = 48000;
            std::uint32_t frame_count = 0;
        };

        /** @brief A mutable container-tree node (branch or Sound leaf). */
        struct AuthoredNode
        {
            ContainerKind kind = ContainerKind::Sound;
            std::uint32_t media_id = INVALID_SOUND; /**< For a Sound leaf. */
            float weight = 1.0f;                    /**< Random weight / Layer gain. */
            std::vector<int> children;              /**< Indices into the project's node pool. */
        };

        /** @brief A named event rooting a container tree. */
        struct AuthoredEvent
        {
            std::string name;
            EventId id = 0;
            int root = -1; /**< Index into the node pool. */
        };

        /** @brief A mixer bus in the authoring project (routing + trim). */
        struct AuthoredBus
        {
            std::string name;
            int parent = -1;     /**< Index of the parent bus, or -1 for master. */
            float gain_db = 0.0f;
        };

        /**
         * @brief The editable audio project: media, a container-node pool, events, and buses.
         *
         * A tool builds it with the add/create helpers, edits nodes in place, then @ref flatten
         * or @ref bake. Node handles are stable indices into @ref nodes.
         */
        class AudioAuthoringProject
        {
            public:
                /** @brief Registers a media asset; returns its id. */
                std::uint32_t add_media(const std::string& name, AudioCodecKind codec,
                                        std::uint32_t channels, std::uint32_t sample_rate,
                                        std::uint32_t frame_count)
                {
                    AuthoredMedia m;
                    m.id = next_media_id_++;
                    m.name = name;
                    m.codec = codec;
                    m.channels = channels;
                    m.sample_rate = sample_rate;
                    m.frame_count = frame_count;
                    media_.push_back(m);
                    return m.id;
                }

                /** @brief Creates a Sound leaf for a media id; returns its node handle. */
                int create_sound(std::uint32_t media_id)
                {
                    AuthoredNode n;
                    n.kind = ContainerKind::Sound;
                    n.media_id = media_id;
                    nodes_.push_back(n);
                    return static_cast<int>(nodes_.size() - 1);
                }

                /** @brief Creates an (initially childless) branch container; returns its handle. */
                int create_container(ContainerKind kind)
                {
                    AuthoredNode n;
                    n.kind = kind;
                    nodes_.push_back(n);
                    return static_cast<int>(nodes_.size() - 1);
                }

                /** @brief Appends @p child to @p parent's child list. */
                void add_child(int parent, int child)
                {
                    if (valid_node(parent) && valid_node(child))
                        nodes_[static_cast<std::size_t>(parent)].children.push_back(child);
                }

                /** @brief Sets a node's Random weight / Layer gain. */
                void set_weight(int node, float weight)
                {
                    if (valid_node(node))
                        nodes_[static_cast<std::size_t>(node)].weight = weight;
                }

                /** @brief Creates a named event rooted at @p root; returns its event id. */
                EventId create_event(const std::string& name, int root)
                {
                    AuthoredEvent e;
                    e.name = name;
                    e.id = hash_name(name);
                    e.root = root;
                    events_.push_back(e);
                    return e.id;
                }

                /** @brief Adds a mixer bus; returns its index. */
                int add_bus(const std::string& name, int parent, float gain_db)
                {
                    buses_.push_back(AuthoredBus{name, parent, gain_db});
                    return static_cast<int>(buses_.size() - 1);
                }

                std::vector<AuthoredMedia>& media() noexcept { return media_; }
                const std::vector<AuthoredMedia>& media() const noexcept { return media_; }
                std::vector<AuthoredNode>& nodes() noexcept { return nodes_; }
                const std::vector<AuthoredNode>& nodes() const noexcept { return nodes_; }
                std::vector<AuthoredEvent>& events() noexcept { return events_; }
                const std::vector<AuthoredEvent>& events() const noexcept { return events_; }
                std::vector<AuthoredBus>& buses() noexcept { return buses_; }
                const std::vector<AuthoredBus>& buses() const noexcept { return buses_; }

                /**
                 * @brief Flattens the mutable tree into a runtime @ref EventDatabase.
                 *
                 * Breadth-first from every event root, allocating each node an output index and
                 * placing its children in one contiguous block, so the result satisfies the
                 * database's contiguous-child-range invariant. Shared subtrees are duplicated per
                 * reference (the flattened form is a forest of trees, not a DAG).
                 *
                 * @param out The database to fill (cleared by @ref EventDatabase's own assign).
                 * @return True if every event had a valid root.
                 */
                bool flatten(EventDatabase& out) const
                {
                    for (const AuthoredEvent& ev : events_)
                        if (!valid_node(ev.root))
                            return false;

                    std::vector<ContainerNode> flat;
                    std::vector<int> authored_of; // authored node index per emitted node
                    std::vector<std::uint32_t> event_roots;
                    event_roots.reserve(events_.size());

                    for (const AuthoredEvent& ev : events_)
                    {
                        const std::uint32_t root_index = static_cast<std::uint32_t>(flat.size());
                        event_roots.push_back(root_index);
                        emit_node(flat, authored_of, ev.root);
                        // Breadth-first over this event's own emitted range; each parent lays its
                        // children down as one contiguous block before the cursor advances to them.
                        for (std::uint32_t cur = root_index; cur < flat.size(); ++cur)
                        {
                            const AuthoredNode& node =
                                nodes_[static_cast<std::size_t>(authored_of[cur])];
                            flat[cur].first_child = static_cast<std::uint32_t>(flat.size());
                            flat[cur].child_count = static_cast<std::uint32_t>(node.children.size());
                            for (int c : node.children)
                            {
                                if (!valid_node(c))
                                    return false;
                                emit_node(flat, authored_of, c);
                            }
                        }
                    }

                    std::vector<EventDef> defs;
                    defs.reserve(events_.size());
                    for (std::size_t i = 0; i < events_.size(); ++i)
                        defs.push_back(EventDef{events_[i].id, event_roots[i]});
                    out.assign(std::move(flat), std::move(defs));
                    return true;
                }

                /**
                 * @brief Bakes a complete bank: media (bytes from @p encoded) + flattened events.
                 * @param builder  The bank builder to populate.
                 * @param encoded  Supplies each media id's already-encoded bytes.
                 * @return True on success.
                 */
                bool bake(BankBuilder& builder,
                          const std::function<const std::vector<std::uint8_t>&(std::uint32_t)>& encoded) const
                {
                    for (const AuthoredMedia& m : media_)
                    {
                        const std::vector<std::uint8_t>& bytes = encoded(m.id);
                        builder.add_media(m.id, m.codec, m.channels, m.sample_rate, m.frame_count,
                                          bytes);
                    }
                    EventDatabase db;
                    if (!flatten(db))
                        return false;
                    builder.set_events(db);
                    return true;
                }

                /** @brief FNV-1a hash of an event name to its stable id. */
                static EventId hash_name(const std::string& name) noexcept
                {
                    std::uint64_t h = 1469598103934665603ull;
                    for (char c : name)
                    {
                        h ^= static_cast<std::uint8_t>(c);
                        h *= 1099511628211ull;
                    }
                    return static_cast<EventId>(h);
                }

            private:
                bool valid_node(int i) const noexcept
                {
                    return i >= 0 && static_cast<std::size_t>(i) < nodes_.size();
                }

                // Appends one flattened node (child range filled later) and records its authored src.
                void emit_node(std::vector<ContainerNode>& flat, std::vector<int>& authored_of,
                               int authored) const
                {
                    const AuthoredNode& n = nodes_[static_cast<std::size_t>(authored)];
                    ContainerNode c;
                    c.kind = n.kind;
                    c.sound = n.media_id;
                    c.weight = n.weight;
                    c.first_child = 0;
                    c.child_count = 0;
                    flat.push_back(c);
                    authored_of.push_back(authored);
                }

                std::vector<AuthoredMedia> media_;
                std::vector<AuthoredNode> nodes_;
                std::vector<AuthoredEvent> events_;
                std::vector<AuthoredBus> buses_;
                std::uint32_t next_media_id_ = 1;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
