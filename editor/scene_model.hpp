/**************************************************************************/
/* scene_model.hpp                                                        */
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

#ifndef SUSHIENGINE_EDITOR_SCENE_MODEL_HPP
#define SUSHIENGINE_EDITOR_SCENE_MODEL_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sushi::editor
{
    /**
     * @brief A local translation/rotation/scale triple for a scene node.
     *
     * This is the editor's own presentation model, deliberately decoupled from any
     * runtime component layout: the editor shell is not a SYCL translation unit and
     * does not yet link the ECS, so it edits plain float triples the inspector can
     * bind directly. When the World is wired in, these map onto real components.
     */
    struct Transform
    {
        float position[3] = {0.0f, 0.0f, 0.0f};
        float rotation[3] = {0.0f, 0.0f, 0.0f};
        float scale[3] = {1.0f, 1.0f, 1.0f};
    };

    /**
     * @brief One node in the editor's scene hierarchy tree.
     *
     * Owns its children; a raw @ref parent back-pointer supports reparenting and
     * upward walks. Identity is a process-stable id assigned by @ref Scene, so the
     * selection and inspector can refer to a node without holding a pointer that a
     * tree edit might invalidate.
     */
    struct SceneNode
    {
        std::uint64_t id = 0;
        std::string name;
        bool visible = true;
        Transform transform;
        SceneNode* parent = nullptr;
        std::vector<std::unique_ptr<SceneNode>> children;
    };

    /**
     * @brief The editor-side scene: a forest of @ref SceneNode roots.
     *
     * Provides the create/rename/reparent/remove operations the hierarchy panel
     * drives, plus id-based lookup used by the inspector. All mutation goes through
     * here so ids stay unique and parent pointers stay consistent.
     */
    class Scene
    {
    public:
        /**
         * @brief Create a node and attach it under @p parent (or as a root).
         * @param name Display name for the new node.
         * @param parent Parent node, or nullptr to create a top-level root.
         * @return Non-owning pointer to the created node, owned by the tree.
         */
        SceneNode* create_node(const std::string& name, SceneNode* parent = nullptr)
        {
            auto node = std::make_unique<SceneNode>();
            node->id = next_id_++;
            node->name = name;
            node->parent = parent;
            SceneNode* raw = node.get();
            child_vector(parent).push_back(std::move(node));
            return raw;
        }

        /**
         * @brief Detach and destroy @p node together with its whole subtree.
         * @param node Node to remove; ignored if nullptr.
         */
        void remove_node(SceneNode* node)
        {
            if (node == nullptr)
                return;

            auto& siblings = child_vector(node->parent);
            for (auto it = siblings.begin(); it != siblings.end(); ++it)
            {
                if (it->get() == node)
                {
                    siblings.erase(it);
                    return;
                }
            }
        }

        /**
         * @brief Move @p node under @p new_parent, preserving its subtree.
         *
         * A no-op if the move would create a cycle (dropping a node onto itself or
         * one of its own descendants), keeping the tree acyclic.
         *
         * @param node Node to move; ignored if nullptr.
         * @param new_parent Destination parent, or nullptr to promote to a root.
         */
        void reparent(SceneNode* node, SceneNode* new_parent)
        {
            if (node == nullptr || node->parent == new_parent)
                return;
            if (node == new_parent || is_ancestor(node, new_parent))
                return;

            auto& siblings = child_vector(node->parent);
            std::unique_ptr<SceneNode> owned;
            for (auto it = siblings.begin(); it != siblings.end(); ++it)
            {
                if (it->get() == node)
                {
                    owned = std::move(*it);
                    siblings.erase(it);
                    break;
                }
            }
            if (!owned)
                return;

            owned->parent = new_parent;
            child_vector(new_parent).push_back(std::move(owned));
        }

        /**
         * @brief Find a node by its stable id.
         * @param id Identity returned when the node was created.
         * @return The node, or nullptr if no live node carries that id.
         */
        SceneNode* find(std::uint64_t id)
        {
            return find_in(roots_, id);
        }

        /** @brief The top-level root nodes, in display order. */
        std::vector<std::unique_ptr<SceneNode>>& roots() { return roots_; }

    private:
        std::vector<std::unique_ptr<SceneNode>>& child_vector(SceneNode* parent)
        {
            return parent == nullptr ? roots_ : parent->children;
        }

        static bool is_ancestor(SceneNode* node, SceneNode* candidate)
        {
            for (SceneNode* p = candidate; p != nullptr; p = p->parent)
            {
                if (p == node)
                    return true;
            }
            return false;
        }

        static SceneNode* find_in(std::vector<std::unique_ptr<SceneNode>>& nodes,
                                  std::uint64_t id)
        {
            for (auto& node : nodes)
            {
                if (node->id == id)
                    return node.get();
                if (SceneNode* hit = find_in(node->children, id))
                    return hit;
            }
            return nullptr;
        }

        std::vector<std::unique_ptr<SceneNode>> roots_;
        std::uint64_t next_id_ = 1;
    };
}

#endif
