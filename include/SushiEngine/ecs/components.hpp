/**************************************************************************/
/* components.hpp                                                         */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/* Licensed under the Apache License, Version 2.0.                         */
/**************************************************************************/

#pragma once

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    /**
     * @brief Per-entity components for the Milestone A slice.
     *
     * The store is structure-of-arrays: each component is one contiguous field in
     * World, not a member of a per-entity struct. These aliases name the element
     * type of each field so the rest of the engine reads by intent.
     */

    /** @brief World-space position of an entity. */
    using Position = Vec3;

    /** @brief Per-step velocity of an entity. */
    using Velocity = Vec3;
} // namespace SushiEngine
