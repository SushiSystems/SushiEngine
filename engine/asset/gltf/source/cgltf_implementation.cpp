/**************************************************************************/
/* cgltf_implementation.cpp                                               */
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

// cgltf is a single-header library: exactly one translation unit in the whole
// program may define its implementation, and this is that unit. It used to be
// render/material/stb_impl.cpp, alongside stb_image and stb_truetype — which made
// the glTF parser a private of the renderer, and meant a target that only wanted to
// read a skeleton had to link a Vulkan library to get `cgltf_parse_file`.
//
// It lives here because the importers do, and they link nothing. Nothing else in the
// engine may define CGLTF_IMPLEMENTATION; the renderer includes the header for its
// types and takes the symbols from this target.

#define CGLTF_IMPLEMENTATION
#include <cgltf.h>
