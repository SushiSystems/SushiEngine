/**************************************************************************/
/* glsl_includer.hpp                                                      */
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
 * @file glsl_includer.hpp
 * @brief Resolves `#include` in GLSL against one shader directory.
 *
 * glslang ships no installable includer, so both users of it — the build-time
 * shader_compiler tool and the runtime hot-reload path — share this one. Both
 * resolve headers from the same flat shader directory, which is why a single
 * directory is all it needs to carry.
 */

#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include <glslang/Public/ShaderLang.h>

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief A glslang includer that reads headers from one directory.
         *
         * Local and system includes resolve identically: the shader tree is flat, so
         * `#include "x.glsl"` and `#include <x.glsl>` mean the same file.
         */
        class GlslIncluder final : public glslang::TShader::Includer
        {
            public:
                /**
                 * @brief Binds the includer to the directory headers are read from.
                 * @param directory Directory containing the shader sources.
                 */
                explicit GlslIncluder(std::string directory) : directory_(std::move(directory)) {}

                /**
                 * @brief Resolves a quoted include.
                 * @param header   The header name as written in the shader.
                 * @param includer The including file's name; unused, the tree is flat.
                 * @param depth    Current include depth; unused.
                 * @return The loaded result, or an empty result if unreadable.
                 */
                IncludeResult* includeLocal(const char* header, const char* includer,
                                            std::size_t depth) override
                {
                    (void)includer;
                    (void)depth;
                    return load(header);
                }

                /**
                 * @brief Resolves an angle-bracket include, identically to includeLocal().
                 * @param header   The header name as written in the shader.
                 * @param includer The including file's name; unused.
                 * @param depth    Current include depth; unused.
                 * @return The loaded result, or an empty result if unreadable.
                 */
                IncludeResult* includeSystem(const char* header, const char* includer,
                                             std::size_t depth) override
                {
                    (void)includer;
                    (void)depth;
                    return load(header);
                }

                /**
                 * @brief Frees a result previously returned by this includer.
                 * @param result The result to release; may be nullptr.
                 */
                void releaseInclude(IncludeResult* result) override
                {
                    if (result == nullptr)
                        return;
                    delete static_cast<std::string*>(result->userData);
                    delete result;
                }

            private:
                /**
                 * @brief Reads one header from the shader directory.
                 * @param header The header file name.
                 * @return A result owning the file text, empty when unreadable.
                 */
                IncludeResult* load(const char* header)
                {
                    const std::string path = directory_ + "/" + header;
                    std::ifstream stream(path, std::ios::binary);
                    if (!stream)
                        return new IncludeResult(path, "", 0, nullptr);
                    std::ostringstream buffer;
                    buffer << stream.rdbuf();
                    std::string* text = new std::string(buffer.str());
                    return new IncludeResult(path, text->c_str(), text->size(), text);
                }

                std::string directory_;
        };
    } // namespace Render
} // namespace SushiEngine
