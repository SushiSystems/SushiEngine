# sushiengine_shader_compiler — the host tool cmake/Shaders.cmake drives: GLSL in, a C++ header
# of embedded SPIR-V words out, so the renderer ships its shaders inside the binary.

find_package(glslang CONFIG REQUIRED)

add_executable(sushiengine_shader_compiler main.cpp)
# It resolves #include inside a shader with the renderer's own GlslIncluder, so that the
# build-time compile and the renderer's hot-reload path read a shader the same way. That header
# is renderer internals, and reaching it is what makes this the renderer's tool rather than a
# general-purpose one.
target_include_directories(sushiengine_shader_compiler
    PRIVATE "${CMAKE_SOURCE_DIR}/engine/presentation/render/source")
target_link_libraries(sushiengine_shader_compiler PRIVATE
    glslang::glslang
    glslang::glslang-default-resource-limits
    glslang::SPIRV)
# The target carries the repository's prefix so it cannot collide with a SushiRuntime one in the
# single flat CMake namespace the two trees share; the file on disk keeps the short name.
set_target_properties(sushiengine_shader_compiler PROPERTIES
    OUTPUT_NAME shader_compiler
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
