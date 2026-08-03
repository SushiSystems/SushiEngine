# shader_compiler — the host tool cmake/Shaders.cmake drives: GLSL in, a C++ header of embedded
# SPIR-V words out, so the renderer ships its shaders inside the binary.

find_package(glslang CONFIG REQUIRED)

add_executable(shader_compiler main.cpp)
# It resolves #include inside a shader with the renderer's own GlslIncluder, so that the
# build-time compile and the renderer's hot-reload path read a shader the same way. That header
# is renderer internals, and reaching it is what makes this the renderer's tool rather than a
# general-purpose one.
target_include_directories(shader_compiler
    PRIVATE "${CMAKE_SOURCE_DIR}/engine/presentation/render/source")
target_link_libraries(shader_compiler PRIVATE
    glslang::glslang
    glslang::glslang-default-resource-limits
    glslang::SPIRV)
set_target_properties(shader_compiler PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
