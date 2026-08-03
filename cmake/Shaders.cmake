# Shaders — the build-time GLSL to SPIR-V step. A host tool compiles each shader into a C++
# header of embedded SPIR-V words, so a renderer ships its shaders inside the binary and reads
# no .glsl at runtime. The module that owns the shaders passes its own directory in, so nothing
# here has to know where in the tree they sit.

# The shared GLSL pulled in by #include, as names relative to the shader directory. Every
# compiled shader depends on all of it, because the compiler resolves the includes itself and
# cannot report back what it read — so a name missing from this list is a shader that silently
# stops rebuilding when its include changes.
set(SUSHIENGINE_SHADER_COMMON_SOURCES
    atmosphere_common.glsl
    atmosphere_nest_common.glsl
    blue_noise.glsl
    cloud_field_window.glsl
    cloud_noise_common.glsl
    cloud_shadow_common.glsl
    clustered_lighting.glsl
    clustered_lighting_common.glsl
    gi_common.glsl
    ibl_common.glsl
    particle_common.glsl
    particle_shadow.glsl
    pbr_common.glsl
    punctual_shadow_common.glsl
    scene_weather_tail.glsl
    sdf_common.glsl
    shadow_common.glsl
    shadow_sampling.glsl
    synoptic_field.glsl
    temporal_common.glsl
    terrain_common.glsl)

# sushiengine_configure_shaders(<shader_directory>)
#   Settles what every sushiengine_compile_shader() call made afterwards from the same directory
#   depends on and where it writes: SUSHIENGINE_SHADER_COMMON_SOURCES resolved against
#   <shader_directory>, and a generated/ directory under the caller's build directory, published
#   back as SUSHIENGINE_SHADER_GENERATED_DIRECTORY for the caller to put on its include path.
function(sushiengine_configure_shaders shader_directory)
    set(generated_directory "${CMAKE_CURRENT_BINARY_DIR}/generated")
    file(MAKE_DIRECTORY "${generated_directory}")
    set(SUSHIENGINE_SHADER_GENERATED_DIRECTORY "${generated_directory}" PARENT_SCOPE)

    set(dependencies "")
    foreach(name IN LISTS SUSHIENGINE_SHADER_COMMON_SOURCES)
        list(APPEND dependencies "${shader_directory}/${name}")
    endforeach()
    set(SUSHIENGINE_SHADER_DEPENDENCIES "${dependencies}" PARENT_SCOPE)
endfunction()

# sushiengine_compile_shader(<vert|frag|comp|task|mesh> <input> <symbol> <output_variable>)
#   Emits the rule that compiles <input> into <generated>/<input-name>.h exposing
#   SushiEngine::Render::Shaders::<symbol>, and sets <output_variable> to that header so the
#   consuming target can list it as a source — which is what puts the rule in its build graph.
function(sushiengine_compile_shader stage input symbol output_variable)
    get_filename_component(name "${input}" NAME)
    set(output "${SUSHIENGINE_SHADER_GENERATED_DIRECTORY}/${name}.h")
    add_custom_command(
        OUTPUT "${output}"
        COMMAND sushiengine_shader_compiler ${stage} "${input}" "${output}" ${symbol}
        DEPENDS sushiengine_shader_compiler "${input}" ${SUSHIENGINE_SHADER_DEPENDENCIES}
        COMMENT "Compiling shader ${name} -> SPIR-V header"
        VERBATIM)
    set(${output_variable} "${output}" PARENT_SCOPE)
endfunction()
