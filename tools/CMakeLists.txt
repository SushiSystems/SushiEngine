# tools — the programs the build and its maintainers run, as opposed to anything the engine
# ships. Not a tier and not modules: a tool is a plain executable, and one is allowed to reach
# into the source directory of the module it is a tool for, which no module may do to another.

# The shader compiler is the renderer's build step, so it exists exactly where the renderer does
# and needs the same glslang package the option provisions.
if(SUSHIENGINE_BUILD_RENDER)
    add_subdirectory(shader_compiler)
endif()

add_subdirectory(probes)
