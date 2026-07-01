# SyclTarget — the single source of truth for turning a plain C++ target into a
# SYCL one against the runtime. The engine is header-only, so its kernels
# instantiate inside the consuming translation unit (an example app or the test
# binary); every such target needs the same three things, so they live here once
# instead of being copy-pasted across the root and tests/ (DRY / SRP).

# sushi_apply_sycl(<target> [SOURCES <src>...])
#   Applies the SYCL device-compilation policy to <target>:
#     * the toolchain's add_sycl_to_target() (pass SOURCES so AdaptiveCpp/oneAPI
#       can find the device code; harmless where it is a no-op stub),
#     * an explicit -fsycl on the intel-llvm path, where add_sycl_to_target() is a
#       no-op stub and the flag must be added to the whole target ourselves,
#     * on Windows, the runtime-DLL copy next to the executable.
function(sushi_apply_sycl target)
    cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})

    if(ARG_SOURCES)
        add_sycl_to_target(TARGET ${target} SOURCES ${ARG_SOURCES})
    else()
        add_sycl_to_target(TARGET ${target})
    endif()

    if(SR_SYCL_TOOLCHAIN STREQUAL "intel-llvm")
        target_compile_options(${target} PRIVATE -fsycl)
        target_link_options(${target} PRIVATE -fsycl)
    endif()

    if(WIN32)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:sushiruntime>
                $<TARGET_FILE_DIR:${target}>)
    endif()
endfunction()

# add_sushi_sycl_executable(<name> <source>)
#   A SushiEngine example/app: one SYCL translation unit linking the engine, C++17,
#   with the shared SYCL policy applied. Requires the SushiEngine target to exist.
function(add_sushi_sycl_executable name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE SushiEngine)
    set_target_properties(${name} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
    sushi_apply_sycl(${name} SOURCES ${source})
endfunction()
