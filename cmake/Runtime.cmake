# Runtime — pull SushiRuntime in as an in-tree component (the battery). This
# provides the `sushiruntime` target and defines add_sycl_to_target(), which the
# SYCL policy in SyclTarget applies to the engine's translation units. It is also
# where the runtime's own CMake selects the SYCL toolchain and sets the cache
# variable SR_SYCL_TOOLCHAIN that SyclTarget keys off.
#
# Inside a SushiStack workspace every module is cloned beside the others, so the
# runtime sibling is <workspace>/sushiruntime. Prefer SUSHISTACK_HOME when set
# (the installer exports it); otherwise the in-tree sibling ../sushiruntime,
# which is the same path in a workspace and the right standalone default.
if(DEFINED ENV{SUSHISTACK_HOME})
    set(_SE_DEFAULT_RUNTIME "$ENV{SUSHISTACK_HOME}/sushiruntime")
else()
    set(_SE_DEFAULT_RUNTIME "${CMAKE_CURRENT_SOURCE_DIR}/../sushiruntime")
endif()
set(SUSHIRUNTIME_DIR "${_SE_DEFAULT_RUNTIME}"
    CACHE PATH "Path to the SushiRuntime sibling checkout")

add_subdirectory("${SUSHIRUNTIME_DIR}" "${CMAKE_BINARY_DIR}/sushiruntime")
