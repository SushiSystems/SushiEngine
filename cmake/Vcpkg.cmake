# Vcpkg — (Windows) locate the vcpkg root and point CMAKE_TOOLCHAIN_FILE at it,
# before project(). SushiEngine is the top-level project, so it reaches project()
# first; the runtime it pulls in needs hwloc from vcpkg on Windows, and resolving
# the toolchain file here — ahead of that subproject — is what lets that resolve.
# No-op off Windows.
if(NOT WIN32)
    return()
endif()

# Prefer an explicit VCPKG_ROOT (env or cache), then the shared workspace vcpkg
# tree, so a direct CMake configure inside a SushiStack workspace finds it without
# the `se` CLI injecting it.
if((NOT DEFINED VCPKG_ROOT OR VCPKG_ROOT STREQUAL "") AND DEFINED ENV{VCPKG_ROOT})
    set(VCPKG_ROOT "$ENV{VCPKG_ROOT}" CACHE PATH "Path to vcpkg root")
endif()
if((NOT DEFINED VCPKG_ROOT OR VCPKG_ROOT STREQUAL "") AND DEFINED ENV{SUSHISTACK_HOME}
   AND EXISTS "$ENV{SUSHISTACK_HOME}/dependencies/vcpkg")
    set(VCPKG_ROOT "$ENV{SUSHISTACK_HOME}/dependencies/vcpkg" CACHE PATH "Path to vcpkg root")
endif()

if(VCPKG_ROOT AND (NOT DEFINED CMAKE_TOOLCHAIN_FILE OR CMAKE_TOOLCHAIN_FILE STREQUAL ""))
    set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE FILEPATH "vcpkg toolchain file")
endif()
