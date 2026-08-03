# platform — the window/lifecycle seam (IPlatformWindow) and its SDL2 implementation,
# so a shell can open a window without linking a renderer or ImGui. Vulkan-neutral by
# design: native OS and library types cross the seam as opaque handles, and the Vulkan
# headers and loader are here only for the surface-creation calls.

find_package(SDL2 CONFIG REQUIRED)
find_package(VulkanHeaders CONFIG REQUIRED)
find_package(VulkanLoader CONFIG REQUIRED)

sushiengine_add_module(NAME platform LAYER foundation
    SOURCES
        source/sdl_window.cpp
        source/user_data_directory.cpp
    EXTERNAL_PUBLIC SDL2::SDL2 Vulkan::Headers Vulkan::Loader)

# Its headers are spelled by bare file name, so the directory holding them is public too.
target_include_directories(sushiengine_platform
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/SushiEngine/platform")

# The name the editor and the player link.
add_library(sushi_platform ALIAS sushiengine_platform)
