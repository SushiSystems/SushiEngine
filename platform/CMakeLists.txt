# ============================================================================
# sushi_platform
#
# The window/lifecycle seam (`IPlatformWindow`) and its SDL2/Vulkan
# implementation (`SdlWindow`), extracted from the editor tree so a future
# `se_player` can open a window without linking ImGui. Vulkan-neutral by
# design: native OS/library types cross the seam as opaque handles. Needs no
# renderer — only the Vulkan headers/loader for the surface-creation calls.
# ============================================================================

find_package(SDL2 CONFIG REQUIRED)
find_package(VulkanHeaders CONFIG REQUIRED)
find_package(VulkanLoader CONFIG REQUIRED)

add_library(sushi_platform STATIC sdl_window.cpp user_data_directory.cpp)
target_include_directories(sushi_platform PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(sushi_platform PUBLIC SDL2::SDL2 Vulkan::Headers Vulkan::Loader)
set_target_properties(sushi_platform PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
