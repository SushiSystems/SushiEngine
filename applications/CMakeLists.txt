# applications — the programs the engine ships as products rather than as a library: the
# authoring shell and the runtime shell. Each is off by default and guards itself here, so the
# root file names this directory once and never its contents.

# The Unity-style ImGui editor shell. It links sushiengine_simulation (the runtime-backed
# world), so its final link is SYCL-aware — the plain stock-toolchain lane is preserved by the
# sandbox and sushiengine_render_probe, not the editor.
if(SUSHIENGINE_BUILD_EDITOR)
    add_subdirectory(editor)
endif()

# The ImGui-free player shell (PLATFORM0 S5). It links the same renderer and the same world as
# the editor and never sushiengine_imgui, which is the whole point of the split (see
# player/CMakeLists.txt).
if(SUSHIENGINE_BUILD_PLAYER)
    add_subdirectory(player)
endif()
