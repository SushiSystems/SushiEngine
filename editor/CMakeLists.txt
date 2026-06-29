# ============================================================================
# SushiEngine editor
#
# A Unity-style editor shell: an SDL2 + OpenGL window hosting Dear ImGui (docking
# branch, vendored at third_party/imgui). This first cut is a bare dockspace and
# placeholder panel; later steps wire in a World/Schedule, panels, and a viewport.
#
# OS-agnostic and CMake-clean: SDL2 and OpenGL come from the platform's package
# manager (vcpkg on Windows, the system on Linux/macOS) via find_package; ImGui is
# vendored as a submodule and compiled here, the idiomatic way to ship it. The
# editor is NOT a SYCL translation unit yet, so it neither links the runtime nor
# needs a SYCL compiler — keeping this lane buildable on a stock toolchain.
# ============================================================================

find_package(SDL2 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/third_party/imgui")
if(NOT EXISTS "${IMGUI_DIR}/imgui.cpp")
    message(FATAL_ERROR
        "Dear ImGui sources not found at ${IMGUI_DIR}. "
        "Initialize submodules: git submodule update --init --recursive")
endif()

# ImGui core plus the SDL2 platform backend and the OpenGL3 renderer backend. The
# GL3 backend carries its own loader, so no GLEW/glad is required.
add_library(sushi_imgui STATIC
    "${IMGUI_DIR}/imgui.cpp"
    "${IMGUI_DIR}/imgui_draw.cpp"
    "${IMGUI_DIR}/imgui_tables.cpp"
    "${IMGUI_DIR}/imgui_widgets.cpp"
    "${IMGUI_DIR}/imgui_demo.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp")
target_include_directories(sushi_imgui PUBLIC
    "${IMGUI_DIR}"
    "${IMGUI_DIR}/backends")
target_link_libraries(sushi_imgui PUBLIC SDL2::SDL2 OpenGL::GL)
target_compile_features(sushi_imgui PUBLIC cxx_std_17)

add_executable(se_editor main.cpp)
target_link_libraries(se_editor PRIVATE sushi_imgui)
# SDL2 defines SDL_main on some platforms (Windows); link the entry shim when the
# target exists so WinMain resolves without forcing it on Linux/macOS.
if(TARGET SDL2::SDL2main)
    target_link_libraries(se_editor PRIVATE SDL2::SDL2main)
endif()
set_target_properties(se_editor PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

# On Windows the SDL2 runtime DLL must sit next to the executable.
if(WIN32 AND TARGET SDL2::SDL2)
    add_custom_command(TARGET se_editor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>
            $<TARGET_FILE_DIR:se_editor>)
endif()
