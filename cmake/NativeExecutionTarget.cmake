# NativeExecutionTarget — the native-lane counterpart to cmake/SyclTarget.cmake.
#
# Defines what add_sushi_sycl_executable()/sushi_apply_sycl() define for the
# runtime backend: here, a SushiEngine example built with a stock C++17
# compiler — no SYCL, no SushiRuntime subproject. Only included when
# SE_EXECUTION_BACKEND is "native" (see the root CMakeLists.txt), so the two
# backends' target-creation helpers never collide.

# add_sushiengine_native_executable(<name> <source>)
#   A SushiEngine example/app built against the native Execution backend: one
#   plain C++17 translation unit linking the engine, with no device-compilation
#   policy applied — there is none, by construction, on this lane.
function(add_sushiengine_native_executable name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE SushiEngine)
    set_target_properties(${name} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
endfunction()

add_subdirectory(execution_native)
