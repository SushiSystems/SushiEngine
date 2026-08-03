# samples — the worked examples, one directory per subject. Every one of them is headless and
# self-checking: it exits 0 when the thing it demonstrates still holds, which is what makes a
# sample runnable in CI rather than only readable.
#
# sandbox and pgs_demo sit outside SE_BUILD_EXAMPLES and exist on both execution lanes, because
# they are the only two targets that prove the native lane builds at all — a plain configure has
# to keep producing them. Everything else here is SYCL/SushiRuntime-aware and runtime-lane only.

# Declares one sample against whichever execution lane this configure is for. The two lanes have
# different executable helpers and this is the only tree that needs both, so the choice is made
# once here instead of in every group directory.
function(sushiengine_add_sample name source)
    if(SE_EXECUTION_BACKEND STREQUAL "runtime")
        add_sushi_sycl_executable(${name} ${source})
    else()
        add_sushiengine_native_executable(${name} ${source})
    endif()
endfunction()

# Read by the group directories below: true when this configure builds the demos beyond the two
# lane-proving ones.
set(SUSHIENGINE_BUILD_SAMPLE_DEMOS OFF)
if(SE_BUILD_EXAMPLES AND SE_EXECUTION_BACKEND STREQUAL "runtime")
    set(SUSHIENGINE_BUILD_SAMPLE_DEMOS ON)
endif()

# Always added: sandbox owns one of the two lane-proving targets and physics owns the other,
# plus the sample scene's builder, which the test suite links whatever SE_BUILD_EXAMPLES says.
add_subdirectory(sandbox)
add_subdirectory(physics)

if(SUSHIENGINE_BUILD_SAMPLE_DEMOS)
    add_subdirectory(animation)
    add_subdirectory(audio)
    add_subdirectory(authoring)
    add_subdirectory(networking)
    add_subdirectory(rendering)
endif()
