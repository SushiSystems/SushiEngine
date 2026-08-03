# ============================================================================
# sushi_exec_native
#
# The native (non-SYCL) SushiEngine::Execution backend (RUNTIME-PORT1): a
# thread-pool+DAG implementation of the seam SushiRuntime's backend also
# satisfies, so SushiEngine::Execution has a second, portable implementation.
# The public headers live under include/SushiEngine/execution/ alongside the
# vocabulary and the runtime backend, exactly as that backend's headers do;
# this target supplies the two symbols that must be compiled rather than
# header-only (a thread pool owning real OS threads, and the DAG compiler's
# execution loop) — see execution/backend/native_backend.hpp's own file
# comment for why the rest is header-only.
# ============================================================================

add_library(sushi_exec_native STATIC
    thread_pool.cpp
    dag_compiler.cpp)
target_include_directories(sushi_exec_native PUBLIC "${CMAKE_SOURCE_DIR}/include")
find_package(Threads REQUIRED)
target_link_libraries(sushi_exec_native PUBLIC Threads::Threads)
set_target_properties(sushi_exec_native PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
