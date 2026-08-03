# execution — the Context/Graph/Buffer seam the engine schedules work through, and the
# backends that satisfy it. The seam is a compile-time policy rather than a virtual
# interface (see cmake/ProjectOptions.cmake), so both backends are header-only except
# for the two native symbols below.
sushiengine_add_module(NAME execution LAYER foundation)

# What the native backend cannot express in a header: a thread pool owning real OS
# threads, and the DAG compiler's execution loop. Built only when this configure selects
# that backend — the runtime backend's implementation lives in SushiRuntime.
if(SE_EXECUTION_BACKEND STREQUAL "native")
    find_package(Threads REQUIRED)

    add_library(sushi_exec_native STATIC
        source/backend/native/thread_pool.cpp
        source/backend/native/dag_compiler.cpp)
    target_link_libraries(sushi_exec_native PUBLIC sushiengine_execution Threads::Threads)
    set_target_properties(sushi_exec_native PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
endif()
