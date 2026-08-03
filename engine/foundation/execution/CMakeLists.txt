# execution — the Context/Graph/Buffer seam the engine schedules work through, and the
# backends that satisfy it. The seam is a compile-time policy rather than a virtual
# interface (see cmake/ProjectOptions.cmake), so both backends are header-only except
# for the two native symbols below.
sushiengine_add_module(NAME execution LAYER foundation)

# What the native backend cannot express in a header: a thread pool owning real OS
# threads, and the DAG compiler's execution loop. Built only when this configure selects
# that backend — the runtime backend's implementation lives in SushiRuntime.
if(SUSHIENGINE_EXECUTION_BACKEND STREQUAL "native")
    find_package(Threads REQUIRED)

    add_library(sushiengine_execution_native STATIC
        source/backend/native/thread_pool.cpp
        source/backend/native/dag_compiler.cpp)
    target_link_libraries(sushiengine_execution_native
        PUBLIC sushiengine_execution Threads::Threads)
    set_target_properties(sushiengine_execution_native PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
endif()

# Which implementation Context/Graph/Buffer denote is settled once, here, rather than per
# translation unit, and the matching backend is linked so the seam has something to compile
# against. This module owns the choice because it owns the seam: a consumer that needs an
# execution backend declares this module and gets both, without the whole engine facade.
if(SUSHIENGINE_EXECUTION_BACKEND STREQUAL "runtime")
    target_link_libraries(sushiengine_execution INTERFACE sushiruntime)
    target_compile_definitions(sushiengine_execution INTERFACE SUSHIENGINE_EXECUTION_BACKEND_RUNTIME=1)
elseif(SUSHIENGINE_EXECUTION_BACKEND STREQUAL "native")
    target_link_libraries(sushiengine_execution INTERFACE sushiengine_execution_native)
    target_compile_definitions(sushiengine_execution INTERFACE SUSHIENGINE_EXECUTION_BACKEND_NATIVE=1)
else()
    message(FATAL_ERROR
        "SUSHIENGINE_EXECUTION_BACKEND must be 'runtime' or 'native', got '${SUSHIENGINE_EXECUTION_BACKEND}'")
endif()
