# profiling — the CPU frame profiler: named channels, RAII scopes, and a fixed history
# ring, with an injectable clock so its tests are deterministic. Depends on nothing, so
# any tier may instrument itself with it.
sushiengine_add_module(NAME profiling LAYER foundation SOURCES source/frame_profiler.cpp)
