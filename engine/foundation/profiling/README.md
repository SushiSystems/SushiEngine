# Profiling {#module-profiling}

`profiling` owns CPU frame timing: `FrameProfiler` measures the main thread's frame against
named channels, `ScopedTimer` opens and closes a channel's scope as an RAII guard, and
`FrameProfileSnapshot` is the copied-out result a display reads. The clock is injectable —
`FrameProfiler`'s constructor takes a `ProfilerClock` free function and defaults to
`std::chrono::steady_clock` only when none is given — so a test can advance time by hand
instead of sleeping.

## Tier

`foundation` — the lowest tier in `cmake/EngineLayers.cmake`, so a module here may depend only on
other `foundation` modules.

## Dependencies

- None. Every tier above may reach it to instrument itself, so it links nothing in return.

## Public surface

Headers are relative to `include/SushiEngine/profiling/`.

| Header | Declares |
|---|---|
| `frame_profiler.hpp` | `ChannelId`, `ProfilerClock`, `ChannelValue`, `FrameProfileSnapshot`, `FrameProfiler`, `ScopedTimer`. |

## Tests

Covered by the functional suite in `tests/`. `tests/unit/test_frame_profiler.cpp` exercises a
single scope's elapsed time, two scopes accumulating on one channel within a frame, a new frame
clearing the last one's channel times, nested scopes' inclusive time and depth, the 240-frame
history ring's oldest-first eviction, and the default clock's non-negative output.

## Further reading

- [`profiling_system.md`](../../../docs/design/profiling_system.md) — the design record this
  module implements.
