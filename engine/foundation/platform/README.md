# Platform {#module-platform}

`platform` owns the window and operating-system lifecycle seam, so an application shell can open
a window, pump events and hand a Vulkan surface out without linking a renderer or an interface
toolkit. It is windowing-library-neutral by construction: native operating-system and library
types cross the seam as opaque handles, and the Vulkan packages are here only for the
surface-creation calls.

## Tier

`foundation` — the lowest tier in `cmake/EngineLayers.cmake`, so a module here may depend only on
other `foundation` modules.

## Dependencies

- No engine module. Nothing in the tree is needed to open a window, and keeping it that way is
  what lets a shell take a window without taking the engine.
- `SDL2::SDL2` (public) — the backing implementation. `SDLWindow` owns the SDL video subsystem's
  lifetime, and `user_data_directory` wraps `SDL_GetPrefPath`.
- `Vulkan::Headers`, `Vulkan::Loader` (public) — the surface-creation and instance-extension
  calls the seam publishes.

The module also adds `include/SushiEngine/platform` itself as a public include directory,
because its headers include each other by bare file name.

## Public surface

Headers are relative to `include/SushiEngine/platform/`.

| Header | Declares |
|---|---|
| `platform_window.hpp` | `IPlatformWindow` — event pumping, drawable size, the Vulkan instance extensions to enable, and surface creation, all in neutral types. |
| `sdl_window.hpp` | `SDLWindow`, the SDL2 implementation of that interface. |
| `user_data_directory.hpp` | `user_data_directory` — the per-user, per-application directory a shipped host writes its own state into. |

## Tests

None. No test target links `sushiengine_platform` and no case names `Platform::`, so opening a
window, pumping events and resolving the user data directory are all unverified by the suite.
The obstacle is real — the two window paths need a display server, and the suite runs headless —
but `user_data_directory` does not, and its absence from the suite is a genuine gap rather than
a consequence of the environment.

## Further reading

- [`cross_platform_engineering_plan.md`](../../../docs/design/cross_platform_engineering_plan.md)
  — the platform priority order and what each target needs from this seam.
