# Input {#module-input}

`input` owns the device-neutral action layer: the one event record every source emits, the
binding tables that map a control to a named action, the per-player action maps over them, and
the gesture, haptic, text and replay channels around that. A game names actions; nothing above
this module names a key, a pad button or a windowing library.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `ui` (public) — the pointer seam is spelled in the interface module's types, so a pointer can
  be handed to a canvas without a translation layer. Both modules sit in `domain`, which makes
  this a same-tier edge and legal.

The module itself is header-only. The one part that touches an operating system is built
separately as `sushiengine_input_backend`, gated on `SUSHIENGINE_BUILD_INPUT`: the translator
that turns already-pumped `SDL_Event` records into engine `InputEvent`s. It links
`sushiengine_input` publicly and `SDL2::SDL2` privately — the header crosses native events as
`const void*`, so SDL never leaks into a consumer's translation unit.

## Public surface

`input_manager.hpp` is the façade a consumer includes; it wires sources into the device registry
and the registry into the mapper. Headers are relative to `include/SushiEngine/input/`.

| Group | Headers | Declares |
|---|---|---|
| Events and devices | `events.hpp`, `source.hpp`, `device_registry.hpp`, `controls.hpp`, `gamepad.hpp` | The event record and its identifiers, the source abstraction and its scripted backend, the folded per-device state, the control vocabulary, and stable slot allocation for hot-plugged pads. |
| Actions | `action_map.hpp`, `bindings.hpp`, `bindings_json.hpp`, `rebinding.hpp`, `player.hpp`, `input_manager.hpp` | Named actions and contexts, bindings as data with their processor chain, binding persistence, run-time rebinding with conflict detection, per-player routing, and the façade over all of it. |
| Extra channels | `gestures.hpp`, `haptics.hpp`, `text_input.hpp`, `virtual_controls.hpp`, `ui_pointer.hpp` | The recognizer stage, the rumble seam, text entry as its own channel, on-screen touch controls, and the adapter onto the interface module's pointer. |
| Determinism | `tick_sample.hpp`, `replay.hpp`, `replay_json.hpp` | The edge-safe reduction from host frames to ticks, device-level capture and playback, and the on-disk replay format. |

## Tests

Covered by the functional suite in `tests/`: nine `tests/unit/test_input_*.cpp` files across the
action map, bindings and their persistence, gamepad slots, per-player routing, rebinding, the
replay format, the tick reduction and virtual controls, plus
`tests/integration/test_input_determinism.cpp` for the frame-to-tick contract under replay.

The SDL translator in `sushiengine_input_backend` has no coverage: no test target links it, and
it is the one piece that needs real platform events.

## Further reading

No design document under `docs/design/` covers this module. The section numbers its headers cite
(§2.4 on gestures, §2.6 on local multiplayer, §6 on text entry) name a plan that is not in this
repository, so the headers themselves are the specification.
- [`domain-input.md`](../../../docs/architecture/domain-input.md) — device abstraction and the
  action layer.
