# Input

This file covers the input domain: the device-abstracted action layer, the one SDL-aware backend
under it, rebinding and persistence, touch and virtual controls, and the tick boundary that hands
quantized commands to the deterministic simulation.

## 1. Input (device-abstracted actions, Phase 1)

Design: `docs/design/input_manager.md`. Gameplay and editor code bind to named actions (`"Move"`,
`"Jump"`), never to a device control — a control is what a *binding* names, in data, once.
Keyboard/mouse, gamepad, and touch all reduce to three action shapes: `Button`, `Axis1D`,
`Axis2D`. No consumer branches on device type.

**The header-only action layer (`engine/domain/input/include/SushiEngine/input/`)** carries zero
SDL, zero SYCL, and no runtime link — the same discipline `sushiengine_render`'s abstract side
keeps. Its pieces are seven single-responsibility objects (SRP by construction):

- `controls.hpp` — the engine-owned control enums. `Key` is numbered by USB HID keyboard usage
  IDs (the table SDL scancodes are built on), so the numbering is physical-position based and
  layout-independent, and the SDL translator *reinterprets* a scancode as a `Key` rather than
  looking it up. Every ordinal here is a serialized wire value and must never renumber. A
  `ControlPath` (family + ordinal) is the atom a binding stores.

- `events.hpp` — the single trivially-copyable `InputEvent` record every source emits, the
  `EventType` tags, the stable `DeviceId` slot scheme (keyboard 0, mouse 1, gamepads from 2), and
  a float `Vector2` (distinct from `UI::Vector2`, which is double).

- `source.hpp` — `IInputSource` (one method: drain this frame's events) and the header-only
  `ScriptedInputSource`. Because all state is folded from events *above* the source, a scripted,
  SDL, and (later) virtual source are indistinguishable downstream — Liskov holds by
  construction, which is what lets the whole layer run headless in tests with no mocks.

- `device_registry.hpp` — `DeviceRegistry`, the single owner of "what is held now", folded from
  the event stream. Level state (keys, buttons, stick/trigger values) persists; relative state
  (mouse delta, wheel) accumulates within a frame and resets at its start — the split the
  tick-boundary sampler (Phase 2) needs.

- `bindings.hpp` — bindings as data: `Binding`, `Deadzone` (radial for whole sticks, axial for
  triggers), `ChordGate` (Ctrl+Z), and the composite/`Vector2` bindings. Every evaluator is a
  pure function of the registry, run through a fixed processor order (deadzone → invert → scale,
  then composite assembly and diagonal normalization), so a keyboard composite and an analog
  stick feed one action through identical downstream math. A new binding shape is a new struct
  and evaluator, no consumer change — OCP as data.

- `action_map.hpp` — `Action`/`ActionType`, `InputContext` with fluent builders
  (`add_axis2d("Move").bind_composite(W,S,A,D).bind(GamepadAxis::LeftStick, ...)`), and
  `ActionMapper`. The mapper holds a priority-ordered context stack and resolves every action
  once per host frame into an `ActionSnapshot`; a higher context *consumes* the controls its
  actions reference, so pushing a `"Menu"` context masks gameplay movement with no consumer
  testing a flag. An `InputGate` mirrors ImGui's `WantCapture*` flags to suppress key- or
  mouse-sourced actions in one place.

- `input_manager.hpp` — the `InputManager` façade. `begin_frame()` drains every registered
  source, folds the events into the registry, and resolves the mapper. It never touches the
  `World` and never registers a system.

**The compiled backend (`engine/domain/input/source/`, `sushiengine_input_backend` STATIC).** The
mirror of `sushiengine_render`'s recipe: links `SDL2::SDL2` only, no SYCL, no runtime, C++17. It
holds the one SDL-aware input component, `Input::SDLInputTranslator`
(`engine/domain/input/source/backend/sdl/sdl_input_translator.*`), which turns already-pumped
`SDL_Event` records into engine `InputEvent`s. It does **not** pump SDL — the single
`SDL_PollEvent` loop stays in `SDLWindow` (`engine/foundation/platform/source/sdl_window.cpp`),
and the translator registers on the window's event-handler seam alongside ImGui. That seam grew
from one handler to a handler list: `IPlatformWindow::add_event_handler` appends, so ImGui still
sees events first. The native `SDL_Event` crosses the translator's header as `const void*`, so
SDL leaks into no consumer translation unit — the editor's "only SDL-aware components" set grows
from two (`SDLWindow`, `ImGuiBackend`) to three.

**Gamepad and haptics.** A controller is a device family, not a special case: the same
`"Move"`/`"Jump"` bindings drive keyboard or pad. `SDLWindow` inits `SDL_INIT_GAMECONTROLLER`
(core SDL2) and the translator opens/closes controllers on hot-plug, translating their button and
axis events to the same `InputEvent` shapes a stick binding already reads (SDL's button and axis
ordinals match the engine enums, so the translation is a reinterpretation). A controller's
identity to the engine is its `DeviceId` slot, allocated by `GamepadSlotTable`
(`engine/domain/input/include/SushiEngine/input/gamepad.hpp`) — a header-only, SDL-free,
unit-tested policy that hands a pad the lowest free slot and keeps it across an unplug/replug of
the same ordering, so bindings and player assignments survive a reconnect. The translator also
implements `IHapticsSink` (`.../input/haptics.hpp`), so the object gameplay drives rumble through
is the same one it never sees as SDL — `SDL_GameControllerRumble` behind
`rumble(device, low, high, duration)`.

**Rebinding and persistence.** Because a binding is data, changing one changes no consumer code —
the property `rebinding.hpp` makes operational. `RebindingListener` captures the next control of
an expected shape (a button rebind is deaf to axis noise; an axis rebind demands deflection past
a threshold so a drifting stick cannot bind itself) and cancels on Escape or timeout;
`binding_conflict` warns when a captured control is already used in the context, and
`set_button_binding` writes it back.

Persistence is `bindings_json.hpp` (`bindings_to_json`/`bindings_from_json`), tolerant
field-by-field with defaulted reads exactly like the editor's `render_settings`: a missing action
entry keeps its compiled-in defaults, a malformed entry is ignored rather than throwing, and
unknown actions survive a round-trip. It is the only input header that pulls in nlohmann/json, so
the core action layer and any headless build that never persists stay dependency-free. The editor
holds the serialized document in `Preferences::input_bindings` (as text, so the struct stays
JSON-free) and nests it in `preferences.json`; the game owns where its own file lives — the
engine provides the functions, not a path policy.

**Touch and virtual controls.** Touch decomposes into two layers, and only the first is device
code. Pointers: the translator turns `SDL_FINGERDOWN/MOTION/UP` into touch events (normalized →
pixels, stable finger→slot), and the `DeviceRegistry` folds up to `MAX_TOUCH_POINTS` of them; a
`set_mouse_as_pointer` flag folds the mouse into pointer 0 so a touch UI is developable on
desktop, and `primary_pointer()` is the one pointer the engine UI reads (via the opt-in
`ui_pointer.hpp` adapter) instead of the host hand-feeding `UI::PointerInput`.

Virtual controls: `VirtualControlSource` (`.../input/virtual_controls.hpp`) owns a screen-space
layout of sticks and buttons and, each frame, claims the pointers inside them and emits ordinary
gamepad-shaped events — a virtual stick is `GamepadAxis::LeftStick` on a dedicated slot, so a
`"Move"` binding resolves it through the exact path a hardware stick takes. It runs on the
manager's second-pass virtual-source stage (`add_virtual_source`), polled after the primary fold
so it reads this frame's pointer state and its output is folded back for the mapper. Adding touch
to a shipped gamepad game is placing controls, not writing input code; rendering them is the
engine UI's job, and gesture recognition is a recorded follow-on.

**Editor migration.** The editor's shortcuts and tool keys, once scattered `ImGui::IsKeyPressed`
polls, are now two rebindable contexts
(`applications/editor/source/input/editor_contexts.hpp`): `EditorGlobal`
(Undo/Redo/Save/Copy/Cut/Paste, chorded on Control) and `EditorViewport` (gizmo W/E/R).
`applications/editor/source/main.cpp` pushes them, applies any overrides saved in
`Preferences::input_bindings`, and consumes `input.snapshot().pressed(...)` for the global
shortcuts; the toolbar reads the same snapshot for the gizmo keys (via a non-owning
`ActionSnapshot*` on `EditorContext`). The `!WantTextInput` guards each poll wrote by hand are
now the mapper's single capture gate. The Preferences window gained a rebind page — list an
action, click Rebind, press a key — that runs the `RebindingListener`, rewrites the binding, and
serializes the set so a rebind survives a restart. Camera flight (WASD while right-mouse is held)
stays on the viewport's own `Editor::InputState` seam
(`applications/editor/source/input/input_state.hpp`), unchanged.

**Local-multiplayer routing.** Binding resolution asks a `DeviceAssignment` — held per
`ActionMapper` — which device answers each control family, instead of hard-coding the first
connected pad. The default (keyboard, mouse, first pad) reproduces single-player exactly, so the
routing costs nothing until used. A local-multiplayer game gives each player a `PlayerHandle`
(`.../input/player.hpp`; its own mapper, tick accumulator, and assignment) resolving the *shared*
contexts against its own devices, collected in a `PlayerRoster` that routes claims and answers
"press A to join" (`join_candidates`). Device events are folded once into the one shared
registry; N players is N reductions of that state, one per `Command` — the exact shape
SushiLoop's "a player is an ECS entity, input a per-tick command" already prescribes.

**Completion pass.** Four recorded follow-ons close the system out. *Buffering* (the design doc's
§2.2): the `TickSampleAccumulator` stamps each tick with ticks-since-press per action, so
`TickSample::pressed_within(name, window)` gives jump-buffer and coyote-time windows at the tick
cadence. *Gestures* (`gestures.hpp`, the design doc's §2.4): a `GestureRecognizer` turns the
registry's pointers into tap/long-press/drag/pinch results, time-driven by `update(dt)` and left
for the consumer to map onto actions — a pure sensor.

*Replay* (`replay.hpp`, the design doc's
§6): an `InputRecorder` captures the frame event stream and replays it through a
`ScriptedInputSource`, reproducing *mapper* behaviour the way `InputHistory` reproduces the *sim*
— the two straddle the tick boundary, the design's central line; `replay_json.hpp` is an opt-in
file format. *Text* (`text_input.hpp`, the design doc's §6): a `TextInputChannel` is an
active-gated UTF-8 buffer the translator feeds `SDL_TEXTINPUT` into, with actions suppressed for
its duration by the same capture gate — text is a channel, not an action. The editor exposes all
of it through an **Edit > Input Manager** window
(`applications/editor/source/input/input_manager_window.cpp`): contexts and their actions with
the current binding, click-to-rebind with conflict flagging, and Reset to Defaults, persisted to
preferences.

**The tick boundary (`tick_sample.hpp`).** Two consumers read the input at two cadences. The
editor and any immediate-mode UI read the per-frame `ActionSnapshot`. The simulation reads a
per-tick `TickSample`, and only ever as a reduced, quantized value — because the fixed-step loop
(`App::advance`) runs zero, one, or several ticks per host frame, per-frame state cannot cross
directly (a tap in a zero-tick frame would vanish; one press in a two-tick hitch would fire
twice).

`TickSampleAccumulator` resolves this: `accumulate` folds each frame's snapshot, `consume`
(called once per tick from inside `sample_command`) hands out one sample and then clears the
edges and zeroes the relative axes it returned. Three laws hold by construction — edges stay
sticky until a tick consumes them, the first tick of a burst consumes the edge while later ticks
see level only, and relative axes (mouse deltas) sum since the last tick while absolute axes
(sticks) are latest-wins. A sub-frame tap survives because the `DeviceRegistry` folds
press/release *edges* independently of the final level, so the mapper's button edges do not
depend on a frame-boundary level change.

`quantize_axis` maps a normalized value to a symmetric `std::int16_t` *before* it enters the
game's `Command`, so `InputHistory`, rollback replay, and server reconciliation all operate on
bit-identical values — prediction misses from float jitter cannot exist. The manager stays on the
OS-event thread forever, and `consume_tick_sample()` returns a value snapshot, which is already
the thread-crossing currency the future sim-thread split prescribes.

**Layering.** The one-way `SushiEngine → SushiRuntime` arrow is untouched — input never sees the
runtime. Headless examples and tests use `ScriptedInputSource` and link neither SDL nor
`sushiengine_input_backend`; only a binary that opens a window needs the compiled library.
`SUSHIENGINE_BUILD_INPUT` gates it (forced on by `SUSHIENGINE_BUILD_EDITOR`). All seven roadmap
phases and the design's recorded follow-ons (buffering, gestures, replay, text) have landed; the
editor consumes the action snapshot for its shortcuts and tool keys and configures bindings
through its Input Manager window. The one deliberately-untouched seam is the viewport's
camera-flight `Editor::InputState` fill, kept on ImGui because its per-mode latching
(right-mouse-held owns WASD) is viewport state, not a binding — a good seam the design keeps
as-is.
