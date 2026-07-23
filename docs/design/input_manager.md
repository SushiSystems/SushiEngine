# Input Manager — Device-Abstracted Action Input for the Editor and the Deterministic Simulation

**Status:** Proposed design document (2026-07). Nothing in this document is implemented yet.
The audit in §1 is verified against source as of this writing; every file and symbol named
there exists today. The design reconciles three constraints that already exist in the tree:
the editor's ImGui-owned event flow, the one-way `SushiEngine → SushiRuntime` layering, and
SushiLoop's locked decision that *the only nondeterminism source is player input, captured as
a numbered per-tick command* ([SUSHILOOP.md](../slop/SUSHILOOP.md), "Core decisions").

Companion documents: [ARCHITECTURE.md](../guides/ARCHITECTURE.md) §8–§9 (SushiLoop core,
rollback, loopback net), [render_pipeline_refactor.md](render_pipeline_refactor.md) §2 (the
layering pattern `sushi_input` copies).

---

## 0. North star

- **Actions, not keys.** Gameplay and editor code binds to named actions (`"Move"`,
  `"Jump"`, `"CameraLook"`), never to a device control. A control is something a *binding*
  names, in data, once. Where the two concepts meet, **the action wins** — no consumer ever
  branches on device type.
- **Three device families, one abstraction.** Keyboard/mouse, gamepad, and touch all reduce
  to the same three action shapes: button, 1D axis, 2D axis. A virtual on-screen stick is a
  device like any other, not a special case in gameplay code.
- **The determinism boundary is sacred.** Raw device input lives on the host frame, outside
  the simulation island. The only thing that crosses into the world is the game's
  trivially-copyable `Command`, built once per fixed tick inside `sample_command` and
  recorded into `InputHistory<Command>`. The Input Manager feeds that reduction; it never
  touches the `World`.
- **SDL stays quarantined.** Exactly one new SDL-aware component is added (the event
  translator inside a compiled `sushi_input` static library). The action layer is
  header-only engine code with zero SDL includes, zero SYCL, and no `sushiruntime` link —
  the same discipline `sushi_render` already proves out.
- **Rebindable and persistent.** Every binding is data, serializable through the same
  nlohmann/json pattern `Preferences::render_settings` already uses, and rebindable at
  runtime with conflict detection.
- **Testable without hardware.** The scripted input source is a first-class
  `IInputSource` implementation used by tests and demos — a real implementation, not a mock,
  honouring the project's no-mocks rule.

---

## 1. Where we are — verified audit (2026-07)

### 1.1 What exists

| Area | State |
| --- | --- |
| Event pump | `SdlWindow::pump_events()` (`editor/window/sdl_window.cpp:62`) is the **only** `SDL_PollEvent` loop. It classifies only `SDL_QUIT`/window-close; everything else is forwarded opaquely. |
| Event routing seam | `IPlatformWindow::set_event_handler(std::function<void(const void*)>)` (`editor/window/platform_window.hpp`) — events cross as an opaque `const SDL_Event*`. Today the sole sink is `ImGui_ImplSDL2_ProcessEvent` (`editor/ui/imgui_backend.cpp:111`). |
| SDL initialization | `SDL_Init(SDL_INIT_VIDEO \| SDL_INIT_TIMER)` (`sdl_window.cpp:40`). **No game-controller, joystick, haptic, or sensor subsystem is initialized.** |
| Keyboard/mouse consumption | Entirely through ImGui IO: global shortcuts polled ad hoc in `editor/main.cpp:316–334`, gizmo hotkeys in `editor/ui/editor_panels.cpp:3845`, camera capture in `editor/ui/viewport_panel.cpp:339–391`, gizmo dragging in `editor/gizmo/gizmo_controller.cpp:172`. |
| Camera input abstraction | `Editor::InputState` (`editor/input/input_state.hpp`) — a neutral per-frame snapshot the viewport fills from ImGui and the stateless `CameraController` consumes. The proven seam this design generalizes. Bindings (WASDQE, Shift, right/middle mouse) are **hard-coded** in the fill site and the controller's documentation. |
| Deterministic command stream | `Loop::InputHistory<Command>` (`include/SushiEngine/loop/input.hpp`), sampled once per fixed tick by `Loop::App::step_once()` via the `sample_command(TickId)` hook (`include/SushiEngine/loop/app.hpp:376`), re-applied verbatim by rollback reconciliation. `Command` is game-defined, trivially copyable, `operator==`. |
| Engine UI pointer model | `UI::PointerInput { position, down }` (`include/SushiEngine/ui/interaction.hpp`) — host-fed, device-agnostic, already documented as determinism-compatible. |
| Binding persistence precedent | `IPreferencesStore` + nlohmann/json with tolerant field-by-field reads (`editor/core/preferences.cpp:586–667`); `%APPDATA%/SushiEngine/preferences.json`. |

### 1.2 The gap

1. **No gamepad, no touch, anywhere.** SDL's controller subsystem is not even initialized;
   ImGui IO — the de-facto input state store — only surfaces keyboard and mouse.
2. **No action concept.** Every consumer polls concrete keys. Rebinding W/E/R or Ctrl+Z
   means editing four call sites; a gamepad camera orbit means rewriting the viewport panel.
3. **No route from device to `Command`.** `sample_command` exists and is the right hook, but
   every example feeds it scripted values. A real game today would read ImGui IO inside
   `sample_command` — coupling the sim's input to the UI library and breaking the moment the
   fixed-tick loop runs zero or two ticks in a frame (edges lost or double-consumed).
4. **No persistence, no per-player device routing, no haptics.**

---

## 2. Target architecture

```
          OS / SDL2 event queue
                   │
                   ▼
   IPlatformWindow::pump_events()          ← unchanged, still the only pump
                   │
        EventHandler seam (const void*)
                   │
        ┌──────────┴───────────┐
        ▼                      ▼
 ImGui_ImplSDL2_ProcessEvent   SdlInputTranslator            [sushi_input STATIC]
 (editor UI, unchanged)        SDL_Event → InputEvent         the ONLY SDL-aware
                               engine codes, device ids       input code
                                       │
                                       ▼
                               DeviceRegistry                 [header-only]
                               per-device state tables,       include/SushiEngine/input/
                               hotplug, player assignment
                                       │
                                       ▼
                               ActionMapper
                               InputContext stack × BindingSet
                               processors: deadzone, invert,
                               scale, chords, composites
                                       │
              ┌────────────────────────┴────────────┐
              ▼                                     ▼
   ActionSnapshot (per host frame)         TickSampleAccumulator
   editor camera, tools, shortcuts,        edge-safe reduction over
   UI overlays                             0..N fixed ticks per frame
                                                    │
                                                    ▼
                                  Loop::App::sample_command(tick)
                                  game builds its quantized Command
                                                    │
                                                    ▼
                                  InputHistory<Command> → rollback / net
                                  (existing, unchanged)
```

Two consumers, two cadences, one source of truth. The editor and any immediate-mode UI read
the **per-frame** `ActionSnapshot`. The simulation reads the **per-tick** sample through the
accumulator, and only ever as a reduced `Command`. Everything above the dashed seam in the
diagram is host-side; nothing below `sample_command` knows devices exist.

### 2.1 The device layer

**Engine-owned control enums, not SDL codes.** `include/SushiEngine/input/controls.hpp`
defines `Key` (ordered by USB HID usage-page scancodes, so the numbering is stable and
layout-independent), `MouseButton`, `GamepadButton` / `GamepadAxis` (SDL's
`SDL_GameController` Xbox-style logical layout: south/east/west/north, sticks, triggers,
bumpers, d-pad), and `TouchPhase { Began, Moved, Ended, Cancelled }`. All are
`enum class ... : std::uint16_t` with explicit ordinals — they appear in serialized bindings
and must never renumber.

**One event type.** `InputEvent` is a trivially-copyable struct: `{ DeviceId device;
EventType type; std::uint16_t control; float value; float x, y; std::uint64_t frame; }`.
Buttons carry value 0/1; axes carry the normalized value; pointer/touch events carry
positions in window pixels. No unions, no inheritance — events are records.

**`IInputSource` — where events come from.**

```cpp
class IInputSource
{
public:
    virtual ~IInputSource() = default;

    /** @brief Drain this source's events for the current host frame into `out`. */
    virtual void poll(std::vector<InputEvent>& out) = 0;
};
```

Three planned implementations:

- `SdlInputTranslator` (compiled, in `sushi_input`) — **does not pump SDL.** It exposes
  `void handle_native_event(const void* sdl_event)` and is registered alongside ImGui on the
  *existing* `IPlatformWindow` event-handler seam. The seam grows from a single handler to a
  handler list (a three-line change in `SdlWindow`); the pump stays unique and stays where it
  is. Decision recorded: **the Input Manager must not own a second `SDL_PollEvent` loop** —
  SDL has one event queue, and two pumps would steal events from each other.
- `ScriptedInputSource` (header-only) — replays a programmed event list, one frame at a
  time. This is the test and demo backend, and doubles as the replay-file reader later.
- `VirtualControlSource` (header-only) — synthesizes gamepad-shaped events from touch
  pointers routed through on-screen controls (§2.4).

**Device state is derived from events, never queried from SDL.** The `DeviceRegistry`
maintains per-device state tables (key bitset, button bitset, axis values, pointer table) by
folding the frame's events. This is what makes `ScriptedInputSource` behaviourally identical
to the real translator — same events in, same state out — and what keeps the action layer
free of SDL calls (Liskov holds by construction: sources are interchangeable because state
lives above them).

**Hotplug and identity.** `SDL_CONTROLLERDEVICEADDED/REMOVED` translate to
`DeviceConnected/DeviceDisconnected` events. `DeviceId` is a stable small integer slot
(keyboard and mouse are fixed slots 0 and 1; gamepads claim the lowest free slot and keep it
until unplugged), so bindings and player assignments survive reconnection of the same
controller ordering.

**SDL prerequisites.** `SdlWindow::initialize` adds `SDL_INIT_GAMECONTROLLER` (which implies
joystick). Game-controller support is core SDL2 — `sdl2[vulkan]` in
`cli/sushistack.deps.toml` already carries it; **no new vcpkg feature is required.** Haptics
go through `SDL_GameControllerRumble` (also core), exposed as
`IHapticsSink::rumble(low_frequency, high_frequency, duration_seconds)` implemented by the
translator, so gameplay can shake a pad without seeing SDL.

### 2.2 The action layer

**Action shapes.** `ActionType { Button, Axis1D, Axis2D }`. A button action exposes edges
(`pressed`, `released`) and level (`held`); an axis action exposes a `float` or `Vector2`
value. That is the complete consumer-facing vocabulary.

**Bindings are data.**

```cpp
struct Binding
{
    ControlPath control;              ///< device family + control ordinal, e.g. {Gamepad, LeftStickX}.
    std::uint16_t modifier_chord[2];  ///< optional held-control requirements (Ctrl+S, L2+face button).
    float scale = 1.0f;               ///< multiplier applied after processors.
    bool invert = false;
    Deadzone deadzone;                ///< radial (sticks, renormalized) or axial (triggers).
};
```

A **composite binding** assembles an `Axis2D` from four buttons (WASD → move vector,
normalized on the diagonal) or an `Axis1D` from two (Q/E). Composites are how keyboard and
stick bind to the *same* action with the same downstream math. Mouse deltas and wheel bind
as *relative* axis controls — per-frame deltas that accumulate rather than sample.

**Contexts.** An `InputContext` is a named set of actions with bindings — `"Gameplay"`,
`"Vehicle"`, `"Menu"`, `"EditorViewport"`, `"EditorGlobal"`. The `ActionMapper` holds a
priority-ordered context stack; an action resolved by a higher context **consumes** its
controls, so opening the pause menu silently masks gameplay movement without any consumer
checking flags. Push/pop is the entire mode-switching API.

**Processors run in a fixed order** — deadzone → invert → scale → composite assembly —
evaluated in plain `float` on the host. (Determinism is unaffected: processed values are
quantized before they enter the `Command`, and the server-authoritative model never requires
two machines to agree on raw float math — only on the quantized command stream.)

**Editor/ImGui gating.** In the editor, the `ActionMapper` is fed a per-frame gate:
`WantCaptureKeyboard` suppresses key-sourced button actions, `WantCaptureMouse` suppresses
mouse-sourced ones, exactly the gating `main.cpp` shortcuts do by hand today — but in one
place, once. The Play-mode viewport pushes the `"Gameplay"` context only while focused.

**Buffering.** `ActionState` keeps a small ring of timestamped edge events, so a consumer
can implement input-buffer windows (jump pressed up to N ticks before landing) by reading
events instead of levels. The buffer is a query convenience on the host side; what enters
the `Command` is still whatever the game's reduction chooses to encode.

### 2.3 The determinism boundary — per-tick sampling

This is the most consequential decision in the design, and it exists because the fixed-step
accumulator (`FixedTimestepClock`, `App::advance`) runs **zero, one, or several** simulation
ticks per host frame:

- A 240 Hz host frame against a 60 Hz tick runs *zero* ticks three frames out of four. A
  key tapped and released inside a zero-tick frame must not vanish.
- A hitchy frame runs *two or more* ticks back-to-back. A single key press must not become
  two presses.

The `TickSampleAccumulator` resolves both by construction:

- **Edges are sticky until consumed.** `pressed` / `released` flags accumulate across host
  frames and are cleared only when a tick actually samples them. The zero-tick tap surfaces
  on the next tick that runs.
- **The first tick of a burst consumes the edges; subsequent ticks in the same burst see
  level state only.** The two-tick hitch sees `pressed` once, `held` twice. This mirrors
  how a real 60 Hz poll would have observed the world, which is exactly the contract
  rollback replay needs.
- **Analog values sample latest-wins.** No averaging: averaging is a filter the game did not
  ask for, and latest-wins is what a hardware poll at the tick boundary would read. Relative
  axes (mouse deltas) instead *sum* across the frames since the last tick and are zeroed on
  consumption — a delta is a quantity, not a level.

The consumption API is deliberately shaped for the existing hook:

```cpp
app.sample_command([&input](Loop::TickId tick) -> MoveCommand
{
    const Input::TickSample sample = input.consume_tick_sample();

    MoveCommand command;
    command.move_x = Input::quantize_axis(sample.axis2d("Move").x);   // → std::int16_t
    command.move_y = Input::quantize_axis(sample.axis2d("Move").y);
    command.jump   = sample.pressed("Jump");
    return command;
});
```

`quantize_axis` maps [-1, 1] to `std::int16_t` (with exact 0 at rest, symmetric range). The
game's `Command` stays trivially copyable, `operator==`-comparable, small on the future
wire, and — because quantization happens *before* recording — `InputHistory`, rollback
replay, and server reconciliation all operate on values that are bit-identical by
construction. Prediction misses caused by float jitter cannot exist.

**What the Input Manager never does:** it never appears in the `World`, never registers a
system, never runs on the (future) sim thread. When the sim/render/net thread split ships
(SUSHILOOP.md, "Threading"), the manager stays on the host/OS thread — the same thread that
must pump SDL events on every platform anyway — and `consume_tick_sample()` returns a value
snapshot, which is already the thread-crossing currency that design prescribes.

### 2.4 Touch and virtual controls

Touch decomposes into two layers, and only the first is device code:

1. **Pointers.** `SDL_FINGERDOWN/MOTION/UP` translate to pointer events (id, phase,
   normalized position). The `DeviceRegistry` tracks up to `MAX_TOUCH_POINTS` (8) concurrent
   pointers. Mouse can masquerade as pointer 0 behind a flag, so touch UIs are developable
   on desktop. This layer also feeds the existing `UI::PointerInput` for the engine UI —
   one pointer source instead of the host hand-feeding it.
2. **Virtual controls.** `VirtualControlSource` owns a screen-space description of sticks,
   buttons, and swipe regions (anchored rectangles, radii). Each host frame it claims
   pointers that land in its regions and **emits ordinary gamepad-shaped `InputEvent`s** —
   a virtual stick is `GamepadAxis::LeftStickX/Y` on a dedicated device slot. Bindings
   cannot tell it from hardware; gameplay code cannot either. Adding touch support to a
   shipped gamepad game is *placing controls*, not writing input code — open for extension,
   closed for modification, literally.

Gesture recognition (tap, long-press, drag, pinch) is a later, optional recognizer stage
that consumes unclaimed pointers and emits button/axis actions; it is explicitly out of
Phase 5's minimum and recorded as an extension point, not designed in detail here.

Rendering the virtual controls is the engine UI's job (they are widgets), not the Input
Manager's; the source only owns hit-testing and emission.

### 2.5 Rebinding and persistence

- **`RebindingListener`**: enters a capture mode filtered by expected shape (a button
  rebind ignores axis noise; an axis rebind requires deflection past a threshold so stick
  drift cannot bind itself), reports the captured `ControlPath`, and surfaces conflicts
  against the containing context. Cancel on Escape/timeout. No consumer code changes when a
  binding changes — that is the entire point of bindings-as-data.
- **Serialization**: `bindings_to_json` / `bindings_from_json`, field-by-field with
  defaulted reads, exactly the `render_settings_to_json` pattern
  (`editor/core/preferences.cpp:88`). Unknown actions are preserved on round-trip; missing
  bindings fall back to the context's compiled-in defaults, so a stale file after an update
  degrades gracefully.
- **Editor**: an `input_bindings` field joins `struct Preferences`, riding the existing
  load/dirty/save lifecycle in `main.cpp`. **Game**: the game owns where its bindings file
  lives; the engine provides the (de)serialization functions, not a file-path policy.

### 2.6 Local multiplayer routing

A `PlayerSlot` maps a player index to a set of `DeviceId`s. Default policy: keyboard+mouse
and gamepad 0 both feed player 0 until a second player claims a device ("press A to join" —
a claim is just routing the device's events to that player's `ActionMapper` instance; each
player owns one). This composes untouched with SushiLoop's model — "a player is an ECS
entity, input is a per-tick command buffer" — because N local players simply means the
game's `Command` (or command stream) carries N reductions, each from its own mapper. Slotted
for the last phase; the data model (per-mapper routing) costs nothing to carry from day one.

---

## 3. Module layout, build, and layering

```
include/SushiEngine/input/          header-only action layer (no SDL, no SYCL)
    controls.hpp                    Key, MouseButton, GamepadButton/Axis, TouchPhase, ControlPath
    events.hpp                      InputEvent, EventType, DeviceId
    source.hpp                      IInputSource, ScriptedInputSource
    device_registry.hpp             per-device state folded from events, hotplug, players
    bindings.hpp                    Binding, Deadzone, composites, (de)serialization decls
    action_map.hpp                  Action, InputContext, ActionMapper, ActionSnapshot
    tick_sample.hpp                 TickSample, TickSampleAccumulator, quantize_axis
    virtual_controls.hpp            VirtualControlSource
    haptics.hpp                     IHapticsSink
    input_manager.hpp               InputManager façade wiring the above; the one include consumers need

input/                              compiled backend (mirrors render/'s discipline)
    CMakeLists.txt                  add_library(sushi_input STATIC ...)
    sdl/sdl_input_translator.cpp    the only SDL-aware input code (+ haptics impl)
    sdl/sdl_input_translator.hpp
```

- `sushi_input` is a plain **STATIC** library: links `SDL2::SDL2` only, compiles no SYCL,
  links no `sushiruntime`, `CXX_STANDARD 17` — the `sushi_render` recipe verbatim. Root
  `CMakeLists.txt` gains `if(SE_BUILD_INPUT) add_subdirectory(input) endif()` with
  `SE_BUILD_EDITOR` forcing it on, next to the existing render/editor ordering block; the
  option lands in `cmake/ProjectOptions.cmake`.
- The header-only action layer needs **no build change at all** — it is picked up by the
  `SushiEngine` INTERFACE target like `loop/` and `ui/` are. Headless examples and tests use
  `ScriptedInputSource` without linking SDL or `sushi_input` — the compiled library is only
  needed by binaries that open a window.
- Layering verdict: the one-way `SushiEngine → SushiRuntime` arrow is untouched — input
  never sees the runtime. The editor's "only SDL-aware components" doc comment grows from
  two members to three (`SdlWindow`, `ImGuiBackend`, `SdlInputTranslator`), all behind the
  same seam.

All headers carry the standard license banner, file-level Doxygen with rationale, Allman
braces, `PascalCase`/`snake_case`/trailing-underscore naming, and fully-spelled names, per
the tree's conventions.

---

## 4. Worked example — `first_game` grown a real input path

The single-player → interactive diff the design optimizes for:

```cpp
Input::InputManager input;                       // owns registry, mapper, accumulator
Input::SdlInputTranslator translator{input};     // only in windowed builds

window.add_event_handler([&](const void* event)
{
    translator.handle_native_event(event);       // ImGui's handler stays first in the list
});

Input::InputContext gameplay{"Gameplay"};
gameplay.add_axis2d("Move")
    .bind_composite(Key::W, Key::S, Key::A, Key::D)
    .bind(GamepadAxis::LeftStick, Deadzone::radial(0.20f));
gameplay.add_button("Jump")
    .bind(Key::Space)
    .bind(GamepadButton::South);
input.push_context(gameplay);

app.sample_command([&input](Loop::TickId) -> MoveCommand { /* §2.3 */ });

while (running)
{
    running = window.pump_events();              // unchanged
    input.begin_frame();                         // fold events → snapshot + accumulator
    app.advance(real_delta_seconds);             // 0..N ticks; each consumes one TickSample
    render(app.interpolation());
}
```

Multiplayer remains `client.connect(&transport)` — the command stream the transport carries
is already made of quantized, input-manager-fed commands, so nothing else changes.

---

## 5. Roadmap

### Phase 1 — Core action model, keyboard/mouse, SDL translator

Controls/events/source headers, `DeviceRegistry`, `Binding` with processors and composites,
`InputContext` stack, `ActionMapper`, per-frame `ActionSnapshot`, `ScriptedInputSource`,
`SdlInputTranslator` for keyboard/mouse, the handler-list growth of `IPlatformWindow`, and
the `sushi_input` target. Acceptance: `Unit_Input*` tests drive the full mapper through
`ScriptedInputSource` (binding resolution, context masking, composite normalization,
ImGui-style gating) with no window; the editor still builds and behaves identically with the
translator registered but unconsumed.

### Phase 2 — The tick boundary

`TickSampleAccumulator`, `TickSample`, `quantize_axis`, `sample_command` integration.
Acceptance: unit tests prove the three accumulator laws (zero-tick tap survives; N-tick
burst sees one edge; relative axes sum-and-clear), and a regression test replays the same
scripted event stream through two `App` instances asserting bit-identical
`InputHistory<Command>` — the determinism contract, tested the same way
`test_net_client_server` tests reconciliation.

### Phase 3 — Gamepad

`SDL_INIT_GAMECONTROLLER`, controller translation, hotplug slots, deadzones proven on
hardware, `IHapticsSink` rumble. Acceptance: `first_game` (windowed variant) playable with
keyboard *or* pad with zero gameplay-code difference; unplug/replug mid-play keeps bindings.

### Phase 4 — Rebinding and persistence

`RebindingListener`, conflict detection, `bindings_to_json/from_json`,
`Preferences::input_bindings`, an editor Preferences page listing actions with a
click-to-rebind flow. Acceptance: rebind survives editor restart; a stale/partial JSON loads
to defaults without error.

### Phase 5 — Touch and virtual controls

Pointer translation, mouse-as-pointer-0, `VirtualControlSource` (stick + buttons),
`UI::PointerInput` fed from the registry. Acceptance: `ui_demo` driven by real pointers; a
virtual stick moves the `first_game` player through the *unchanged* `"Move"` binding path.
Gesture recognizers recorded as a follow-on, not in this phase.

### Phase 6 — Editor migration

`EditorGlobal` and `EditorViewport` contexts absorb the ad-hoc shortcut polls in `main.cpp`
and `editor_panels.cpp`; the viewport fills `Editor::InputState` from the snapshot (the
struct survives as the camera controller's argument — it is a good seam; only its fill site
changes). Acceptance: shortcut behaviour byte-for-byte identical; W/E/R and Ctrl+Z
rebindable from the Preferences page as the proof.

### Phase 7 — Local multiplayer routing

`PlayerSlot`, device claiming, per-player mappers. Acceptance: two pads drive two player
entities in a `net_demo` variant through one `Command` stream carrying both reductions.

---

## 6. Cross-cutting concerns

- **Threading.** Everything here is host-thread-only today (as is the whole loop). The
  design's only thread-split obligation is already met: the sim-facing surface is a value
  snapshot (`TickSample`), and the manager itself stays on the OS-event thread forever.
- **Text input** is not an action. IME/character events route to whoever owns text focus
  (ImGui in the editor; a future engine-UI text field via a dedicated channel). Actions are
  suppressed while text capture is active — the existing `WantTextInput` gate, centralized.
- **Latency.** Sampling order per host frame is pump → fold → tick(s), so a tick sees input
  at most one frame old, which is the floor for a polled pipeline. Frames-in-flight latency
  is the renderer's ledger ([render_pipeline_refactor.md](render_pipeline_refactor.md),
  Phase 11), not this one's.
- **Replay files.** `ScriptedInputSource` reading a recorded event stream gives
  device-level replay for free, complementing (not replacing) `InputHistory` replay: the
  former reproduces mapper behaviour, the latter reproduces the sim. Both exist because the
  boundary between them is the design's central line.
- **Testing.** All action-layer logic is header-only and driven by `ScriptedInputSource`
  under the existing `Unit_*`/`Regression_*` GTest lanes — real implementations end to end,
  no mocks, no display required. Only the SDL translator itself needs a windowed smoke
  check, which Phase 3's hardware acceptance covers.

---

## 7. How this satisfies SOLID

- **SRP** — pump (window), translate (`SdlInputTranslator`), store (`DeviceRegistry`), map
  (`ActionMapper`), reduce-to-tick (`TickSampleAccumulator`), persist (json functions),
  rebind (`RebindingListener`) are seven objects with seven reasons to change.
- **OCP** — a new device family is a new `IInputSource` emitting existing event shapes
  (virtual touch controls prove it: gamepad events without a gamepad); a new binding
  processor extends data, not consumers.
- **LSP** — sources are substitutable by construction: state is folded from events above
  the source, so scripted, SDL, and virtual sources are indistinguishable downstream.
- **ISP** — consumers see only what they need: gameplay sees `TickSample`, the editor sees
  `ActionSnapshot`, rumble callers see `IHapticsSink`, nobody sees SDL or the registry.
- **DIP** — high-level policy (`ActionMapper`, the loop) depends on `IInputSource` /
  `TickSample` abstractions; the SDL detail depends inward. The existing
  `InputState`/`CameraController` pattern, generalized engine-wide.
