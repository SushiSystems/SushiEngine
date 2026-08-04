# Glossary

The corpus names its work in phase codes — `P7-C`, `UX4`, `RHI2`, `S9`. A code is only
meaningful next to the document that mints it, and several families reuse the same letters.
This page maps every family to its owner so a code encountered in a commit message, a source
comment or a changelog entry can be resolved without guessing.

## Phase code families

| Family | Owner | What the codes number |
| --- | --- | --- |
| `M0`–`M5` | [SUSHILOOP.md](../design/SUSHILOOP.md) | Milestones of the deterministic, network-ready game loop: fixed-step core, snapshots, rollback, loopback reconciliation, cloth. |
| `P0`–`P9`, `PX` | [physics_system.md](../design/physics_system.md) | Phases of the unified XPBD solver and the cooking pipeline. `PX` is the editor-exposure stream that runs alongside them. Sub-phases are lettered: `P7-C`. |
| `P0`–`P11` | [solar_system_overhaul.md](../design/solar_system_overhaul.md) | Phases of planetary terrain. **These collide with the physics family** — see below. |
| `A0`–`A9` | [animation_system.md](../design/animation_system.md) | Phases of the skeletal animation stack, from clip assets to retargeting. |
| `S0`–`S10` | [audio_system.md](../design/audio_system.md) | Phases of the from-scratch audio engine, from the device seam to the authoring surface. |
| `PLATFORM0`, `S1`–`S6` | [cross_platform_engineering_plan.md](../design/cross_platform_engineering_plan.md) | Stages of the port to Linux, Android, macOS, iOS and PlayStation. **`S` collides with the audio family** — see below. |
| `RHI0`–`RHI9` | [cross_platform_engineering_plan.md](../design/cross_platform_engineering_plan.md) | Steps of the render hardware interface extraction: the golden harness, the neutral vocabulary, the command list, the backend split. |
| `BB-1`–`BB-8` | [cross_platform_engineering_plan.md](../design/cross_platform_engineering_plan.md) | Requests against SushiRuntime's backbone, tracked in that repository's `ENGINE_BACKBONE_REFACTOR.md`. `BB-1` is the interop import half the renderer's export half waits on. |
| `R1`–`R9` | [cross_platform_engineering_plan.md](../design/cross_platform_engineering_plan.md) | Real-time capability requests against SushiRuntime, tracked in its `PHYSICS_SUBSTRATE_REQUIREMENTS.md`. |
| `T1`, `T2`, phases `A`–`F` | [atmosphere_system.md](../design/atmosphere_system.md) | `T1` is the global dynamical core, `T2` the regional nest over it. The lettered phases are the build order. |
| `W0`–`W6` | retired | The weather and cloud roadmap. Its document was removed in 2026-07; `atmosphere_system.md` §1 records why. A `W` code survives only as a historical reference. |
| `VFX1`–`VFX6` | [vfx_particle_system.md](../design/vfx_particle_system.md) | Phases of the particle system, from the authoring model to the effect timeline. Sub-phases are lettered: `VFX2c`. |
| `UX0`–`UX6` | [editor_ux_overhaul.md](../design/editor_ux_overhaul.md) | Phases of the editor overhaul. All shipped as of 2026-07-30. |
| `UHM` | [unified_hazard_model.md](../design/unified_hazard_model.md) | The one execution vocabulary shared by simulation, compute and render. Its own sections are cited as `UHM §4`. |
| `RESTRUCTURE0` | [repository_restructure.md](../design/repository_restructure.md) | The four-phase reorganization of this repository. |

## The two collisions

Two families reuse the same letters, and neither can be renamed without invalidating the
commit messages and source comments that already cite them:

- **`P<n>` is ambiguous.** `P0`–`P9` belong to physics; `P0`–`P11` belong to planetary terrain.
  A bare `P2` is unresolvable. Cite the document with the code — `physics_system.md` §P2 or
  `solar_system_overhaul.md` §P2 — whenever the surrounding sentence does not already fix which
  subsystem is meant.
- **`S<n>` is ambiguous.** `S0`–`S10` belong to audio; `S1`–`S6` belong to the cross-platform
  plan, where they are always written under the `PLATFORM0` umbrella. Write `PLATFORM0 S4` for
  the porting stage and a bare `S4` for the audio phase.

New code families must not reuse a letter already claimed above.

## Terms

| Term | Meaning |
| --- | --- |
| Tier | One of `foundation`, `domain`, `asset`, `presentation`, `world` — the layer a module sits in. A module may depend on its own tier and on anything below it, enforced at configure time by `sushiengine_add_module()`. |
| Module | One directory under `engine/<tier>/`, owning its `include/`, `source/`, `tests/` and `README.md`. The unit of ownership and of the layer rule. |
| Execution lane | Which implementation backs `SushiEngine::Execution`. `runtime` dispatches through SushiRuntime; `native` is the SushiRuntime-free path. Selected by `SUSHIENGINE_EXECUTION_BACKEND`, and each lane configures into its own build tree. |
| Golden | A recorded render output a later run is compared against, byte for byte, to catch unintended visual change. Recorded and compared through `se render --probe golden`. |
| Cook | The offline step that turns an imported mesh into a simulation asset — `.sushicollision`, `.sushisoft`, `.sushinodebeam`. Host-only, cached by content hash. |
| Blob table | The indirection that lets a scene snapshot name a cooked asset by content hash instead of carrying a copy, so an undo stack of fifty steps holds one copy of a body rather than fifty. |
| Design document | A file under `docs/design/`. It records why a subsystem is shaped the way it is and what is planned, and it is not a description of what is built today — the architecture chapters are. |
