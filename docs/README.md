# SushiEngine documentation

This is the index. Nothing here holds facts of its own — every entry points at the document
that owns them, so there is exactly one place to change when something becomes false.

The project's front door, with the requirements, the setup and the first build, is the
[root `README.md`](../README.md).

## Start here

| Document | What it answers |
| --- | --- |
| [Getting started](getting-started/introduction.md) | How do I build a world, from an empty program? |
| [Command line guide](guides/command-line-interface.md) | What can `se` do, and with which flags? |
| [Contributing](CONTRIBUTING.md) | What is a change held to before it merges? |
| [Documentation style guide](documentation-style-guide.md) | How is prose in this repository written, and what is checked? |

## How it works

[`architecture/`](architecture/README.md) has one chapter per subject, grouped by the tier the
subject lives in. It describes the tree as it is today.

| Chapter | Covers |
| --- | --- |
| [Overview](architecture/overview.md) | The head and battery split, and the layers |
| [Foundation](architecture/foundation.md) | The entity-component-system, the system graph, the value-type seam |
| [Physics](architecture/domain-physics.md) | The constraint solver, cloth, colliders, soft bodies |
| [Animation](architecture/domain-animation.md) | Clips, the animator, blend trees, layers, inverse kinematics |
| [Audio](architecture/domain-audio.md) | The digital signal processing graph, spatialization, reverb |
| [Input](architecture/domain-input.md) | Device abstraction and the action layer |
| [User interface](architecture/domain-ui.md) | The retained canvas and its overlay pass |
| [Visual effects](architecture/domain-vfx.md) | Emitters, the two simulation backends, effect assets |
| [Atmosphere](architecture/domain-atmosphere.md) | The global dynamical core and the regional nest |
| [Terrain](architecture/domain-terrain.md) | Planetary terrain, from a metre to orbit |
| [Astronomy](architecture/domain-astro.md) | Ephemerides, gravity and reference frames |
| [Render](architecture/presentation-render.md) | The frame graph, materials, the temporal core, shadows, lighting |
| [World](architecture/world.md) | Snapshots, rollback, reconciliation, the loop core |
| [Tooling](architecture/tooling.md) | Validation and the probes |
| [Roadmap](architecture/roadmap.md) | The milestones |

## What is here

[`modules/`](modules/README.md) indexes every module's own `README.md`, which lives beside its
code under `engine/<tier>/<module>/` and is the single source of what that module owns, which
tier it sits in, what it links and what covers it in test.

## Guides

| Guide | Covers |
| --- | --- |
| [Command line](guides/command-line-interface.md) | Every `se` command, subcommand and flag |
| [Vehicles](guides/vehicles.md) | Authoring and simulating a node-and-beam vehicle |

## Reference

| Document | Covers |
| --- | --- |
| [Changelog](reference/changelog.md) | What changed, when, and why |
| [Glossary](reference/glossary.md) | Phase codes, their owning documents, and the shared vocabulary |
| API reference | The generated Doxygen site. Build it with `se doxygen`; it lands in `docs/api-site/html/`. Its landing page is [`api/mainpage.md`](api/mainpage.md). |

## Why it is shaped this way

[`design/`](design/README.md) is the engineering corpus: the plan for each subsystem and the
audits against it. It records intent, so a document there may describe work that does not
exist. Read the one covering your subsystem before you change that subsystem, and never read it
as a description of the tree.

## Project conduct

- [Contributing](CONTRIBUTING.md)
- [Code of conduct](CODE_OF_CONDUCT.md)
- [Security policy](SECURITY.md)
