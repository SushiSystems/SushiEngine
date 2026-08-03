# rendering — the presentation side of the engine. Only the part of it a headless run can prove
# lives here; the renderer's own device path is measured by tools/probes/ instead, because it
# needs a Vulkan device rather than an assertion.

# particle_demo exercises the VFX particle system's CPU-verifiable half: the authoring model,
# the emitter compiler's POD boundary, and the deterministic CPU backend's spawn/lifetime
# accounting plus its determinism and rollback-replay guarantees. The GPU cosmetic backend is
# validated visually in the editor.
sushiengine_add_sample(particle_demo particle_demo.cpp)
