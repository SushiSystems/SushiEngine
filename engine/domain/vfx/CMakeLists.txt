# vfx — a particle effect as authored and as compiled: the module stack, the curves and
# gradients it is parameterised by, and the POD emitter table both backends read. Header-only
# so the deterministic CPU backend's kernels instantiate inside the consuming translation
# unit; the GPU cosmetic backend consumes the same table from the renderer.
sushiengine_add_module(NAME vfx LAYER domain PUBLIC_DEPENDS core)
