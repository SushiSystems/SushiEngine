# ecs — archetype/chunk component storage, the World composed from it, and the schedule
# that turns systems into Execution nodes. Header-only so its kernels instantiate inside
# the consuming translation unit, which is the safe way to ship device code.
sushiengine_add_module(NAME ecs LAYER foundation PUBLIC_DEPENDS execution)
