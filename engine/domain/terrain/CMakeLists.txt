# terrain — the planet's surface as data: the cube-sphere mapping, the quadtree over it, the
# tile addressing and residency a streamer works in, and the height and layer stack a tile is
# baked from. Header-only and graphics-free on purpose, so the offline baker, the test suite
# and the renderer all read one definition of what a tile is.
sushiengine_add_module(NAME terrain LAYER domain PUBLIC_DEPENDS core)
