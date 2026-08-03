# environment — the scene's outdoor state as plain data: the sun, the planet it lights and
# the sky layers drawn around it, the punctual lights under that, and the atmospheric fields
# (the regional nest's parameters and readback, the synoptic placement, the weather grid) a
# frame is shaded against. Header-only and device-free on purpose: the simulation authors
# these values, the renderer consumes them, and the editor and the serializer both read them,
# so all four agree on one definition rather than a shared graphics stack.
sushiengine_add_module(NAME environment LAYER domain PUBLIC_DEPENDS core material)
