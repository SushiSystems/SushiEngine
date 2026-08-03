# astro — where the sun, moon, planets and stars actually are: Julian dates, orbital
# elements, the ephemeris over them, body orientation, gravity, and the topocentric and
# scene frames an observer is placed in. Header-only, double-precision host C++ answering in
# real units at real epochs, so the sky a frame is lit by and a headless test asserting an
# altitude at a date come from the same numbers. It fills the environment the renderer reads,
# which is why that module is a public dependency here.
sushiengine_add_module(NAME astro LAYER domain PUBLIC_DEPENDS core environment)
