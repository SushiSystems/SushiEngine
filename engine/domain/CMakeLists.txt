# domain — the tier that models the world itself: the shapes it is made of, the physics
# that moves them, and the systems that dress them. A domain module may read foundation
# and its own tier, and knows nothing about how any of it is presented or scheduled into
# a session.

add_subdirectory(geometry)
add_subdirectory(imaging)
add_subdirectory(physics)
add_subdirectory(material)
add_subdirectory(environment)
add_subdirectory(animation)
add_subdirectory(astro)
add_subdirectory(atmosphere)
add_subdirectory(terrain)
add_subdirectory(vfx)
add_subdirectory(ui)
add_subdirectory(audio)
add_subdirectory(input)
