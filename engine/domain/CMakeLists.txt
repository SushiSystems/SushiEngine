# domain — the tier that models the world itself: the shapes it is made of, the physics
# that moves them, and the systems that dress them. A domain module may read foundation
# and its own tier, and knows nothing about how any of it is presented or scheduled into
# a session.

add_subdirectory(geometry)
add_subdirectory(physics)
