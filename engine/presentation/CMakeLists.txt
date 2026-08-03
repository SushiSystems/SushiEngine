# presentation — the tier that turns what the world holds into pixels. It sits above asset and
# below world deliberately: the renderer is a service the world consumes, not a consumer of it,
# which is what the tree already said. EngineLayers refuses render -> simulation outright, so
# reordering the tiers cannot quietly legalise the renderer reaching into the simulation.

# The renderer needs the Vulkan, VMA, vk-bootstrap and glslang packages, which a plain configure
# does not provision, so it exists only when the option asks for it.
if(SUSHIENGINE_BUILD_RENDER)
    add_subdirectory(render)
endif()
