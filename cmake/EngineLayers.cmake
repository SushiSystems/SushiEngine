# EngineLayers — the architecture's shape as data: the tier order, which tier every
# module sits in, and the edges that are refused outright. A module exists to the build
# only once it is listed in SUSHIENGINE_MODULE_LAYERS below, so adding a module means
# adding it here first; sushiengine_add_module() rejects a name it cannot find.

# Lowest tier first. A module may depend on its own tier and on anything below it, and
# `application` covers applications/editor and applications/player.
set(SUSHIENGINE_LAYER_ORDER
    foundation
    domain
    asset
    presentation
    world
    application)

# Flat <module>;<layer> pairs rather than one list per tier, so a module's tier resolves
# by name whatever order the module directories happen to be added in.
set(SUSHIENGINE_MODULE_LAYERS
    core            foundation
    ecs             foundation
    execution       foundation
    platform        foundation
    geometry        domain
    physics         domain
    material        domain
    environment     domain
    animation       domain
    astro           domain
    atmosphere      domain
    terrain         domain
    vfx             domain
    ui              domain
    audio           domain
    input           domain
    gltf            asset
    model           asset
    render          presentation
    loop            world
    simulation      world
    serialization   world
    authoring       world
    editor          application
    player          application)

# Invariants stated directly rather than left to the tier comparison, as
# <consumer>;<dependency> pairs. Deliberately redundant while world sits above
# presentation, so that reordering SUSHIENGINE_LAYER_ORDER cannot quietly legalise the
# renderer reaching into the simulation.
set(SUSHIENGINE_FORBIDDEN_EDGES
    render          simulation)

# Enumerated debt, as <consumer>;<dependency>;<date>;<reason>. The list exists so that an
# upward edge that has to be tolerated is written down and dated rather than silently
# legalised, and it may only ever shrink. Empty: there is nothing to excuse today.
set(SUSHIENGINE_LAYER_EXCEPTIONS "")

# sushiengine_layer_index(<layer> <output_variable>)
#   Sets <output_variable> to the layer's position in SUSHIENGINE_LAYER_ORDER, or to -1
#   when <layer> is not a tier this repository defines.
function(sushiengine_layer_index layer output_variable)
    list(FIND SUSHIENGINE_LAYER_ORDER "${layer}" position)
    set(${output_variable} ${position} PARENT_SCOPE)
endfunction()

# sushiengine_module_layer(<module> <output_variable>)
#   Sets <output_variable> to the module's tier from SUSHIENGINE_MODULE_LAYERS, or to the
#   empty string when the module is not manifested. Strides the pair list two at a time,
#   so a module name can never be matched against a tier slot.
function(sushiengine_module_layer module output_variable)
    set(${output_variable} "" PARENT_SCOPE)

    list(LENGTH SUSHIENGINE_MODULE_LAYERS entry_count)
    math(EXPR last_pair "${entry_count} - 2")
    set(index 0)
    while(index LESS_EQUAL last_pair)
        list(GET SUSHIENGINE_MODULE_LAYERS ${index} listed_module)
        if("${listed_module}" STREQUAL "${module}")
            math(EXPR layer_slot "${index} + 1")
            list(GET SUSHIENGINE_MODULE_LAYERS ${layer_slot} listed_layer)
            set(${output_variable} "${listed_layer}" PARENT_SCOPE)
            return()
        endif()
        math(EXPR index "${index} + 2")
    endwhile()
endfunction()
