# atmosphere — T1, the global dynamical core, and the Fourier transform its elliptic inversion
# is built on. It reaches no device and no graphics stack, deliberately: gameplay queries, the
# regional nest's parent forcing and a headless probe all ask it what the weather is a thousand
# kilometres away, and none of the three should have to bring one up to find out
# (docs/slop/atmosphere_system.md §5). The regional nest (T2) is a Vulkan compute service and
# stays beside the renderer; the seam between the two tiers is plain data.
sushiengine_add_module(NAME atmosphere LAYER domain PUBLIC_DEPENDS core
    SOURCES
        source/climatology.cpp
        source/fourier_transform.cpp
        source/quasigeostrophic_core.cpp)

# Position-independent because sushi_sim, its one library consumer, is.
set_target_properties(sushiengine_atmosphere PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The name the simulation, the test suite and the global probe link.
add_library(sushi_atmosphere ALIAS sushiengine_atmosphere)
