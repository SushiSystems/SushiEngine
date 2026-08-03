# physics — the constraint solvers and the deformable bodies built on them, each checked
# against a host reference or a closed form rather than against a picture.

# pgs_demo is the second of the two samples declared on both execution lanes and outside
# SE_BUILD_EXAMPLES: a hanging chain solved with Projected Gauss-Seidel is the smallest thing
# that proves a lane can actually run a solve.
sushiengine_add_sample(pgs_demo pgs_demo.cpp)

if(SUSHIENGINE_BUILD_SAMPLE_DEMOS)
    # pgs_demo's chain again, ported to the rigid-body XPBD generalization, and then generalized
    # once more into a pinned cloth grid over the same constraint type.
    sushiengine_add_sample(xpbd_demo xpbd_demo.cpp)
    sushiengine_add_sample(cloth_demo cloth_demo.cpp)

    # The volumetric XPBD lattice and positional contact resolution (Phase 4 physics).
    sushiengine_add_sample(soft_body_demo soft_body_demo.cpp)

    # soft_body_budget measures docs/slop/physics_system.md §13.1's soft-body line — one body,
    # 20 000 tetrahedra, 32 substeps, ≤ 3 ms/tick — and reports the number rather than asserting
    # it, because the target is stated against one desktop-class GPU and a suite that fails on
    # the hardware it happens to run on is a suite people stop reading. The scene's *shape* is
    # pinned by the conformance suite; this reports its *cost*.
    sushiengine_add_sample(soft_body_budget soft_body_budget.cpp)

    # §6.5's second half: the same cosmetic soft body stepped once with `float` storage and once
    # through `SoftBodyHalfStorage`'s `sycl::half` narrow/widen seam, reporting the wall-clock
    # difference rather than a keep-or-drop verdict — the measurement P8 gates that decision on.
    sushiengine_add_sample(soft_body_half_storage_budget soft_body_half_storage_budget.cpp)
endif()

# The sample scene is a library plus a generator rather than a translation unit each consumer
# compiles for itself, so it needs the live world. Only a configure that builds a shell or the
# test suite declares one.
if(TARGET sushi_sim)
    add_subdirectory(sample_scene)
endif()
