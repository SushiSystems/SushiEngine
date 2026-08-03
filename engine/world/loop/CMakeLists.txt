# loop — the orchestration surface a session is driven through: Loop::App, the fixed-timestep
# clock, the rollback ring and the reconciliation the network path replays against. World rather
# than domain because it schedules the model instead of being part of it, which is what turns a
# domain module reaching up here for a clock or a seed into an edge the configure refuses.
# Header-only, so its systems instantiate inside the consuming translation unit.
sushiengine_add_module(NAME loop LAYER world PUBLIC_DEPENDS core ecs execution)
