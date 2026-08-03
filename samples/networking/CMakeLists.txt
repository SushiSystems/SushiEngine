# networking — the client/server half of the loop. net_demo drives Loop::Net's loopback channel
# and reconciliation with a real per-tick gameplay command, modelling both roles as two ECS
# worlds in one process, which is the scope the loopback transport commits to.
sushiengine_add_sample(net_demo net_demo.cpp)
