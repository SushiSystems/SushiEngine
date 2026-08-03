# animation — what a character's motion is: skeletons and clips, the animator state machine
# and blend trees that choose between them, the IK and pose modifiers that correct the result,
# and the evaluators that turn all of it into a skin palette. Header-only so its kernels
# instantiate inside the consuming translation unit, which is the safe way to ship device code
# — the batched device evaluator is the reason execution is a public dependency here.
sushiengine_add_module(NAME animation LAYER domain PUBLIC_DEPENDS core execution)
