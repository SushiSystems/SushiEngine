# sandbox — the ECS worked example, and one of the two samples declared on both execution
# lanes and outside SUSHIENGINE_BUILD_EXAMPLES. `se run` defaults to it, and the native lane has nothing
# else that exercises a schedule end to end.
sushiengine_add_sample(sandbox main.cpp)
