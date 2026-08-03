# foundation — the bottom tier: the shared value types, the ECS, the Execution seam that
# work is scheduled through, and the window/lifecycle seam. Nothing here may depend on a
# module above it, which is what makes it safe for everything else to depend on.

add_subdirectory(core)
add_subdirectory(ecs)
add_subdirectory(execution)

# platform/ is added by the root CMakeLists.txt instead, because whether it is built at
# all depends on options that file settles further down.
