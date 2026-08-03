# world — the tier that runs a session: the loop that drives it, the live simulation behind the
# ISimulation seam, the file shapes a session is saved as, and the authoring services that edit
# one. It may read every tier below it, which is what makes the renderer a service the world
# consumes rather than a consumer of the world.

add_subdirectory(loop)

# simulation/, serialization/ and authoring/ are added by the root CMakeLists.txt instead,
# because whether they are built at all depends on options that file settles further down.
