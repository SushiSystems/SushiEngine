# The engine's C++ modules, one directory per tier of the order
# cmake/EngineLayers.cmake states. Each tier directory adds the modules it owns, so
# this file never has to know what a tier contains.
add_subdirectory(foundation)
