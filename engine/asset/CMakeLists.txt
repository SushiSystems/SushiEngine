# asset — the tier that turns an authored file into the domain's own types, and the only tier
# allowed to link a format parser. An asset module may read foundation and domain, so an
# importer answers in the vocabulary the rest of the engine already speaks, and nothing above
# it ever learns which library read the file.

add_subdirectory(gltf)
