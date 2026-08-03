# material — the authored surface: its maps and scalars, the ids the renderer registers a
# texture or a mesh under, and the alpha, blend, cull and wrap state that decides how it is
# drawn. Header-only and graphics-free, so the importer that writes a surface, the serializer
# that persists it and the inspector that edits it all describe it in the same vocabulary
# without any of them naming a device.
sushiengine_add_module(NAME material LAYER domain PUBLIC_DEPENDS core)
