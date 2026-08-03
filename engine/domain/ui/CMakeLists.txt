# ui — the retained interface as ECS components (canvas, anchors, buttons, text) and the
# immediate draw list they flatten into, so a panel is entities the schedule steps and the
# renderer only ever sees vertices. Header-only.
sushiengine_add_module(NAME ui LAYER domain PUBLIC_DEPENDS core ecs)
