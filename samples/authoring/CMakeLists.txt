# authoring — how a game is written against the engine, as opposed to what a subsystem does
# internally. Both demos here are about the surface a game author touches.

# first_game is the smallest complete game written against the settled SushiLoop authoring API
# (Loop::App): pure-ECS systems, a fixed-step deterministic loop, and multiplayer reached by one
# connect() — headless and self-checking.
sushiengine_add_sample(first_game first_game.cpp)

# ui_demo exercises the ECS UI (canvas + button): anchor layout, pointer-driven button states,
# and click callbacks, checked headlessly against the draw list the renderer would consume.
sushiengine_add_sample(ui_demo ui_demo.cpp)
