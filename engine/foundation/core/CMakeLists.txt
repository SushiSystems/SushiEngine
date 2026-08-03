# core — the single seam the engine's scalar and vector types come from, plus the
# seeded generator every deterministic system draws its randomness from. Header-only
# and depends on nothing, so any module may reach it.
sushiengine_add_module(NAME core LAYER foundation)
