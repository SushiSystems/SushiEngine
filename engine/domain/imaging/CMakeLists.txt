# imaging — pixel-buffer resampling and generic bounded caching, with no device and no file
# format knowledge of its own. It exists so the editor's thumbnail pipeline (which does own a
# device and a decoder) can keep its one piece of real algorithmic content — the downscale
# filter and the cache eviction order — testable without a GPU, the same reason `geometry` was
# split out from behind Vulkan.
sushiengine_add_module(NAME imaging LAYER domain
    SOURCES
        source/box_downscale.cpp)
