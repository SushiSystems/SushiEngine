// The tail of SceneBlock, from planet_ring onward — included *inside* a `uniform SceneBlock
// { ... }` declaration by every shader that needs to reach the weather-field members at the
// very end of the block.
//
// std140 offsets are positional, so a shader cannot declare a later member without also
// declaring every earlier one. Four cloud shaders need the last three vec4s and previously
// stopped at sky_stars; rather than have each repeat (and eventually mis-copy) the same nine
// intervening members, they include this. Anything appended to `Scene::SceneUniforms` past
// light_shadow_b belongs here, in the same order.

vec4 planet_ring;      // x = near-field ring inner radius (m), y = outer radius (m); 0 = none
vec4 planet_precision; // ellipsoid terms formed in double; see scene_uniforms.hpp
// Every body lighting the scene, brightest first, 2 vec4 each:
// [2i+0] = direction to the body.xyz, irradiance; [2i+1] = colour.rgb, emits.
vec4 lights[10];
vec4 light_counts;    // x = light count
vec4 light_shadow_a;  // lights 0-3's punctual-atlas shadow record index, -1 = unshadowed
vec4 light_shadow_b;  // light 4 in x, y = ground wetness, zw spare
// The simulated weather field's addressing (atmosphere_system.md §7):
//   map.xy    = scene-XZ -> field UV scale, map.zw = offset (camera position already folded in)
//   levels.xyz = the three band centre altitudes, metres above the surface, ascending
//   levels.w   = 1 when a field was published this frame, 0 to ignore the field entirely
//   reference.xyz = the coverage the deck stack was compiled from, per band — the denominator
//                   that turns the field's absolute coverage into a scale about the observer
vec4 weather_field_map;
vec4 weather_field_levels;
vec4 weather_field_reference;
