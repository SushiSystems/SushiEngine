// The tail of SceneBlock, from planet_ring onward — included *inside* a `uniform SceneBlock
// { ... }` declaration by every shader that needs to reach the weather-field members at the
// very end of the block.
//
// std140 offsets are positional, so a shader cannot declare a later member without also
// declaring every earlier one. Four cloud shaders need the last three vec4s; rather than have
// each repeat (and eventually mis-copy) the same nine intervening members, they include this.
// Anything appended to `Scene::SceneUniforms` past light_shadow_b belongs here, in the same
// order.

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
// Only the cloudscape *bake* reads these now: with §7.4's per-column genus the weather field
// is a bake input, not a per-march-sample correction, so the march never touches it.
vec4 weather_field_map;
vec4 weather_field_levels;
// The two camera-centred cloudscape windows the T3 bake writes (atmosphere_system.md §7.2).
// Both map camera-relative XZ metres straight to the window's [0, 1] UV — xy = scale,
// zw = offset, with the eye and the wind residual since the last bake already folded in on
// the CPU in double. The near window is the detailed one the march, the light volume and the
// cloud shadow map all share; the far one carries the same simulated structure out to the
// horizon at a coarser texel, and is what a sample past the near window reads instead of a
// clamped edge.
//   params.x = near window span, metres          params.y = far window span, metres
//   params.z = near skip-field cell, metres      params.w = far field texel, metres
// A window that has never been baked publishes span 0, which cloud_field_window.glsl reads
// as "no field" and every consumer then treats as clear sky.
vec4 cloud_field_near;
vec4 cloud_field_far;
vec4 cloud_field_params;
// The GPU regional nest's optical extinction field (atmosphere_system.md §7.1), addressed for
// the cloudscape bake. This is where cloud *shape* comes from when the nest is running: the
// bake reads how much water is actually suspended here rather than instantiating a genus
// profile, so a cumulus has the outline the condensate has.
//   nest_map.xy    = camera-relative XZ metres -> the nest's horizontal UV
//   nest_map.zw    = the matching offset
//   nest_params.x  = the nest's domain top, metres above the surface
//   nest_params.y  = the inverse of the vertical stretch exponent, so altitude -> W is one pow
//   nest_params.z  = 1 when the nest is running and the bake should read it, 0 otherwise
//   nest_params.w  = the extinction of 1 g/m^3 of liquid water, the scale sigma is stated against
vec4 atmosphere_nest_map;
vec4 atmosphere_nest_params;
// The pattern frame the cloudscape windows were baked in (CloudsV2): xy = the near window's
// texel-(0,0) corner in scene-absolute-plus-wind metres, zw = the far window's. The view
// march's analytic carve reconstructs `pattern = origin + window_uv * span` from these so
// its noise is evaluated in exactly the frame the bake evaluated the envelope's weather in —
// the shapes stand still in the world and advect with the wind, like the envelope itself.
// float32 like the bake's own push constants, which already accept that precision.
vec4 cloud_field_pattern;
// Where the weather is over the whole body (atmosphere_system.md, WM-SEED). This is what the
// cloud march reads out past every baked window, where the two camera-centred windows above
// have nothing to say and a lattice of any affordable size would resolve nothing: at planetary
// distance one texel of a 64-cell global grid is six hundred kilometres.
//   synoptic_params.x = populated centre count      synoptic_params.y = ITCZ latitude (rad)
//   synoptic_params.z = 1 when the body has a latitudinal cloud structure at all
// The last two say different things: a planet with an atmosphere has an ITCZ and a clear
// subtropical belt even with no systems placed on it, so a provider publishing zero centres
// still wants the climatology.
//   centre_a[i].xyz = unit direction from the planet centre, **already in scene space**
//   centre_a[i].w   = chord falloff; weight is exp(-w * (1 - dot(radial, xyz)))
//   centre_b[i].x   = signed coverage anomaly (+ a low, − a high)
//   centre_b[i].y   = convective fraction     centre_b[i].z = surface precipitation
vec4 synoptic_params;
vec4 synoptic_centre_a[12];
vec4 synoptic_centre_b[12];
