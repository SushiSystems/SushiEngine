// The clouds' shadow on a lit surface, cheap enough for the geometry pass.
//
// The sky pass already shadows the analytic ground by marching the full cloud medium,
// which it can afford because it has the three noise volumes bound and is marching that
// medium anyway. The geometry pass has neither, so this takes the same physical quantity
// — transmittance of direct sunlight down through the deck — from the one texture that
// carries the large-scale cloud field: the weather map. That is also the only scale that
// matters here. A cumulus deck's shadow on the ground is kilometres of soft light and
// shade, not the metre-scale detail the erosion noise adds, so one warped lookup along
// the sun ray reproduces what the eye actually reads for the cost of two fetches.
//
// The parameterisation deliberately matches the sky pass's, warp included, so a surface
// mesh sitting on the analytic ground is shadowed by the same cloud the ground is.

// Where the sun ray leaves the deck, projected onto the plane tangent to the planet.
// Requires the scene block to have been declared through cloud_deck_d.
float cloud_sun_transmittance(sampler2D weather_texture, vec3 position, vec3 sun)
{
    if (scene.misc.w <= 0.5 || scene.cloud_global.x <= 0.0 || scene.cloud_global.w <= 0.0)
        return 1.0;

    vec3 centre = scene.planet_center.xyz;
    float deck = scene.planet_center.w + (scene.cloud_global.y + scene.cloud_global.z) * 0.5;

    // Where the ray from the surface toward the sun crosses the middle of the deck.
    vec3 offset = position - centre;
    float b = dot(offset, sun);
    float c = dot(offset, offset) - deck * deck;
    float discriminant = b * b - c;
    if (discriminant <= 0.0)
        return 1.0;
    float distance = -b + sqrt(discriminant);
    if (distance <= 0.0)
        return 1.0;

    vec3 hit = position + sun * distance;
    vec3 up = normalize(hit - centre);
    // The same tangent basis the sky pass builds, so both agree on where a cloud is.
    vec3 t1 = normalize(cross(up, vec3(0.1, 0.0, 1.0)));
    vec3 t2 = cross(up, t1);
    vec3 surface = hit - centre;

    float weather_scale = max(scene.cloud_deck_d[0].y, 1.0);
    vec2 uv = vec2(dot(surface, t1), dot(surface, t2)) / weather_scale;
    vec2 warp = texture(weather_texture, uv * 0.15 + vec2(0.37)).rg - 0.5;
    uv += warp * 0.9;
    float coverage = texture(weather_texture, uv).r;

    // Coverage is a fraction of sky filled, not an optical depth, so it is turned into
    // one before Beer's law rather than used as an opacity directly — otherwise a fully
    // covered sky would still let 0% through as a hard black rather than as deep shade.
    float thickness = max(scene.cloud_global.z - scene.cloud_global.y, 1.0);
    float optical_depth = coverage * coverage * thickness * scene.cloud_light.x * 0.0009;
    // A grazing sun travels much further through the deck than an overhead one.
    float slant = clamp(abs(dot(up, sun)), 0.15, 1.0);
    float transmittance = exp(-optical_depth / slant);
    return mix(1.0, transmittance, clamp(scene.cloud_global.x, 0.0, 1.0));
}
