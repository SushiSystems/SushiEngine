#ifndef ATMOSPHERE_COMMON_GLSL
#define ATMOSPHERE_COMMON_GLSL

// Shared parameterization for the Hillaire 2020 atmosphere LUT stack: the
// transmittance LUT (view-independent optical depth to the top of atmosphere) and
// the multiple-scattering LUT (the infinite-order isotropic scattering the single
// march cannot capture). Both LUTs are built by their own compute passes and sampled
// by sky.frag; keeping the (r, mu) ↔ uv mappings and the medium model in one included
// file is what guarantees the builder and the sampler agree bit for bit.
//
// The medium is spherical (mean radius), which is the universal assumption for
// scattering LUTs even under a WGS84 ground — the ellipsoid only matters for the hard
// ground intersection, which stays in sky.frag. Extinction matches sky.frag's own
// march: Rayleigh scattering doubles as its extinction, and Mie extinction is the Mie
// scattering scaled by the same 1.1 absorption factor the view march already applies.

const float ATMO_PI = 3.14159265359;

// LUT resolutions; the sampler mirrors these so a texel centre maps to the exact
// (r, mu) the builder integrated.
const float TRANSMITTANCE_LUT_WIDTH = 256.0;
const float TRANSMITTANCE_LUT_HEIGHT = 64.0;
const float MULTISCATTER_LUT_SIZE = 32.0;
const float SKY_VIEW_LUT_WIDTH = 192.0;
const float SKY_VIEW_LUT_HEIGHT = 108.0;
// Aerial-perspective froxel volume: a camera-frustum-aligned 3D grid of the in-scatter
// and transmittance from the camera out to each froxel's depth, so a mesh reads the air
// in front of it as one fetch instead of a march. Squared depth distribution packs the
// slices toward the camera, where the aerial gradient is strongest.
const float AERIAL_SLICES = 32.0;
const float AERIAL_MAX_DISTANCE = 32000.0;

// The atmosphere medium, unpacked from a pass's push constants or reconstructed by the
// sampler from the scene block. Radii are camera-independent (metres from the planet
// centre); coefficients are per-metre at sea level.
struct AtmosphereMedium
{
    vec3 rayleigh_scattering;
    float mie_scattering;
    float mie_extinction;
    float rayleigh_scale_height;
    float mie_scale_height;
    float bottom_radius;
    float top_radius;
};

float atmo_safe_sqrt(float x) { return sqrt(max(x, 0.0)); }

// Bruneton's half-texel remaps between a unit value and a texture coordinate, so the
// LUT's edge texels sit exactly on the parameter domain's boundary.
float atmo_unit_to_tex(float u, float n) { return 0.5 / n + u * (1.0 - 1.0 / n); }
float atmo_tex_to_unit(float u, float n)
{
    return clamp((u - 0.5 / n) / (1.0 - 1.0 / n), 0.0, 1.0);
}

// Rayleigh and Henyey-Greenstein Mie phase functions (atmo_-prefixed so a shader may
// include this alongside its own phase helpers without a redefinition).
float atmo_phase_rayleigh(float mu) { return 3.0 / (16.0 * ATMO_PI) * (1.0 + mu * mu); }
float atmo_phase_mie(float mu, float g)
{
    float g2 = g * g;
    return (1.0 / (4.0 * ATMO_PI)) * (1.0 - g2) /
           pow(max(1.0 + g2 - 2.0 * g * mu, 1e-4), 1.5);
}

// Ray versus a sphere of the given radius centred at c: returns (t_near, t_far), or a
// negative t_far on a miss. General centre (the sky-view build works camera-relative).
vec2 atmo_ray_sphere2(vec3 ro, vec3 rd, vec3 c, float radius)
{
    vec3 oc = ro - c;
    float b = dot(oc, rd);
    float cc = dot(oc, oc) - radius * radius;
    float h = b * b - cc;
    if (h < 0.0)
        return vec2(-1.0, -1.0);
    h = sqrt(h);
    return vec2(-b - h, -b + h);
}

// Nearest positive distance from a point at radius r with zenith cosine mu to the
// sphere of radius rad centred on the planet, or -1 on a miss.
float atmo_ray_sphere(float r, float mu, float rad)
{
    float disc = r * r * (mu * mu - 1.0) + rad * rad;
    if (disc < 0.0)
        return -1.0;
    disc = atmo_safe_sqrt(disc);
    float t0 = -r * mu - disc;
    float t1 = -r * mu + disc;
    if (t1 < 0.0)
        return -1.0;
    return t0 >= 0.0 ? t0 : t1;
}

// Scattering and extinction coefficients of the medium at radius r.
void atmo_coefficients(AtmosphereMedium m, float r, out vec3 scattering, out vec3 extinction)
{
    float altitude = max(r - m.bottom_radius, 0.0);
    float rho_rayleigh = exp(-altitude / m.rayleigh_scale_height);
    float rho_mie = exp(-altitude / m.mie_scale_height);
    scattering = m.rayleigh_scattering * rho_rayleigh + vec3(m.mie_scattering) * rho_mie;
    extinction = m.rayleigh_scattering * rho_rayleigh + vec3(m.mie_extinction) * rho_mie;
}

// --- transmittance LUT -----------------------------------------------------------

vec2 transmittance_lut_uv(float r, float mu, float bottom, float top)
{
    float H = atmo_safe_sqrt(top * top - bottom * bottom);
    float rho = atmo_safe_sqrt(r * r - bottom * bottom);
    float d = max(-r * mu + atmo_safe_sqrt(r * r * (mu * mu - 1.0) + top * top), 0.0);
    float d_min = top - r;
    float d_max = rho + H;
    float x_mu = (d - d_min) / max(d_max - d_min, 1e-6);
    float x_r = rho / max(H, 1e-6);
    return vec2(atmo_unit_to_tex(x_mu, TRANSMITTANCE_LUT_WIDTH),
                atmo_unit_to_tex(x_r, TRANSMITTANCE_LUT_HEIGHT));
}

void transmittance_lut_rmu(vec2 uv, float bottom, float top, out float r, out float mu)
{
    float x_mu = atmo_tex_to_unit(uv.x, TRANSMITTANCE_LUT_WIDTH);
    float x_r = atmo_tex_to_unit(uv.y, TRANSMITTANCE_LUT_HEIGHT);
    float H = atmo_safe_sqrt(top * top - bottom * bottom);
    float rho = H * x_r;
    r = atmo_safe_sqrt(rho * rho + bottom * bottom);
    float d_min = top - r;
    float d_max = rho + H;
    float d = d_min + x_mu * (d_max - d_min);
    mu = d == 0.0 ? 1.0 : clamp((H * H - rho * rho - d * d) / (2.0 * r * d), -1.0, 1.0);
}

// Optical-depth transmittance from radius r along zenith cosine mu to the top of the
// atmosphere, integrated directly (this is what the transmittance LUT stores).
vec3 transmittance_to_top(AtmosphereMedium m, float r, float mu)
{
    const int STEPS = 40;
    float dist = max(-r * mu + atmo_safe_sqrt(r * r * (mu * mu - 1.0) +
                                              m.top_radius * m.top_radius),
                     0.0);
    float dt = dist / float(STEPS);
    vec3 depth = vec3(0.0);
    for (int i = 0; i < STEPS; ++i)
    {
        float t = (float(i) + 0.5) * dt;
        float ri = atmo_safe_sqrt(r * r + t * t + 2.0 * r * t * mu);
        vec3 scattering, extinction;
        atmo_coefficients(m, ri, scattering, extinction);
        depth += extinction * dt;
    }
    return exp(-depth);
}

vec3 sample_transmittance(sampler2D lut, AtmosphereMedium m, float r, float mu)
{
    return texture(lut, transmittance_lut_uv(r, mu, m.bottom_radius, m.top_radius)).rgb;
}

// --- multiple-scattering LUT -----------------------------------------------------

vec2 multiscatter_lut_uv(float r, float mu_sun, float bottom, float top)
{
    float x = mu_sun * 0.5 + 0.5;
    float y = clamp((r - bottom) / max(top - bottom, 1.0), 0.0, 1.0);
    return vec2(atmo_unit_to_tex(x, MULTISCATTER_LUT_SIZE),
                atmo_unit_to_tex(y, MULTISCATTER_LUT_SIZE));
}

void multiscatter_lut_rmu(vec2 uv, float bottom, float top, out float r, out float mu_sun)
{
    float x = atmo_tex_to_unit(uv.x, MULTISCATTER_LUT_SIZE);
    float y = atmo_tex_to_unit(uv.y, MULTISCATTER_LUT_SIZE);
    mu_sun = clamp(x * 2.0 - 1.0, -1.0, 1.0);
    r = mix(bottom, top, y);
}

vec3 sample_multiscatter(sampler2D lut, AtmosphereMedium m, float r, float mu_sun)
{
    return texture(lut, multiscatter_lut_uv(r, mu_sun, m.bottom_radius, m.top_radius)).rgb;
}

// --- sky-view LUT ----------------------------------------------------------------
//
// A latitude-longitude image of the background sky's in-scattered radiance in the
// camera's local frame, marched once per frame at low resolution so a background pixel
// is a single fetch instead of a full march. The builder (a compute pass) and the
// sampler (sky.frag) derive the local frame and the elevation/azimuth mapping from the
// SAME functions here, which is what keeps the built sky aligned with the drawn one.

// The camera's local tangent frame from the planet centre in camera-relative space.
// Derived from the up vector alone so the builder and the sampler always agree.
void sky_view_frame(vec3 center_relative, out vec3 up, out vec3 right, out vec3 forward)
{
    up = normalize(-center_relative);
    vec3 seed = abs(up.y) < 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    right = normalize(cross(seed, up));
    forward = cross(up, right);
}

// The horizon's zenith angle from a camera at radius r_camera above a planet of the
// given bottom radius, and the angular span below it to the nadir.
void sky_view_horizon(float r_camera, float bottom, out float zenith_horizon, out float beta)
{
    float v_horizon = atmo_safe_sqrt(r_camera * r_camera - bottom * bottom);
    float cos_beta = v_horizon / max(r_camera, 1.0);
    beta = acos(clamp(cos_beta, -1.0, 1.0));
    zenith_horizon = ATMO_PI - beta;
}

// Direction (camera-relative) → sky-view uv. The elevation axis is split at the
// horizon with a square-root warp on each side (Hillaire), concentrating texels where
// the sky gradient is steepest; azimuth is linear and wraps at the ±pi seam.
vec2 sky_view_uv(vec3 up, vec3 right, vec3 forward, vec3 rd, float r_camera, float bottom)
{
    float cos_zenith = clamp(dot(rd, up), -1.0, 1.0);
    float azimuth = atan(dot(rd, forward), dot(rd, right));

    float zenith_horizon, beta;
    sky_view_horizon(r_camera, bottom, zenith_horizon, beta);
    float view_zenith = acos(cos_zenith);

    float y;
    if (view_zenith < zenith_horizon)
    {
        float coord = view_zenith / max(zenith_horizon, 1e-4);
        y = (1.0 - sqrt(1.0 - coord)) * 0.5;
    }
    else
    {
        float coord = (view_zenith - zenith_horizon) / max(beta, 1e-4);
        y = sqrt(clamp(coord, 0.0, 1.0)) * 0.5 + 0.5;
    }
    return vec2(azimuth / (2.0 * ATMO_PI) + 0.5, y);
}

// The inverse used by the builder: sky-view uv → direction (camera-relative).
vec3 sky_view_dir(vec3 up, vec3 right, vec3 forward, vec2 uv, float r_camera, float bottom)
{
    float azimuth = (uv.x - 0.5) * 2.0 * ATMO_PI;

    float zenith_horizon, beta;
    sky_view_horizon(r_camera, bottom, zenith_horizon, beta);

    float view_zenith;
    if (uv.y < 0.5)
    {
        float coord = 1.0 - 2.0 * uv.y;
        view_zenith = zenith_horizon * (1.0 - coord * coord);
    }
    else
    {
        float coord = 2.0 * uv.y - 1.0;
        view_zenith = zenith_horizon + beta * coord * coord;
    }

    float cos_zenith = cos(view_zenith);
    float sin_zenith = atmo_safe_sqrt(1.0 - cos_zenith * cos_zenith);
    return up * cos_zenith + right * (sin_zenith * cos(azimuth)) +
           forward * (sin_zenith * sin(azimuth));
}

vec3 sample_sky_view(sampler2D lut, vec3 center_relative, vec3 rd, float bottom)
{
    vec3 up, right, forward;
    sky_view_frame(center_relative, up, right, forward);
    float r_camera = length(center_relative);
    return texture(lut, sky_view_uv(up, right, forward, rd, r_camera, bottom)).rgb;
}

// --- aerial-perspective froxel volume --------------------------------------------

// View distance from the camera to the centre of froxel depth slice s.
float aerial_slice_center_distance(float s)
{
    float w = (s + 0.5) / AERIAL_SLICES;
    return w * w * AERIAL_MAX_DISTANCE;
}

// The froxel volume's depth texture coordinate for a view distance (inverse of the
// squared slice distribution).
float aerial_depth_texcoord(float view_distance)
{
    return sqrt(clamp(view_distance / AERIAL_MAX_DISTANCE, 0.0, 1.0));
}

// Sample the froxel volume for a pixel at the given screen uv and view distance:
// rgb = in-scatter per unit sun illuminance, a = scalar transmittance to that depth.
vec4 sample_aerial(sampler3D volume, vec2 screen_uv, float view_distance)
{
    return texture(volume, vec3(screen_uv, aerial_depth_texcoord(view_distance)));
}

#endif
