// The scene distance clipmap parameterization, shared by the pass that populates it and
// the probe tracer that marches it. Analytic primitive distances, camera-relative
// addressing, and a sphere-trace with a gradient normal — written once so the builder and
// the tracer can never disagree on where a voxel sits. No bindings and no version here.

struct SdfPrimitive
{
    vec4 center_kind; // xyz camera-relative centre, w = kind (0 box, 1 sphere, 2 cylinder)
    vec4 extent;      // xyz half-extents (box) or radius.x + half-height.y
    vec4 albedo;      // rgb surface albedo, w spare
};

struct SdfClipmapConfig
{
    vec4 origin_voxel;  // xyz camera-relative min corner, w = voxel size in metres
    ivec4 resolution;   // xyz voxel counts, w = live primitive count
    ivec4 extra;        // x = mesh-instance count, yzw spare
};

struct SdfMeshInstance
{
    mat4 inv_model;  // camera-relative world -> mesh local
    vec4 aabb_min;   // xyz local AABB min, w = brick slot
    vec4 aabb_max;   // xyz local AABB max, w = local-to-world distance scale
    vec4 albedo;     // rgb bounce albedo
};

// Axis-aligned analytic primitive distances (Inigo Quilez forms).
float sdf_box(vec3 p, vec3 b)
{
    vec3 q = abs(p) - b;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}

float sdf_sphere(vec3 p, float r)
{
    return length(p) - r;
}

float sdf_cylinder(vec3 p, float r, float h)
{
    vec2 d = vec2(length(p.xz) - r, abs(p.y) - h);
    return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
}

// Signed distance from a camera-relative world point to one primitive.
float sdf_primitive(SdfPrimitive prim, vec3 world)
{
    vec3 local = world - prim.center_kind.xyz;
    int kind = int(prim.center_kind.w + 0.5);
    if (kind == 1)
        return sdf_sphere(local, prim.extent.x);
    if (kind == 2)
        return sdf_cylinder(local, prim.extent.x, prim.extent.y);
    return sdf_box(local, prim.extent.xyz);
}

// Transforms a camera-relative world point into a mesh instance's brick and returns its
// [0,1] texcoord, or false when the point is outside the mesh's local AABB. Pure math: the
// brick fetch that follows references the atlas buffer the caller declares.
bool sdf_mesh_texcoord(SdfMeshInstance instance, vec3 world, out vec3 texcoord)
{
    vec3 local = (instance.inv_model * vec4(world, 1.0)).xyz;
    vec3 span = instance.aabb_max.xyz - instance.aabb_min.xyz;
    texcoord = (local - instance.aabb_min.xyz) / max(span, vec3(1e-6));
    return all(greaterThanEqual(texcoord, vec3(0.0))) && all(lessThanEqual(texcoord, vec3(1.0)));
}

// The camera-relative world centre of a voxel.
vec3 sdf_voxel_center(SdfClipmapConfig cfg, ivec3 voxel)
{
    return cfg.origin_voxel.xyz + (vec3(voxel) + 0.5) * cfg.origin_voxel.w;
}

// The [0,1] texture coordinate of a camera-relative world point in the clipmap.
vec3 sdf_texcoord(SdfClipmapConfig cfg, vec3 world)
{
    return (world - cfg.origin_voxel.xyz) / (cfg.origin_voxel.w * vec3(cfg.resolution.xyz));
}

// The interpolated signed distance at a point (the clipmap stores it in .a).
float sdf_sample_distance(sampler3D clipmap, SdfClipmapConfig cfg, vec3 world)
{
    return texture(clipmap, sdf_texcoord(cfg, world)).a;
}

// Central-difference gradient of the distance field: the surface normal at a hit.
vec3 sdf_gradient(sampler3D clipmap, SdfClipmapConfig cfg, vec3 world)
{
    float e = cfg.origin_voxel.w;
    vec3 g = vec3(
        sdf_sample_distance(clipmap, cfg, world + vec3(e, 0.0, 0.0)) -
            sdf_sample_distance(clipmap, cfg, world - vec3(e, 0.0, 0.0)),
        sdf_sample_distance(clipmap, cfg, world + vec3(0.0, e, 0.0)) -
            sdf_sample_distance(clipmap, cfg, world - vec3(0.0, e, 0.0)),
        sdf_sample_distance(clipmap, cfg, world + vec3(0.0, 0.0, e)) -
            sdf_sample_distance(clipmap, cfg, world - vec3(0.0, 0.0, e)));
    float len = length(g);
    return len > 1e-5 ? g / len : vec3(0.0, 1.0, 0.0);
}

// Sphere-traces the clipmap from a camera-relative origin along a unit direction. Returns
// true on a surface hit within max_distance, with the hit's albedo and gradient normal.
// Steps that leave the clipmap cube miss (the sky/ground fallback handles them).
bool sdf_trace(sampler3D clipmap, SdfClipmapConfig cfg, vec3 origin, vec3 dir,
               float max_distance, out vec3 hit_albedo, out vec3 hit_normal)
{
    float voxel = cfg.origin_voxel.w;
    float t = voxel; // start a voxel out so a probe on a surface does not self-hit
    for (int i = 0; i < 48; ++i)
    {
        vec3 p = origin + dir * t;
        vec3 tc = sdf_texcoord(cfg, p);
        if (any(lessThan(tc, vec3(0.0))) || any(greaterThan(tc, vec3(1.0))))
            return false;
        vec4 sampled = texture(clipmap, tc);
        float d = sampled.a;
        if (d < voxel * 0.75)
        {
            hit_albedo = sampled.rgb;
            hit_normal = sdf_gradient(clipmap, cfg, p);
            return true;
        }
        t += max(d, voxel * 0.5);
        if (t > max_distance)
            return false;
    }
    return false;
}
