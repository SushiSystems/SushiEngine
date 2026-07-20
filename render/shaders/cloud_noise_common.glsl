// Tileable Perlin and Worley noise, shared by the cloud-volume and weather-map
// compute shaders. This is the GPU port of the recipe the volumes were previously
// built with on the CPU (Schneider/Guerrilla "Nubis"): Perlin fbm supplies connected
// shapes, inverted Worley fbm supplies the puffy billows, and remapping the first by
// the second yields cloud-like base density. Every lattice lookup wraps on its own
// period so the volumes tile seamlessly under a REPEAT sampler.

float noise_fade(float t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

float noise_remap(float v, float a, float b, float c, float d)
{
    return c + (v - a) / (b - a) * (d - c);
}

uint noise_hash(uint x)
{
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

uint noise_hash3(int x, int y, int z, int period)
{
    int px = ((x % period) + period) % period;
    int py = ((y % period) + period) % period;
    int pz = ((z % period) + period) % period;
    return noise_hash(uint(px) * 73856093u ^ uint(py) * 19349663u ^ uint(pz) * 83492791u);
}

float noise_rand01(uint h) { return float(h & 0xffffffu) / float(0x1000000); }

float noise_gradient_dot(int ix, int iy, int iz, vec3 f, int period)
{
    const vec3 G[16] = vec3[16](
        vec3(1, 1, 0),  vec3(-1, 1, 0),  vec3(1, -1, 0), vec3(-1, -1, 0),
        vec3(1, 0, 1),  vec3(-1, 0, 1),  vec3(1, 0, -1), vec3(-1, 0, -1),
        vec3(0, 1, 1),  vec3(0, -1, 1),  vec3(0, 1, -1), vec3(0, -1, -1),
        vec3(1, 1, 0),  vec3(-1, 1, 0),  vec3(0, -1, 1), vec3(0, -1, -1));
    uint h = noise_hash3(ix, iy, iz, period) & 15u;
    return dot(G[h], f);
}

float noise_perlin(vec3 p, int period)
{
    ivec3 i = ivec3(floor(p));
    vec3 f = p - vec3(i);
    float u = noise_fade(f.x);
    float v = noise_fade(f.y);
    float w = noise_fade(f.z);

    float n000 = noise_gradient_dot(i.x, i.y, i.z, f, period);
    float n100 = noise_gradient_dot(i.x + 1, i.y, i.z, f - vec3(1, 0, 0), period);
    float n010 = noise_gradient_dot(i.x, i.y + 1, i.z, f - vec3(0, 1, 0), period);
    float n110 = noise_gradient_dot(i.x + 1, i.y + 1, i.z, f - vec3(1, 1, 0), period);
    float n001 = noise_gradient_dot(i.x, i.y, i.z + 1, f - vec3(0, 0, 1), period);
    float n101 = noise_gradient_dot(i.x + 1, i.y, i.z + 1, f - vec3(1, 0, 1), period);
    float n011 = noise_gradient_dot(i.x, i.y + 1, i.z + 1, f - vec3(0, 1, 1), period);
    float n111 = noise_gradient_dot(i.x + 1, i.y + 1, i.z + 1, f - vec3(1, 1, 1), period);

    float nx00 = mix(n000, n100, u);
    float nx10 = mix(n010, n110, u);
    float nx01 = mix(n001, n101, u);
    float nx11 = mix(n011, n111, u);
    return mix(mix(nx00, nx10, v), mix(nx01, nx11, v), w) * 0.5 + 0.5;
}

float noise_perlin_fbm(vec3 p, int freq, int octaves)
{
    float sum = 0.0;
    float amp = 0.5;
    float total = 0.0;
    int f = freq;
    for (int i = 0; i < octaves; ++i)
    {
        sum += amp * noise_perlin(p * float(f), f);
        total += amp;
        amp *= 0.5;
        f *= 2;
    }
    return sum / total;
}

float noise_worley(vec3 p, int freq)
{
    vec3 scaled = p * float(freq);
    ivec3 base = ivec3(floor(scaled));
    float md = 1e9;
    for (int dz = -1; dz <= 1; ++dz)
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
            {
                ivec3 cell = base + ivec3(dx, dy, dz);
                uint h = noise_hash3(cell.x, cell.y, cell.z, freq);
                vec3 feature = vec3(cell) + vec3(noise_rand01(h),
                                                 noise_rand01(noise_hash(h + 1u)),
                                                 noise_rand01(noise_hash(h + 2u)));
                vec3 e = feature - scaled;
                md = min(md, dot(e, e));
            }
    return 1.0 - min(sqrt(md), 1.0);
}

float noise_worley_fbm(vec3 p, int freq)
{
    return noise_worley(p, freq) * 0.625 + noise_worley(p, freq * 2) * 0.25 +
           noise_worley(p, freq * 4) * 0.125;
}
