#ifndef BLUE_NOISE_GLSL
#define BLUE_NOISE_GLSL

// The one sampling source every screen-space stochastic pass draws from — sun and
// punctual shadows, GTAO, SSR, contact shadows, and the output dither all take their
// per-pixel offset from here instead of each hashing its own. Centralising it (SRP) is
// what lets the whole engine's noise be swapped to a baked spatiotemporal blue-noise
// texture in one place later (§5.1); today it is the interleaved-gradient hash, which is
// already spatially even, plus a frame-advance for passes that animate without binding
// the temporal block.

// Interleaved gradient noise (Jimenez, Siggraph 2014). A white-noise hash clumps: runs
// of neighbouring pixels land on similar values and the eye reads the clumps as dirty
// speckle. This spreads the values so every 3x3 neighbourhood spans the whole range —
// smoother to look at raw, and exactly what a 3x3 temporal-resolve neighbourhood expects.
float interleaved_gradient_noise(vec2 pixel)
{
    return fract(52.9829189 * fract(dot(pixel, vec2(0.06711056, 0.00583715))));
}

// The same hash advanced by a frame index, for a pass that animates its offset per frame
// but does not bind the temporal block (where temporal_dither() belongs instead). Frame N
// shifts the lattice by N scrambled steps, so successive frames decorrelate while each one
// stays spatially even; the 64-frame wrap keeps the argument small enough to stay exact.
float interleaved_gradient_noise(vec2 pixel, uint frame)
{
    return interleaved_gradient_noise(pixel + 5.588238 * float(frame & 63u));
}

// A triangular-PDF dither in [-amplitude, amplitude] from two decorrelated draws. Applied
// before an N-bit quantise it spreads each level's rounding across its boundary, so a
// smooth gradient dissolves into sub-threshold grain instead of printing bands.
float tpdf_dither(vec2 pixel, float amplitude)
{
    float d1 = interleaved_gradient_noise(pixel);
    float d2 = interleaved_gradient_noise(pixel + vec2(113.0, 271.0));
    return (d1 + d2 - 1.0) * amplitude;
}

#endif // BLUE_NOISE_GLSL
