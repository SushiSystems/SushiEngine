# Golden references (RHI0)

One file per case, written by `se render --probe golden -- --update`. Each records the frame's
FNV-1a hash, a 32x18 thumbprint of it, and one hash per pass output — all in text,
so a change is reviewable in a diff rather than only as a hash that moved.

**A golden here is a statement about one GPU and one driver.** The hash is over
every byte of the output image, so a driver update that changes rounding in a
handful of pixels invalidates it. That is why the thumbprint exists: on a mismatch
the harness reports the largest and mean per-channel distance against it, and a
distance of a level or two is rounding while tens of levels is a pass that stopped
doing its job. Re-record deliberately, after looking at the dump (`--dump`), never
because the harness went red.

The set covers the mesh-shading half of the frame — depth, shadows, opaque, tonemap,
temporal resolve. Sky and cloud are switched off at the `Environment`, because that
work is still moving and a golden that goes red every day is one people learn to
ignore. They join the set when it settles.

## The pass section

The whole-frame hash says the image changed. The `passes` section says *which pass*
changed it, which is the question a 39-pass port has to keep asking. Each line is

    <hash> <pass name> <texture name>

in the order the frame produced them. Entries are matched between runs by pass name,
texture name, and which repeat of that pair they are — so a *reordered* frame still
compares cleanly, while a *renamed* pass reads as a difference, which it is.

Three findings, deliberately distinguished on a mismatch: `gone` (the golden had this
output and this run did not — the pass was culled or removed), `new` (the reverse),
and `changed` (same output, different bytes — the one a bisect is looking for).

A frame whose image is byte-identical but whose pass hashes differ is *also* reported.
That is either a refactor doing exactly what it promised to do internally, or a bug
the final image happened to hide.

## What the pass hashes do not cover

Stated because a hash that silently covers less than it appears to is worse than none:

- **Mip 0 only.** A regression confined to a lower mip is invisible here.
- **The depth aspect only**, for depth/stencil targets. A stencil-only change is invisible.
- **Formats the capture can size.** Anything block-compressed or otherwise unlisted is
  reported as un-copyable rather than guessed at, and simply does not appear.
- **What fits the per-frame staging budget.** Outputs past it are dropped and counted —
  and a run that dropped any refuses to record a golden at all, because an incomplete
  capture is not a smaller reference, it is one that cannot notice the passes it never
  saw. The first budget tried (96 MiB) was spent almost entirely by the cascade atlas
  and truncated the frame at sixteen outputs; the harness now also pins the shadow
  resolution, which both bounds that cost and keeps the image off the quality tier.
- **Passes still being rewritten**, excluded by name: anything matching `cloud`, `sky`,
  `weather`, or `atmosphere`. Those passes register and write whether or not the
  `Environment` switched their effect off, so switching the effect off is not enough to
  keep them out. Note the residual risk this does *not* remove: those passes still write
  into the shared scene target, so a change in what they write while disabled can still
  move the whole-frame hash.

## Capture changes how the frame allocates

Capture copies out of every transient a pass writes, so under capture every transient
gains `TRANSFER_SRC` usage — and usage is part of the transient pool's reuse key. A
captured frame therefore aliases its transients differently from an uncaptured one.

Contents are unaffected by that, with one honest exception: a pass reading a transient
it never initialised. That is a bug, and this is how to catch it — the goldens are
recorded with capture on, so `render_golden --no-capture` re-renders the frame the way
a shipping build allocates it and compares the whole-frame hash against the same
reference. A frame that only matches with capture on is that bug.

`--update` refuses to run with `--no-capture`: a golden without a pass section is not
a golden, and a harness that can quietly weaken its own baseline is not one either.

## Recording one

    se render --probe golden                 # compare against what is here
    se render --probe golden -- --dump       # ...and write a PPM of any mismatch
    se render --probe golden -- --update     # re-record, deliberately

`--update` is an act, not a remedy. A red run is the harness doing its job; look at the
dump and the per-pass lines first, and re-record only once the change is understood and
wanted.

No baseline is checked in yet. Every recording attempted so far predated a fix that was
found by running the harness — a name-normalisation bug in the harness itself, and an
under-declared image usage that hid the whole post-processing tail from the per-pass
half. A reference is worth keeping once nothing is still being fixed underneath it.
