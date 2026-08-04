# Golden references

A golden here is one recorded frame, in text. `render_golden` renders a fixed scene at
512x288 for 12 frames, reads the output image back, and writes what it found: the frame's
FNV-1a hash, a 32x18 thumbprint of the image, and one hash per pass output. All of it is
plain text, so a change is reviewable in a diff rather than only as a hash that moved.

Two cases are recorded, one file each:

- **`opaque_lit.golden`** — the scene with shadows on, 15 pass outputs.
- **`opaque_unshadowed.golden`** — the same scene with the shadow pass switched off, 13 pass
  outputs, so a shadow regression is distinguishable from a shading one rather than
  reddening a single case.

The scene, the environment, the camera and the render settings are all built in code by
`tools/probes/render_golden/main.cpp` — no asset file, because an asset is a second thing
that can change underneath a reference. Dynamic resolution is off and the shadow cascade
count, shadow resolution and light shadow atlas size are pinned, so nothing in the image is
a function of how fast the machine ran or which quality tier it picked.

**A golden is a statement about one GPU and one driver.** The hash is over every byte of the
output image, so a driver update that changes rounding in a handful of pixels invalidates
it. That is why the thumbprint exists: on a mismatch the harness reports the largest and
mean per-channel distance against it, and a distance of a level or two is rounding while
tens of levels is a pass that stopped doing its job. Re-record deliberately, after looking
at the dump (`--dump`), never because the harness went red.

The set covers the mesh-shading half of the frame — depth prepass, shadow cascades, contact
shadows, ambient occlusion and its resolve, opaque, ground shadow resolve, screen-space
reflections, temporal resolve, bloom and tonemap. Sky and cloud are switched off at the
`Environment`, because that work is still moving and a golden that goes red every day is one
people learn to ignore. They join the set when it settles.

## The pass section

The whole-frame hash says the image changed. The `passes` section says *which pass* changed
it, which is the question a large port has to keep asking. Each line is

    <hash> <pass name> <texture name>

in the order the frame produced them. Entries are matched between runs by pass name, texture
name, and which repeat of that pair they are — so a *reordered* frame still compares
cleanly, while a *renamed* pass reads as a difference, which it is. Names are folded to a
single token (spaces and tabs become underscores) on both sides of every comparison, so the
recorded spelling always equals the live one.

Three findings, deliberately distinguished on a mismatch: `gone` (the golden had this output
and this run did not — the pass was culled or removed), `new` (the reverse), and `changed`
(same output, different bytes — the one a bisect is looking for).

A frame whose image is byte-identical but whose pass hashes differ is *also* reported. That
is either a refactor doing exactly what it promised to do internally, or a bug the final
image happened to hide.

## What the pass hashes do not cover

Stated because a hash that silently covers less than it appears to is worse than none:

- **Mip 0 only.** A regression confined to a lower mip is invisible here.
- **The depth aspect only**, for depth/stencil targets. A stencil-only change is invisible.
- **Formats the capture can size.** Anything block-compressed or otherwise unlisted is
  reported as un-copyable rather than guessed at, and simply does not appear.
- **What fits the per-frame staging budget**, which is 256 MiB per frame slot
  (`PassCapture::DEFAULT_BUDGET`). Outputs past it are dropped and counted — and a run that
  dropped any refuses to record a golden at all, because an incomplete capture is not a
  smaller reference, it is one that cannot notice the passes it never saw. The budget is
  sized against the frame's largest single output rather than its total, since the shadow
  cascade atlas dwarfs every screen-sized target beneath it.
- **Passes still being rewritten**, excluded by name: anything matching `cloud`, `sky`,
  `weather`, or `atmosphere`. Those passes register and write whether or not the
  `Environment` switched their effect off, so switching the effect off is not enough to keep
  them out. Note the residual risk this does *not* remove: those passes still write into the
  shared scene target, so a change in what they write while disabled can still move the
  whole-frame hash.

A golden is also refused when the recorded extent or frame count does not match the build's.
A reference taken at another size or after another number of frames answers a different
question, and silently comparing against it is the one failure mode a regression harness
cannot have.

## Capture changes how the frame allocates

Capture copies out of every transient a pass writes, so under capture every transient gains
`TRANSFER_SRC` usage — and usage is part of the transient pool's reuse key. A captured frame
therefore aliases its transients differently from an uncaptured one.

Contents are unaffected by that, with one honest exception: a pass reading a transient it
never initialised. That is a bug, and this is how to catch it — the goldens are recorded
with capture on, so `--no-capture` re-renders the frame the way a shipping build allocates
it and compares the whole-frame hash against the same reference. A frame that only matches
with capture on is that bug.

`--update` refuses to run with `--no-capture`: a golden without a pass section is not a
golden, and a harness that can quietly weaken its own baseline is not one either.

## Running the harness

    se render --probe golden                    # compare against what is here
    se render --probe golden -- --dump          # ...and write a PPM of any mismatch
    se render --probe golden -- --update        # re-record, deliberately
    se render --probe golden -- --no-capture    # whole-frame comparison only
    se render --probe golden -- --goldens DIR   # read and write references elsewhere

With no flags the harness compares and prints one line per case: `OK`, or `MISMATCH` with
the two hashes, the thumbprint distance, a verdict on whether that distance looks like
driver rounding, and the per-pass difference lines. It exits non-zero if any case failed.
`--dump` writes `<case>_actual.ppm` into the working directory — the repository root, when
the probe is launched by `se render` — for every case whose frame hash differs.

The build points the harness at this directory (`SE_GOLDEN_DIR`), so it works from any
working directory. `--goldens` overrides that, which is how to record a set somewhere else
without touching the checked-in one.

## When a golden legitimately changes

`--update` is an act, not a remedy. A red run is the harness doing its job, so:

1. Read the thumbprint distance. One or two levels is rounding; tens is a visible change.
2. Read the per-pass lines. They name the pass that moved, which is usually the whole answer
   for a change you meant to make and the start of the investigation for one you did not.
3. Look at the dump before deciding the new image is the right one.
4. Re-run with `--no-capture`. If the whole-frame hash matches only with capture on, the
   difference is a pass reading a transient it never wrote — fix that rather than recording
   it.
5. Only then `--update`, and review the resulting diff: the thumbprint rows and the pass
   hashes both change visibly in text, so the commit shows what was accepted.

Recording is otherwise automatic in exactly one situation: a case with no file here at all is
written on the first run rather than reported as a failure. That is why deleting a golden to
"reset" it is not a safe move — it silently becomes whatever the next run produces, on
whatever machine ran it.
