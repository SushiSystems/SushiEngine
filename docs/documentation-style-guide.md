# Documentation style guide

[`CONTRIBUTING.md`](CONTRIBUTING.md) says what must be documented. This page says how, and
sets the length ceilings a machine can check — because the "short bullet, not an essay" rule
has existed in prose since the project started and has been ignored the whole time. A rule
nothing enforces is a preference.

`tools/documentation/check_documentation_length.py` enforces every ceiling below and runs in
continuous integration.

## Where a document goes

| Directory | Holds | Answers |
| --- | --- | --- |
| `docs/` root | `README.md`, `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, this guide | Project conduct and contribution rules. The three community-health files stay here because GitHub resolves them from the repository root, `docs/` and `.github/` only. |
| `docs/getting-started/` | The tour a first-time reader takes | "How do I run this?" |
| `docs/architecture/` | One chapter per subject, grouped by tier | "How does it work?" |
| `docs/modules/` | An index into the per-module `README.md` files under `engine/` | "What is here?" |
| `docs/guides/` | Task-shaped walkthroughs — the command line tool, vehicles | "How do I do X?" |
| `docs/reference/` | The changelog, the glossary | "What is this thing called?" |
| `docs/design/` | Engineering plans and their audits | "Why is it shaped this way, and what is planned?" |

A module's own facts — what it owns, its tier, its dependencies, its tests — live in
`engine/<tier>/<module>/README.md` and nowhere else. `docs/modules/README.md` links to them and
holds no facts of its own. Two files stating the same fact drift apart; the one beside the code
is the one a change is forced to walk past.

`docs/design/` is a record of intent, not a description of the tree. It may describe work that
does not exist. Everything else in `docs/` describes what is built today, and a sentence that
stops being true is a defect in the change that made it false.

## Length ceilings

| Subject | Ceiling | Why |
| --- | --- | --- |
| Prose line | 100 columns | Matches `.clang-format`'s `ColumnLimit`, so a diff is readable in the same width as the code it describes. |
| Paragraph | 1,200 characters | Past that a paragraph is carrying more than one argument. Split it at the seam. |
| Changelog bullet | 240 characters, hard failure past 400 | A changelog says what changed. Why it changed belongs in a design document, which the bullet may cite. |
| File | 900 lines | Past that a reader cannot hold the shape of the file. Split it by subject. |

Exempt from the line and file ceilings:

- Table rows, which markdown cannot wrap.
- Fenced code blocks, which wrap where the language allows and nowhere else.
- A line whose overflow is a single unbreakable token, such as a URL.
- The changelog, from the **file** ceiling. It grows by append and splits at a release
  boundary, not by subject, so the ceiling would only ever ask for a split that makes the
  history harder to read. Its bullets carry a stricter ceiling of their own instead.
- `docs/design/` and `docs/api/`, from the **file and paragraph** ceilings. A design document
  is a record of one long argument, and splitting it scatters the argument; the generated
  reference is not hand-written at all. Both still obey the line ceiling and must not carry a
  broken link.

## Voice

Write the way [`docs/architecture/`](architecture/README.md) already does.

- **Declarative.** State what the system does. Not "we decided to make the solver…", just
  "the solver…".
- **Specific.** Name the type, the file, the number. "The cache evicts least-recently-bound"
  beats "the cache uses a smart eviction policy".
- **Honest about gaps.** A module with no tests says so, and says why. Documentation that
  hides a hole is worse than no documentation, because it stops anyone from looking.
- **No marketing.** No "powerful", "seamless", "blazing", "robust", "state-of-the-art".
- **No hedging.** If a claim is uncertain, say what is uncertain about it, or verify it.
- **Present tense, no history.** Describe the tree as it is. "Previously", "used to", "now
  that we" and "the old implementation" belong in the changelog and in git, not in a
  description of current state.
- **Second person for instructions**, third for descriptions. A guide tells *you* what to
  run; an architecture chapter describes what *the renderer* does.

## Naming

The rules in [`CONTRIBUTING.md`](CONTRIBUTING.md) §2 apply to prose and headings as well as to
code:

- **No abbreviations.** "configuration", not "config". "parameters", not "params". The one
  exception is an identifier quoted verbatim from the source.
- **Acronyms are fully upper-case.** `GPU`, `CPU`, `API`, `UI`, `ECS`, `VFX`, `IK`, `HRTF`.
  Never `Gpu`, never `Api`.
- **File names are lower-case with hyphens** — `command-line-interface.md`. The exceptions are
  the files GitHub resolves by name: `README.md`, `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`,
  `SECURITY.md`, `LICENSE`, and the design documents, whose names are cited from source
  comments and commit messages that cannot be rewritten.

## Links and paths

- **Every link resolves, anchor included.** The check fails on a link to a file that does not
  exist *and* on a `#anchor` that matches no heading in the file it points at, so a section
  renamed out from under a citation is caught rather than silently rotting. The one exception:
  a module `README.md` links into `docs/design/`, which is not in `Doxyfile`'s `INPUT`, so
  those links resolve in the repository and on GitHub but not on the generated reference site.
  Keeping the design corpus
  out of the published reference is deliberate, and the two places a module README is actually
  read are the two where the link works.
- **Links are relative.** `../design/physics_system.md`, never a URL to the hosting site.
- **A path in prose is a real path.** `engine/domain/physics/include/SushiEngine/physics/`, in
  backticks, spelled as the tree spells it. Verify it before you write it — the restructure
  left several hundred stale paths behind precisely because nobody did.
  `tools/documentation/check_design_citations.py` verifies it now, on every continuous
  integration run: it resolves every backticked file path a `docs/design/` document cites and
  fails on one that names nothing, the way `check_documentation_length.py` enforces the ceilings.
- **Cite a section by link, not by number.** Link the section's own anchor rather than writing
  "see §5.0.1". Section numbers move, and a document that cites them by number rots silently
  because nothing can tell that the number now points somewhere else.
  The one exception is a citation into `docs/design/`, whose sections are stable identifiers
  used from source comments.

## Code samples

A sample in `docs/getting-started/` or `docs/guides/` must compile against the current headers.
It is not decoration; it is the first thing a reader copies. Before you commit one, find every
symbol it names in the tree and confirm the namespace, the header path, the parameter list and
the return type.
