# Post processors

A post processor turns OxCAM's toolpaths into the program one machine control reads. In OxCAM a
post is a **JavaScript module** that OxCAM runs in an engine built into the program
([ADR-0006](../adr/0006-javascript-post-processors.md),
[DL-0039](../design-log/0039-javascript-post-engine.md)). This folder documents how posts work
and how to write one.

| Page | What it covers |
|---|---|
| This page | How posting works, choosing a post, running and checking posts, rules |
| [writing-posts.md](writing-posts.md) | A post module: manifest, options, handlers, sandbox, errors; a walk-through |
| [output-library.md](output-library.md) | `oxcam:post`: number formats, words, blocks, comments, line numbers, buffers |
| [fanuc-base.md](fanuc-base.md) | `oxcam:fanuc`: the generic Fanuc-style base posts and their settings |
| [nc-program.md](nc-program.md) | The NC program (version 2): what a post receives, field by field |

A complete example post is in [examples/posts/example-vmc.js](../../examples/posts/example-vmc.js).

## How posting works

```
operations ──▶ toolpaths ──▶ machine::solve ──▶ post::lower ──▶ NC program ──▶ post (JS) ──▶ text
                (strategies)   (limits, axes)    (sequencing,    (hand-over)     (yours)
                                                   declarations)
```

Everything up to the NC program is done by OxCAM and is the same for every control:

- the toolpaths, solved against the machine (axis travel, spindle and feed limits, rotary
  positions);
- tool changes, spindle and coolant sequencing, the safe approach to each operation;
- canned cycles, or their expansion into moves for cycle types the post does not take;
- arcs split, or made into lines, where the post cannot take them;
- where cutter compensation starts and stops;
- 3+2 indexing and multi-axis rotary motion (tool centre point control or machine positions).
- probing cycles with a spindle probe, as one instruction each with all their moves; calling the
  probe's macro programs is the post's ([probing](fanuc-base.md#probing)).

The NC program is a list of typed instructions: "operation 2 begins", "tool change to T3",
"linear move to X10 Z-2 at 600 mm/min". It contains no G-code. **The post writes all of the
text**: codes, word order, number formats, header and footer, comments, line numbers.

The post also **declares** what its control can take (its manifest): arc planes, helical arcs,
the longest arc, the canned cycles, the multi-axis mode and the machines it is for. OxCAM shapes
the program to fit before the post sees it, so the post never has to undo what OxCAM did.

## The built-in posts

| Post | Machines | Notes |
|---|---|---|
| `fanuc-generic-mill` | 3-axis and 5-axis mills | All canned cycles; 5-axis with tool centre point control (G43.4) |
| `fanuc-generic-mill-4axis` | 3-axis and 4-axis (A) mills | Machine positions, inverse-time feeds (G93) on rotary moves |
| `fanuc-generic-lathe` | 2-axis lathes | Diameters, G96/G97 with a G50 limit, feed per revolution, G32 threads, G84 tapping on the spindle axis (other cycles as moves); bar pulls and the transfer to a sub-spindle, with your machine's codes as options |
| `haas-lathe` | 2-axis lathes, live tooling | As the generic lathe, with Haas's live tool codes (M133, M135, M154, M155) as its starting options |
| `haas-classic-mill` | 3-axis, 4-axis (A) and 5-axis trunnion mills | Haas-style controls without tool centre point control: G93 machine positions, G154 offsets, G187, rotary brakes ([details](haas-classic-mill.md)) |

They are written in JavaScript like any other post ([data/posts](../../data/posts)) on the
`oxcam:fanuc` base, and built into OxCAM.

## Choosing a post

**Setup > Post Process** (or right-click a setup, *Post Process*) first shows the post and its
options, then posts. The post a setup is posted with is, in order:

1. the post the **setup** chooses (the Post Process dialog, or *Post* on the setup page): a
   built-in post or one the project carries;
2. else the **machine's default post**: *Default post* in the machine library (`post` in the
   machine file, version 6), a built-in post or one from your post library;
3. else the built-in post for the machine's kinematics (lathe, 4-axis, or mill).

Only posts that write for the machine's kinematics are offered.

**Posts travel with the project.** Choosing a post file, or a post from your post library
(directly, or as a machine's default), copies it into the project, as custom machines are
copied. The project then posts the same on any computer. When the library has a newer version
of a copied post, the setup page offers *Update from library*.

**Your post library** is a folder of `.js` files: `posts` in OxCAM's application data folder
(on Windows `%APPDATA%\OxCAM\OxCAM\posts`, or the folder in the `postLibraryFolder` setting).
Every file there that loads is offered.

A post refuses machines it does not declare: posting a lathe program with a mill post is an
error naming the machines the post writes for.

### Post options

A post can declare options: a value for the whole program (an optional stop at each tool, a
safe Z height) or one per operation (a smoothing level). The Post Process dialog (and the setup
page's *Configure...* next to *Post*) shows the program options of the chosen post, each
starting from, in order of precedence:

1. **this setup**: the value the setup was last posted with;
2. **machine**: the machine's value for its default post, set in the machine library
   (*Configure...* next to *Default post*);
3. **your default**: saved with *Save as my defaults* in the dialog, per post, for you on this
   computer;
4. **post default**: the post's own.

Each option says which one it shows; *Use defaults* drops the setup's values. Posting saves the
values with the setup, so the project posts the same anywhere (the CLI and Python read the
setup's and the machine's values too). Choosing another post drops the old post's values.
Per-operation options are on each operation's page.

### Work offsets

A post declares the work offsets its control has and how they are written (`workOffsets` in
its manifest, [writing posts](writing-posts.md)): G54-G59 and G54.1 P1-P48 unless it says
otherwise; the Haas-style post has G54-G59 and G154 P1-P99. On the setup page, *Work offset*
chooses the range (*Standard*, *Extended*) and the row under it the number in the post's code
(*G* 54, or *G154 P* 2), as far as the post declares and the machine has (machine library: *Work offsets*; 16 on a Haas-style machine
gives G54-G59 and G154 P1-P10). Posting refuses work offsets beyond either.

## Running a post

**GUI:** *Post Process* on a setup posts it with the setup's post and option values, shows
the program and its warnings, and saves it with the post's file extension.

**Command line:**

```bash
oxcam-cli project post part.ocam --setup "Setup 1" -o part.nc --verify
```

- `--post <id or file.js>` posts with another post than the setup's.
- `--option name=value` sets a program option (repeatable; `true`, `false`, numbers, else
  text), over the setup's own values.
- `--ir program.json` also writes the NC program the post receives.
- `--verify` reads the program back (below), within `--tolerance` mm (default 0.0015).

**Python:**

```python
import oxcam
doc = oxcam.Document.open("part.ocam")
gcode = doc.post()                                      # the setup's post and options
gcode = doc.post(post="haas-classic-mill")              # a built-in post by id
gcode = doc.post(post="my-vmc.js", options={"optionalStop": True})
program = doc.nc_program()                              # what a post receives
```

## Checking the output

OxCAM can read G-code back into motion (`post::backplot`, a Fanuc-style subset) and compare it
with the toolpaths it came from. A post declares in its manifest (`verify`) which dialect, if
any, reads its output:

- `oxcam-cli project post ... --verify` reads the program back and compares it within
  `--tolerance` mm;
- `oxcam.backplot(text)` in Python returns the motion and its statistics.

A post declaring `verify: null` cannot be checked this way. **Always dry-run a new post on the
machine** (single block, feed override low, raised Z) before cutting, and record controller dry
runs in [docs/validation/controller-dry-runs.md](../validation/controller-dry-runs.md).

## Speeds, feeds and ADR-0004

The NC program carries the speeds and feeds the user set, exactly (ADR-0004). A post is the
user's customization for a machine and is exempt from that rule (amendment of 2026-10-24): it
may change anything, feeds and speeds included, for example to clamp a feed to the control's
limit. The built-in posts write the values as they are.

## Speed

Posts run in an interpreter: about 5 s per million NC instructions with the generic mill post
(DL-0039). A typical 3-axis program of 100 000 moves posts in about half a second.

## Clean-room rule for posts

Take G-code syntax facts only from controller programming manuals and public standards. Never
copy or adapt a vendor's post-processor files, even publicly downloadable ones
([CONTRIBUTING.md](../../CONTRIBUTING.md), [IP policy](../ip/IP-POLICY.md)). JavaScript is the
language of commercial CAM posts too; that does not make their posts a source. Name posts
descriptively ("Fanuc-style", "for Haas-style controls"), never after a vendor's product, and
list the manuals a post was written from in its header comment.
