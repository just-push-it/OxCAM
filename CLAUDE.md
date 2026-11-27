# OxCAM: instructions for coding agents

Read [CONTRIBUTING.md](CONTRIBUTING.md) for the clean-room, design-log and code rules. The rule
below overrides convenience everywhere in this repository.

## Lint locally before you push or open a PR

Run the same checks as the CI `lint` job ([.github/workflows/ci.yml](.github/workflows/ci.yml))
and fix every failure before `git push` or opening a pull request. Never push to find out.

- **clang-format 18** (the CI version; other versions format differently):
  `clang-format-18 --dry-run --Werror $(git ls-files '*.cpp' '*.hpp' '*.h')`
- **REUSE**: `reuse lint`
- **DCO**: every commit carries a `Signed-off-by:` line (`git commit -s`).

If a tool is missing locally, say so and get it (or run the checks on the Debian VM) rather
than skip the check.

## The output is exactly what the user sees (ADR-0004)

OxCAM's output drives machine tools. **Every speed and feed in a toolpath or G-code program
must be a value the user can see where they set it, exactly as shown, 100% of the time.**
See [docs/adr/0004-output-is-what-the-user-sees.md](docs/adr/0004-output-is-what-the-user-sees.md).

- **Never** add a fallback, default, minimum or derived substitute for a speed or feed the user
  left at 0. Zero means zero.
- If a zero cannot work in the output (a feed move with no feed, no spindle speed), **refuse to
  generate** with a message naming the value and where to set it. Do not invent a number.
- **Never** scale, clamp or compensate a feed or speed between the preset and the output
  (no arc feed compensation, no automatic slow-downs). If a behaviour must change a feed, make
  it a visible, labelled parameter the user controls.
- Show derived values (spindle speed, feed rate, tapping feed) in the editors next to their
  inputs. Plunge, ramp and lead feeds are their own values, set by the user. Never show a value
  as a fixed text ("at the feed rate"): what the output uses is a value the user can change.
- A parameter's default lives only in its `ParamDef`; read parameters through `ParamReader`,
  never with a second literal default.
- **Post processors are exempt** (ADR-0004 amendment, 2026-10-24). The rule covers everything
  up to the NC program handed to the post. A post is the user's customization for one machine
  and may change the output, feeds and speeds included. Posts are JavaScript programs only
  ([ADR-0006](docs/adr/0006-javascript-post-processors.md)); see [docs/posts](docs/posts/README.md).
- When in doubt, ask the maintainer rather than assume.
