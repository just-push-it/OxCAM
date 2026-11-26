# Clean-room IP policy

This is a project policy, not legal advice. Before any public release, the project should get
counsel, for example through a fiscal sponsor such as Software Freedom Conservancy or Software
in the Public Interest.

## Goals

1. No copied or derived code, data or text from proprietary or incompatibly licensed sources.
2. Each algorithm can be traced to public, citable prior art.
3. A recorded review before implementing techniques in areas known to be patented.
4. No confusion with vendors' trademarks.

## Copyright: sources

| Allowed | Prohibited |
|---|---|
| Peer-reviewed papers, textbooks, theses | Commercial CAM source code, binaries (decompiled or disassembled), captured formats or traffic |
| Public standards, paraphrased (ISO 6983, 13399, 1832, ...) | NDA, leaked or employer-confidential material |
| Controller manuals for G-code syntax facts | Vendor post processors, tool libraries, sample parts |
| GPL-3-compatible open source, with attribution | Non-commercial, JSON-license ("Good, not Evil"), GPL-2.0-only, proprietary-SDK code |

Black-box observation of commercial CAM output (for example, comparing G-code it generates) is
not to be used as a design input.

Contributors who work on commercial CAM must disclose that and avoid overlapping areas
unless their employer gives written clearance.

## Design log

Each algorithm gets `docs/design-log/NNNN-*.md` (see the template) before or with its first
PR. An entry records:

- the method and where it comes from (author, title, year, DOI or ISBN);
- deviations from the cited sources;
- the patent-gate outcome and a link to the `docs/ip/reviews/` record, if one exists.

## Patent review gate

Patents protect methods, so clean-room authorship alone does not protect against them.

**Process**

1. A PR that adds a new strategy, optimization or automation gets the `needs-patent-review`
   label.
2. A small, designated review group (not every contributor, to limit willful-infringement
   exposure) records in `docs/ip/reviews/NNNN-*.md`, using the
   [review template](reviews/0000-template.md):
   - a method summary;
   - the oldest prior art found (more than 20 years old is ideal);
   - a summary of what was searched;
   - the outcome.
3. High-risk categories need counsel sign-off before any implementation work starts.

**Risk categories**

| Risk | Areas | Requirement |
|---|---|---|
| High | Constant-engagement, adaptive, dynamic or trochoidal clearing with engagement control | Counsel sign-off before work starts |
| Medium | Morphed or spiral pocketing with continuous engagement; feed optimization from simulated material removal or forces; automatic rest machining driven by in-process stock; automatic 5-axis tilt or collision avoidance; automatic multi-channel (Swiss) balancing | Prior-art search recorded and reviewed |
| Low | Offset and zigzag pocketing, contouring with lead-in/out, standard canned cycles, drop-cutter, waterline, dexel and tri-dexel simulation | Design-log citation only |

Use neutral working names for features, such as "engagement-controlled roughing". Never use a
vendor's marketing name.

## Trademarks

- Descriptive (nominative) use only: "Fanuc-style generic post", "for Haas-style controls".
  Add a disclaimer where needed, and never use logos.
- Feature names must not reuse vendor marks.

## Third-party code

- Vendored code lives only in `third_party/<name>/`, with its original license file.
- It is listed in `THIRD_PARTY_NOTICES.md` and covered by REUSE metadata.
