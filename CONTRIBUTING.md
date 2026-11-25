# Contributing to OxCAM

Thanks for helping. OxCAM is a **clean-room** project: how code is written matters as much
as what it does. Please read this whole page before opening a pull request.

## 1. Developer Certificate of Origin (DCO)

Every commit must carry a sign-off certifying the
[Developer Certificate of Origin 1.1](https://developercertificate.org/):

```
Signed-off-by: Your Name <you@example.com>
```

Use `git commit -s`. There is no CLA. The project is GPL-3.0-or-later and contributions are
accepted under that license.

## 2. Clean-room rules (mandatory)

The full policy is [docs/ip/IP-POLICY.md](docs/ip/IP-POLICY.md). In short:

**Never use as a source**

- Source code, decompiled or disassembled binaries, or captured file formats or network
  traffic from commercial CAM software.
- Documents under NDA, leaked material, or knowledge confidential to your employer.
- Vendor post-processor files, tool-library files or sample parts, even if publicly
  downloadable. Readable is not the same as licensed.
- Code under licenses incompatible with GPL-3.0-or-later (see
  [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)).

If you work, or recently worked, on commercial CAM software, say so in your first PR and stay
out of areas that overlap your employer's products unless you have written clearance.

**Allowed sources**

- Peer-reviewed papers, textbooks and theses.
- Public standards (ISO 6983, ISO 13399, ISO 1832, ...). Paraphrase; never copy text.
- Controller programming manuals, for G-code syntax facts only.
- Open-source code under compatible licenses, with attribution and license preserved in
  `third_party/`.

AI-assisted contributions are allowed. You certify their provenance under the DCO like any
other contribution, and the same rules apply.

## 3. Design log and patent gate

- Every algorithm (toolpath strategy, simulation method, recognition heuristic) needs a
  design-log entry in `docs/design-log/NNNN-short-name.md`, using the
  [template](docs/design-log/0000-template.md), with citations.
- PRs that touch `libs/strategies/` or `libs/sim/` must link their design-log entry.
- New strategies and optimizations get the `needs-patent-review` label and go through the
  [patent gate](docs/ip/IP-POLICY.md#patent-review-gate) before merge. Do not do your own
  patent searching; the review group handles it.

## 4. Trademarks

Use vendor names only descriptively ("Fanuc-style generic post"). Never name a feature after a
vendor's feature or product.

## 5. Code standards

- C++20. Format with the repository `.clang-format`. CI checks it.
- Every source file starts with:
  ```cpp
  // SPDX-FileCopyrightText: OxCAM contributors
  // SPDX-License-Identifier: GPL-3.0-or-later
  ```
- Internal units: millimetres, radians, seconds. Convert only at I/O and UI boundaries.
- **The output is exactly what the user sees** ([ADR-0004](docs/adr/0004-output-is-what-the-user-sees.md)).
  Speeds and feeds come only from what the user set: no fallbacks or defaults for a value left
  at 0, no hidden scaling or compensation, and generation refuses (naming the value) rather
  than inventing a number. Derived values are shown in the editors. Post processors are exempt:
  they may change anything in the output for their machine ([docs/posts](docs/posts/README.md)).
- Libraries must respect the layering in [cmake/ModuleGraph.cmake](cmake/ModuleGraph.cmake).
  OpenCASCADE is used only in `libs/cad`, and Qt only in `apps/opencam-gui`.
- Long-running kernels take an `ocam::CancelToken&` and an `ocam::ProgressSink&`, and keep
  no global mutable state.
- Every behavior change needs a test. Golden-file updates (`OCAM_UPDATE_GOLDEN=1`) need
  explicit reviewer approval.

## 6. Building and testing

See [README.md](README.md#building). Before pushing:

```bash
cmake --build --preset <preset>
ctest --preset <preset> -j <number of CPU threads>
```

Each process running tests is capped at 8 GB (`OCAM_TEST_MEMORY_CAP_GB` in
[cmake/OpenCAMHelpers.cmake](cmake/OpenCAMHelpers.cmake)), so a runaway test fails instead of
taking the machine with it. Set `OCAM_MEMORY_CAP_GB` in the environment to change it for one run;
0 means no cap. The apps are only capped when run as tests.
