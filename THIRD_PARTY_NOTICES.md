# Third-party notices

OxCAM is licensed under GPL-3.0-or-later. It uses the third-party components below. vcpkg
fetches them at build time; none are vendored into this repository unless a `third_party/`
path is listed.

| Component | License | Used by | Phase |
|---|---|---|---|
| Eigen | MPL-2.0 | foundation | 0 |
| {fmt} | MIT | foundation | 0 |
| CLI11 | BSD-3-Clause | oxcam-cli | 0 |
| Catch2 v3 | BSL-1.0 | tests | 0 |
| OpenCASCADE Technology | LGPL-2.1 with OCCT exception | cad, oxcam-gui | 1 |
| Qt 6 | LGPL-3.0 | oxcam-gui | 1 |
| Clipper2 | BSL-1.0 | geom2d | 1 |
| nlohmann/json | MIT | tooling, document | 2 |
| json-schema-validator (pboettch) | MIT | tooling, document | 2 |
| miniz | MIT | document | 2 |
| oneTBB | Apache-2.0 | sim, strategies | 5 |
| nanobind (with tsl::robin_map) | BSD-3-Clause (robin_map: MIT) | Python bindings (optional, `OCAM_BUILD_PYTHON`) | 6 |
| QuickJS-ng | MIT (its Unicode data tables: Unicode-3.0) | post (JavaScript post engine) | ADR-0006 |
| RapidCheck | BSD-2-Clause | tests | 3 |
| Manifold | Apache-2.0 | tests (oracle only) | 5 |

## Not allowed

- Shewchuk's Triangle (non-commercial license).
- Anything licensed non-commercial, under the JSON license, or GPL-2.0-only.
- Proprietary geometry, CAM or simulation SDKs.
- Vendor post-processor or tool-library files.
