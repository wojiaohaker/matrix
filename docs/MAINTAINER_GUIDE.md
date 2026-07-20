# MATRiX Architecture and Maintainer Guide

This document defines the boundaries and maintenance rules of the public
MATRiX repository. Read it before changing launch, installation, conversion, or
release behavior.

## Repository boundary

MATRiX is the public release/runtime repository. It contains:

- user-facing launch and environment-check scripts;
- release download, verification, installation, packaging, and upload tooling;
- simulator configuration, scene examples, and RViz configuration;
- documentation and selected binary system dependencies;
- placeholders populated by published release assets.

The Unreal Engine source project is maintained separately as
`jszr_mujoco_ue2`. Changes that require engine compilation must be coordinated
across repositories and documented in the public issue or pull request without
exposing private implementation details.

## Runtime flow

1. `scripts/install_deps.sh` installs OS and ROS dependencies.
2. `scripts/release_manager/install_chunks.sh` downloads versioned artifacts,
   verifies their manifest/checksums, and installs runtime content.
3. `scripts/check_env.sh` validates commands, assets, binaries, and shared
   libraries for the selected mode.
4. `scripts/run_sim.sh` normalizes arguments/configuration and starts the
   simulator processes.
5. `scripts/run_custom_urdf.sh` converts and caches custom robot data before
   delegating to `run_sim.sh`.

The configuration files under `config/` and `scene/` are user-editable sources
of truth. Generated runtime copies must not silently become authoritative.

## Script ownership and invariants

| Area | Entry point | Maintenance invariant |
| --- | --- | --- |
| Environment | `scripts/check_env.sh` | Diagnostics must be safe on incomplete installations. |
| Launch | `scripts/run_sim.sh` | Track and stop only processes owned by the current launch. |
| Custom robots | `scripts/run_custom_urdf.sh` | Cache keys include source hash and conversion pipeline version. |
| Downloads | `scripts/release_manager/install_chunks.sh` | Verify artifacts before extraction; interrupted downloads remain recoverable. |
| Releases | `scripts/release_manager/release_pipeline.sh` | Build, manifest, local install, check, then upload/publish. |
| Shared release logic | `scripts/release_manager/common.sh` | Reusable behavior belongs here, including the canonical version reader. |

Complex scripts should be split when a unit can be given a clear input/output
contract and tested without launching the simulator. Prefer small helpers over
additional global state.

## Commenting standard

Comments are required for:

- safety boundaries around process or filesystem cleanup;
- data-format invariants and compatibility workarounds;
- cross-repository assumptions;
- non-obvious failure recovery and retry behavior;
- public environment variables and their effects.

Comments should explain why a constraint exists and what may break if it is
removed. Avoid comments that only translate the command immediately below.
User-facing interfaces belong in the appropriate guide as well as in `--help`.

## Version policy

The repository root `VERSION` file is the single source of truth for the
current release version. Release-manager scripts load it through
`scripts/release_manager/common.sh`. User-facing installers may default to the
current version; commands that create or publish release artifacts require an
explicit version to prevent accidental release work.

To prepare a release:

1. Update `VERSION` in a dedicated pull request.
2. Run `python3 scripts/ci/check_repo.py`.
3. Run the release pipeline locally with the intended version.
4. Verify the generated manifest and SHA256 files.
5. Install the generated artifacts locally and run the runtime environment
   check.
6. Create and publish the matching `v<version>` tag/release only after review.

Do not change a default version inside an individual script. Historical
documentation may name a concrete artifact version when the URL itself is
version-specific.

## Review and validation

Every pull request should run:

```bash
find scripts src -type f -name '*.sh' -print0 | xargs -0 -n1 bash -n
python3 -m py_compile scripts/validate_xml_contract.py scripts/ci/check_repo.py
python3 scripts/ci/check_repo.py
```

Runtime changes also require the relevant `scripts/check_env.sh` mode and a
manual smoke test on supported Ubuntu/ROS/GPU hardware. State the exact test
environment in the pull request.

## Documentation ownership

User-facing changes must update both English and Chinese navigation or guides
when equivalent documents exist. Internal-only endpoints, credentials, private
logs, and personal data must not be committed to public documentation.
