# MATRiX Release 0.1.2

## Required Packages

- **assets-0.1.2.tar.gz** (~675MB)
  - Launcher files: `bin/sim_launcher`, `bin/sim_launcher.bin`
  - MuJoCo runtime binary and dynamic map payloads
  - MC runtime binaries and shared libraries, including WBC libraries
  - ONNX controller models: `xg`, `xg_wheel`, `zg_wheels`
  - UE runtime dependencies, including OpenCV shared libraries

- **base-0.1.2.tar.gz** (~2.0GB)
  - Chunk 0 / EmptyWorld core content
  - Core Blueprints and system files
  - Published robot model directories: `xgb`, `xgw`, `zgws`, `go2`, `go2w`
  - Runtime template directories: `Content/model/config`, `Content/model/SceneLoder`

Internal robot model directories are not included in this release. `Content/model/dynamicmap` is created at runtime for MoonWorld from `dynamicmaps/moonworld.bin`.

## Recommended / Optional Packages

- **shared-0.1.2.tar.gz** (~3.3GB): shared resources used by multiple maps. This file exceeds GitHub's single-asset size limit and is uploaded as split parts when needed.
- **Map packages**: install only the maps you need.

## Checksums

- `assets-0.1.2.tar.gz`: `ec18164489775c9f5ac1f73da4790c8c5d18f701cddf48ec1a8e8d7b3861e8fc`
- `base-0.1.2.tar.gz`: `2cbb40861e89c40735cd64b24e8b64d88d012f335bdb405b5ed52db86f8b4e38`

Verify all downloaded packages with:

```bash
sha256sum -c checksums-0.1.2.sha256
```

## Install

```bash
bash scripts/release_manager/install_chunks.sh 0.1.2
```

For offline/local installation, place downloaded packages in `releases/` and run:

```bash
bash scripts/release_manager/install_chunks_local.sh 0.1.2
```
