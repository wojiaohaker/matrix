#!/usr/bin/env python3
"""Run fast, dependency-free repository consistency checks.

This intentionally avoids simulator assets and external services so it can run
for every pull request. Runtime and GPU behavior still require separate smoke
tests on supported hardware.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[2]
SEMVER = re.compile(r"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)$")
MARKDOWN_LINK = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")


def check_version(errors: list[str]) -> None:
    version_file = ROOT / "VERSION"
    version = version_file.read_text(encoding="utf-8").strip()
    if not SEMVER.fullmatch(version):
        errors.append(f"VERSION is not semantic x.y.z: {version!r}")

    release_scripts = (ROOT / "scripts" / "release_manager").glob("*.sh")
    hardcoded_default = re.compile(r'^\s*VERSION=.*["\']\d+\.\d+\.\d+["\']', re.MULTILINE)
    for script in release_scripts:
        text = script.read_text(encoding="utf-8")
        if hardcoded_default.search(text):
            errors.append(f"hard-coded VERSION default in {script.relative_to(ROOT)}")

    command_with_version = re.compile(
        r"install_chunks(?:_local)?\.sh\s+\d+\.\d+\.\d+"
    )
    for document in [ROOT / "README.md", ROOT / "docs" / "README_CN.md"]:
        if command_with_version.search(document.read_text(encoding="utf-8")):
            errors.append(f"hard-coded installer command in {document.relative_to(ROOT)}")


def check_governance(errors: list[str]) -> None:
    required = [
        "CONTRIBUTING.md",
        "CODE_OF_CONDUCT.md",
        "SECURITY.md",
        ".github/pull_request_template.md",
        ".github/ISSUE_TEMPLATE/bug_report.yml",
        ".github/ISSUE_TEMPLATE/feature_request.yml",
    ]
    for relative in required:
        if not (ROOT / relative).is_file():
            errors.append(f"missing collaboration file: {relative}")


def check_json(errors: list[str]) -> None:
    for directory in (ROOT / "config", ROOT / "scene"):
        for path in sorted(directory.glob("*.json")):
            try:
                json.loads(path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                errors.append(f"invalid JSON in {path.relative_to(ROOT)}: {exc}")


def check_local_markdown_links(errors: list[str]) -> None:
    for document in sorted(ROOT.rglob("*.md")):
        if ".git" in document.parts:
            continue
        text = document.read_text(encoding="utf-8")
        for raw_target in MARKDOWN_LINK.findall(text):
            target = raw_target.strip().split(maxsplit=1)[0].strip("<>")
            if not target or target.startswith(("#", "http://", "https://", "mailto:")):
                continue
            path_text = unquote(target.split("#", 1)[0])
            if not path_text:
                continue
            resolved = (document.parent / path_text).resolve()
            if not resolved.exists():
                errors.append(
                    f"broken local link in {document.relative_to(ROOT)}: {target}"
                )


def main() -> int:
    errors: list[str] = []
    check_version(errors)
    check_governance(errors)
    check_json(errors)
    check_local_markdown_links(errors)

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    print("Repository consistency checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
