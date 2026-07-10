#!/usr/bin/env python3

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
EXAMPLES_ROOT = REPO_ROOT / "examples"
NICLA_VISION_FQBN = "arduino:mbed_nicla:nicla_vision"
MKR1000_FQBN = "arduino:samd:mkr1000"


@dataclass(frozen=True)
class SketchTarget:
    sketch_dir: Path
    ino_file: Path
    fqbn: str | None
    reason: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compile all Arduino examples in the repo and optionally upload a "
            "selected subset."
        )
    )
    parser.add_argument(
        "--arduino-cli",
        default="arduino-cli",
        help="Path to arduino-cli binary.",
    )
    parser.add_argument(
        "--library",
        default=str(REPO_ROOT),
        help="Library path passed to arduino-cli --library.",
    )
    parser.add_argument(
        "--examples-root",
        default=str(EXAMPLES_ROOT),
        help="Examples root to scan.",
    )
    parser.add_argument(
        "--default-fqbn",
        default=None,
        help="Fallback FQBN for examples whose target cannot be inferred.",
    )
    parser.add_argument(
        "--all-filesystem-examples",
        action="store_true",
        help=(
            "Scan every sketch currently present under examples/. By default, "
            "only git-tracked examples are tested."
        ),
    )
    parser.add_argument(
        "--pattern",
        action="append",
        default=[],
        help="Only include sketches whose relative path contains this substring.",
    )
    parser.add_argument(
        "--exclude-pattern",
        action="append",
        default=[],
        help="Exclude sketches whose relative path contains this substring.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List detected sketches and inferred targets without compiling.",
    )
    parser.add_argument(
        "--upload",
        action="store_true",
        help="Upload compiled sketches after a successful compile pass.",
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port for uploads, for example /dev/ttyACM0.",
    )
    parser.add_argument(
        "--upload-pattern",
        action="append",
        default=[],
        help=(
            "Only upload sketches whose relative path contains this substring. "
            "If omitted with --upload, every compiled sketch is uploaded."
        ),
    )
    parser.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="Keep compiling after the first failure.",
    )
    return parser.parse_args()


def sketch_dirs(examples_root: Path) -> list[Path]:
    result: list[Path] = []
    for ino_file in sorted(examples_root.glob("*/*.ino")):
        result.append(ino_file.parent)
    return result


def git_tracked_sketch_dirs() -> list[Path]:
    result = run_command(["git", "ls-files", "examples/*/*.ino"])
    if result.returncode != 0:
        return []

    sketch_dirs: list[Path] = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        sketch_dirs.append((REPO_ROOT / line).parent)
    return sorted(set(sketch_dirs))


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


def infer_fqbn(sketch_dir: Path, ino_file: Path, default_fqbn: str | None) -> tuple[str | None, str]:
    rel = sketch_dir.relative_to(REPO_ROOT).as_posix()
    ino_text = read_text(ino_file)
    joined = f"{rel}\n{ino_text}"

    if "ARDUINO_NICLA_VISION" in ino_text or re.search(r"NiclaVision|akd1500_infer_fast|interupt", rel):
        return NICLA_VISION_FQBN, "nicla_vision inferred from sketch content/path"

    if "ARDUINO_SAMD_MKR1000" in ino_text or "MKR1000" in joined:
        return MKR1000_FQBN, "mkr1000 inferred from sketch content/path"

    if default_fqbn:
        return default_fqbn, "default FQBN"

    return None, "no target inference"


def discover_targets(
    examples_root: Path,
    default_fqbn: str | None,
    include_patterns: Iterable[str],
    exclude_patterns: Iterable[str],
    all_filesystem_examples: bool,
) -> list[SketchTarget]:
    include_patterns = list(include_patterns)
    exclude_patterns = list(exclude_patterns)
    targets: list[SketchTarget] = []

    source_sketch_dirs = (
        sketch_dirs(examples_root)
        if all_filesystem_examples
        else git_tracked_sketch_dirs()
    )

    for sketch_dir in source_sketch_dirs:
        ino_file = sketch_dir / f"{sketch_dir.name}.ino"
        if not ino_file.exists():
            ino_matches = sorted(sketch_dir.glob("*.ino"))
            if len(ino_matches) != 1:
                continue
            ino_file = ino_matches[0]

        rel = sketch_dir.relative_to(REPO_ROOT).as_posix()
        if include_patterns and not any(pattern in rel for pattern in include_patterns):
            continue
        if any(pattern in rel for pattern in exclude_patterns):
            continue

        fqbn, reason = infer_fqbn(sketch_dir, ino_file, default_fqbn)
        targets.append(SketchTarget(sketch_dir=sketch_dir, ino_file=ino_file, fqbn=fqbn, reason=reason))

    return targets


def run_command(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def compile_target(cli: str, library: str, target: SketchTarget) -> subprocess.CompletedProcess[str]:
    return run_command(
        [
            cli,
            "compile",
            "-b",
            target.fqbn,
            "--library",
            library,
            str(target.sketch_dir),
        ]
    )


def upload_target(cli: str, port: str, target: SketchTarget) -> subprocess.CompletedProcess[str]:
    return run_command(
        [
            cli,
            "upload",
            "-b",
            target.fqbn,
            "-p",
            port,
            str(target.sketch_dir),
        ]
    )


def print_target(target: SketchTarget) -> None:
    rel = target.sketch_dir.relative_to(REPO_ROOT).as_posix()
    if target.fqbn is None:
        print(f"SKIP  {rel}  target=unknown  reason={target.reason}")
    else:
        print(f"PLAN  {rel}  target={target.fqbn}  reason={target.reason}")


def main() -> int:
    args = parse_args()
    examples_root = Path(args.examples_root).resolve()
    targets = discover_targets(
        examples_root=examples_root,
        default_fqbn=args.default_fqbn,
        include_patterns=args.pattern,
        exclude_patterns=args.exclude_pattern,
        all_filesystem_examples=args.all_filesystem_examples,
    )

    if not targets:
        print("No sketches matched the current filters.", file=sys.stderr)
        return 1

    if args.list:
        for target in targets:
            print_target(target)
        return 0

    skipped: list[SketchTarget] = []
    compiled: list[SketchTarget] = []
    failed: list[tuple[SketchTarget, subprocess.CompletedProcess[str]]] = []

    for target in targets:
        rel = target.sketch_dir.relative_to(REPO_ROOT).as_posix()
        if target.fqbn is None:
            print(f"SKIP  {rel}  reason={target.reason}")
            skipped.append(target)
            continue

        print(f"TEST  {rel}  target={target.fqbn}")
        result = compile_target(args.arduino_cli, args.library, target)
        if result.returncode == 0:
            print(f"PASS  {rel}")
            compiled.append(target)
        else:
            print(f"FAIL  {rel}")
            print(result.stdout.rstrip())
            failed.append((target, result))
            if not args.continue_on_failure:
                break

    upload_failures: list[tuple[SketchTarget, subprocess.CompletedProcess[str]]] = []
    if args.upload:
        if not args.port:
            print("--upload requires --port", file=sys.stderr)
            return 2

        upload_patterns = list(args.upload_pattern)
        upload_targets = [
            target
            for target in compiled
            if not upload_patterns
            or any(pattern in target.sketch_dir.relative_to(REPO_ROOT).as_posix() for pattern in upload_patterns)
        ]

        for target in upload_targets:
            rel = target.sketch_dir.relative_to(REPO_ROOT).as_posix()
            print(f"UPLOAD  {rel}  port={args.port}")
            result = upload_target(args.arduino_cli, args.port, target)
            if result.returncode == 0:
                print(f"UPPASS  {rel}")
            else:
                print(f"UPFAIL  {rel}")
                print(result.stdout.rstrip())
                upload_failures.append((target, result))
                if not args.continue_on_failure:
                    break

    print(
        "SUMMARY "
        f"compiled_pass={len(compiled)} "
        f"compile_fail={len(failed)} "
        f"skipped={len(skipped)} "
        f"upload_fail={len(upload_failures)}"
    )

    if failed or upload_failures:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
