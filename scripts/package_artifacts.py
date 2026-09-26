#!/usr/bin/env python3
"""Discover, build, and package autonomous_rc into a versioned artifacts/ tree.

Wraps build.py's environment validation and colcon invocation, then collects
the resulting executables/libraries plus their non-system runtime dependencies
(resolved via `ldd`) into a self-contained artifacts/ folder suitable for
copying onto the Jetson Orin Nano target.

Usage:
    python autonomous_rc/scripts/package_artifacts.py [--packages P1 P2 ...]
        [--clean] [--force] [--debug] [--skip-firmware] [--dry-run]
"""
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent
SRC_DIR = WORKSPACE_ROOT / "src"
ARTIFACTS_DIR = WORKSPACE_ROOT / "artifacts"
ARTIFACT_SUBDIRS = ("bin", "lib", "firmware", "deps", "logs")
SKIP_DIR_NAMES = {"build", "install", "log", ".git"}

sys.path.insert(0, str(WORKSPACE_ROOT))
import build as build_module  # noqa: E402  (reuse check_environment/run_colcon, not duplicate it)

_START_RE = re.compile(r"^Starting >>> (\S+)$")
_END_RE = re.compile(r"^(Finished|Failed)\s+<<<\s+(\S+)")
_SO_RE = re.compile(r"\.so(\.\d+)*$")
_LDD_LINE_RE = re.compile(r"^\s*\S+\s+=>\s+(\S+)\s+\(0x")
_SYSTEM_LIB_PREFIXES = ("/lib/", "/lib64/", "/usr/lib/")


def _parse_package_name(package_xml: Path) -> str | None:
    text = package_xml.read_text(encoding="utf-8", errors="replace")
    match = re.search(r"<name>\s*([^<]+?)\s*</name>", text)
    return match.group(1) if match else None


def discover_packages() -> list[dict]:
    """Scans src/** for CMakeLists.txt/package.xml/platformio.ini, plus the
    workspace-root build.py, and returns a manifest of discovered build
    definitions (name, path, build_type)."""
    manifest: list[dict] = []
    if SRC_DIR.exists():
        for path in sorted(p for p in SRC_DIR.rglob("*") if p.is_dir()):
            rel_parts = path.relative_to(SRC_DIR).parts
            if any(part in SKIP_DIR_NAMES for part in rel_parts):
                continue
            platformio_ini = path / "platformio.ini"
            package_xml = path / "package.xml"
            if platformio_ini.exists():
                manifest.append({"name": path.name, "path": str(path), "build_type": "platformio"})
            elif package_xml.exists():
                name = _parse_package_name(package_xml) or path.name
                build_type = "ament_cmake" if (path / "CMakeLists.txt").exists() else "ament_python"
                manifest.append({"name": name, "path": str(path), "build_type": build_type})

    build_py = WORKSPACE_ROOT / "build.py"
    if build_py.exists():
        manifest.append({"name": "build.py", "path": str(build_py), "build_type": "build_script"})
    return manifest


def ensure_artifacts_structure(clean: bool) -> tuple[list[str], list[str]]:
    """Creates artifacts/{bin,lib,firmware,deps,logs} (idempotent). Returns
    (created, already_existing) folder path lists for logging."""
    if clean and ARTIFACTS_DIR.exists():
        shutil.rmtree(ARTIFACTS_DIR)

    created: list[str] = []
    existing: list[str] = []
    for name in ("", *ARTIFACT_SUBDIRS):
        directory = ARTIFACTS_DIR / name if name else ARTIFACTS_DIR
        (existing if directory.exists() else created).append(str(directory))
        directory.mkdir(parents=True, exist_ok=True)
    return created, existing


def _flush_package_log(logs_dir: Path, name: str | None, lines: list[str],
                        status: str, results: dict) -> None:
    if name is None:
        return
    log_path = logs_dir / f"{name}.log"
    log_path.write_text("\n".join(lines), encoding="utf-8")
    results[name] = {"status": status, "log": str(log_path)}


def _split_per_package_logs(stdout: str, logs_dir: Path) -> dict:
    """Splits combined colcon console_direct+ output into artifacts/logs/<pkg>.log
    per package. Requires --executor sequential so output isn't interleaved."""
    results: dict = {}
    current_name: str | None = None
    current_lines: list[str] = []

    for line in stdout.splitlines():
        start_match = _START_RE.match(line)
        if start_match:
            _flush_package_log(logs_dir, current_name, current_lines, "unknown", results)
            current_name, current_lines = start_match.group(1), [line]
            continue
        if current_name is not None:
            current_lines.append(line)
        end_match = _END_RE.match(line)
        if end_match and end_match.group(2) == current_name:
            status = "success" if end_match.group(1) == "Finished" else "failed"
            _flush_package_log(logs_dir, current_name, current_lines, status, results)
            current_name, current_lines = None, []

    _flush_package_log(logs_dir, current_name, current_lines, "unknown", results)
    return results


def run_build(packages: list[str], debug: bool, force: bool, logs_dir: Path) -> tuple[bool, dict]:
    env_ok = build_module.check_environment()
    if not env_ok and not force:
        print("\nEnvironment validation failed. Fix the MISS items above, or re-run with --force.")
        return False, {}

    build_type = "Debug" if debug else "Release"
    cmd = [
        "colcon", "build", "--symlink-install",
        # Sequential (not colcon's default parallel executor): keeps per-package
        # output un-interleaved so it can be split into artifacts/logs/<pkg>.log.
        "--executor", "sequential",
        "--cmake-args", f"-DCMAKE_BUILD_TYPE={build_type}",
        "--event-handlers", "console_direct+",
    ]
    if packages:
        cmd += ["--packages-select", *packages]

    print("Running:", " ".join(cmd))
    proc = subprocess.run(cmd, cwd=WORKSPACE_ROOT, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True)

    (logs_dir / "colcon_build.log").write_text(proc.stdout, encoding="utf-8")
    package_results = _split_per_package_logs(proc.stdout, logs_dir)
    return proc.returncode == 0, package_results


def run_firmware_build(logs_dir: Path, skip_firmware: bool) -> dict | None:
    firmware_dir = SRC_DIR / "safety_manager" / "firmware" / "esp32_estop"
    if skip_firmware or not firmware_dir.exists():
        return None

    pio = shutil.which("pio") or shutil.which("platformio")
    if not pio:
        print(f"[firmware] platformio not found on PATH, skipping {firmware_dir}")
        return {"status": "skipped", "reason": "platformio not found"}

    print(f"Running: {pio} run (cwd={firmware_dir})")
    proc = subprocess.run([pio, "run"], cwd=firmware_dir, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True)
    log_path = logs_dir / "esp32_estop.log"
    log_path.write_text(proc.stdout, encoding="utf-8")
    return {"status": "success" if proc.returncode == 0 else "failed", "log": str(log_path)}


def _is_shared_lib(path: Path) -> bool:
    return bool(_SO_RE.search(path.name))


def _is_static_lib(path: Path) -> bool:
    return path.suffix == ".a"


def _real_file(path: Path) -> Path | None:
    """Resolves symlinks (colcon --symlink-install) to their real target file."""
    if path.is_symlink():
        try:
            resolved = path.resolve()
        except OSError:
            return None
        return resolved if resolved.is_file() else None
    return path if path.is_file() else None


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _collect_runtime_deps(binary_path: Path, copied: dict, seen_keys: set) -> None:
    if sys.platform != "linux":
        print(f"[deps] skipping ldd resolution for {binary_path.name}: not running on Linux")
        return
    ldd = shutil.which("ldd")
    if not ldd:
        print("[deps] ldd not found on PATH, skipping runtime dependency resolution")
        return

    try:
        result = subprocess.run([ldd, str(binary_path)], capture_output=True, text=True, timeout=15)
    except Exception as exc:  # noqa: BLE001 - ldd failures shouldn't abort packaging
        print(f"[deps] ldd failed for {binary_path}: {exc}")
        return

    for line in result.stdout.splitlines():
        match = _LDD_LINE_RE.match(line)
        if not match:
            continue
        dep_path = Path(match.group(1))
        if not dep_path.is_file():
            continue
        dep_str = str(dep_path)
        if dep_str.startswith(_SYSTEM_LIB_PREFIXES) or dep_str.startswith(str(WORKSPACE_ROOT)):
            continue  # system lib, or already captured from our own install/ tree

        dep_hash = _sha256(dep_path)
        key = (dep_path.name, dep_hash)
        if key in seen_keys:
            continue
        seen_keys.add(key)

        dest = ARTIFACTS_DIR / "deps" / dep_path.name
        shutil.copy2(dep_path, dest)
        copied["deps"].append({"src": dep_str, "dest": str(dest), "size": dest.stat().st_size})


def collect_binaries() -> dict:
    """Walks install/**/lib/** and install/**/bin/**, copying executables to
    artifacts/bin, shared/static libs to artifacts/lib, resolving each
    non-static binary's runtime deps into artifacts/deps, and ESP32 firmware
    outputs into artifacts/firmware."""
    install_dir = WORKSPACE_ROOT / "install"
    copied: dict = {"bin": [], "lib": [], "firmware": [], "deps": []}
    seen_dep_keys: set = set()

    if install_dir.exists():
        for pkg_dir in sorted(p for p in install_dir.iterdir() if p.is_dir()):
            for sub in ("lib", "bin"):
                base = pkg_dir / sub
                if not base.exists():
                    continue
                for entry in sorted(base.rglob("*")):
                    resolved = _real_file(entry)
                    if resolved is None:
                        continue

                    if _is_shared_lib(resolved) or _is_static_lib(resolved):
                        dest_dir = ARTIFACTS_DIR / "lib"
                    elif (resolved.stat().st_mode & 0o111) != 0:
                        dest_dir = ARTIFACTS_DIR / "bin"
                    else:
                        continue

                    dest = dest_dir / resolved.name
                    shutil.copy2(resolved, dest)
                    copied[dest_dir.name].append(
                        {"src": str(resolved), "dest": str(dest), "size": dest.stat().st_size})

                    if not _is_static_lib(resolved):
                        _collect_runtime_deps(resolved, copied, seen_dep_keys)

    firmware_build_dir = SRC_DIR / "safety_manager" / "firmware" / "esp32_estop" / ".pio" / "build"
    if firmware_build_dir.exists():
        for f in sorted(firmware_build_dir.rglob("*")):
            if f.is_file() and f.suffix in (".bin", ".elf", ".map"):
                dest = ARTIFACTS_DIR / "firmware" / f.name
                shutil.copy2(f, dest)
                copied["firmware"].append({"src": str(f), "dest": str(dest), "size": dest.stat().st_size})

    return copied


def _git_commit_hash() -> str | None:
    try:
        result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=WORKSPACE_ROOT,
                                 capture_output=True, text=True, timeout=5)
    except Exception:  # noqa: BLE001
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def write_manifest(discovered: list[dict], package_results: dict, firmware_result: dict | None,
                    copied: dict, started_at: datetime, dry_run: bool = False) -> dict:
    manifest = {
        "timestamp": started_at.isoformat(),
        "git_commit": _git_commit_hash(),
        "dry_run": dry_run,
        "duration_sec": (datetime.now(timezone.utc) - started_at).total_seconds(),
        "discovered_packages": discovered,
        "build_results": package_results,
        "firmware_result": firmware_result,
        "artifacts": copied,
    }
    (ARTIFACTS_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def print_summary(package_results: dict, firmware_result: dict | None, copied: dict) -> None:
    print("\n=== Summary ===")
    print(f"{'Package':30} {'Status':10} Log")
    for name, info in sorted(package_results.items()):
        print(f"{name:30} {info['status']:10} {info.get('log', '')}")
    if firmware_result:
        print(f"{'esp32_estop (firmware)':30} {firmware_result['status']:10} {firmware_result.get('log', '')}")
    print(
        f"\nCollected: {len(copied['bin'])} executables, {len(copied['lib'])} libraries, "
        f"{len(copied['firmware'])} firmware files, {len(copied['deps'])} runtime deps -> {ARTIFACTS_DIR}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--packages", nargs="*", default=[], help="Build/package only these packages")
    parser.add_argument("--clean", action="store_true", help="Wipe artifacts/ before recreating it")
    parser.add_argument("--force", action="store_true", help="Build even if environment validation fails")
    parser.add_argument("--debug", action="store_true", help="Build with CMAKE_BUILD_TYPE=Debug")
    parser.add_argument("--skip-firmware", action="store_true", help="Skip the ESP32 PlatformIO build")
    parser.add_argument("--dry-run", action="store_true",
                         help="Only run discovery + folder validation, skip build and copy")
    args = parser.parse_args()

    started_at = datetime.now(timezone.utc)
    print("== package_artifacts.py ==")
    print(f"Workspace: {WORKSPACE_ROOT}")

    discovered = discover_packages()
    print(f"\nDiscovered {len(discovered)} build definitions:")
    for entry in discovered:
        print(f"  [{entry['build_type']:12}] {entry['name']:24} {entry['path']}")

    created, existing = ensure_artifacts_structure(args.clean)
    print(f"\nartifacts/ folders created: {created or 'none'}")
    print(f"artifacts/ folders already existing: {existing or 'none'}")

    logs_dir = ARTIFACTS_DIR / "logs"

    if args.dry_run:
        print("\n--dry-run: discovery and folder validation complete, skipping build/copy.")
        write_manifest(discovered, {}, None, {"bin": [], "lib": [], "firmware": [], "deps": []},
                       started_at, dry_run=True)
        return 0

    build_ok, package_results = run_build(args.packages, args.debug, args.force, logs_dir)
    firmware_result = run_firmware_build(logs_dir, args.skip_firmware)

    copied = collect_binaries() if build_ok else {"bin": [], "lib": [], "firmware": [], "deps": []}

    write_manifest(discovered, package_results, firmware_result, copied, started_at)
    print_summary(package_results, firmware_result, copied)

    any_failed = (
        not build_ok
        or any(info.get("status") == "failed" for info in package_results.values())
        or (firmware_result is not None and firmware_result.get("status") == "failed")
    )
    return 1 if any_failed else 0


if __name__ == "__main__":
    sys.exit(main())
