#!/usr/bin/env python3
"""Generate the SHA-256 manifest for the published bundle.

This is the mechanism the whole priority claim rests on, so it is deliberately
small and auditable: hash every published file, write the hashes to a plain
manifest, and let OpenTimestamps anchor that manifest in Bitcoin.

`manifest.txt` is NOT listed inside itself (that would be circular). Its own
hash is recorded in `manifest.txt.sha256`, which is what gets stamped.

Usage:
    python3 scripts/build_manifest.py
"""

from __future__ import annotations

import hashlib
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BLOG = ROOT / "blog"

# Published site content, in reading order.
SITE_FILES = [
    "index.html",
    "style.css",
    "robots.txt",
    ".nojekyll",
    "verify.html",
    # The proof artifacts themselves are deliberately NOT listed. manifest.txt.ots
    # and manifest.tsr are created *after* this manifest and are verified by the
    # tools that consume them (`ots verify`, `openssl ts -reply`). Listing a proof
    # inside the manifest it proves would be circular, and listing a hash-file
    # whose content is this manifest's own digest is circular twice over.
    "evidence/jev_run_1789798395.json",
    "evidence/jev_closed_loop_run.txt",
    "evidence/control_law_diagnosis.json",
    "evidence/mcp_e2e_test_output.txt",
    "evidence/turtle_verify.png",
    "notes/system-one-and-system-two.html",
    "notes/ros-mcp-verified.html",
    "notes/jev-latency.html",
    "notes/jev-closed-loop.html",
    "notes/what-did-not-work.html",
    "notes/reproduce.html",
    "notes/on-publication.html",
]

# Source that produced the published evidence.
#
# NOTE: IDEA.md is deliberately absent. It is my private scratch file and it
# contains a live API key. Hashing a secret is not a leak by itself, but the
# bundle is published, and a manifest is a list of things worth looking at.
# Secrets stay out of anything that ships.
#
# build_manifest.py is also absent: it would be hashing a copy of itself, and
# the staged copy would not match the running copy, so the hash would drift
# between runs and the manifest would never verify twice.
SOURCE_FILES = [
    "scripts/jev_controller.py",
    "scripts/control_law_diagnosis.py",
    "scripts/jev_smoke.py",
    "scripts/jev_discriminate.py",
    "scripts/mcp_e2e_test.py",
    "scripts/probe_rosbridge.py",
    "scripts/sim_demo_motion.py",
    "scripts/start_sim.sh",
    "sim/Dockerfile.sim",
    "sim/launch_turtlesim.launch.py",
]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def third_party_revision() -> str:
    """Pin the exact ros-mcp-server revision rather than hashing vendored code.

    Committing or hashing a whole third-party checkout would attribute their
    source to me in the manifest, which is the opposite of the point.
    """
    repo = ROOT / "vendor" / "ros-mcp-server"
    if not (repo / ".git").exists():
        return "present, no .git (revision not pinned)"
    try:
        rev = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "HEAD"],
            capture_output=True, text=True, check=True, timeout=20,
        ).stdout.strip()
        return rev
    except Exception:  # noqa: BLE001
        return "present, git rev-parse failed"


def stage_sources() -> list[str]:
    """Copy the source that produced the evidence into the bundle as src/.

    Two reasons this exists rather than listing paths back into the repo. The
    manifest is published, so its paths must resolve for whoever downloads the
    bundle - a path like `scripts/foo.py` is meaningless outside my machine.
    And a reader should be able to check the control-law claim against the code
    without being handed the whole repository.
    """
    staged: list[str] = []
    for rel in SOURCE_FILES:
        src = ROOT / rel
        if not src.exists():
            continue
        dst = BLOG / "src" / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(src.read_bytes())
        staged.append(f"src/{rel}")
    return staged


def main() -> int:
    lines: list[str] = []
    add = lines.append
    missing: list[str] = []

    add("# SHA-256 manifest - Working Notes on System 1 robot control")
    add(f"# Generated: {datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')}")
    add("# Author: Justin (github.com/BrewHog)")
    add("#")
    add("# Verify from this directory:")
    add("#     shasum -a 256 -c manifest.txt    (macOS)")
    add("#     sha256sum -c manifest.txt        (Linux)")
    add("#")
    add("# manifest.txt is not listed inside itself. It is anchored directly:")
    add("#   manifest.txt.ots  - OpenTimestamps proof, Bitcoin-blockchain backed")
    add("#   manifest.tsr      - RFC 3161 token from a commercial timestamp authority")
    add("# Both cover this file's own SHA-256, so verify them as described in")
    add("# verify.html rather than looking for them in the list below.")

    staged = stage_sources()

    for title, files in (("PUBLISHED SITE", SITE_FILES), ("SOURCE THAT PRODUCED THE EVIDENCE", staged)):
        add(f"# --- {title} ---")
        for rel in files:
            path = BLOG / rel
            if not path.exists():
                missing.append(rel)
                add(f"# MISSING (not published): {rel}")
                continue
            add(f"{sha256(path)}  {rel}")

    add("# --- THIRD-PARTY DEPENDENCY (not mine, not hashed) ---")
    add(f"# ros-mcp-server revision: {third_party_revision()}")
    add("# https://github.com/robotmcp/ros-mcp-server - Contoro")
    add("# ROS 2 Jazzy + turtlesim : Open Robotics / ROS community")
    add("# System 1 model (JEV)    : TypeSafe - https://docs.typesafe.ai/introduction")
    add("# OpenTimestamps          : https://opentimestamps.org")

    manifest = BLOG / "manifest.txt"
    manifest.write_text("\n".join(lines) + "\n")

    digest = sha256(manifest)

    print(f"manifest : {manifest}")
    print(f"files    : {sum(1 for ln in lines if ln and not ln.startswith('#'))}")
    print(f"sha256   : {digest}")
    if missing:
        print("\nWARNING - listed but not found:")
        for m in missing:
            print(f"  {m}")
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
