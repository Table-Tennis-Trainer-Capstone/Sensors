#!/usr/bin/env python3
"""
run_rviz.py  —  M.A.R.T.Y. cross-platform RViz / MoveIt2 launcher

Works on macOS, Linux, and Windows — no X11 / XQuartz setup required.
RViz renders inside a virtual display in the container. View it at:

    http://localhost:8080/vnc.html

Usage:
    python3 src/ttt_bringup/scripts/run_rviz.py              # build image (first time) then launch
    python3 src/ttt_bringup/scripts/run_rviz.py --rebuild    # force-rebuild the Docker image
    python3 src/ttt_bringup/scripts/run_rviz.py --clean      # delete cached build volumes then exit

Requirements:
    Docker Desktop (macOS / Windows) or Docker Engine (Linux)
    Download: https://www.docker.com/products/docker-desktop
"""

import os
import sys
import platform
import subprocess
import shutil
import argparse

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
IMAGE_TAG     = "marty-rviz:latest"
DOCKERFILE    = os.path.join(SCRIPT_DIR, "Dockerfile.rviz")
BUILD_VOL     = "marty-colcon-build"
INSTALL_VOL   = "marty-colcon-install"


def _run(cmd: str, **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, shell=True, **kwargs)

def _ok(cmd: str) -> bool:
    return subprocess.run(cmd, shell=True, capture_output=True).returncode == 0

def die(msg: str):
    print(f"\nERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def require_docker():
    if not shutil.which("docker"):
        print("\nDocker is not installed or not on PATH.")
        print("  macOS / Windows : https://www.docker.com/products/docker-desktop")
        print("  Ubuntu / Debian : sudo apt-get install docker.io")
        die("Install Docker and re-run.")
    if not _ok("docker info"):
        print("\nDocker daemon is not running.")
        print("  macOS / Windows : open Docker Desktop and wait for it to start")
        print("  Linux           : sudo systemctl start docker")
        die("Start Docker and re-run.")
    print("  ✓ Docker is running")


def image_exists() -> bool:
    return subprocess.run(
        ["docker", "image", "inspect", IMAGE_TAG],
        capture_output=True,
    ).returncode == 0

def build_image(force: bool = False):
    if image_exists() and not force:
        print(f"  ✓ Image '{IMAGE_TAG}' already built  (use --rebuild to refresh)")
        return
    verb = "Rebuilding" if force else "Building"
    print(f"\n  {verb} Docker image '{IMAGE_TAG}'")
    print("  First build downloads ~2 GB and takes ~10 min; later builds use cache.")
    if _run(f'docker build -t {IMAGE_TAG} -f "{DOCKERFILE}" "{WORKSPACE_DIR}"').returncode != 0:
        die("docker build failed — see output above.")
    print(f"  ✓ Image ready: {IMAGE_TAG}")


def docker_path(host_path: str) -> str:
    """Convert host path to a Docker-compatible path string."""
    if platform.system() == "Windows":
        p = host_path.replace("\\", "/")
        if len(p) >= 2 and p[1] == ":":
            p = "/" + p[0].lower() + p[2:]
        return p
    return host_path

def clean_volumes():
    print("\n  Removing colcon build cache volumes...")
    for vol in (BUILD_VOL, INSTALL_VOL):
        _run(f"docker volume rm {vol} 2>/dev/null || true")
    print("  ✓ Done. Next launch will do a clean build.")


def launch():
    src_path = docker_path(os.path.join(WORKSPACE_DIR, "src"))

    print("\n  Launching M.A.R.T.Y. RViz  (MoveIt2 visualisation)")
    print("  Building workspace and starting nodes — this takes ~30 s on first run.")
    print()
    print("  ┌──────────────────────────────────────────┐")
    print("  │  Open in your browser once RViz starts:  │")
    print("  │                                          │")
    print("  │    http://localhost:8080/vnc.html        │")
    print("  │                                          │")
    print("  └──────────────────────────────────────────┘")
    print()
    print("  Press Ctrl+C to stop.\n")

    _run(
        f'docker run --rm -it '
        f'--name marty-rviz '
        f'-p 8080:8080 '
        f'-v "{src_path}":/ws/src:ro '
        f'-v {BUILD_VOL}:/ws/build '
        f'-v {INSTALL_VOL}:/ws/install '
        f'{IMAGE_TAG}'
    )


def main():
    parser = argparse.ArgumentParser(
        description="M.A.R.T.Y. cross-platform RViz / MoveIt2 launcher"
    )
    parser.add_argument("--rebuild", action="store_true",
                        help="Force-rebuild the Docker image")
    parser.add_argument("--clean", action="store_true",
                        help="Delete colcon build volumes then exit")
    args = parser.parse_args()

    print("\n========================================")
    print("  M.A.R.T.Y.  RViz  Launcher")
    print(f"  Platform : {platform.system()} {platform.machine()}")
    print("========================================")

    require_docker()

    if args.clean:
        clean_volumes()
        return

    build_image(force=args.rebuild)
    launch()


if __name__ == "__main__":
    main()