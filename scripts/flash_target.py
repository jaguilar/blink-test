#!/usr/bin/env python3

import argparse
import subprocess
import sys
import os
from pathlib import Path

# Add scripts directory to path to import debug_tools
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from debug_tools import get_toolchain, get_bazel_artifact_path
except ImportError:
    def get_toolchain():
        return {
            "interface": "interface/stlink.cfg",
            "target": "target/stm32g4x.cfg"
        }
    def get_bazel_artifact_path(target):
        return None

def get_elf_path(target_name, build_dir="bazel-bin"):
    """Search for the ELF file corresponding to the target name."""
    # Look for target_name.elf in the build directory recursively
    # In Bazel, the output might be in a subdirectory corresponding to the package
    results = list(Path(build_dir).rglob(f"{target_name}.elf"))
    if not results:
        # Try without .elf suffix
        results = list(Path(build_dir).rglob(f"{target_name}"))
        # Filter for files, avoiding .map files or directories
        results = [r for r in results if r.is_file() and not r.suffix == ".map"]
    
    if not results:
        return None
    
    # Return the most likely one (shortest path or first found)
    # Often Bazel has multiple symlinks, we want the one in bazel-bin directly or its subdirs
    return results[0]

def main():
    tools = get_toolchain()
    parser = argparse.ArgumentParser(description="Build and flash a Bazel target to the device.")
    parser.add_argument("target", help="The Bazel target to build and flash (e.g., blink-test or //tests:blink-tests).")
    parser.add_argument("--build-dir", default="bazel-bin", help="The build directory (default: bazel-bin)")
    parser.add_argument("--interface", default=tools.get("interface", "interface/stlink.cfg"), help="OpenOCD interface config")
    parser.add_argument("--target-cfg", default=tools.get("target", "target/stm32g4x.cfg"), help="OpenOCD target config")
    parser.add_argument("--config", help="Bazel config to use (e.g., dbg, opt)")
    
    args = parser.parse_args()

    # 1. Build the target
    bazel_target = args.target
    if not bazel_target.startswith("//") and not bazel_target.startswith(":"):
        # If it's just a name, assume it's in the root BUILD file
        bazel_target = f"//:{bazel_target}"

    print(f"--- Building target: {bazel_target} ---")
    build_cmd = ["bazel", "build", bazel_target]
    if args.config:
        build_cmd.extend(["-c", args.config])
        
    try:
        subprocess.check_call(build_cmd)
    except subprocess.CalledProcessError as e:
        print(f"Build failed for target {bazel_target}")
        sys.exit(1)

    # 2. Find the ELF file
    # First try querying Bazel directly
    elf_path = get_bazel_artifact_path(bazel_target)
    
    # Fallback to searching in bazel-bin
    if not elf_path:
        # Extract the name from the target (e.g., //tests:blink-tests -> blink-tests)
        target_name = bazel_target.split(":")[-1]
        elf_path = get_elf_path(target_name, args.build_dir)
        
    if not elf_path:
        print(f"Could not find ELF file for target: {bazel_target}")
        sys.exit(1)
    
    print(f"Found ELF: {elf_path}")

    # 3. Flash the target
    print(f"--- Flashing: {elf_path} ---")
    flash_cmd = [
        "openocd",
        "-f", args.interface,
        "-f", args.target_cfg,
        "-c", f"program {elf_path} verify reset exit"
    ]
    
    try:
        subprocess.check_call(flash_cmd)
    except subprocess.CalledProcessError as e:
        print("Flashing failed.")
        sys.exit(1)

    print("\nSuccessfully built and flashed target!")

if __name__ == "__main__":
    main()
