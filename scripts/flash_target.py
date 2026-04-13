#!/usr/bin/env python3

import argparse
import subprocess
import sys
import os
from pathlib import Path

# Add scripts directory to path to import debug_tools
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from debug_tools import get_toolchain
except ImportError:
    def get_toolchain():
        return {
            "interface": "interface/stlink.cfg",
            "target": "target/stm32g4x.cfg"
        }

def get_elf_path(target_name, build_dir="build"):
    """Search for the ELF file corresponding to the target name."""
    # Look for target_name.elf in the build directory recursively
    results = list(Path(build_dir).rglob(f"{target_name}.elf"))
    if not results:
        # Try without .elf suffix if it was already included
        results = list(Path(build_dir).rglob(f"{target_name}"))
        # Filter for files
        results = [r for r in results if r.is_file() and not r.suffix == ".map"]
    
    if not results:
        return None
    
    # Return the most likely one (shortest path or first found)
    return results[0]

def main():
    tools = get_toolchain()
    parser = argparse.ArgumentParser(description="Build and flash a CMake target to the device.")
    parser.add_argument("target", help="The CMake target to build and flash.")
    parser.add_argument("--build-dir", default="build", help="The build directory (default: build)")
    parser.add_argument("--interface", default=tools.get("interface", "interface/stlink.cfg"), help="OpenOCD interface config")
    parser.add_argument("--target-cfg", default=tools.get("target", "target/stm32g4x.cfg"), help="OpenOCD target config")
    
    args = parser.parse_args()

    # 1. Build the target
    print(f"--- Building target: {args.target} ---")
    build_cmd = ["cmake", "--build", args.build_dir, "--target", args.target]
    try:
        subprocess.check_call(build_cmd)
    except subprocess.CalledProcessError as e:
        print(f"Build failed for target {args.target}")
        sys.exit(1)

    # 2. Find the ELF file
    elf_path = get_elf_path(args.target, args.build_dir)
    if not elf_path:
        print(f"Could not find ELF file for target: {args.target} in {args.build_dir}")
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
