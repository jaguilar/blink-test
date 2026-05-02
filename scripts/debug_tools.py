import os
import glob
import subprocess

def get_toolchain():
    return {
        "gdb_server": "openocd",
        "gdb": "gdb-multiarch",
        "interface": "interface/stlink.cfg",
        "target": "target/stm32g4x.cfg"
    }

def get_bazel_artifact_path(target):
    """Query Bazel for the output path of a target."""
    try:
        # If it's just a name, assume it's in the root
        if not target.startswith("//") and not target.startswith(":"):
            target = f"//:{target}"
            
        # Use cquery to find the output file for the target
        result = subprocess.check_output(
            ["bazel", "cquery", "--output=files", target],
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
        
        if not result:
            return None
            
        # If there are multiple files, take the first one
        path = result.split('\n')[0]
        
        # Return absolute path if possible, otherwise relative to current dir
        if os.path.exists(path):
            return os.path.abspath(path)
            
        # Fallback to searching in bazel-bin if cquery gives something weird
        # or if we are not at the root
        return path
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None

def start_gdb_server(port=3333):
    tools = get_toolchain()
    
    cmd = [
        tools['gdb_server'],
        "-f", tools['interface'],
        "-f", tools['target'],
        "-c", f"gdb_port {port}",
        "-c", "tcl_port disabled",
        "-c", "telnet_port disabled",
        "-c", "init"
    ]
    # Start in background
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

if __name__ == "__main__":
    tools = get_toolchain()
    print(f"GDB Server: {tools['gdb_server']}")
    print(f"GDB: {tools['gdb']}")
    print(f"Config: {tools['interface']} + {tools['target']}")
