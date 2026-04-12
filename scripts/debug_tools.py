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
