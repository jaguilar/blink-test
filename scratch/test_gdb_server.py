import scripts.debug_tools as dt
import subprocess
import time
import os
import sys

# Ensure scripts is in path
sys.path.append(os.path.join(os.getcwd(), "scripts"))

tools = dt.get_toolchain()
print(f"GDB Server: {tools['gdb_server']}")
print(f"Programmer Dir: {tools['programmer_dir']}")

cmd = [
    tools['gdb_server'],
    "-cp", tools['programmer_dir'],
    "-p", "3334",
    "-l", "1",
    "-m", "1"
]

print(f"Running: {' '.join(cmd)}")
p = subprocess.Popen(cmd)
try:
    time.sleep(10)
except KeyboardInterrupt:
    pass
finally:
    p.terminate()
