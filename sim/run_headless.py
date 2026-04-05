import sys
import subprocess
import time
import os

import atexit

import signal

if len(sys.argv) < 3:
    print("Usage: run_headless.py <test_runner.resc> <firmware.elf>")
    sys.exit(1)

resc_file = os.path.abspath(sys.argv[1])
elf_file = os.path.abspath(sys.argv[2])

print(f"Starting Renode Headless Simulation... {elf_file}")

# Start renode in its own process group
proc = subprocess.Popen(['renode', '--disable-xwt', '--port', '-1', '-e', f'$elf_path=@{elf_file}; i @{resc_file}'], 
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

def cleanup():
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

atexit.register(cleanup)

log_file = os.path.join(os.path.dirname(resc_file), "uart.log")
if os.path.exists(log_file):
    os.remove(log_file)

# Wait for log file to be created
for _ in range(10):
    if os.path.exists(log_file):
        break
    time.sleep(0.5)

passed = False
failed = False
start_time = time.time()
pos = 0

try:
    while time.time() - start_time < 120:
        if os.path.exists(log_file):
            with open(log_file, "r", errors="ignore") as f:
                f.seek(pos)
                new_content = f.read()
                if new_content:
                    sys.stdout.write(new_content)
                    sys.stdout.flush()
                    pos = f.tell()
                
                # Re-read full content to check for exit condition
                f.seek(0)
                full_content = f.read()
                if "Global test environment tear-down" in full_content or "test suites ran." in full_content:
                    if "[  FAILED  ]" in full_content:
                        failed = True
                    else:
                        passed = True
                    break
                    
        if proc.poll() is not None:
            break
            
        time.sleep(0.2)
finally:
    cleanup()

if passed and not failed:
    print("\n\nTests Passed!")
    sys.exit(0)
else:
    if not passed and not failed:
        print("\n\nTests Timed Out.")
    else:
        print("\n\nTests Failed.")
    sys.exit(1)
