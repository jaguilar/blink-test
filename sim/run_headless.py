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

log_file = os.path.join(os.path.dirname(resc_file), "uart.log")
if os.path.exists(log_file):
    os.remove(log_file)

# Open log file for renode's own output
renode_log_path = os.path.join(os.path.dirname(resc_file), "renode.log")
renode_log = open(renode_log_path, "w")

# Start renode in its own process group, disconnected from terminal stdin.
# We add a line hook to the UART to automatically quit when we see the test completion marker.
# This ensures that even if output is buffered, it is flushed to disk as soon as tests finish.
renode_cmds = [
    f'$elf_path=@{elf_file};',
    f'$uart_log=@{log_file};',
    f'i @{resc_file};',
    'sysbus.usart1 AddLineHook "test suites ran." "monitor.Parse(\'quit\')"'
]

args = ['renode', '--disable-xwt', '--plain', '--console', '-e', " ".join(renode_cmds)]
proc = subprocess.Popen(args,
                        preexec_fn=os.setsid, 
                        stdin=subprocess.PIPE,
                        stdout=renode_log,
                        stderr=subprocess.STDOUT)

def cleanup():
    if proc.poll() is None:
        try:
            # Tell the whole process group to exit
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            time.sleep(0.5)
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    renode_log.close()

atexit.register(cleanup)

# Wait for log file to be created
for _ in range(10):
    if os.path.exists(log_file):
        break
    time.sleep(0.5)

passed = False
failed = False
start_time = time.time()
pos = 0
renode_pos = 0

try:
    while time.time() - start_time < 120:
        full_content = ""
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
                
        if os.path.exists(renode_log_path):
            with open(renode_log_path, "r", errors="ignore") as f:
                f.seek(renode_pos)
                new_content = f.read()
                if new_content:
                    sys.stdout.write(new_content)
                    sys.stdout.flush()
                    renode_pos = f.tell()

        if "Global test environment tear-down" in full_content or "test suites ran." in full_content:
            if "[  FAILED  ]" in full_content:
                failed = True
            else:
                passed = True
            break

        poll_res = proc.poll()            
        if poll_res is not None:
            print(f'proc.poll returned {poll_res}')
            break
            
        time.sleep(0.2)
finally:
    cleanup()
    # Final check of the logs after termination to capture any remaining output
    if os.path.exists(log_file):
        with open(log_file, "r", errors="ignore") as f:
            full_content_final = f.read()
            if "Global test environment tear-down" in full_content_final or "test suites ran." in full_content_final:
                # Update passed/failed status if we found the result in the final check
                if "[  FAILED  ]" in full_content_final:
                    failed = True
                else:
                    passed = True


if passed and not failed:
    print("\n\nTests Passed!")
    sys.exit(0)
else:
    if not passed and not failed:
        print("\n\nTests Timed Out.")
    else:
        print("\n\nTests Failed.")
    sys.exit(1)
