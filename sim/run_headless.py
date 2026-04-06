import sys
import subprocess
import time
import os
import atexit
import signal
import re

def tail_log(file_path, pos):
    """Reads and prints new content from a file, returns the new position and full content."""
    if not os.path.exists(file_path):
        return pos, ""
    
    with open(file_path, "r", errors="ignore") as f:
        # Get new content from pos
        f.seek(pos)
        new_content = f.read()
        if new_content:
            sys.stdout.write(new_content)
            sys.stdout.flush()
            pos = f.tell()
        
        # Get full content for analysis
        f.seek(0)
        full_content = f.read()
        return pos, full_content

def check_results(content):
    """Checks content for test completion and status."""
    # Pattern for total tests to run
    # [==========] Running 4 tests from 2 test suites.
    m_start = re.search(r"\[==========\] Running (\d+) tests", content)
    total_tests = int(m_start.group(1)) if m_start else 0
    
    # Pattern for finished
    finished = "Global test environment tear-down" in content or "test suites ran." in content
    
    # Count OKs and Fails
    oks = content.count("[       OK ]")
    fails = content.count("[  FAILED  ]")
    
    if fails > 0:
        return finished, False, True, "FAILED"
    
    if finished:
        if total_tests > 0 and oks == total_tests:
            return True, True, False, "PASSED"
        else:
            return True, False, False, "UNKNOWN"
            
    return False, False, False, "RUNNING"

if len(sys.argv) < 3:
    print("Usage: run_headless.py <test_runner.resc> <firmware.elf>")
    sys.exit(1)

resc_file = os.path.abspath(sys.argv[1])
elf_file = os.path.abspath(sys.argv[2])
sim_dir = os.path.dirname(resc_file)

log_file = os.path.join(sim_dir, "uart.log")
renode_log_path = os.path.join(sim_dir, "renode.log")

if os.path.exists(log_file):
    os.remove(log_file)

# Open log file for renode's own output
renode_log_handle = open(renode_log_path, "w")

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
                        stdout=renode_log_handle,
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
    if not renode_log_handle.closed:
        renode_log_handle.close()

atexit.register(cleanup)

# Wait for log file to be created
for _ in range(10):
    if os.path.exists(log_file):
        break
    time.sleep(0.5)

status = "UNKNOWN"
start_time = time.time()
uart_pos = 0
renode_pos = 0

try:
    while time.time() - start_time < 120:
        # Tail UART log
        uart_pos, full_uart = tail_log(log_file, uart_pos)
        
        # Tail Renode log
        renode_pos, _ = tail_log(renode_log_path, renode_pos)

        finished, passed, failed, current_status = check_results(full_uart)
        if finished or failed:
            status = current_status
            break

        if proc.poll() is not None:
            break
            
        time.sleep(0.2)
finally:
    cleanup()
    # Final flush of all logs
    uart_pos, full_uart = tail_log(log_file, uart_pos)
    renode_pos, _ = tail_log(renode_log_path, renode_pos)
    
    # One last result check
    finished, passed, failed, current_status = check_results(full_uart)
    if finished or failed:
        status = current_status
    elif status == "RUNNING" or status == "UNKNOWN":
        if time.time() - start_time >= 120:
            status = "TIMED OUT"
        else:
            status = "UNKNOWN"

print(f"\n\nTests {status}")
if status == "PASSED":
    sys.exit(0)
else:
    sys.exit(1)
