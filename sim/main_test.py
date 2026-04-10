import sys
import subprocess
import time
import os
import atexit
import signal
import re
import argparse

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
    # Pattern for CppUTest success: "OK (N tests, N ran, N checks, ...)"
    # Pattern for CppUTest failure: "Errors (N failures, N tests, ...)"
    
    finished = "test suites ran." in content
    
    m_ok = re.search(r"OK \((\d+) tests", content)
    m_fail = re.search(r"Errors \((\d+) failures", content)
    
    if m_fail:
        return True, False, True, "FAILED"
    
    if m_ok or finished:
        if m_ok:
            return True, True, False, "PASSED"
        else:
            # If we saw the "finished" hook but not the OK/Errors yet, wait a bit
            return finished, "OK" in content, "Errors" in content, "RUNNING"
            
    return False, False, False, "RUNNING"

parser = argparse.ArgumentParser(description="Run tests in Renode with optional filtering.")
parser.add_argument("resc", help="The Renode script file (.resc)")
parser.add_argument("elf", help="The firmware ELF file")
parser.add_argument("-t", "--test-filter", help="Test filter arguments for CppUTest (e.g. '-sn ShouldPass' or '-sg TimersTest')")
parser.add_argument("--renode", default="renode", help="Path to Renode executable (e.g. 'mono /path/to/Renode.exe')")
args_parsed = parser.parse_args()

resc_file = os.path.abspath(args_parsed.resc)
elf_file = os.path.abspath(args_parsed.elf)
test_filter = args_parsed.test_filter or "-v"
renode_cmd_raw = args_parsed.renode
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

# Handle renode command which might be multiple words (e.g. 'mono Renode.exe')
args = renode_cmd_raw.split() + ['--disable-xwt', '--plain', '--console', '-e', " ".join(renode_cmds)]
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
command_sent = False

try:
    while time.time() - start_time < 30: # Increased timeout for potential handshake
        # Tail UART log
        uart_pos, full_uart = tail_log(log_file, uart_pos)
        
        # Tail Renode log
        renode_pos, _ = tail_log(renode_log_path, renode_pos)

        # Implementation of handshake
        if not command_sent and "READY_FOR_TESTS" in full_uart:
            print(f"---> Sending test filter: {test_filter}")
            # We send the command to Renode's monitor via stdin
            # Some Renode versions/models don't expose FeedString to the monitor.
            # Using a character-by-character WriteChar loop is a more robust fallback.
            full_cmd = f"START_TESTS {test_filter}\n"
            for char in full_cmd:
                cmd = f'sysbus.usart1 WriteChar {ord(char)}\n'
                proc.stdin.write(cmd.encode())
            proc.stdin.flush()
            command_sent = True

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
