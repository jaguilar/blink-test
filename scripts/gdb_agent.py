import subprocess
import os
import sys
import threading
import queue
import time
import json
import re
from debug_tools import get_toolchain

class GDBAgent:
    def __init__(self, elf_file, gdb_path, server_port=3334):
        self.elf_file = elf_file
        self.gdb_path = gdb_path
        self.server_port = server_port
        self.process = None
        self.output_queue = queue.Queue()
        self.token = 1

    def start(self):
        cmd = [
            self.gdb_path,
            "--interpreter=mi2",
            "--quiet",
            self.elf_file
        ]
        self.process = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        # Thread to read stdout
        self.thread = threading.Thread(target=self._read_output, daemon=True)
        self.thread.start()
        
        # Wait for initial prompt
        self.wait_for_prompt()

    def wait_for_prompt(self, timeout=10):
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                line = self.output_queue.get(timeout=1)
                if line == "(gdb)":
                    return True
            except queue.Empty:
                continue
        return False

    def _read_output(self):
        while True:
            line = self.process.stdout.readline()
            if not line:
                break
            self.output_queue.put(line.strip())

    def send_command(self, cmd, wait_for_result=True):
        token = self.token
        self.token += 1
        full_cmd = f"{token}{cmd}\n"
        self.process.stdin.write(full_cmd)
        self.process.stdin.flush()
        
        if not wait_for_result:
            return None

        # Wait for result record starting with ^done, ^error, etc.
        while True:
            try:
                line = self.output_queue.get(timeout=10)
                if line.startswith(f"{token}^"):
                    return line
            except queue.Empty:
                return f"{token}^error,msg=\"Timeout waiting for GDB\""

    def connect(self):
        return self.send_command(f"-target-select remote 127.0.0.1:{self.server_port}")

    def reset(self):
        # STM32 reset via GDB
        self.send_command("monitor reset halt")
        return self.send_command("-exec-continue", wait_for_result=False)

    def get_backtrace(self):
        return self.send_command("-stack-list-frames")

    def read_var(self, var_name):
        return self.send_command(f"-data-evaluate-expression {var_name}")

    def quit(self):
        self.send_command("-gdb-exit")
        if self.process:
            self.process.terminate()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 gdb_agent.py <command> [args]")
        sys.exit(1)

    command = sys.argv[1]
    elf = "build/tests/blink-tests.elf" # Default
    
    tools = get_toolchain()
    gdb_path = os.getenv('GDB_PATH', tools['gdb'])
    agent_port = int(os.getenv('GDB_PORT', '3333'))
    
    agent = GDBAgent(elf, gdb_path, server_port=agent_port)
    agent.start()
    
    try:
        if command == "reset":
            print(agent.connect())
            print(agent.reset())
        elif command == "backtrace":
            print(agent.connect())
            print(agent.get_backtrace())
        elif command == "read":
            if len(sys.argv) < 3:
                print("Usage: read <var_name>")
            else:
                print(agent.connect())
                print(agent.read_var(sys.argv[2]))
        else:
            print(f"Unknown command: {command}")
    finally:
        agent.quit()

if __name__ == "__main__":
    main()
