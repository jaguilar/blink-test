import sys
import os
import time
import serial

from debug_tools import get_toolchain
from gdb_agent import GDBAgent

def validate_gdb():
    elf = "build/tests/blink-tests.elf"
    tools = get_toolchain()
    
    if not tools['gdb']:
        print("Error: arm-none-eabi-gdb not found.")
        return

    print(f"Using GDB: {tools['gdb']}")
    print(f"Using ELF: {elf}")

    agent = GDBAgent(elf, tools['gdb'], server_port=3334)
    agent.start()
    
    try:
        print("Connecting to GDB server...")
        resp = agent.connect()
        print(f"Connect response: {resp}")
        if "^error" in resp:
            print("Failed to connect. Check if ST-LINK GDB server is running on 3334.")
            return

        # Set breakpoints
        # breakpoint 1: Start of Loop
        print("Setting breakpoint at main_test.cc:Loop...")
        agent.send_command("-break-insert tests/main_test.cc:Loop")
        
        # breakpoint 2: StartOutOfPhase calculation point
        print("Setting breakpoint at foc_types.h:380...")
        header_path = os.path.abspath("Core/Inc/foc_types.h")
        agent.send_command(f"-break-insert {header_path}:380")
        
        # Reset and Run
        print("Resetting and halting...")
        agent.send_command("monitor reset halt")
        
        print("Continuing...")
        agent.send_command("-exec-continue", wait_for_result=False)
        
        # Wait for any breakpoint hit
        print("Waiting for breakpoint hit...")
        start_time = time.time()
        hit_phase_delay = False
        
        # We might need to send START_TESTS if it gets stuck at the read()
        sent_start = False
        
        while time.time() - start_time < 30: # 30 second timeout
            try:
                line = agent.output_queue.get(timeout=1)
                if "*stopped" in line:
                    print(f"Stopped: {line}")
                    if "main_test.cc" in line and "line=\"421\"" in line:
                        print("Hit Loop breakpoint. It's probably waiting for START_TESTS...")
                        # We are at printf, the next thing is read().
                        # Let's continue and send START_TESTS.
                        agent.send_command("-exec-continue", wait_for_result=False)
                        
                        if not sent_start:
                            print("Sending START_TESTS to /dev/ttyUSB0...")
                            try:
                                with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
                                    ser.write(b"START_TESTS\n")
                                sent_start = True
                            except Exception as e:
                                print(f"Failed to send START_TESTS: {e}")
                    
                    if "foc_types.h" in line and "line=\"380\"" in line:
                        print("Hit StartOutOfPhase breakpoint!")
                        hit_phase_delay = True
                        break
            except:
                continue
        
        if not hit_phase_delay:
            print("Timed out waiting for StartOutOfPhase breakpoint.")
            return

        # Read phase_delay
        print("Reading phase_delay...")
        resp = agent.read_var("phase_delay")
        print(f"Read variable response: {resp}")
        
        import re
        match = re.search(r'value="([^"]+)"', resp)
        if match:
            print(f"\nSUCCESS: phase_delay = {match.group(1)}")
        else:
            print("Could not parse value from response.")

    finally:
        agent.quit()

if __name__ == "__main__":
    validate_gdb()
