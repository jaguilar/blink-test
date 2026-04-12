import sys
import os
import time
from gdb_agent import GDBAgent
from debug_tools import get_toolchain, start_gdb_server

def diagnose():
    elf = "build/tests/blink-tests.elf"
    tools = get_toolchain()
    
    print("Starting GDB server...")
    server_process = start_gdb_server(3334)
    time.sleep(2)
    
    try:
        agent = GDBAgent(elf, tools['gdb'], server_port=3334)
        agent.start()
        
        print("Connecting...")
        print(agent.connect())
        
        print("Resetting and continuing...")
        # Try monitor reset
        agent.send_command("monitor reset")
        agent.send_command("-exec-continue", wait_for_result=False)
        
        print("Waiting for fault (2 seconds)...")
        time.sleep(2)
        
        print("Interrupting...")
        agent.send_command("-exec-interrupt")
        
        # Wait a bit for it to stop
        time.sleep(1)
        
        print("Backtrace:")
        print(agent.get_backtrace())
        
        print("Registers:")
        print(agent.send_command("-data-list-register-values x"))
        
        print("Fault Registers (CFSR, HFSR):")
        print("CFSR:", agent.send_command("-data-evaluate-expression *(uint32_t*)0xE000ED28"))
        print("HFSR:", agent.send_command("-data-evaluate-expression *(uint32_t*)0xE000ED2C"))
        print("BFAR:", agent.send_command("-data-evaluate-expression *(uint32_t*)0xE000ED38"))
        
    finally:
        agent.quit()
        server_process.terminate()

if __name__ == "__main__":
    diagnose()
