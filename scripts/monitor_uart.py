#! /usr/bin/env python3

import serial
import time
import sys
import os
import subprocess

def reset_target():
    print("Resetting target via OpenOCD...")
    try:
        # Run OpenOCD to trigger a hardware reset.
        subprocess.run([
            'openocd', 
            '-f', 'interface/stlink.cfg', 
            '-f', 'target/stm32g4x.cfg', 
            '-c', 'init; reset; exit'
        ], capture_output=True, check=True)
        print("Target reset successfully.")
    except Exception as e:
        print(f"Warning: Failed to reset target: {e}")

def main():
    try:
        reset_target()

        port = os.getenv('UART_PORT', '/dev/ttyUSB0')
        baud = int(os.getenv('UART_BAUD', '115200'))
        
        # Give OpenOCD a moment to release the ST-Link
        time.sleep(0.5)
        
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud.")
        print("Monitoring UART... (Ctrl+C to stop)")
        
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting).decode('ascii', errors='replace')
                sys.stdout.write(data)
                sys.stdout.flush()
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nStopping monitor.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()
