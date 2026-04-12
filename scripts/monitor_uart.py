import serial
import time
import sys
import os

def main():
    try:
        # Use UART_PORT from environment or default to /dev/ttyUSB0
        port = os.getenv('UART_PORT', '/dev/ttyUSB0')
        baud = int(os.getenv('UART_BAUD', '115200'))
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud.")
        
        start_time = time.time()
        handshake_done = False
        
        print("Waiting for READY_FOR_TESTS handshake...")
        
        while time.time() - start_time < 30: # Max 30 seconds for the whole test
            if ser.in_waiting:
                data = ser.read(ser.in_waiting).decode('ascii', errors='replace')
                sys.stdout.write(data)
                sys.stdout.flush()
                
                if "READY_FOR_TESTS" in data and not handshake_done:
                    print("\n[Handshake] Detected READY_FOR_TESTS. Sending START_TESTS...")
                    ser.write(b"START_TESTS\n")
                    handshake_done = True
                
                if "test suites ran" in data or "Tests finished" in data:
                    print("\n[Monitor] Tests completed detected.")
                    break
                    
            time.sleep(0.01)
            
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
