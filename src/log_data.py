import serial
import time
import os

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
OUTPUT_FILE = '/home/daino/Desktop/Research Projects/FCDataLab/05_Sensor_integeration/ESP_DataLogger/ESP_Datalogger/Data_stored/sensor_data.csv'

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for connection to establish

print("=" * 60)
print("BMM350 Multi-Sensor Data Logger")
print("=" * 60)
print(f"Port: {SERIAL_PORT}")
print(f"Baud Rate: {BAUD_RATE}")
print(f"Output File: {OUTPUT_FILE}")
print("=" * 60)
print("\nWaiting for sensors to initialize and complete self-test...")
print("(This may take 10-20 seconds)")
print("-" * 60)

# Create directory if it doesn't exist
os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)

# State machine
data_logging_started = False
header_written = False
line_count = 0

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if not line:
            continue
        
        # Display all lines (including comments) to console for monitoring
        print(line)
        
        # Check if data logging has started
        if not data_logging_started:
            if "STARTING DATA COLLECTION" in line:
                print("\n" + "=" * 60)
                print("✅ Self-test complete! Starting data logging...")
                print("=" * 60 + "\n")
                data_logging_started = True
            continue
        
        # Once data logging started, filter and save only data lines
        if data_logging_started:
            # Skip comment lines (lines starting with #)
            if line.startswith('#'):
                continue
            
            # Write to CSV file
            with open(OUTPUT_FILE, 'a') as f:
                f.write(line + '\n')
                f.flush()
            
            line_count += 1
            
            # Print progress every 50 lines
            if line_count % 50 == 0:
                print(f"\n📊 Logged {line_count} data points")
                
except KeyboardInterrupt:
    print("\n" + "=" * 60)
    print(f"✅ Logging stopped")
    print(f"📁 Total data points logged: {line_count}")
    print(f"💾 Data saved to: {OUTPUT_FILE}")
    print("=" * 60)
    
except Exception as e:
    print(f"\n❌ Error occurred: {e}")
    
finally:
    ser.close()
    print("🔌 Serial connection closed")