import serial
import time
import os
from datetime import datetime

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
OUTPUT_FILE = '/home/daino/Desktop/Research Projects/FCDataLab/05_Sensor_integeration/ESP_DataLogger/ESP_Datalogger/Data_stored/sensor_data.csv'
ALERT_FILE = '/home/daino/Desktop/Research Projects/FCDataLab/05_Sensor_integeration/ESP_DataLogger/ESP_Datalogger/Data_stored/sensor_alerts.log'

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for connection to establish

print("=" * 60)
print("BMM350 Multi-Sensor Data Logger")
print("=" * 60)
print(f"Port: {SERIAL_PORT}")
print(f"Baud Rate: {BAUD_RATE}")
print(f"Output File: {OUTPUT_FILE}")
print(f"Alert File: {ALERT_FILE}")
print("=" * 60)
print("\nWaiting for sensors to initialize and complete self-test...")
print("(This may take 10-20 seconds)")
print("-" * 60)

# Create directory if it doesn't exist
os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)

# State tracking
data_logging_active = False
line_count = 0
initial_self_test_passed = False
in_periodic_test = False
last_good_data_count = 0
suspicious_data_start = None

def log_alert(message):
    """Log alerts to both console and alert file"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    alert_msg = f"[{timestamp}] {message}"
    print(alert_msg)
    
    with open(ALERT_FILE, 'a') as f:
        f.write(alert_msg + '\n')
        f.flush()

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if not line:
            continue
        
        # Display all lines to console for monitoring
        print(line)
        
        # Check for initialization failures
        if "Initialization FAILED" in line or "ERROR: Sensor" in line:
            print("\n" + "!" * 60)
            print("❌ SENSOR INITIALIZATION FAILED!")
            print("   Check wiring and connections!")
            print("!" * 60 + "\n")
            log_alert("CRITICAL: Sensor initialization failed. Check wiring!")
            break  # Stop the script
        
        # Check for initial self-test results
        if not initial_self_test_passed:
            # Check if any sensor failed initial self-test
            if "failed self-test" in line or "FAILED ❌" in line:
                print("\n" + "!" * 60)
                print("❌ INITIAL SELF-TEST FAILED!")
                print("   Cannot start data logging.")
                print("   Please check sensor connections and restart ESP32")
                print("!" * 60 + "\n")
                log_alert("CRITICAL: Initial self-test failed. Data logging aborted.")
                break  # Stop the script
            
            # Check if all sensors passed
            if "All sensors passed self-test!" in line:
                initial_self_test_passed = True
                print("\n" + "=" * 60)
                print("✅ All sensors passed initial self-test!")
                print("=" * 60 + "\n")
                log_alert("SUCCESS: All sensors passed initial self-test")
        
        # Check if data collection is starting (only after successful self-test)
        if initial_self_test_passed and not data_logging_active:
            if "STARTING DATA COLLECTION" in line:
                print("\n" + "=" * 60)
                print("🚀 Starting data logging...")
                print("=" * 60 + "\n")
                data_logging_active = True
                last_good_data_count = 0
                log_alert("Data logging started")
                continue
        
        # If data logging is not active yet, skip saving data
        if not data_logging_active:
            continue
        
        # Check for periodic self-test start
        if "Running Periodic Self-Test" in line:
            in_periodic_test = True
            last_good_data_count = line_count
            print("\n" + "-" * 60)
            print("🔄 Periodic self-test in progress...")
            print(f"   Data logged so far: {line_count} points")
            print("-" * 60)
        
        # Check for periodic self-test failure
        if in_periodic_test and ("FAILED ❌" in line or "failed periodic self-test" in line):
            suspicious_data_start = last_good_data_count
            
            print("\n" + "!" * 60)
            print("❌ PERIODIC SELF-TEST FAILED!")
            print("!" * 60)
            
            alert_msg = (
                f"CRITICAL: Periodic self-test failed!\n"
                f"   Total data points logged: {line_count}\n"
                f"   Last verified good data: line {last_good_data_count}\n"
                f"   Potentially suspicious data: lines {last_good_data_count + 1} to {line_count}\n"
                f"   STOPPING DATA LOGGING - Please check sensor connections!"
            )
            log_alert(alert_msg)
            print(alert_msg)
            
            # Mark suspicious data in CSV
            with open(OUTPUT_FILE, 'a') as f:
                f.write(f"\n# ALERT: Periodic self-test failed at {datetime.now()}\n")
                f.write(f"# Last known good data: line {last_good_data_count}\n")
                f.write(f"# Suspicious data range: lines {last_good_data_count + 1} to {line_count}\n")
                f.write(f"# Data logging stopped. Check sensor connections!\n")
                f.flush()
            
            print("!" * 60 + "\n")
            break  # Stop logging
        
        # Check for periodic self-test success
        if "Resuming Data Collection" in line:
            in_periodic_test = False
            print("-" * 60)
            print("✅ Periodic self-test passed - data is valid")
            print("-" * 60 + "\n")
            log_alert(f"Periodic self-test passed at {line_count} data points")
        
        # Process data lines
        if data_logging_active:
            # Skip comment lines
            if line.startswith('#'):
                continue
            
            # Check if this is the CSV header
            if 'Timestamp' in line and 'S0_X' in line:
                with open(OUTPUT_FILE, 'w') as f:  # Overwrite file with header
                    f.write(line + '\n')
                    f.flush()
                print("📝 CSV Header written")
                continue
            
            # Check if this is actual CSV data
            if line and ',' in line:
                # Check for sensor read errors
                if 'ERR' in line:
                    error_msg = f"⚠️  WARNING: Line {line_count + 1} contains sensor read errors!"
                    print(f"\n{error_msg}")
                    log_alert(error_msg)
                    
                    # This might indicate a wiring issue
                    print("   Possible wiring problem detected!")
                    print("   Stopping data logging for safety.\n")
                    log_alert("CRITICAL: Sensor read errors detected. Stopping logging.")
                    
                    with open(OUTPUT_FILE, 'a') as f:
                        f.write(f"\n# ERROR: Sensor read failure at line {line_count + 1}\n")
                        f.write(f"# Data logging stopped due to possible wiring issue\n")
                        f.flush()
                    
                    break  # Stop logging on read errors
                
                # Write valid data to file
                with open(OUTPUT_FILE, 'a') as f:
                    f.write(line + '\n')
                    f.flush()
                
                line_count += 1
                
                # Print progress every 50 lines
                if line_count % 50 == 0:
                    print(f"\n📊 Logged {line_count} valid data points ✅")
                
except KeyboardInterrupt:
    print("\n" + "=" * 60)
    print(f"✅ Logging stopped by user")
    print(f"📁 Total data points logged: {line_count}")
    
    if suspicious_data_start is not None:
        suspect_count = line_count - suspicious_data_start
        print(f"\n⚠️  WARNING: {suspect_count} potentially suspicious data points!")
        print(f"   Suspicious range: lines {suspicious_data_start + 1} to {line_count}")
        print(f"   Last verified good data: line {suspicious_data_start}")
        log_alert(f"Session ended with {suspect_count} suspicious data points (lines {suspicious_data_start + 1}-{line_count})")
    else:
        print(f"✅ All {line_count} data points are verified valid")
    
    print(f"💾 Data saved to: {OUTPUT_FILE}")
    print(f"📋 Alert log: {ALERT_FILE}")
    print("=" * 60)
    
except Exception as e:
    print(f"\n❌ Error occurred: {e}")
    log_alert(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    
finally:
    ser.close()
    print("🔌 Serial connection closed")