import serial
import time
import os
from datetime import datetime

# -------------------------
# Configuration
# -------------------------
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
OUTPUT_FILE = '/home/daino/Desktop/Research Projects/FCDataLab/05_Sensor_integeration/ESP_DataLogger/ESP_Datalogger/Data_stored/sensor_data.csv'
ALERT_FILE = '/home/daino/Desktop/Research Projects/FCDataLab/05_Sensor_integeration/ESP_DataLogger/ESP_Datalogger/Data_stored/sensor_alerts.log'

# -------------------------
# Setup serial
# -------------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # wait for serial

os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)

# -------------------------
# State variables
# -------------------------
line_count = 0
suspicious_data_start = None

def log_alert(message):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    alert_msg = f"[{timestamp}] {message}"
    print(alert_msg)
    with open(ALERT_FILE, 'a') as f:
        f.write(alert_msg + '\n')
        f.flush()

# -------------------------
# Write header if file doesn't exist
# -------------------------
if not os.path.exists(OUTPUT_FILE):
    with open(OUTPUT_FILE, 'w') as f:
        f.write("Timestamp(ms),"
                "S0_X,S0_Y,S0_Z,S0_T,"
                "S1_X,S1_Y,S1_Z,S1_T,"
                "S2_X,S2_Y,S2_Z,S2_T,"
                "S3_X,S3_Y,S3_Z,S3_T,"
                "S4_X,S4_Y,S4_Z,S4_T,"
                "tS0_start,tS0_end,"
                "tS1_start,tS1_end,"
                "tS2_start,tS2_end,"
                "tS3_start,tS3_end,"
                "tS4_start,tS4_end,"
                "Δ01,Δ12,Δ23,Δ34,Δ04_total\n")

# -------------------------
# Main loop
# -------------------------
try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue

        # Print all lines for monitoring
        print(line)

        # Skip comment lines
        if line.startswith('#'):
            continue

        # Write CSV data
        with open(OUTPUT_FILE, 'a') as f:
            f.write(line + '\n')
            f.flush()

        line_count += 1

except KeyboardInterrupt:
    print(f"\n✅ Logging stopped by user. Total lines: {line_count}")
    log_alert(f"Session ended with {line_count} data lines")

except Exception as e:
    log_alert(f"ERROR: {e}")
    import traceback
    traceback.print_exc()

finally:
    ser.close()
    print("🔌 Serial connection closed")
