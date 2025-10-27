import serial
import time

# Change COM3 to your port (COM3, /dev/ttyUSB0, etc.)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for connection

with open('/home/daino/Desktop/Research Projects/FCDataLab/05_Sensor_integeration/ESP_DataLogger/ESP_Datalogger/Data_stored/sensor_data.csv', 'w') as f:
    print("Logging data... Press Ctrl+C to stop")
    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(line)  # Display on screen
                f.write(line + '\n')  # Write to file
                f.flush()  # Ensure data is saved
    except KeyboardInterrupt:
        print("\nLogging stopped")
        ser.close()