import time
import json
import socket
import select
import paho.mqtt.client as mqtt
import psutil
import csv
import os
import math

    # ThingsBoard Config
THINGSBOARD_HOST = "mqtt.thingsboard.cloud"
ACCESS_TOKEN = "d4o0zhyonwl93gagkbsd"

   # UDP Config
UDP_IP = "0.0.0.0"
# UDP_PORT = 5005
UDP_PORT = 3333 # Reese's port

    # CSV File Setup
CSV_FILE = "ecg_data.csv"
if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "ecg_sample"])

    # MQTT Setup
client = mqtt.Client()
client.username_pw_set(ACCESS_TOKEN)
client.connect(THINGSBOARD_HOST, 1883, 60)
client.loop_start()

    # UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(0)

    # Storage Variables
ecg_samples_batch = []
sensors = {}

    # Helper Functions
def safe_float(x):
    try:
        if x is None:
            return float('nan')
        x = str(x).lower()
        if x == "nan":
            return float('nan')
        return float(x)
    except (TypeError, ValueError):
        return float('nan')

def flush_ecg_to_csv(samples):
    if not samples:
        return
    timestamp = time.time()
    try:
        with open(CSV_FILE, "a", newline="") as f:
            writer = csv.writer(f)
            for sample in samples:
                writer.writerow([timestamp, sample])
            f.flush()
    except Exception as e:
        print(f"Error writing to CSV: {e}")

def ecg_batch_to_csv_string(ecg_batch):
    if not ecg_batch:
        return ""
    valid_samples = [v for v in ecg_batch if not math.isnan(v)]
    return ",".join([f"{v:.3f}" for v in valid_samples])


print("Listening for ECG packets...")

    # Aggregation Timer
last_publish = time.time()

try:
    while True:
            # Read UDP
        ready = select.select([sock], [], [], 0.05)
        if ready[0]:
            data, addr = sock.recvfrom(4096)
            decoded = data.decode().strip().split(",")

            ecg_samples_batch.clear()     # Clear for new batch
            sensors.clear()               # Clear last readings

            for item in decoded:
                if ":" in item:
                    key, val = item.split(":")
                    val = safe_float(val)
                    sensors[key] = val 
                    
                    if key == "amb":           # MLX Ambient Temp / Converts to F
                        sensors["ambient_f"] = (val * 9/5 + 32) if val is not None else None

                    elif key == "obj":         # MLX Object Temp / Converts to F
                        sensors["object_f"] = (val * 9/5 + 32) if val is not None else None

                    elif key == "avgECG":      #ECG Average
                        sensors["ecg_avg"] = val
                        
                    elif key == "BPM":         #BPM
                        sensors["bpm"] = val

                    elif key == "SpO2":        #SpO2
                        sensors["spo2"] = val

                else:
                    val = safe_float(item)
                    ecg_samples_batch.append(val)

                # Save ECG Batch to CSV       
            flush_ecg_to_csv(ecg_samples_batch)
            
            # Publish Aggregated Telemetry Every 1 Sec
        if time.time() - last_publish >= 1.0:
            last_publish = time.time()
            
                # Compute ECG Average, Ignoring NaN
            valid_samples = [v for v in ecg_samples_batch if not math.isnan(v)]
            ecg_avg = sum(valid_samples)/len(valid_samples) if valid_samples else float('nan')
            sensors["ecg_avg"] = ecg_avg                                                            # ensure avg is included
            sensors["ecg_batch"] = ecg_batch_to_csv_string(ecg_samples_batch)
            
                # Publish to ThingsBoard
            client.publish("v1/devices/me/telemetry", json.dumps(sensors, allow_nan=True))
            print(f"Sent To ThingsBoard GUI: {sensors}")

        time.sleep(0.001)

except KeyboardInterrupt:
    print("Shutting down safely...")

finally:
    sock.close()
    client.loop_stop()
    client.disconnect()
