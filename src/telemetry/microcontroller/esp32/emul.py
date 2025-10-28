import serial
import time
import json

# this is a simulation of an esp32
ser = serial.Serial('/tmp/ttyV1', 115200)

V_FULL, V_EMPTY, DIVIDER_RATIO = 4.2, 1.0, 2.0
soc = 0
voltage = 1.0

while True:
    voltage += 0.01
    if voltage > V_FULL:
        voltage = V_EMPTY
    soc = max(0, min(100, (voltage - V_EMPTY) / (V_FULL - V_EMPTY) * 100))
    
    payload = json.dumps({"voltage": round(voltage, 3), "soc": round(soc, 2)})
    
    # sending message
    ser.write(f"\x02{payload}\x03".encode('utf-8'))
    
    time.sleep(0.5)