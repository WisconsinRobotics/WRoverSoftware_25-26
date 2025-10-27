from machine import ADC, Pin
import time
import struct
import sys

adc_pin = 34
adc = ADC(Pin(adc_pin))
adc.atten(ADC.ATTN_11DB)
adc.width(ADC.WIDTH_12BIT)

V_FULL = 4.20
V_EMPTY = 1
V_REF = 3.3
DIVIDER_RATIO = 2.0

def voltage_to_soc(v):
    if v >= V_FULL:
        return 100.0
    elif v <= V_EMPTY:
        return 0.0
    else:
        return (v - V_EMPTY) / (V_FULL - V_EMPTY) * 100.0

while True:
    raw = adc.read()
    v_meas = (raw / 4095.0) * V_REF
    v_batt = v_meas * DIVIDER_RATIO
    soc = voltage_to_soc(v_batt)

    data = struct.pack('<ff', v_batt, soc)

    #print(v_batt, soc)
    
    #struck
    sys.stdout.buffer.write(data)
    sys.stdout.flush()

    time.sleep(0.5)
