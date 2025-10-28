from machine import ADC, Pin
import time, json, sys

adc = ADC(Pin(34))
adc.atten(ADC.ATTN_11DB)
adc.width(ADC.WIDTH_12BIT)

V_FULL, V_EMPTY, V_REF, DIVIDER_RATIO = 4.20, 1.0, 3.3, 2.0

def voltage_to_soc(v):
    return max(0, min(100, (v - V_EMPTY) / (V_FULL - V_EMPTY) * 100))

while True:
    raw = adc.read()
    v_meas = (raw / 4095.0) * V_REF
    v_batt = v_meas * DIVIDER_RATIO
    soc = voltage_to_soc(v_batt)

    payload = json.dumps({"voltage": round(v_batt, 3), "soc": round(soc, 2)})
    # endpoints STX e ETX
    sys.stdout.write(f"\x02{payload}\x03")
    sys.stdout.flush()
    time.sleep(0.5)