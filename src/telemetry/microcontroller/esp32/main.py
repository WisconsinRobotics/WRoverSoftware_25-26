from machine import ADC, Pin
import time
import sys

adc_pin = 34 
adc = ADC(Pin(adc_pin))

adc.atten(ADC.ATTN_11DB)  # leitura até ~3.3V
adc.width(ADC.WIDTH_12BIT)  # resolução de 12 bits (0–4095)

while True:
    raw_value = adc.read()
    
    #tension converter
    voltage = (raw_value / 4095.0) * 3.3
    
    sys.stdout.write(f"{voltage:.3f}\n")
    sys.stdout.flush()
    
    time.sleep(0.2)