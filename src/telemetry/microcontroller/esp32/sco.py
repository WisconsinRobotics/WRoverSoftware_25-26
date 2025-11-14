import time
import json
from machine import ADC, Pin, UART

# Configuração da UART (serial via USB)
uart = UART(0, baudrate=115200)

# Configuração do ADC
adc_pin = 34  # pino analógico (use 32, 33, 34, 35 ou 36)
adc = ADC(Pin(adc_pin))
adc.atten(ADC.ATTN_11DB)   # Permite leitura até ~3.6 V
adc.width(ADC.WIDTH_12BIT) # 12 bits -> valores 0 a 4095

# Constantes
V_REF = 3.3           # tensão de referência do ADC (em V)
DIVIDER_RATIO = 2.0   # se usou divisor de 2:1
V_FULL = 4.2
V_EMPTY = 3.0          # mínimo seguro da célula (ajuste conforme sua bateria)

while True:
    # Lê valor ADC e converte em volts
    adc_val = adc.read()
    voltage = (adc_val / 4095) * V_REF * DIVIDER_RATIO

    # Calcula o estado de carga (SoC)
    soc = max(0, min(100, (voltage - V_EMPTY) / (V_FULL - V_EMPTY) * 100))

    # Cria JSON
    payload = json.dumps({
        "voltage": round(voltage, 3),
        "soc": round(soc, 2)
    })

    # Envia mensagem com delimitadores STX/ETX
    uart.write(b"\x02" + payload.encode('utf-8') + b"\x03")

    time.sleep(0.5)
