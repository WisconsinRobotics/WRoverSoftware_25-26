#include <Preferences.h>

Preferences prefs;

#define REFRESH_RATE 10 // Hz
#define ANALOG_PORT_IN 0 
#define STORAGE "rover_storage"
#define TOTALCHARGE "total_charge"
#define LASTVOLTAGE "last_voltage"
#define LASTCURRENT "last_current"

// LED azul da ESP32 (GPIO2)
#define BLUE_LED 2

// Valor de voltagem que ativa o LED
#define TRIGGER_VOLTAGE 50  


// -----------------------------
// STRUCTS
// -----------------------------

struct __attribute__((packed)) BatteryData {
  float voltage = 0;
  float current = 0;
  float ampHours = 0;

  String toString(){
    return "[voltage:" + String(voltage) + 
           ", current:" + String(current) +
           ", ampHours:" + String(ampHours) + "]";
  }
};

struct __attribute__((packed)) Packet {
  uint8_t start = 0xAA;
  BatteryData data;
  uint8_t checksum = 0;
  uint8_t end = 0x55;

  String toString(){
    return "[data:" + data.toString() + 
           ", checksum:" + String(checksum) + "]";
  }
};


// -----------------------------
// GLOBALS
// -----------------------------

BatteryData d;

uint8_t rx_buffer[20];
int rx_index = 0;
bool receiving = false;

int lastWrite = 0;
int lastLoopCall = 0;


// -----------------------------
// LOAD & STORE
// -----------------------------

void loadData(){
  d.ampHours = prefs.getFloat(TOTALCHARGE, 0);
  d.voltage  = prefs.getFloat(LASTVOLTAGE, 0);
  d.current  = prefs.getFloat(LASTCURRENT, 0);
}

void storeData(){
  prefs.putFloat(TOTALCHARGE, d.ampHours);
  prefs.putFloat(LASTVOLTAGE, d.voltage);
  prefs.putFloat(LASTCURRENT, d.current);
}


// -----------------------------
// PROCESS RAW DATA (simulação)
// -----------------------------

void processRawData(float dt){
  d.current = random(7,10);
  d.voltage = random(5,7);

  if(d.ampHours > 0){
    d.ampHours -= d.current * dt;
    if(d.ampHours < 0) d.ampHours = 0;
  }
}


// -----------------------------
// SEND PACKET (ESP → PC)
// -----------------------------

void packageData(){
  Packet p;
  p.data = d;

  uint8_t *ptr = (uint8_t*)&p.data;
  for (int i=0; i < sizeof(p.data); i++){
    p.checksum ^= ptr[i];
  }

  Serial.write((uint8_t*)&p, sizeof(p));
  Serial.flush();
}


// -----------------------------
// RECEIVE PACKET (PC → ESP)
// -----------------------------

void readSerialFromPC() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    if (!receiving) {
      if (b == 0xAA) {
        receiving = true;
        rx_index = 0;
        rx_buffer[rx_index++] = b;
      }
      continue;
    }

    rx_buffer[rx_index++] = b;

    if (rx_index == 15) {
      receiving = false;

      if (rx_buffer[14] != 0x55) return;

      uint8_t *data_ptr = &rx_buffer[1];

      uint8_t chk = 0;
      for (int i = 0; i < 12; i++){
        chk ^= data_ptr[i];
      }

      if (chk != rx_buffer[13]) {
        Serial.println("Checksum error on incoming packet!");
        return;
      }

      float *fv = (float*)data_ptr;

      float newV  = fv[0];
      float newC  = fv[1];
      float newAh = fv[2];

      d.voltage = newV;
      d.current = newC;
      d.ampHours = newAh;

      // ------------------------------
      // TRIGGER LED
      // ------------------------------
      if (d.voltage == TRIGGER_VOLTAGE) {
        digitalWrite(BLUE_LED, HIGH);
        Serial.println("LED ON – Trigger voltage received!");
        delay(3000);
      } else {
        digitalWrite(BLUE_LED, LOW);
      }

      Serial.print("Received from PC → ");
      Serial.println(d.toString());
    }
  }
}


// -----------------------------
// SETUP
// -----------------------------

void setup() {
  Serial.begin(9600);
  prefs.begin(STORAGE, false);
  loadData();

  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW);

  lastLoopCall = millis();
}


// -----------------------------
// MAIN LOOP
// -----------------------------

void loop() {

  readSerialFromPC();  // recebe dados do ROS2

  int now = millis();
  float dt = (now - lastLoopCall) / 1000.0f;
  lastLoopCall = now;

  processRawData(dt);

  if (now - lastWrite > 500) {
    storeData();
    lastWrite = now;
  }

  packageData();       // envia dados para o ROS2

  delay(1000 / REFRESH_RATE);
}
