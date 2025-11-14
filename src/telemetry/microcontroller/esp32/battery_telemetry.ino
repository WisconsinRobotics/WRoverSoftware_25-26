#include <Preferences.h>

Preferences prefs;

#define REFRESH_RATE 10 // Hz
#define ANALOG_PORT_IN 0 
#define STORAGE "rover_storage"
#define TOTALCHARGE "total_charge"
#define LASTVOLTAGE "last_voltage"
#define LASTCURRENT "last_current"

// battery voltage data for storing data
struct __attribute__((packed)) BatteryData{
  float voltage = 0;
  float current = 0;
  float ampHours= 0;

  String toString(){
    return "[voltage:" + String(voltage) + 
    ", current: " + String(current) +
    ", ampHours: " + String(ampHours) + "]";
  }
};



// a packet for sending over battery data etc
struct __attribute__((packed)) Packet{
  // start bit for decoding
  uint8_t start = 0xAA;
  BatteryData data;
  uint8_t checksum = 0;
  uint8_t end = 0x55;

  String toString(){
    return "[data: " + data.toString() + ", checksum: " + String(checksum) + "]";
  }
};



// data storage
//int data = 0;
BatteryData d;



// delta time variables
int lastWrite = 0;
int lastLoopCall = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // init VMS data & load the stuff
  prefs.begin(STORAGE,false);
  loadData();
}





/**
Load the data from VMS storage into memory
*/
void loadData(){
  float total_charge = prefs.getFloat(TOTALCHARGE, 0);
  float last_voltage = prefs.getFloat(LASTVOLTAGE, 0);
  float last_current = prefs.getFloat(LASTCURRENT, 0);

  d.ampHours = total_charge;
  d.voltage = last_voltage;
  d.current = last_current;
}



/**
Stashes the data into VMS storage for persistant data
*/
void storeData(){
  prefs.putFloat(TOTALCHARGE, d.ampHours);
  prefs.putFloat(LASTVOLTAGE, d.voltage);
  prefs.putFloat(LASTCURRENT, d.current);
}




/**
Takes the raw data and computes the voltage volumes
*/
void processRawData(float dt){
  // simulate a change in voltage and currnet
  d.current = random(7,10);
  d.voltage = random(5,7);


  // do the voltage calculations
  if(d.ampHours > 0){
    d.ampHours -= d.current * dt;
    if(d.ampHours <= 0){
      d.ampHours = 0;
    }
  }
}


/**
Packages the data into a packet, computes the checksum, sends out the byte data
*/
void packageData(){
  // construct the packet to send out to the ROS2 Node
  Packet p;
  p.data = d;

  // check sum
  byte* ptr = (byte*)&p.data;
  size_t len = sizeof(p.data);
  for(size_t i = 0; i < len; i++){
    p.checksum ^= ptr[i];
  }


  // convert package into bytes and send over
  byte* ptr_package = (byte*)&p;
  for(size_t i = 0; i < (size_t)sizeof(p); i ++){
    Serial.write(ptr_package[i]);
  }

  Serial.flush();

}



void loop() {
  // delta time loop stuff
  int now = millis();
  int dt = now - lastLoopCall;
  lastLoopCall = now;

  // get data and process it here
  processRawData(dt);

  // Cache the charge 2 times every second
  if(now - lastWrite > 500){
    storeData();
    lastWrite = millis();
  }

  // package and send out the data
  packageData();

  //String output = "Data: " + d.toString() + " | dt: " + String(dt) + "| len: " + String(sizeof(d));
  // sleep for the polling rate
  sleep(1/REFRESH_RATE);
}


