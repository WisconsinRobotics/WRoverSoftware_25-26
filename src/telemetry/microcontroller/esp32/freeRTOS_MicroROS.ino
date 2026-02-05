#include <Preferences.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

#define LED_PIN 2

// Non volitile storage
#define STORAGE "rover_storage"
#define CURRENT "current"
#define VOLTAGE "voltage"
#define AMP_HOURS "amp_hours"
Preferences prefs;

// Struct for general battery data
struct __attribute__((packed)) BatteryData{
  float voltage = 0;
  float current = 0;
  float ampHour = 0;

  String toString(){
    return "{\"voltage\":" + String(voltage) +
      ",\"current\":" + String(current) +
      ",\"ampHours\":" + String(ampHour)+"}}"; 
  }
};

BatteryData batteryDataBufferOne;
BatteryData batteryDataBufferTwo;

BatteryData* readBuffer = &batteryDataBufferOne;
BatteryData* writeBuffer = &batteryDataBufferTwo;

//uROS
rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


/* ------         ------ */
/* ------ STORAGE ------ */
/* ------         ------ */
/**
Load the data from NVS storage
*/
void loadData(){
  float current = prefs.getFloat(CURRENT, 0.0);
  float voltage = prefs.getFloat(VOLTAGE, 0.0);
  float ampHours = prefs.getFloat(AMP_HOURS, 0.0);

  // set both buffers to be the data
  batteryDataBufferOne.ampHour = ampHours;
  batteryDataBufferOne.voltage = voltage;
  batteryDataBufferOne.current = current;

  batteryDataBufferTwo.ampHour = ampHours;
  batteryDataBufferTwo.voltage = voltage;
  batteryDataBufferTwo.current = current;
}

/**
Stashes the data int VMS for persistant data
*/
void storeData(){
  // take the most recently commited read buffer and stash it
  prefs.putFloat(CURRENT, readBuffer->current);
  prefs.putFloat(VOLTAGE, readBuffer->voltage);
  prefs.putFloat(AMP_HOURS, readBuffer->ampHour);
}




/* ------      ------ */
/* ------ uROS ------ */
/* ------      ------ */
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    String batteryData = readBuffer->toString();
    char charBatteryData[batteryData.length()];
    batteryData.toCharArray(charBatteryData, batteryData.length());

    // assign the data and publish it
    rosidl_runtime_c__String__assign(&msg.data, charBatteryData);
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

/* ------          ------ */
/* ------ FreeRTOS ------ */
/* ------          ------ */
TaskHandle_t uROSTaskHandler = NULL;
TaskHandle_t binkTaskHandler = NULL;

/*
the task responsible for running ROS2
*/
void uROSTask(void * arg){
  //set up the transports for uROS
  set_microros_transports();

  // set up uROS2
  while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN,HIGH);
    delay(500);
  }

  allocator = rcl_get_default_allocator();

  //create uROS init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create uROS node
  RCCHECK(rclc_node_init_default(&node, "battery_telemetry_node", "", &support));

  // create node publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "battery_telemetry"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  std_msgs__msg__String__init(&msg);
  rosidl_runtime_c__String__assign(&msg.data, "test");
  //msg.data = "";


  // for loop that runs uROS spin
  while(true){
    vTaskDelay(100 / portTICK_PERIOD_MS);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
}

void blinkTast(void * args){
  while(true){
    digitalWrite(4, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(4, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // write the data
    writeBuffer->current = (float)random(-100, 100);
    writeBuffer->voltage = (float)random(-100, 100);
    writeBuffer->ampHour = (float)random(-100, 100);

    // swap buffers
    BatteryData * temp = writeBuffer;
    writeBuffer = readBuffer;
    readBuffer = temp;

    // load data into storage
    storeData();
  }
}
/* ----- -----*/

// setups
void setup() {
  // set up pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  // start non volitile storage and initialize data
  prefs.begin(STORAGE, false);
  loadData();

  // task that runs reading tasks
  xTaskCreatePinnedToCore(
    blinkTast,   // Task function
    "uROSTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    5,                 // Priority
    &binkTaskHandler,  // Task handle
    0                  // Core 0
  );

  // Task that runs ROS
  xTaskCreatePinnedToCore(
    uROSTask,   // Task function
    "uROSTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &uROSTaskHandler,  // Task handle
    1                  // Core 1
  );
  
}

void loop() {}