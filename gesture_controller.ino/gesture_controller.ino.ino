#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// #include <BleGamepad.h>
#include <HardwareSerial.h>
#include "sbus.h"
#include "math.h"
#include "types.h"

#define TRAINER_MODE_SBUS
// #define TRAINER_MODE_PPM

#define SERIAL1_RX -1
#define SERIAL1_TX 39

#ifdef TRAINER_MODE_SBUS
#define SBUS_UPDATE_TASK_MS 15
uint8_t sbusPacket[SBUS_PACKET_LENGTH] = {0};
HardwareSerial sbusSerial(1);
uint32_t nextSbusTaskMs = 0;
#endif

// BleGamepad bleGamepad;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
dataOutput_t output;
int counter = 0;
float formattedX = 0;
float formattedY = 0;
float formattedZ = 0;

bool isOffset = false;
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;


// Rotary Encoder Inputs
#define inputCLK 12  
#define inputDT 13

// LED Outputs
#define ledCW 8
#define ledCCW 9

int counter_scroll = 0; 
int currentStateCLK;
int previousStateCLK; 

String encdir ="";


int angleToRcChannel(float angle)
{
    const float value = fconstrainf(applyDeadband(angle, 5.0f), -45.0f, 45.0f); //5 deg deadband
    return (int)fscalef(value, -45.0f, 45.0f, -500, 500);
  // return map(angle, -90f, 90f, 500, 500);
}
int joystickToRcChannel(float angle)
{
    const float value = fconstrainf(applyDeadband(angle, 0.02f), -1.0f, 1.0f);
    return (int)fscalef(value, -1.0f, 1.0f, -200, 200);
}

int getRcChannel_wrapper(uint8_t channel)
{
  if (channel >= 0 && channel < SBUS_CHANNEL_COUNT)
  {
    return output.channels[channel];
  }
  else
  {
    return DEFAULT_CHANNEL_VALUE;
  }
}

float adjustFloat(float cur, float off)
{
  float adjusted = cur - off;
  if (adjusted < 180.0)
  {
    return adjusted;
  }
  return adjusted - 360.0;
}

void setup(void) 
{
  Serial.begin(115200);
  #ifdef TRAINER_MODE_SBUS
    sbusSerial.begin(100000, SERIAL_8E2, SERIAL1_RX, SERIAL1_TX, false, 100UL);
  #endif
  //Serial2.begin(100000, SERIAL_8E2, SERIAL1_RX, SERIAL1_TX, false, 100UL);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  // bleGamepad.begin();
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    // Set encoder pins as inputs  
  pinMode (inputCLK,INPUT);
  pinMode (inputDT,INPUT);
   
  // Set LED pins as outputs
  pinMode (ledCW,OUTPUT);
  pinMode (ledCCW,OUTPUT);
   
  // Setup Serial Monitor
  Serial.begin(115200);  
  // Read the initial state of inputCLK
  // Assign to previousStateCLK variable
  previousStateCLK = digitalRead(inputCLK);
  delay(1000);
    
  bno.setExtCrystalUse(true);
  
}
float altitude_state(int sum){
  float thrust = 0.0f;
  if (sum > 0){
    thrust = 8.0f;
  }
  else if (sum < 0){
    thrust = -8.0f;
  }
  else {
    thrust = 0.0f;
  }
 
  return thrust;
  
}

void loop(void) 
{
    // Read the current state of inputCLK
   currentStateCLK = digitalRead(inputCLK);
   
   // If the previous and the current state of the inputCLK are different then a pulse has occured
   if (currentStateCLK != previousStateCLK){ 
       
     // If the inputDT state is different than the inputCLK state then 
     // the encoder is rotating counterclockwise
     if (digitalRead(inputDT) != currentStateCLK) { 
       counter_scroll -= 10;
       encdir ="CCW";
       digitalWrite(ledCW, LOW);
       digitalWrite(ledCCW, HIGH);
       
     } else {
       // Encoder is rotating clockwise
       counter_scroll += 10;
       encdir ="CW";
       digitalWrite(ledCW, HIGH);
       digitalWrite(ledCCW, LOW);
       
     }
//     Serial.print("Direction: ");
//     Serial.print(encdir);
//     Serial.print(" -- Value: ");
 
   } 
  
   // Update previousStateCLK with the current state
   previousStateCLK = currentStateCLK; 
  // if (bleGamepad.isConnected())
  // {
    // Serial.println("Press buttons 5, 16 and start. Move all enabled axes to max. Set DPAD (hat 1) to down right.");
    // bleGamepad.press(BUTTON_5);
    // bleGamepad.press(BUTTON_16);
    // bleGamepad.pressStart();

    /* Get a new sensor event */ 
    sensors_event_t event;
    bno.getEvent(&event);
    
    if (!isOffset or counter < 500)
    {
      counter ++;
      offsetX = event.orientation.x;
      // offsetY = event.orientation.y;
      // offsetZ = event.orientation.z;
      isOffset = true;
    }
    
    formattedX = adjustFloat(event.orientation.x, offsetX);
    formattedY = event.orientation.y;
    formattedZ = event.orientation.z;
    // formattedY = adjustFloat(event.orientation.y, offsetY);
    // formattedZ = adjustFloat(event.orientation.z, offsetZ);

    /* Display the floating point data */
    // Serial.print("X: ");
    // Serial.print(formattedX, 4);
    // Serial.print("\tY: ");
    // Serial.print(formattedY, 4);
    // Serial.print("\tZ: ");
    // Serial.print(formattedZ, 4);
    // Serial.println("");

//    float opX = formattedX;
//    float opY = formattedY;
//    float opZ = formattedZ;

      float opX = map(formattedX, -180, 180, 90, -90); // yaw
      float opY = map(formattedY, -90, 90, 90, -90); // roll
      float opZ = map(formattedZ, -180, 180, -90, 90); // pitch
    
    //bleGamepad.setAxes(formattedX, formattedY, formattedZ, 0, 0, 0, 0, 0);
    // (x axis, y axis, z axis, rx axis, ry axis, rz axis, slider 1, slider 2)
    
    // bleGamepad.setHat1(HAT_DOWN_RIGHT);
    // All axes, sliders, hats etc can also be set independently. See the IndividualAxes.ino example

    output.channels[ROLL] = DEFAULT_CHANNEL_VALUE + angleToRcChannel(opY);
    output.channels[PITCH] = DEFAULT_CHANNEL_VALUE + angleToRcChannel(opZ);
    output.channels[YAW] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(opX); // why is it minus?
    output.channels[THROTTLE_T] = DEFAULT_CHANNEL_VALUE + joystickToRcChannel(altitude_state(counter_scroll));
    output.channels[4] = DEFAULT_CHANNEL_VALUE + 1000;
    if (counter_scroll > 0 ){
      counter_scroll -= 1;
    }
    else if (counter_scroll < 0) {
      counter_scroll ++;
    }
    //Serial.println(counter_scroll);
    /* Display the formatted floating point data */
//    Serial.print("X: ");
//    Serial.print(angleToRcChannel(opX));
//    Serial.print("\tY: ");
//    Serial.print(angleToRcChannel(opY));
//    Serial.print("\tZ: ");
//    Serial.print(angleToRcChannel(opZ));
//    Serial.print("\toffset: ");
//    Serial.print(offsetX);
//    Serial.print("\tizz_offset: ");
//    Serial.print(isOffset);
//    Serial.println("");
    Serial.println(joystickToRcChannel(altitude_state(counter_scroll)));
    for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++) {
      output.channels[i] = constrain(output.channels[i], 1000, 2000);
    }

    sbusPreparePacket(sbusPacket, false, false, getRcChannel_wrapper);
    // Serial2.write(sbusPacket, SBUS_PACKET_LENGTH);
    //Serial.print(sbusPacket);
    sbusSerial.write(sbusPacket, SBUS_PACKET_LENGTH);

    nextSbusTaskMs = millis() + SBUS_UPDATE_TASK_MS;
    delay(10);
  // }
  
  
}
