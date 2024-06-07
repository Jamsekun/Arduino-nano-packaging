//
//    FILE: nano_package.cpp
//  AUTHOR: James Kyle S. Balolong
// PURPOSE: Nano packaging 2 jars at a time.
//     URL: https://github.com/Jamsekun/Arduino-nano-packaging/blob/main/src/nano_package.cpp
// COMMENTS:
// CODE INSPIRATION: their group project is inspired by https://www.youtube.com/watch?v=26YH8Mtsxzg&t=1107s

#include <Arduino.h>
#include <Wire.h>
#include "HX711.h"
#include "AccelStepper.h"
#include <Servo.h>

#define DEBUG // Uncomment to disable debugging printing
#ifdef DEBUG
#define debug_print(x) Serial.print(x)
#define debug_println(x) Serial.println(x)
#else
#define debug_print(x)
#define debug_println(x)
#endif
//------------stepper--------
#define motorInterfaceType 1
const int stepPin = 3;
const int dirPin = 4;
AccelStepper rotatorStepper(motorInterfaceType, stepPin, dirPin);

//------------scale-------------
HX711 scale;
uint8_t dataPin = A4;
uint8_t clockPin = A5;

//---------------servo------------
Servo hopper;
const int hopperServopin = 11; // servo signal to ng mg996r model

float val_hopper = 20;
float val_hopperFiltered = 20;

float filter(float prevValue, float currentValue, int filter)
{
  float lengthFiltered = (prevValue + (currentValue * filter)) / (filter + 1);
  return lengthFiltered;
}

//-------Time----------
unsigned long currentMillis;
unsigned long previousMillis = 0; // set up timer for whole machine
const unsigned long interval = 10;

unsigned long previousPackagingMillis = 0;
unsigned long previousCappingMillis = 0;
unsigned long previousHopperMillis = 0;
//-------------------------------
const int sensor1 = 7; // signal ng sensor na initial position, it ust detect the initial position
int sensor1Flag = 0;
const int sensor2 = 8; // pandetect ng jar sa hopper, near the servo hopper
int sensor2Flag = 0;
const int sensor3 = 9; // pandetect ng jar near sa capping.
int sensor3Flag = 0;

int led = A1;
// int w_pump = 12;
int a_pump = 2; // air solenoid tlga to
int cap_m = 5;  // capping relay
int conv = 6;   // conyeyor 12Vdc motor
int buzzer = 10;

// Variable to keep track of loop count
int loopCount = 0;

// For i2c comms
char receivedChar = '\0'; // after the whole process is done the recievedchar must be reset

// auto start
bool autoStart = true;
int autoStartFlag = 0;

//---------automation---------
int state = 0;
int stepFlagCapping = 0;
int stepFlagHopper = 0;

void receiveEvent(int bytes);

void setup()
{
  {
    // Pin modes

    //-------------STEPPER MOTORS--------
    rotatorStepper.setMaxSpeed(1000);
    rotatorStepper.setAcceleration(50);
    rotatorStepper.setSpeed(200);
    rotatorStepper.moveTo(100); // For a 180-degree rotation, you’ll need 100 steps (180 / 1.8 = 100).

    hopper.attach(hopperServopin);
    // pinMode(startt, INPUT_PULLUP);
    pinMode(sensor1, INPUT);
    pinMode(sensor2, INPUT);
    pinMode(sensor3, INPUT);
    // pinMode(w_pump, OUTPUT);
    pinMode(cap_m, OUTPUT);
    pinMode(conv, OUTPUT);
    pinMode(led, OUTPUT);
    // pinMode(stepPin, OUTPUT);
    // pinMode(dirPin, OUTPUT);
    scale.begin(dataPin, clockPin);
    scale.set_scale(420.0983); // you can change this value in loadcell calibration
    scale.tare(20);
    scale.get_units(1); // you get the value of the scale

    // Find initial position
    // while (digitalRead(sensor1) == HIGH)
    // {
    //   rotatorStepper.runSpeed();
    //   if (digitalRead(sensor1) == LOW)
    //   {
    //     rotatorStepper.stop();
    //     break;
    //   }
    // }
  }

  //-----I2C communication----------
  Wire.begin(9);                // Start the I2C Bus as Slave on address 9 (same as master) or address 0x70
  Wire.onReceive(receiveEvent); // Attach a function to trigger when something is received

#ifdef DEBUG
  Serial.begin(9600); // Adjust baud rate as needed
  debug_println("Debug mode enabled");
  debug_println(__FILE__);
  debug_println("LIBRARY VERSION: ");
  debug_println("JAMES KYLE S. BALOLONG");
  debug_println();
#endif
}

void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    // start event
    previousMillis = currentMillis;
    // once na rerecieve
    if (receivedChar == 's')
    {
      state = 1;
      receivedChar = '\0';
    }

    if (Serial.read() == 's')
    {
      debug_println("done serial reading");
      state = 1;
      // loopCount = 4;
    }

    //------------auto start for testing purposes---------------
    // comment out this code kapag nakaconnect na sa arduino mega
    if (autoStart && autoStartFlag == 0)
    {
      state = 1;
      autoStartFlag = 1;
    }
    //---------------------------------------------------------

    // naka input pull up. hopper fill
    if (sensor2 == HIGH && sensor2Flag == 0)
    {
      debug_println("sensed jar in hopper");
      sensor2Flag = 1;
      stepFlagHopper = 1;
    }
    else if (sensor2 == LOW && sensor2Flag == 1)
    {
      delay(250);
      sensor2Flag = 0;
    }

    if (sensor3 == HIGH && sensor3Flag == 0)
    {
      debug_println("sensed jar in capper");
      sensor3Flag = 1;
      stepFlagCapping = 1;
    }

    else if (sensor3 == LOW && sensor3Flag == 1)
    {
      delay(250);
      sensor3Flag = 0;
    }

    if (state == 0 && currentMillis - previousPackagingMillis > 500)
    {
      debug_println("doing nothing");
      previousPackagingMillis = currentMillis;
    }
    else if (state == 1 && currentMillis - previousPackagingMillis > 1000)
    {
      // run dc motor
      digitalWrite(conv, HIGH);
      debug_println("running conv");
      state = 2;
      // digitalWrite(a_pump, HIGH);
      previousPackagingMillis = currentMillis;
    }

    else if (state == 2 && currentMillis - previousPackagingMillis > 450)
    {
      // off motor
      digitalWrite(conv, LOW);

      debug_println("stopped conv");
      state = 3;
      previousPackagingMillis = currentMillis;
    }

    else if (state == 3 && currentMillis - previousPackagingMillis > 6000)
    {

      rotatorStepper.moveTo(100); // MOVE 180 DEGREES
      stepFlagHopper = 1;
      // digitalWrite(a_pump, LOW);
      // For a 180-degree rotation, you’ll need 100 steps (180 / 1.8 = 100).
      debug_println("running servo and stepper 180 degrees");
      state = 4;
      previousPackagingMillis = currentMillis;
    }

    else if (state == 4 && currentMillis - previousPackagingMillis > interval)
    {
      if (loopCount < 5)
      {
        loopCount++;
        state = 1;
      }
      else
      {
        loopCount = 0;     // Reset loop count on start
        autoStartFlag = 0; // reset autostart flag
        debug_println("reseting back");
        state = 0;
      }
    }

    if (stepFlagCapping == 1 && currentMillis - previousHopperMillis > 1500)
    {
      digitalWrite(cap_m, HIGH);
      digitalWrite(a_pump, HIGH);
      debug_println("capping");
      previousCappingMillis = currentMillis;
      stepFlagCapping = 2;
    }

    else if (stepFlagCapping == 2 && currentMillis - previousCappingMillis > 4500)
    {
      digitalWrite(cap_m, LOW);
      digitalWrite(a_pump, LOW);
      debug_println("done cap");
      previousCappingMillis = currentMillis;
      stepFlagCapping = 0;
    }

    if (stepFlagHopper == 1 && currentMillis - previousHopperMillis > 1500)
    {
      val_hopper = 20; // open value haopper
      debug_println("open hopper");
      previousHopperMillis = currentMillis;
      stepFlagHopper = 2;
    }

    else if (stepFlagHopper == 2 && currentMillis - previousHopperMillis > 1500)
    {
      val_hopper = 180; // close value hopper
      debug_println("close hopper");
      previousHopperMillis = currentMillis;
      stepFlagHopper = 0;
    }
  }

  // servo smoothing
  val_hopperFiltered = filter(val_hopper, val_hopperFiltered, 20);

  hopper.write(val_hopperFiltered);
}

void receiveEvent(int bytes)
{
  receivedChar = Wire.read(); // Read the received character
}