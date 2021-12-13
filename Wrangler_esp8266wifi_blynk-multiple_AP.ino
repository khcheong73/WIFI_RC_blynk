/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example runs directly on ESP8266 chip.

  Note: This requires ESP8266 support package:
    https://github.com/esp8266/Arduino

  Please be sure to select the right ESP8266 module
  in the Tools -> Board menu!

  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "YourAuthToken";

//Array to store Network Credentials for different WiFi Networks
const char* KNOWN_SSID[] = {"SSID#1", "SSID#2", "SSID#3"}; //Put all Your WiFi Network Names
const char* KNOWN_PASSWORD[] = {"PASSWORD#1", "PASSWORD#2", "PASSWORD#3"}; //Put the WiFi Passwords in same order. For Open networks leave the password blank inside the double quotes.


const int KNOWN_SSID_COUNT = sizeof(KNOWN_SSID) / sizeof(KNOWN_SSID[0]); // number of known networks

// Defining variables
int X; // X-Axis value
int Y; // Y-Axis value
int A; // Accelerator value

// Assigning ports
int LED_HEAD = D0;   // Headlight (digitial I/O)
int MOTOR_PHASE = D5; // Motor phase (FWD = 0, BWD = 1)
int MOTOR_PWM = D6;   // Motor speed control (PWM)
int SERVO = D8;       // Servo control (PWM)
int LED_BRAKE = D7;   // Brake light
int LED_LEFT = D4;   // Left turn light (digital I/O)
int LED_RIGHT = D3;  // Right turn light (digital I/O)
int LED_BACK = MOTOR_PHASE;        // Function and port shared with MOTOR_PHASE

// define LED mode
int EMERGENCY = LOW;
int LT_TURN = LOW;
int RT_TURN = LOW;
int LED_LT_STATE = LOW;
int LED_RT_STATE = LOW;
int EMRG_LED_STATE = LOW;
unsigned long TimePrev = 0;
unsigned long TimeNow;

unsigned long StopPrev = 0;
unsigned long StopNow;

const long interval = 500;

// Defining ports for Widget LED on Blynk app
// V0: 1st Joystick-X (Steering)
// V1: 2nd Joystick-Y (Up - forward/ Down - backward)
// V3: Emergency Light
WidgetLED WBRAKE(V4);
WidgetLED WBACK(V5);
WidgetLED WLEFT(V6);
WidgetLED WRIGHT(V7);
// V8: Measured Voltage

// define voltage sensor parameters
float R1 = 2000;
float R2 = 1000;
int sensorPin = A0;
float Vref;
float Vsense;
float Vcc = 3.15 ; // Vcc means maximum voltage for Analog input
float Vth = 6.2; // threshold value to warn low battery voltage

// define DC motor with DRV8838
int speed;

// define servo
Servo servo;
float SERVO_CENTER = 83;
//float SERVO_LEFT = SERVO_CENTER - 45;
//float SERVO_RIGHT = SERVO_CENTER + 45;
int SERVO_ANGLE;

// Define functions
void stopMotor()
{
//  digitalWrite(MOTOR_PHASE, LOW);
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(LED_BRAKE, HIGH); // WBRAKE.on();
  digitalWrite(LED_BACK, LOW); // WBACK.off();
}

void setMotor(int SPEED, boolean FORWARD)
{
//  digitalWrite(MOTOR_PHASE, FORWARD);
  analogWrite(MOTOR_PWM, SPEED);
  digitalWrite(LED_BRAKE, LOW); // WBRAKE.off();
}

void servoCenter()
{
  servo.write(SERVO_CENTER);
}

void setServo(int STEER) {
  servo.write(SERVO_ANGLE);

  if ( EMERGENCY == LOW) {
    if ( X < 410 ) { // Left Turn
      LT_TURN = HIGH;
      RT_TURN = LOW;
    }
    else if ( X > 614 ) { // Right Turn
      LT_TURN = LOW;
      RT_TURN = HIGH;  
    }
    else { // Center
      LT_TURN = LOW;
      RT_TURN = LOW;
    }
  }
}

void ReadVoltage() {
  Vsense = analogRead(sensorPin) * Vcc / 1023.0;
  Vref = ((R1+R2) / R2) * Vsense;
  Blynk.virtualWrite(V8, Vref);
  if ( Vref < Vth) { 
    Blynk.virtualWrite(V9, "Low Batt.");
  }
  else {    Blynk.virtualWrite(V9, "Batt. OK");
  }
}

BLYNK_WRITE(V0) //X-Axis
{
  int pinValue = param.asInt(); // assigning incoming value from pin V0 to a variable
  X = pinValue;
  // Servo control
  SERVO_ANGLE = -90*X/1024 + 135;
  setServo(SERVO_ANGLE);

}

BLYNK_WRITE(V1) // Y-Axis
// Y determines forward or backward
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Blynk.virtualWrite(V10, speed);
  if (pinValue > 512) { // forward
    speed = ( pinValue - 512 )*2;
    analogWrite(MOTOR_PWM, speed);
    // setMotor(speed, HIGH);
    digitalWrite(LED_BACK, LOW); // WBACK.off();
    digitalWrite(LED_BRAKE, LOW); // WBRAKE.off();  
  }
  else if (pinValue < 512) { // backward
    speed = (512 - pinValue) *2;
    analogWrite(MOTOR_PWM, speed);
    //setMotor(speed, LOW);
    digitalWrite(LED_BACK, HIGH); // WBACK.on();
    digitalWrite(LED_BRAKE, LOW); // WBRAKE.off();
  }
  else {
    stopMotor();
    speed=0;
  }
}

BLYNK_WRITE(V3) // Emergency Lights
{
  int pinValue = param.asInt(); // assigning incoming value from pin V3 to a variable
  EMERGENCY = pinValue;
}
// ------------- END of Fucntions

void setup()
{
  boolean wifiFound = false;
  int i, n;

  Serial.begin(115200);
//  Serial.println("Waiting for connections...");

  u8g2.begin();
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g_font_profont12); // choose a suitable font
  u8g2.drawStr(0,20, "OH35A01"); 
  u8g2.sendBuffer();
  

  // Blynk will work through Serial
  // Do not read or write this serial manually in your sketch
//  Blynk.begin(auth, ssid, pass);
//  Serial.print ("SSID: "); Serial.println (ssid);
  

// Defining pin mode for digital I/O
  pinMode(LED_HEAD, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_BRAKE, OUTPUT);
  pinMode(LED_BACK, OUTPUT);
  pinMode(MOTOR_PHASE, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  digitalWrite(LED_HEAD, LOW);
  digitalWrite(LED_RIGHT, LOW); // WRIGHT.off();
  digitalWrite(LED_LEFT, LOW); // WLEFT.off();
  digitalWrite(LED_BRAKE, LOW); // WBRAKE.off();
  digitalWrite(LED_BACK, LOW); // WBACK.off();

  servo.attach(SERVO);
  servoCenter(); // Set Steering wheel to center

  stopMotor();

  // ----------------------------------------------------------------
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  // ----------------------------------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("Setup done");

  // ----------------------------------------------------------------
  // WiFi.scanNetworks will return the number of networks found
  // ----------------------------------------------------------------
  Serial.println(F("Scan start"));
  int nbVisibleNetworks = WiFi.scanNetworks();
  Serial.println(F("Scan Completed"));
  if (nbVisibleNetworks == 0) {
  Serial.println(F("No networks found. Reset to try again"));
  while (true); // no need to go further, hang in there, will auto launch the Soft WDT reset
  }

  // ----------------------------------------------------------------
  // if you arrive here at least some networks are visible
  // ----------------------------------------------------------------
//  Serial.print(nbVisibleNetworks);
//  Serial.println(" network(s) found");

  // ----------------------------------------------------------------
  // check if we recognize one by comparing the visible networks
  // one by one with our list of known networks
  // ----------------------------------------------------------------

  for (i = 0; i < nbVisibleNetworks; ++i) {
//    Serial.println(WiFi.SSID(i)); // Print current SSID
    for (n = 0; n < KNOWN_SSID_COUNT; n++) { // walk through the list of known SSID and check for a match
      if (strcmp(KNOWN_SSID[n], WiFi.SSID(i).c_str())) {
        Serial.print(F("\tNot matching "));
        Serial.println(KNOWN_SSID[n]);
      } else { // we got a match
        wifiFound = true;
        break; // n is the network index we found
      }
    } // end for each known wifi SSID
    if (wifiFound) break; // break from the "for each visible network" loop
  }

  u8g2.clearBuffer();          // clear the internal memory
  
  if (!wifiFound) {
    Serial.println(F("No Known Network identified. Reset to try again"));
    u8g2.drawStr(0,10,"No WIFI");  // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the display

    while (true);
  }

  const char* ssid = (KNOWN_SSID[n]);
  const char* pass = (KNOWN_PASSWORD[n]);
  Serial.println(WiFi.localIP());
  Blynk.begin(auth, ssid, pass);
  delay(1000);
  Serial.println("Blynk Connected"); //Connected and Authenticated with Blynk Server
  u8g2.drawStr(0,10,"SSID: ");
  u8g2.drawStr(35,10, ssid); 
  u8g2.sendBuffer();          // transfer internal memory to the display  
}

void loop()
{
  
//  bool CONNECTED = Blynk.connected();
//  if ( CONNECTED == false ) { Serial.println ("Wifi disconnected!"); }
//  else { Serial.println ("Wifi connected"); }
  
  Blynk.run();
  
  ReadVoltage();
  char buffer[4];
  const char* VOLT = dtostrf(Vref, 2, 1, buffer);
//  Serial.println (s);
  u8g2.drawStr(0,20, "Batt. = "); 
  u8g2.drawStr(50,20, VOLT);
  u8g2.drawStr(70,20, "V"); 
  u8g2.sendBuffer();

  // LED Control
  if ( EMERGENCY == HIGH ) { // Emergency mode
    TimeNow = millis();
    if (TimeNow - TimePrev >= interval) {
      TimePrev = TimeNow;
      LED_LT_STATE = !LED_LT_STATE;
      LED_RT_STATE = !LED_RT_STATE;
    }
  }
  else { // Normal mode
    if (LT_TURN == HIGH ) {
      TimeNow = millis();
      if (TimeNow - TimePrev >= interval) {
        TimePrev = TimeNow;
        LED_LT_STATE = !LED_LT_STATE;
        LED_RT_STATE = LOW;
      }
    } else LED_LT_STATE = LOW;
    if (RT_TURN == HIGH) {
      TimeNow = millis();
      if (TimeNow - TimePrev >= interval) {
        TimePrev = TimeNow;
        LED_LT_STATE = LOW;
        LED_RT_STATE = !LED_RT_STATE;
      }
    } else LED_RT_STATE =LOW;
  }
  digitalWrite(LED_LEFT, LED_LT_STATE); 
  digitalWrite(LED_RIGHT, LED_RT_STATE);
}
