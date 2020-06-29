// *******************************************************************
//  Arduino Nano 3.3V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Left Sensor cable (long wired cable)
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// • Option 2: Serial on Right Sensor cable (short wired cable) - recommended, so the ADCs on the other cable are still available
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// *******************************************************************
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <DNSServer.h>
#include <ESPUI.h>
#include <ESP8266WiFi.h>

// ########################## DEFINES ##########################

#define TITLE               "Hoverboard Control v1.0"

#define HOVER_SERIAL_BAUD   38400       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
#define DEBUG_INPUT

#define THROTTLE            A0
#define INVERT              D0
#define PIXEL_PIN           D1
#define NUM_PIXELS          16

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
SoftwareSerial HoverSerial(D3,D4);        // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
int sensorValue = 0;  // variable to store the value coming from the sensor
int buttonState = 0;  // variable for reading the pushbutton status
int SPEED_MAX_TEST = 600; // [-] Maximum speed

// Loop
unsigned long iTimeSend = 0;
int iTest = 0;

//Neopixel
String LED_PATTERN = "knightrider";
unsigned long lastUpdate = 0;
uint32_t old_val[NUM_PIXELS]; // up to 256 lights!
uint32_t color = 0xFF1000;
uint8_t width = 4;
uint16_t speed = 32;
boolean forward = true;
uint16_t count = 0;

//ESPUI
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;
const char *ssid = "HOVERCAR";
const char *password = "hovercar2020";
const char *hostname = "HOVERCAR";
uint16_t maxSpeedLabelId;
uint16_t ledPatternLabelId;
uint16_t ledColorLabelId;


typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

void handleSpeedUpdate(Control *sender, int value) {
  Serial.print("Select: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");
  Serial.println(sender->value);
  SPEED_MAX_TEST = sender->value.toInt();
  ESPUI.updateControlValue(maxSpeedLabelId, String(SPEED_MAX_TEST));
}

void handleLedPatternUpdate(Control *sender, int value) {
  Serial.print("Select: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");
  Serial.println(sender->value);
  count=0;
  forward = true;
  LED_PATTERN = sender->value;
  ESPUI.updateControlValue(ledPatternLabelId, LED_PATTERN);
}


void handleLedColorUpdate(Control *sender, int value) {
  Serial.print("Select: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");
  Serial.println(sender->value);
  if(sender->value == "red") {
    color = 0xFF1000;
  }
  else if(sender->value == "blue") {
    color = 0x0000FF;
  }
  else if(sender->value == "green") {
    color = 0x00FF00;
  }
  else if(sender->value == "white") {
    color = 0xFFFFFF;
  }
  else if(sender->value == "purple") {
    color = 0xFF00FF;
  }
  else if(sender->value == "yellow") {
    color = 0xFFFF00;
  }
  else if(sender->value == "cyan") {
    color = 0x00FFFF;
  }
  else {
    color = 0xFF1000;
  }
  ESPUI.updateControlValue(ledColorLabelId, sender->value);
}

void handleLedSpeedSlider(Control *sender, int type) {
  Serial.print("Slider: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");
  Serial.println(sender->value);
  speed = sender->value.toInt();
}

// ########################## SETUP ##########################
void setup() {
  ESPUI.setVerbosity(Verbosity::VerboseJSON);
  Serial.begin(SERIAL_BAUD);
  Serial.println(TITLE);
    
  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INVERT, INPUT_PULLDOWN_16);

  strip.begin();

  WiFi.hostname(hostname);
  Serial.print("\n\nCreating hotspot");

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);

  dnsServer.start(DNS_PORT, "*", apIP);

  Serial.println("\n\nWiFi parameters:");
  Serial.print("Mode: ");
  Serial.println(WiFi.getMode() == WIFI_AP ? "Station" : "Client");
  Serial.print("IP address: ");
  Serial.println(WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP());

  maxSpeedLabelId = ESPUI.addControl(ControlType::Label, "MaxSpeed:", String(SPEED_MAX_TEST), ControlColor::Emerald, Control::noParent);
  uint16_t speedSelect = ESPUI.addControl(ControlType::Select, "Select speed:", "", ControlColor::Emerald, Control::noParent, &handleSpeedUpdate);
  ESPUI.addControl(ControlType::Option, "Normal", "600", ControlColor::Emerald, speedSelect);
  ESPUI.addControl(ControlType::Option, "Slow", "300", ControlColor::Emerald, speedSelect);
  ESPUI.addControl(ControlType::Option, "Fast", "1000", ControlColor::Emerald, speedSelect);

  ledPatternLabelId = ESPUI.addControl(ControlType::Label, "LED Pattern:", "knightrider", ControlColor::Alizarin, Control::noParent);
  uint16_t ledPatternSelect = ESPUI.addControl(ControlType::Select, "Select LED pattern:", "", ControlColor::Alizarin, Control::noParent, &handleLedPatternUpdate);
  ESPUI.addControl(ControlType::Option, "Knight Rider", "knightrider", ControlColor::Alizarin, ledPatternSelect);
  ESPUI.addControl(ControlType::Option, "Running Lights", "runninglights", ControlColor::Alizarin, ledPatternSelect);
  ESPUI.addControl(ControlType::Option, "Fading Loop", "fadingloop", ControlColor::Alizarin, ledPatternSelect);

  ESPUI.addControl(ControlType::Slider, "LED speed (pause in ms)", "32", ControlColor::Wetasphalt, Control::noParent, &handleLedSpeedSlider);

  ledColorLabelId = ESPUI.addControl(ControlType::Label, "LED Color:", "red", ControlColor::Turquoise, Control::noParent);
  uint16_t ledColorSelect = ESPUI.addControl(ControlType::Select, "Select LED color:", "", ControlColor::Turquoise, Control::noParent, &handleLedColorUpdate);
  ESPUI.addControl(ControlType::Option, "Red", "red", ControlColor::Turquoise, ledColorSelect);
  ESPUI.addControl(ControlType::Option, "Blue", "blue", ControlColor::Turquoise, ledColorSelect);
  ESPUI.addControl(ControlType::Option, "Green", "green", ControlColor::Turquoise, ledColorSelect);
  ESPUI.addControl(ControlType::Option, "White", "white", ControlColor::Turquoise, ledColorSelect);
  ESPUI.addControl(ControlType::Option, "Purple", "purple", ControlColor::Turquoise, ledColorSelect);
  ESPUI.addControl(ControlType::Option, "Yellow", "yellow", ControlColor::Turquoise, ledColorSelect);
  ESPUI.addControl(ControlType::Option, "Cyan", "cyan", ControlColor::Turquoise, ledColorSelect);

  ESPUI.begin(TITLE);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive() {
  // Check for new data availability in the Serial buffer
  if (HoverSerial.available()) {
    incomingByte    = HoverSerial.read();                                 // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;   // Construct the start frame    
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
    Serial.print(incomingByte);
    return;
  #endif      
  
  // Copy received data
  if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
    p     = (byte *)&NewFeedback;
    *p++  = incomingBytePrev;
    *p++  = incomingByte;
    idx   = 2;  
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++  = incomingByte; 
    idx++;
  } 
  
  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {    
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
  
    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      
      // Print data to built-in Serial
      Serial.print("1: ");   Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }
  
  // Update previous states
  incomingBytePrev  = incomingByte;
}

uint32_t dimColor(uint32_t color, uint8_t width) {
   return (((color&0xFF0000)/width)&0xFF0000) + (((color&0x00FF00)/width)&0x00FF00) + (((color&0x0000FF)/width)&0x0000FF);
}

void updateKnightRider() {
    if(forward && count<NUM_PIXELS ) {
        strip.setPixelColor(count, color);
        old_val[count] = color;
        for(int x = count; x>0; x--) {
          old_val[x-1] = dimColor(old_val[x-1], width);
          strip.setPixelColor(x-1, old_val[x-1]); 
        }
        strip.show();
        count++;
        if(count == NUM_PIXELS) {
             forward = !forward;
        }
    } else if(!forward && count>=0) {
      strip.setPixelColor(count, color);
      old_val[count] = color;
      for(int x = count; x<=NUM_PIXELS ;x++) {
        old_val[x-1] = dimColor(old_val[x-1], width);
        strip.setPixelColor(x+1, old_val[x+1]);
      }
      strip.show();  
      count--;
      if(count == 0) {
        forward = !forward;
      }
   }
}

void updateRunningLights() {
  byte green = ((color>>(0*8)) & 0xff);
  byte red = ((color>>(1*8)) & 0xff);
  byte blue = ((color>>(2*8)) & 0xff);

  int Position=0;
 
  if(count<NUM_PIXELS*2 ) {
      Position++; // = 0; //Position + Rate;
      for(int i=0; i<NUM_PIXELS; i++) {
        strip.setPixelColor(i,
                   ((sin(i+Position) * 127 + 128)/255)*red,
                   ((sin(i+Position) * 127 + 128)/255)*green,
                   ((sin(i+Position) * 127 + 128)/255)*blue);
      }     
      strip.show();  
      count++;
      if(count == NUM_PIXELS*2) {
        count = 0;
      }
    }
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_PIXELS; i++ ) {
    strip.setPixelColor(i, red, green, blue);
  }
  strip.show();  
}

void updateRGBLoop() {
  float r, g, b;
  byte green = ((color>>(0*8)) & 0xff);
  byte red = ((color>>(1*8)) & 0xff);
  byte blue = ((color>>(2*8)) & 0xff);
  r = (count*4/256.0)*red;
  g = (count*4/256.0)*green;
  b = (count*4/256.0)*blue;
    if(forward && count<64 ) {
      setAll(r,g,b);
      strip.show();  
      count++;
      if(count == 64) {
         forward = !forward;
      }
    } else if(!forward && count>=0) {
      setAll(r,g,b);
      strip.show();  
      count--;
      if(count == 0) {
        forward = !forward;
      }
   }
}

void updateNeopixel() {
  unsigned long now = millis();
  if (now > lastUpdate+speed) {  
    if(LED_PATTERN == "knightrider") {
      updateKnightRider();
    }
    else if(LED_PATTERN == "runninglights")  {
      updateRunningLights();
    }
    else if(LED_PATTERN == "fadingloop")  {
      updateRGBLoop();
    }    
   lastUpdate = now; 
  }  
}

// ########################## LOOP ##########################


void loop(void) { 
  dnsServer.processNextRequest();
  
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;

  buttonState = digitalRead(INVERT);
  sensorValue = analogRead(THROTTLE);

#ifdef DEBUG_INPUT
  Serial.print("Button: ");
  Serial.print(buttonState);
  Serial.print(", Org: ");
  Serial.print(sensorValue);
#endif

  sensorValue = sensorValue - 50;
  if(sensorValue > 600) {
    sensorValue = SPEED_MAX_TEST;
  }
  if(buttonState == HIGH) {
    sensorValue = -sensorValue / 2;
  }
  
#ifdef DEBUG_INPUT
  Serial.print(", Mod: ");
  Serial.println(sensorValue);
#endif
  
  Send(0, sensorValue <  SPEED_MAX_TEST ? sensorValue : SPEED_MAX_TEST);

  updateNeopixel();

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}

// ########################## END ##########################
