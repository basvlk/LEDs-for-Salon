
/**
 This is the contolr program for a series of 50 Diffuse NeoPixel LEDs, by Bas Vellekoop. A bit part of the program is about Serial communication
 It uses a lot of space for Global Variables - something I'll need to figure out. For now the solution is to usa an Arduino Mega

 TO DO:
 1. Change that control program doesn't send 3 bytes (RGB) but just one colorByte
 2. Look at global / local variables and memory usage

 **************************
 ***** SERIAL COMMUNICATION
 **************************
 * The main objectives are:
 * a fast way of receiving short commands, responding with low latency
 * also allowing longer messages to be sent, where latency is not a problem
 * Robust for messages being too short or too long, or invalid
 * ability to send error messages back
 * a Diagnostic on/off function, sending a lot more information back
 *
 * Principle:
 * (1) Header Bytes and start reading from Serial
 * - the first Byte of any message is '255'
 * - the second byte is 'Mode' which is used further down in the program for 'cases'
 * - the third Byte is 'DataLength" and says how many bytes will follow: this is the actual data. if '0' it means no further Bytes follow
 * - As a result, messages are always a minimum of THREE Bytes: 1:Begin ("255") 2:Mode 3:DataLength
 * - the program waits with reading in Bytes until there are a minimum of 3 Bytes in the buffer. Then it starts reading them.
 * - Once >3 Bytes are received if the first is a '255' it kicks off a reading cycle. If a different value is received, it is discarded
 * (2) Reading in the DataBytes
 * - if DataLength > 0, first a check is being done on whether the number of Bytes in the buffer is correct for the Datalength of the message:
 * -- If there are less Bytes in the buffer than expected, the program waits until (CommsTimeout) and if they haven't arrived, discards the Bytes
 * -- If there are too many Bytes in the buffer, the program doesn't trust the information, and discards all Bytes in the buffer.
 * - If the Bytes in the buffer correspond to the Datalength, the bytes can be read either into an array or into variables - but this is done within the 'cases' or Modes
 * (3) 2 kinds of Modes:
 * 0-99 are "OnceModes": they make the Arduino 'do' something once, like:
     * (a) Do things like switch all LEDs on and off (1=Off, 2=White, 3=Red. 4=Green, 5=Blue
     * (b) Mode 10-98 is to activate presets 10-98
     * (c) Mode 9 sets the variable preset StateX, based on incoming bytes
     * (d) Mode 99 is for changing the Diagnostic settings
 * 100-199  Preset modes: allow the loading of preset configurations of the entire LED string. They are slow to load but stored on the Arduino for quick retrieval
 * - Modes
 * - Mode 0 does Nothing. There's a reason for that:
 * - Mode 1-99 One-OFF modes, for example reading in a set of bytes to store in an Array, set 'Mode = 0' at the end of doing their job: this way the case for One-off modes runs only once
 * - Mode 100 - 199 CONTINUOUS  modes, where until the mode is changed, we want the program to keep executing this mode.
 **/

//LED SETUP
#include <Adafruit_NeoPixel.h>
#define PIN 7 //Pin that connects to the neopixels
const  int nLEDs = 50; // standard arraylength
Adafruit_NeoPixel strip = Adafruit_NeoPixel(nLEDs, PIN, NEO_RGB + NEO_KHZ800); // the Neopixel stick is "NEO_GRB", the 8mm LEDs are "NEO_RGB"

//PROGRAM CONTROL
const int ArduinoLedPin =  13  ;   // NOT the NeoPixel data pin!! the number of the Arduino LED pin - it's blinking helps seeing if the program runs
unsigned long previousMillis = 0;  // will store last time at the end of the previous loop
unsigned long currentMillis = 0;  // will store the current time
int BytesInBuffer = 0;             // number of unread Bytes waiting in the serial buffer
int DiscardedBytes = 0;            // number of Bytes discarded (read out of the serial buffer but not used) since the last start of a read operation. Indicates something is wrong
unsigned long CommsTimeout = 200;    // When the program is expecting X bytes to arrive, this is how long it will wait before discarding
unsigned long WaitedForBytes = 0;      //variable to time how long we've been waiting for the required bytes to arrive. To use to break out of a loop when Bytes don't arrive

//DIAGNOSTIC TOOLS
int Diagnostic = 0;                // switches on all kinds of diagnostic feedback from various locations in the program
int LooptimeDiag = 0;              // minimal feedback for checking efficiency: only feeds back looptime
int ArrayDiag = 0;                 // if switched on, prints all arrays every cycle
unsigned long Slowdown = 0;                  // Delay value (ms) added to each loop, only in 'Diagnostic' mode to allow inspecting the data coming back over serial
unsigned long msTable[8] = {0, 100, 200, 500, 1000, 1500, 2000, 5000}; //Delay values in ms to 'Slow Down' program for diagnostic purposes
int LoopIteration = 0;             // to track loop iterations

// SERIAL- required for the core functionality of the Serial communication
byte Mode = 0;
byte PrevContMode = 0; //Previous Continuous Mode and Once Mode are temporarily stored,
byte PrevOnceMode = 0; //If data turns out to be invalid, ContMode and OnceMode are restored to the previous version
byte ContMode = 0;
byte OnceMode = 0;
byte DataLength = 0;
int colorByte = 0;

// PRESETS - The following arrays form Presets for 10 LEDs: They are 10x3 long - patterns are repeated every 10 LEDs
byte STATEX[30] = {
  0, 10, 0, 0, 0, 10, 10, 10, 10, 10, 0, 0, 0, 10, 0, 0, 0, 10, 10, 10, 10, 10, 0, 0, 10, 0, 0, 10, 0, 0,
};
byte STATE10[nLEDs * 3] = {
  0, 48, 56, 43, 78, 0, 8, 74, 0, 0, 74, 25, 0, 85, 72, 0, 50, 86, 0, 11, 85, 31, 0, 83, 0, 48, 56, 43, 78, 0,
  0, 48, 56, 43, 78, 0, 8, 74, 0, 0, 74, 25, 0, 85, 72, 0, 50, 86, 0, 11, 85, 31, 0, 83, 0, 48, 56, 43, 78, 0,
  0, 48, 56, 43, 78, 0, 8, 74, 0, 0, 74, 25, 0, 85, 72, 0, 50, 86, 0, 11, 85, 31, 0, 83, 0, 48, 56, 43, 78, 0,
  0, 48, 56, 43, 78, 0, 8, 74, 0, 0, 74, 25, 0, 85, 72, 0, 50, 86, 0, 11, 85, 31, 0, 83, 0, 48, 56, 43, 78, 0,
  0, 48, 56, 43, 78, 0, 8, 74, 0, 0, 74, 25, 0, 85, 72, 0, 50, 86, 0, 11, 85, 31, 0, 83, 0, 48, 56, 43, 78, 0,
};
byte STATE11[nLEDs * 3] = {
  105, 0, 71, 0, 3, 101, 0, 23, 67, 0, 153, 205, 0, 153, 205, 0, 23, 67, 0, 3, 101, 105, 0, 71, 105, 0, 71, 0, 3, 101,
  105, 0, 71, 0, 3, 101, 0, 23, 67, 0, 153, 205, 0, 153, 205, 0, 23, 67, 0, 3, 101, 105, 0, 71, 105, 0, 71, 0, 3, 101,
  105, 0, 71, 0, 3, 101, 0, 23, 67, 0, 153, 205, 0, 153, 205, 0, 23, 67, 0, 3, 101, 105, 0, 71, 105, 0, 71, 0, 3, 101,
  105, 0, 71, 0, 3, 101, 0, 23, 67, 0, 153, 205, 0, 153, 205, 0, 23, 67, 0, 3, 101, 105, 0, 71, 105, 0, 71, 0, 3, 101,
  105, 0, 71, 0, 3, 101, 0, 23, 67, 0, 153, 205, 0, 153, 205, 0, 23, 67, 0, 3, 101, 105, 0, 71, 105, 0, 71, 0, 3, 101,
};
byte STATE12[nLEDs * 3] = {
  86, 29, 0, 138, 120, 0, 0, 43, 8, 0, 99, 34, 35, 91, 0, 47, 59, 0, 48, 75, 0, 28, 13, 0,
};
byte STATE13[nLEDs * 3] = {
  92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102,
};
byte STATE14[nLEDs * 3] = {
  23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0,
};
byte STATE15[nLEDs * 3] = {
  96, 0, 2, 101, 0, 42, 80, 0, 105, 0, 3, 94, 0, 80, 91, 0, 86, 25, 39, 96, 0, 86, 87, 0,
};
byte STATE16[nLEDs * 3] = {
  0, 10, 0, 0, 0, 10, 10, 10, 10, 10, 0, 0, 0, 10, 0, 0, 0, 10, 10, 10, 10, 10, 0, 0,
};
byte STATE17[nLEDs * 3] = {
  0, 48, 56, 43, 78, 0, 8, 74, 0, 0, 74, 25, 0, 85, 72, 0, 50, 86, 0, 11, 85, 31, 0, 83,
};
byte STATE18[nLEDs * 3] = {
  105, 0, 71, 0, 3, 101, 0, 23, 67, 0, 153, 205, 0, 153, 205, 0, 23, 67, 0, 3, 101, 105, 0, 71,
};
byte STATE19[nLEDs * 3] = {
  86, 29, 0, 138, 120, 0, 0, 43, 8, 0, 99, 34, 35, 91, 0, 47, 59, 0, 48, 75, 0, 28, 13, 0,
};
byte STATE20[nLEDs * 3] = {
  0, 48, 56, 43, 78, 0, 8, 74, 0, 0, 74, 25, 0, 85, 72, 0, 50, 86, 0, 11, 85, 31, 0, 83,
};
byte STATE21[nLEDs * 3] = {
  105, 0, 71, 0, 3, 101, 0, 23, 67, 0, 153, 205, 0, 153, 205, 0, 23, 67, 0, 3, 101, 105, 0, 71,
};
byte STATE22[nLEDs * 3] = {
  86, 29, 0, 138, 120, 0, 0, 43, 8, 0, 99, 34, 35, 91, 0, 47, 59, 0, 48, 75, 0, 28, 13, 0,
};
byte STATE23[nLEDs * 3] = {
  92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102,
};
byte STATE24[nLEDs * 3] = {
  23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0,
};
byte STATE25[nLEDs * 3] = {
  96, 0, 2, 101, 0, 42, 80, 0, 105, 0, 3, 94, 0, 80, 91, 0, 86, 25, 39, 96, 0, 86, 87, 0,
};
byte STATE26[nLEDs * 3] = {
  0, 10, 0, 0, 0, 10, 10, 10, 10, 10, 0, 0, 0, 10, 0, 0, 0, 10, 10, 10, 10, 10, 0, 0,
};
byte STATE27[nLEDs * 3] = {
  0, 48, 56, 43, 78, 0, 8, 74, 0, 0, 74, 25, 0, 85, 72, 0, 50, 86, 0, 11, 85, 31, 0, 83,
};
byte STATE28[nLEDs * 3] = {
  105, 0, 71, 0, 3, 101, 0, 23, 67, 0, 153, 205, 0, 153, 205, 0, 23, 67, 0, 3, 101, 105, 0, 71,
};
byte STATE29[nLEDs * 3] = {
  86, 29, 0, 138, 120, 0, 0, 43, 8, 0, 99, 34, 35, 91, 0, 47, 59, 0, 48, 75, 0, 28, 13, 0,
};
byte STATE30[nLEDs * 3] = {
  92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102, 92, 117, 0, 23, 0, 102,
};

void setup() {
  Serial.begin(9600);
  pinMode(ArduinoLedPin, OUTPUT);
  strip.begin();
  strip.show();
  for (int i = 0; i < nLEDs; i++) {
    strip.setPixelColor(i, strip.Color(100, 0, 0));
    delay(50);
    strip.show();              // Refresh LED states
  }
  for (int i = 0; i < nLEDs; i++) {
    strip.setPixelColor(i, strip.Color(0, 100, 0));
    delay(50);
    strip.show();              // Refresh LED states
  }
  for (int i = 0; i < nLEDs; i++) {
    strip.setPixelColor(i, 0);
    delay(50);
    strip.show();              // Refresh LED states
  }

}

//**********************************************************
//*************         M A I N       **********************
//**********************************************************

void loop()
{
  ++LoopIteration;
  LoopBlink(LoopIteration);
  BytesInBuffer = Serial.available();
  Serial.print("]");
  Serial.println(BytesInBuffer);
  if (BytesInBuffer == 0) {
    DiscardedBytes = 0;
  }
  if (Diagnostic == 1) {                  //Diag
    delay(Slowdown);                      //Diag
    Serial.print(F("[ **** NEW LOOP: "));    //Diag
    Serial.println(LoopIteration);        //Diag
    Serial.print(F("[ currentMillis: "));    //Diag
    Serial.println(currentMillis);        //Diag
    Serial.print(F("[ Slowdown: "));         //Diag
    Serial.println(Slowdown);             //Diag
  }                                       //Diag

  // Start when data arrived
  if (BytesInBuffer > 2) // all messages are minimum 3 Bytes so are waiting for 3 before getting going.
  {
    if (Serial.read() == 255)

      //*************        R E A D       **********************

    { // SECTION 1: MODE and LENGTH
      if (Diagnostic == 1) {
        Serial.println(F("Entered reading section, ergo >=3 Bytes in buffer, first is 255"));
      }
      PrevContMode = ContMode; //before reading in the mode, backing up the previous modes
      PrevOnceMode = OnceMode;

      Mode = Serial.read();
      DataLength = Serial.read();
      BytesInBuffer = Serial.available();

      if (Mode > 99) {
        ContMode = Mode;
      }
      if (Mode < 100) {
        OnceMode = Mode;
      }
      
      if (Diagnostic == 1) {                     //Diag
        Serial.print(F("[ Mode: "));             //Diag
        Serial.print(Mode);                      //Diag
        Serial.print(F(" // New ContMode: "));   //Diag
        Serial.print(ContMode);                  //Diag
        Serial.print(F(" // New OnceMode: "));   //Diag
        Serial.println(OnceMode);                //Diag
        Serial.print(F("[ DataLength: "));       //Diag
        Serial.print(DataLength);                //Diag
        Serial.print(F(" - Bytes in Buffer: ")); //Diag
        Serial.println(BytesInBuffer);           //Diag
      }                                          //Diag

      //NOT ENOUGH BYTES
      if (BytesInBuffer < DataLength) {
        if (Diagnostic == 1) {
          Serial.println("[ Entering 'NOT ENOUGH BYTES");
        }
        WaitedForBytes = 0;
        unsigned long StartMillis = millis();

        while ( (BytesInBuffer < DataLength) && (WaitedForBytes < CommsTimeout )) {
          BytesInBuffer = Serial.available();
          
          if (Diagnostic == 1) {                      //Diag
            Serial.print(F("[ DataLength: "));        //Diag
            Serial.print(DataLength);                 //Diag
            Serial.print(F("=> BytesInBuffer: "));    //Diag
            Serial.println (BytesInBuffer);           //Diag
            Serial.print(F("[ CommsTimeout: "));      //Diag
            Serial.print(CommsTimeout);               //Diag
            Serial.print(F("=> WaitedForBytes: "));  //Diag
            Serial.println(WaitedForBytes);
          }
          WaitedForBytes = (millis() - StartMillis);
        }
        /// End of while loop. Now there are 2 options: either the bytes arrived, or they didn't and the thing timed out
        if (BytesInBuffer == DataLength) {
          if (Diagnostic == 1) {                                 //Diag
            Serial.print(F("[ Bytes arrived after waiting for ")); //Diag
            Serial.print(WaitedForBytes);                        //Diag
            Serial.println(F("ms"));                                //Diag
          }                                                      //Diag
        }

        if (BytesInBuffer < DataLength) {
          Serial.print("[ ERROR: Missing ");
          Serial.print(DataLength - BytesInBuffer);
          Serial.print("Bytes in buffer, Waited ");
          Serial.print(WaitedForBytes);
          Serial.println("ms, Aborting read operation, dumping data");
          char dump[BytesInBuffer];
          Serial.readBytes(dump, BytesInBuffer);
          ContMode = PrevContMode; //restoring the previous modes so operation continues unchanged despite the invalid data
          OnceMode = PrevOnceMode ;
        }

      } // End of 'not enough Bytes'

      //TOO MANY BYTES
      if (BytesInBuffer > DataLength) {
        if (Diagnostic == 1) {                               //Diag
          Serial.println("[ Entering 'TOO MANY BYTES");      //Diag
        }                                                    //Diag
        Serial.print("[ ERROR: ");                          
        Serial.print(BytesInBuffer - DataLength);
        Serial.println(" too many Bytes in buffer. Dumping all data, not doing anything");
        char dump[BytesInBuffer];
        Serial.readBytes(dump, BytesInBuffer);
        ContMode = PrevContMode; //restoring the previous modes so operation continues unchanged despite the invalid data
        OnceMode = PrevOnceMode ;
      }

    }
    else // IF 3 bytes or more AND the first is not '255'; cycle back to start without doing anything with the read Byte: 
        // effectively this just removes the first Byte, and leaves the remaining in the buffer
    {
      ++DiscardedBytes;
      Serial.print("[ ERROR: Bytes received not starting with '255' Discarded: ");
      Serial.println(DiscardedBytes);
    } //End invalid data section (ie data did not start with '255' and is non-255 byte is discarded. The rest is left intact in case it is the start of a valid message)
  } // End reading / discarding data section which only runs when there are 3 Bytes or ore in the buffer



  ////// BRIDGE section, between validating the incoming data, and reading in the data (if valid)
  if (Diagnostic == 1) {                    //Diag
    Serial.print("[ Last read Mode: ");     //Diag
    Serial.print(Mode);                     //Diag
    Serial.print(" // Current ContMode: "); //Diag
    Serial.print(ContMode);                 //Diag
    Serial.print(" // Current OnceMode: "); //Diag
    Serial.println(OnceMode);               //Diag
  }


  //*************       O N C E  M O D E S      **********************
  // this section represents modes that run once only - a state change, or a download of settings
  // MODE 1-9 are special modes (1= all Off, 2=StateX etc)
  // MODE 10 -xx are Presets, stored in Arduino, or overwritten by Host


  switch (OnceMode) {
    case 1: //All Off
      {
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
        }
        strip.show();
        OnceMode = 0;      // Refresh LED states
        break;
      }
    case 2: //All RED
      {
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(255, 255, 255)); // Erase pixel, but don't refresh!
        }
        strip.show();
        OnceMode = 0;      // Refresh LED states
        break;
      }
    case 3: //All RED
      {
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(255, 0, 0)); // Erase pixel, but don't refresh!
        }
        strip.show();
        OnceMode = 0;      // Refresh LED states
        break;
      }
    case 4: //All GREEN
      {
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(0, 255, 0)); // Erase pixel, but don't refresh!
        }
        strip.show();
        OnceMode = 0;      // Refresh LED states
        break;
      }
    case 5: //All RED
      {
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(0, 0, 255)); // Erase pixel, but don't refresh!
        }
        strip.show();
        OnceMode = 0;      // Refresh LED states
        break;
      }
    case 9:
      {
        Serial.readBytes((char *)STATEX, DataLength);
        ArrayToPixels(STATEX, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 10:
      {
        Serial.readBytes((char *)STATE10, DataLength);
        ArrayToPixels(STATE10, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 11:
      {
        Serial.readBytes((char *)STATE11, DataLength);
        ArrayToPixels(STATE11, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 12:
      {
        Serial.readBytes((char *)STATE12, DataLength);
        ArrayToPixels(STATE12, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 13:
      {
        Serial.readBytes((char *)STATE13, DataLength);
        ArrayToPixels(STATE13, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 14:
      {
        Serial.readBytes((char *)STATE14, DataLength);
        ArrayToPixels(STATE14, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 15:
      {
        Serial.readBytes((char *)STATE15, DataLength);
        ArrayToPixels(STATE15, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 20:
      {
        ArrayToPixels(STATE10, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 21:
      {
        ArrayToPixels(STATE11, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 22:
      {
        ArrayToPixels(STATE12, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 23:
      {
        ArrayToPixels(STATE13, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 24:
      {
        ArrayToPixels(STATE14, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 25:
      {
        ArrayToPixels(STATE15, nLEDs * 3);
        OnceMode = 0;
        break;
      }
    case 99: //Mode 99 is CONFIG. Bytes set: Diagnostic, Delay
      {
        Diagnostic = Serial.read();
        int i = Serial.read();
        Slowdown = msTable[i];
        LooptimeDiag = Serial.read();
        ArrayDiag = Serial.read();
        CommsTimeout = msTable[Serial.read()];
        if (Diagnostic == 1) {
          Serial.print("[ Diagnostic set to: ");
          Serial.println(Diagnostic);
          Serial.print("[ Slowdown set to: ");
          Serial.println(Slowdown);
          Serial.print("[ CommsTimeout set to: ");
          Serial.println(CommsTimeout);
        }
        OnceMode = 0;
        break;
      }

    default:
      if (Diagnostic == 1) {
        Serial.print("[ Once Mode ");
        Serial.print(OnceMode);
        Serial.println(" not yet implemented");
      }
  }

  //*************       C O N T I N U O U S  M O D E S      **********************
  // this section represents modes that run continuously, once with every main loop cycle- to be used for time-base effects

  switch (ContMode)

  { //Start MODES Section
    case 0:
      {
        if (Diagnostic == 1) {
          Serial.println("[ Continuous Mode 0 - not doing anything");
        }
        break;
      }

    default:
      if (Diagnostic == 1) {
        Serial.print("[ Cont Mode ");
        Serial.print(ContMode);
        Serial.println(" not yet implemented");
      }
  }  // END Cont Modes Section



  //*************       L A S T  P A R T  O F  M A I N  P R O G R A M      **********************

  currentMillis = millis();
  if (LooptimeDiag == 1) {
    Serial.print("[ Looptime: ");
    Serial.println(currentMillis - previousMillis);
  }
  if (ArrayDiag == 1)
  {
    ArrayToSerial(STATEX, nLEDs * 3);
    ArrayToSerial(STATE10, nLEDs * 3);
    ArrayToSerial(STATE11, nLEDs * 3);
    ArrayToSerial(STATE12, nLEDs * 3);
    ArrayToSerial(STATE13, nLEDs * 3);
    ArrayToSerial(STATE14, nLEDs * 3);
    ArrayToSerial(STATE15, nLEDs * 3);
  }
  if (Diagnostic == 1) {
    Serial.print("[ **** END OF LOOP ");
    Serial.print(LoopIteration);
    Serial.print(",   Looptime: ");
    Serial.println(currentMillis - previousMillis);
  }
  previousMillis = currentMillis;
} //End main loop


//*************       F U N C T I O N S      **********************

// Array to NeoPixels
void ArrayToPixels(byte Array[], int N) {
  for (int i = 0; i < N / 3; i++)
  {
    int pix = i;
    int r = Array[i * 3];
    int g = Array[i * 3 + 1];
    int b = Array[i * 3 + 2];
    strip.setPixelColor(pix, strip.Color(r, g, b)); // Set new pixel 'on'
    strip.show();              // Refresh LED states
  }
}

// PrintArrayToSerial
void ArrayToSerial(byte Array[], int N) {
  for (int i = 0; i < N ; i++)
  {
    Serial.print(" ");
    Serial.print(Array[i], DEC);
    Serial.print(" ");
  }
  Serial.println("");
}

// Blink ArduinoLED to show program is running. Toggles On/Off every loop
void LoopBlink(int Loop)
{
  if (LoopIteration % 2)
  {
    digitalWrite(ArduinoLedPin, HIGH);
  }
  else
  {
    digitalWrite(ArduinoLedPin, LOW);
  }
}
// End blinkLed


































