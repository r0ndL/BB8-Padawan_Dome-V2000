/*
    ===================================================================================================
    ==================================== BB-8 PADAWAN DOME ============================================
    ======================================  Version 1.00  =============================================
    ===================================================================================================

    Designed to be used with a second Arduino running the BB-8 Padawan Body code

    Version History
    0.01 - Original Sketch by DavidScott
    0.02 - r0n_dL added annotations & variable for PIN_sound_BUSY as suggested by DavidScott
    0.03 - r0n_dL reassigned pins to align with Padawan control system
         - Changed SoftwareSerial pins (RX/TX) 10/9 to 8/4
    0.04 - Change pinMode for PIN_trigger to INPUT_PULLUP to resolve issue using with Pro Micro
    0.05 - Added support for the BY8301-16P and BY8001-16P sound modules
    0.06 - Using an updated version of MP3FLASH16P (MP3Flash16Pv2) which now removes static serial
           mappings out of the .cpp file
         - Add support to select between using 3 different sound modules by changing 1 variable
    1.00  - Major release - Added RX capabilities to receive commands from dedicated BB-8 Body Arduino

    r0n_dL IMPORTANT NOTE & RECOGNITON: This sketch was written by DavidScott. Thanks for a great base!

    Additionally:
    Much of this code is from various sources including...
    danf, CuriousMarc, Paul Murphy (JoyMonkey), John V, Michael Erwin,
    Michael Smith, Roger Moolay, Chris Reiff and Brad Oakley
    
    This program is free software: you can redistribute it and/or modify it .
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

    EasyTransfer and PS2X_lib libraries by Bill Porter
    (that's an Arduino Library - it needs to be downloaded and extracted into your
    Libraries folder for this sketch to work)


    // DavidScott's original comment below
    
    // This example plays a random file (001 to 010) forever
    // It uses the playFileAndWait() command so no extra code is needed, it will play another file as soon as 
    // the previous one finishes. If it doesn't work, try reversing the RX/TX wires as that's probably going to be
    // the most common mistake. Also check that the player's BUSY line is connected to Arduino pin 3
    // Finally check that the player has a speaker connected as well as ground and VCC

    r0n_dL NOTES:  Basic sketch provided by DavidScott to give sounds and synchronized voice lights for 
                      scale BB-8 Domes

    REQUIRED Libraries to make this sketch work:
    
    MP3Flash16Pv2 Library  
    https://github.com/r0ndL/MP3Flash16Pv2
    
    BY8x0116P Library
    https://github.com/r0ndL/BY8x0116Pv2

    
    IMPORTANT: By default MP3FLASH16P &  BY8x0116P library assumes using Software Serial (RX/TX) ports 10/11.
                  Change this pin assignments if using different pins in the .cpp file.  I'm using 8/4.

    r0n_dL PARTS USED
    
    1 Arduino Pro Micro
    1 MP3-FLASH-16P MP3 Soundboard
    1 Red 3mm Flanged LED (Radar Eye)
    1 Blue 3mm Flanged LED (Side Logic Display)*
    1 White 3mm Flanged LED (Top Front Logic Display)*
    1 White 5mm Flanged LED (PSI/Voice LED)
    1 5v power supply
    2 1K ohm Resistors*
    2 220 ohm Resistors*
    1 56 ohm Resistor*
    1 Switch - local or remote

    *There are actually MORE LEDs required if using BB-8 Club spec (as of 4 Jan 16 T&M beta v033) pg 42
     If using additional LEDs, more resistors and/or different value resistors will be required.

     May want to consider adding a LED to the HP as well which will also require changes to the resistor 
     & wiring

    Example:  
    2 Front(Top&Bottom) Logic Lenses have 5 round 3mm LEDs each - (5v to all (+) & 100ohm resistor on each (-) LED lead
    1 Side Logic Lens has 4 round 3mm LEDs - (5v to all (+) & 100ohm resistor on each (-) LED lead

    r0n_dL BASIC WIRING for testing

 Switch | 5v Power | Pro Micro | MP3-FLASH-16P | Speaker | LEDs
        |   POS    |  VCC      |     VCC       |         | + (3mm Red)- +(3mm Blue) 56 R
    1   |   NEG    |  GND      |     GND       |         | GND all LEDs
        |          |  3        |     BUSY      |         |    
        |          |  9        |               |         | + (3mm Wht 220 R)   
        |          |  8*(1K R) |      TX       |         |     
        |          |  4 (1K R) |      RX       |         |     
        |          |  6        |               |         | + (5mm Wht 220 R) 
        |          |  A2       |     SPK1      |   +     |      
        |          |           |     SPK2      |   -     |     
    2   |          |  A3       |               |         |
    
 NOTE: 3mm Red LED and 3mm Blue LED wired in series with 56 ohm Resistor
 * MP3 TX to Pin 8 not required.  Do not use pin 8 for anything else
 

FUTURE PINS:
 
PS2 Controller
  PS2ATT  10
  PS2CMD  16
  PS2DAT  14
  Ps2CLK  15
 
For Body Signal
  BODYRX RX1

RESERVED
  8 (Analog) - RX (SoftwareSerial)
  4 TX (SoftwareSerial)
 
FREE
  TX0
  5 (Digital)
  7 

Possibly used for WTV020SD-16P
  WTV1BSY A0
  WTV1CLK A1
  WTV1DAT 2

  
 SoftwareSerial Limitations:
 The library has the following known limitations:
  If using multiple software serial ports, only one can receive data at a time.
  
  Not all pins on the Mega and Mega 2560 support change interrupts, so only the following can be used for 
  RX: 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), 
  A14 (68), A15 (69).

  Not all pins on the Leonardo and Micro support change interrupts, so only the following can be used for
  RX: 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
*/

// Type of Arduino you are using
  // 1 = Arduino Pro Mini or Uno or Duemilanove (the cleanest option, but you need an FTDI adapter)
  // 2 = Sparkfun Pro Micro (but be ready to deal with USB and upload issues)
  // 3 = Arduino Micro 
  // 4 = Arduino Nano
#define BOARDtype 4

HardwareSerial* serialPort;
#define BAUDRATE 57600

// SET AUDIO OPTIONS...
// NOTE:  This sketch currently supports 3 different sound modules (MP3-FLASH-16P, BY8001-16P, BY8301-16P)
  
#define AUDIO1  1   // 1=MP3-FLASH-16P
                    // 2=BY8001-16P or BY8301-16P

#define number_of_sounds  32 //enter the # of audio files on sound module
                            

// INCLUDE LIBS AND DECLARE VARIABLES...
#include "SoftwareSerial.h"
SoftwareSerial MP3Serial(8, 4); // SoftwareSerial port assignments (RX, TX) to communicate with sound module

#if (AUDIO1==2)
  //settings for BY8001-16P or BY8301-16P module... 
  #include "BY8x0116Pv2.h"
  BY8x0116Pv2 myPlayer(MP3Serial); // Use SoftwareSerial as the serial port  
#else 
  //settings for MP3-FLASH-16P...
  #include "MP3Flash16Pv2.h"
  MP3Flash16Pv2 myPlayer(MP3Serial); // Use SoftwareSerial as the serial port
#endif


byte displayEffect; // 0 = normal, 1 = alarm, 2 = march, 3 = leia, 4 = failure

// ARDUINO PIN ASSIGNMENTS...
// Default Pin Assignments - BASIC WIRING in NOTES

#define PIN_sound_BUSY    3  //Connect to BUSY pin on sound module
#define PIN_pulse_LED     9  //Logic LED - 3mm White & 220 Ohm Resistor
#define PIN_voice_LED     6  //Voice LED - 5mm White & 220 Ohm Resistor
#define PIN_sound         A2 //Speaker lead for sound & LED sync
#define PIN_trigger       A3 //Connect to Switch
#define hpPin             10 //HP LED


// Set up the EasyTransfer communication
#include <EasyTransfer.h>
EasyTransfer ET;//create object
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int snd; // sound track number
  int sndSTOP; // mute sound command
  int sndRDM; // random sound command
  int sndVOL; // volume control
  int sndVOL2; // volume control 2
  int hpx; // hp movement
  int hpy; // hp movement
  int hpl; // hp light
  int hpa; // hp automation
  int dsp; // 100=no change, 0=random, 1=alarm, 4=whistle, 5=leia, 6=short circut, 10=EQ, 11=alarm2,
};         // 21=speed1, 22=speed2, 23=speed3, 24=logics+, 25=logics-
  
RECEIVE_DATA_STRUCTURE domeData;//give a name to the group of data

// Most builders shouldn't have to edit anything below here 
void setup() 
{
    MP3Serial.begin(9600);
    myPlayer.init(PIN_sound_BUSY);      // Init the player with the MP3 BUSY pin connected to Arduino pin defined
    pinMode(PIN_voice_LED, OUTPUT);
    pinMode(PIN_pulse_LED, OUTPUT); 
    pinMode(PIN_sound, INPUT);
    pinMode(PIN_trigger, INPUT_PULLUP); // Changed INPUT to INPUT_PULLUP because of issues working with Pro Micro.  
                                        // Change back to INPUT if required
    randomSeed(analogRead(0));
	

	pinMode(hpPin, OUTPUT); // Used for holo projector

// Since the serial names are different on different platforms, need to access through a universal pointer instead
// Sparkfun Pro Micro and Arduino Micro are based on Atmega32u4 chips, hardware serial is Serial1
#if (BOARDtype==2 || BOARDtype==3) 
  Serial1.begin(BAUDRATE);
  serialPort=&Serial1;
    ET.begin(details(domeData), &Serial1);
#else
  //Arduino Pro Mini is based on an Atmega328, hardware serial is Serial
  Serial.begin(BAUDRATE);
  serialPort=&Serial;
    ET.begin(details(domeData), &Serial);
#endif

}

// Integer Assignments for animated LEDs
int voiceBrightness;
int pulseBrightness;

void loop() {
    // Checking for trigger going LOW
    if(digitalRead(PIN_trigger) == LOW){
        if(!myPlayer.isBusy()){
            myPlayer.playFile(random(1, number_of_sounds+1));
            delay(100);
        }
    }
    
    // Voice light
    if(myPlayer.isBusy()){
        // Sets the brightness of the light based on the loudness of the voice
        voiceBrightness = constrain(map(analogRead(PIN_sound), 700, 1024, 0, 255), 0, 255);
        analogWrite(PIN_voice_LED, voiceBrightness);
    }else{
        // No voice playing, light is off
        analogWrite(PIN_voice_LED, 0);
    }
    
    // Slow pulsing light on the side of BB-8's head
    pulseBrightness = 170+(sin(millis()/400.00)*80);
    analogWrite(PIN_pulse_LED, pulseBrightness);

	if(ET.receiveData())
	{
		if (domeData.hpl==1)
		{
			digitalWrite(hpPin, HIGH);//turn on hp light
		}
		else
		{
			digitalWrite(hpPin, LOW);//turn off hp light
		}
		
		if(domeData.snd != 0){
            if(!myPlayer.isBusy()){
                myPlayer.playFile(domeData.snd);
                delay(100);
            }
        }
		
		if (domeData.sndSTOP==1){
				myPlayer.stopPlay();
			}
	 
	}//end ET.receiveData
	
}
