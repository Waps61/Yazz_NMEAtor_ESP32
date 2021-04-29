#include <Arduino.h>
/*
  Project:  Yazz_NMEAtor_ESP32.cpp, Copyright 2020, Roy Wassili
  Contact:  waps61 @gmail.com
  URL:      https://www.hackster.io/waps61
  VERSION:  0.12
  Date:     10-10-2020
  Last
  Update:   29-04-2021
            Beta version with display communication to Nextion NX4832K035
            Not all parameters are show yet
            26-04-2021
            Preparation for display implementation, compiled version and runnable
            26-04-2021 V0.10
            First runnable MVP reading NMEA data,converting and sending NMEA0183 data
            18-04-2021 V0.02
            Fixed a bug in tthe Depth calculation
            10-10-2020 V0.01
            Port to ESP32 from Arduino V1.04
            
  Achieved: 
            
  Purpose:  Build an NMEA0183 manupulator and animator for on board of my sailing boat
            supporting following types of tasks:
            - Reading NMEA0183 v1.5 data without a checksum on 4800 Bd,
            - Convert V1.5 to V2.0 with a checksum
            - Converting Robertson specific data to valid NMEA0183 v2.0 data
            - Sending NMEA0183 V2.0 data on 38400 Bd

  NOTE:     NMEA encoding conventions in short
            An NMEA sentence consists of a start delimiter, followed by a comma-separated sequence
            of fields, followed by the character '*' (ASCII 42), the checksum and an end-of-line marker.
            i.e. <start delimiter><field 0>,<field 1>,,,<field n>*<checksum><end-of-linemarker>
            The start delimiter is either $ or !. <field 0> contains the tag and the remaining fields
            the values. The tag is normaly a 5 character wide identifier where the 1st 2 characters
            identify the talker ID and the last 3 identify the sentence ID.
            Maximum sentence length, including the $ and <CR><LF> is 82 bytes.

  Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_nmea_0183_physical_protocol_layer


  
 
  Serial1 Rx1 (GPIO 18) and Tx1 (GPIO 19) are reserved for the NMEA listener on 4800Bd
  Serial2 Rx2 (GPIO 16) and Tx2 (GPIO17) are reserved for communicating with the Nextion
  GPIO 22 (and 23) are reserved for NMEA talker via SoftSerial on 38400 Bd
  
  Hardware setup:

  Wiring Diagram (for RS-232 <-> TTL shifter)
  The RS-232 TX+ wire runs through the TTL shifter
  The GND is directly plugged into GND of the ESP32
  This is used in my case.
  RS-232  | TTL shifter | ESP32
          | RS232 | TTL |
    TX+   | +     | +   | GPIO 18
    GND   |       |     | GND

Wiring Diagram (for RS-232 to NMEA0183 device)
  ESP32     | NMEA device
     Pin 23 |  RX +   
     GND    |  RX - 
            |  GND (if isolated input available)

Wiring Diagram (for ESP32 to Nextion display)
  ESP32     | Nextion
     Pin 16 |  TX +   
     Pin 17 |  TX +   
     GND    |  GND 
      3,3V  |   5V


---------------
Terms of use:
---------------
The software is provided "AS IS", without any warranty of any kind, express or implied,
including but not limited to the warranties of mechantability, fitness for a particular
purpose and noninfringement. In no event shall the authors or copyright holders be liable
for any claim, damages or other liability, whether in an action of contract, tort or
otherwise, arising from, out of or in connection with the software or the use or other
dealings in the software.

-----------
Warning:
-----------
Do NOT use this compass in situations involving safety to life
such as navigation at sea.  
        
TODO: 

Credit:   
*/

/*
    Include the necessary libraries
*/
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>
//*** Since the signal from the RS422-TTL converter is inverted
//*** a digital input is used as a software serial port because
//*** it can invert te signal back to its orignal pulse set
#include <SoftwareSerial.h>
#include <Nextion.h> //All other Nextion classes come with this libray

/*
   Definitions go here
*/
// *** Conditional Debug & Test Info to Serial Monitor
// *** by commenting out the line(s) below the debugger and or test statements will
// *** be ommitted from the code
#define DEBUG 1
//#define TEST 1
//#define DISPLAY_ATTACHED 1
#define NEXTION_ATTACHED 1 //out comment if no display available

#define VESSEL_NAME "YAZZ"
#define PROGRAM_NAME "NMEAtor ESP32"
#define PROGRAM_VERSION "0.02"

#define SAMPLERATE 115200

#define LISTENER_RATE 4800 // Baudrate for the listner
#define LISTENER_RX 18     // Serial1 Rx port
#define LISTENER_TX 19     // Serial1 TX port
#define TALKER_RATE 38400  // Baudrate for the talker
#define TALKER_PORT 23     // SoftSerial port 2

#define NMEA_RX 22
#define NMEA_TX 23
#define NEXTION_RX (int8_t)16
#define NEXTION_TX (int8_t)17
#define NEXTION_RCV_DELAY 100
#define NEXTION_SND_DELAY 50
//*** Some conversion factors
#define FTM 0.3048    // feet to meters
#define MTF 3.28084   // meters to feet
#define NTK 1.852     // nautical mile to km
#define KTN 0.5399569 // km to nautical mile

//*** The NMEA defines in totl 82 characters including the starting
//*** characters $ or ! and the checksum character *, the checksum
//*** AND last but not least the <CR><LF> chacters.
//*** we define one more for the terminating '\0' character for char buffers
#define NMEA_BUFFER_SIZE 82 // According NEA0183 specs the max char is 82
#define NMEA_TERMINATOR "\r\n"

//*** The maximum number of fields in an NMEA string
//*** The number is based on the largest sentence MDA,
//***  the Meteorological Composite sentence
#define MAX_NMEA_FIELDS 21

#define STACKSIZE 10 // Size of the stack; adjust according use

#define TALKER_ID "EP"
#define VARIATION "1.57,E" //Varition in Lemmer on 12-05-2020, change 0.11 per year
//*** On my boat there is an ofsett of 0.2V between the battery monitor and what
//*** is measured by the Robertson Databox
#define BATTERY_OFFSET 0.2 //Volts
//*** define NMEA tags to be used
//*** make sure you know your Talker ID used in the sentences
//*** In my case next to GP for navigation related sentences
//*** II is used for Integrated Instruments and
//*** PS is used for vendor specific tags like Stowe Marine
//*** EP is used for my ESP32 generated sentences

/* for lab testing with an NMEA simulator tool
#define _DBK "$SDDBK"   // Depth below keel
#define _DBS "$SDDBS"   // Depth below surface
#define _DBT "$SDDBT"   // Depth below transducer
*/
#define _DBK "$IIDBK" // Depth below keel
#define _DBS "$IIDBS" // Depth below surface
#define _DBT "$IIDBT" // Depth below transducer
#define _HDG "$IIHDG" // Heading  Deviation & Variation
#define _HDM "$IIHDM" // Heading Magnetic
#define _HDT "$IIHDT" // Heading True
#define _MWD "$IIMWD" // Wind Direction & Speed
#define _MTW "$IIMTW" // Water Temperature
/* for lab testing with an NMEA simulator tool
#define _MWV "$WIMWV"  // Wind Speed and Angle
*/
#define _MWV "$IIMWV" // Wind Speed and Angle
#define _ROT "$IIROT" // Rate of Turn
#define _RPM "$IIRPM" // Revolutions
#define _RSA "$IIRSA" // Rudder sensor angle
#define _VDR "$IIVDR" // Set and Drift
#define _VHW "$IIVHW" // Water Speed and Heading
#define _VLW "$IIVLW" //  Distance Traveled through Water
#define _VTG "$IIVTG" //  Track Made Good and Ground Speed
#define _VWR "$IIVWR" //  Relative Wind Speed and Angle
#define _XDR "$IIXDR" //  Cross Track Error  Dead Reckoning
#define _XTE "$IIXTE" //  Cross-Track Error  Measured
#define _XTR "$IIXTR" //  Cross Track Error  Dead Reckoning
#define _ZDA "$IIZDA" //  Time & Date - UTC, day, month, year and local time zone
//*** Some specific GPS sentences
#define _GLL "$GPGLL" // Geographic Position  Latitude/Longitude
#define _GGA "$GPGGA" // GPS Fix Data. Time, Position and fix related data for a GPS receiver
#define _GSA "$GPGSA" // GPS DOP and active satellites
#define _GSV "$GPGSV" // Satellites in view
#define _RMA "$GPRMA" // Recommended Minimum Navigation Information
#define _RMB "$GPRMB" // Recommended Minimum Navigation Information
#define _RMC "$GPRMC" // Recommended Minimum Navigation Information

//*** Some specific Robertson / Stowe Marine tags below
#define _TON "$PSTON" // Distance Nautical since reset
#define _TOE "$PSTOE" // Engine hours
#define _TOB "$PSTOB" // Battery voltage
#define _TOD "$PSTOD" // depth transducer below waterline in feet
//*** Arduino generated TAGS
#define _xDR "$" TALKER_ID "" \
             "XDR" // Arduino Transducer measurement
#define _dPT "$" TALKER_ID "" \
             "DPT" // Arduino Transducer measurement
#define _hDG "$" TALKER_ID "" \
             "HDG" // Arduino Transducer measurement
/* SPECIAL NOTE:
  XDR - Transducer Measurement
        1 2   3 4            n
        | |   | |            |
  $--XDR,a,x.x,a,c--c, ..... *hh<CR><LF>
  Field Number:   1:Transducer Type
                2:Measurement Data
                3:Units of measurement
                4:Name of transducer

  There may be any number of quadruplets like this, each describing a sensor. The last field will be a checksum as usual.
  Example:
  $HCXDR,A,171,D,PITCH,A,-37,D,ROLL,G,367,,MAGX,G,2420,,MAGY,G,-8984,,MAGZ*41
*/

/*
   If there is some special treatment needed for some NMEA sentences then
   add the their definitions to the NMEA_SPECIALTY definition
   The pre-compiler concatenates string literals by using "" in between
*/
#define NMEA_SPECIALTY "" _DBK "" _TOB

//*** define the oject tags of the Nextion display
#define WINDDISPLAY_STATUS "status"
#define WINDDISPLAY_STATUS_VALUE "winddisplay.status.val"
#define WINDDISPLAY_NMEA "nmea"
#define FIELD_BUFFER 10 //nr of char used for displaying info on Nextion

//*** A structure to hold the NMEA data
typedef struct
{
  String fields[MAX_NMEA_FIELDS];
  byte nrOfFields = 0;
  String sentence = "";

} NMEAData;

char nmeaBuffer[NMEA_BUFFER_SIZE + 1] = {0};
char nb_AWA[FIELD_BUFFER] = {0};
char nb_COG[FIELD_BUFFER] = {0};
char nb_SOG[FIELD_BUFFER] = {0};
char nb_AWS[FIELD_BUFFER] = {0};
char nb_BAT[FIELD_BUFFER] = {0};
char nb_DPT[FIELD_BUFFER] = {0};
char nb_DIR[FIELD_BUFFER] = {0};
char nb_TWS[FIELD_BUFFER] = {0};
char nb_STW[FIELD_BUFFER] = {0};
char nb_HDG[FIELD_BUFFER] = {0};
char nb_LOG[FIELD_BUFFER] = {0};
char nb_WTR[FIELD_BUFFER] = {0};
char nb_TRP[FIELD_BUFFER] = {0};
char oldVal[255] = {0}; // holds previos _BITVALUE to check if we need to send

enum NMEAReceiveStatus
{
  INVALID,
  VALID,
  RECEIVING,
  CHECKSUMMING,
  TERMINATING,
  NMEA_READY
};
byte nmeaStatus = INVALID;
byte nmeaIndex = 0;
bool nmeaDataReady = false;
bool newData = false;
unsigned long tmr1 = 0;

//*** Global scope variable declaration goes here
NexPicture dispStatus = NexPicture(1, 35, WINDDISPLAY_STATUS);
NexText nmeaTxt = NexText(1, 16, WINDDISPLAY_NMEA);
NexText versionTxt = NexText(0, 3, "version");

SoftwareSerial nmeaSerialOut; // // signal need to be inverted for RS-232

#define WHITE 0xFFFF /* 255, 255, 255 */
#define BLACK 0x0000 /*   0,   0,   0 */
int16_t current_color;

#ifdef DISPLAY_ATTACHED
/*
TFT screen specific definitions go here
*/
#define YP A3 // must be an analog pin, use "An" notation!
#define XM A2 // must be an analog pin, use "An" notation!
#define YM 9  // can be a digital pin
#define XP 8  // can be a digital pin

//param calibration from kbv
/*#define TS_MINX 298
#define TS_MAXX 814

#define TS_MINY 114 
#define TS_MAXY 867
*/
#define TS_MINX 116
#define TS_MAXX 906

#define TS_MINY 92
#define TS_MAXY 952

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
//touch sensitivity for press
#define MINPRESSURE 10
#define MAXPRESSURE 1000

#define BLUE 0x001F        /*   0,   0, 255 */
#define RED 0xF800         /* 255,   0,   0 */
#define GREEN 0x07E0       /*   0, 255,   0 */
#define CYAN 0x07FF        /*   0, 255, 255 */
#define MAGENTA 0xF81F     /* 255,   0, 255 */
#define YELLOW 0xFFE0      /* 255, 255,   0 */
#define NAVY 0x000F        /*   0,   0, 128 */
#define DARKGREEN 0x03E0   /*   0, 128,   0 */
#define DARKCYAN 0x03EF    /*   0, 128, 128 */
#define MAROON 0x7800      /* 128,   0,   0 */
#define PURPLE 0x780F      /* 128,   0, 128 */
#define OLIVE 0x7BE0       /* 128, 128,   0 */
#define LIGHTGREY 0xC618   /* 192, 192, 192 */
#define DARKGREY 0x7BEF    /* 128, 128, 128 */
#define ORANGE 0xFD20      /* 255, 165,   0 */
#define GREENYELLOW 0xAFE5 /* 173, 255,  47 */
#define LOG_COLOR 0xFD20

#define BUTTON_H 60  //button height
#define BUTTON_W 110 //button wodth
#define BUTTON_X 5   // x position of button column
#define BUTTON_Y 260 // y position of button column

//if the IC model is known or the modules is unreadable,you can use this constructed function
LCDWIKI_KBV my_lcd(ILI9486, A3, A2, A1, A0, A4); //model,cs,cd,wr,rd,reset
//if the IC model is not known and the modules is readable,you can use this constructed function
//LCDWIKI_KBV my_lcd(320,480,A3,A2,A1,A0,A4);//width,height,cs,cd,wr,rd,reset

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

int16_t flag_colour;
boolean show_flag = true;
int16_t screen_row = 0;

//*** Define the units of measurement in a string pointer array
//*** and use an enum to get the index for the specific label
//*** DEG is used for degrees and DEG< and DEG> for left and right indicators
const char *screen_units[] = {"Kts", "NM", "DEG", "DEG<", "DEG>", "M", "C", "V"};
enum units
{
  SPEED,
  DIST,
  DEG,
  DEGR,
  DEGL,
  MTRS,
  TEMP,
  VOLT
};
//*** the screen is divided in 4 quadrants
//***   Q1      Q2
//***   Q3      Q4
enum screen_quadrant
{
  Q1,
  Q2,
  Q3,
  Q4
};

//*** a structure to hold the button info
typedef struct
{
  char button_name[10];
  uint8_t button_name_size; // the text size i.e. 1,2,...,n based on a 5x8 char
  uint16_t button_name_colour;
  uint16_t button_colour;
  uint16_t button_x;
  uint16_t button_y;
} button_info;

//*** define the buttons used with an enumerated tag
enum menu_buttons
{
  SPD,
  CRS,
  LOG,
  MEM
};
uint8_t active_menu_button = SPD; //holds the active menu button pressed
//*** the definition of buttons menu
button_info menu_button[4] =
    {
        "Speed",
        3,
        BLACK,
        LIGHTGREY,
        BUTTON_X,
        BUTTON_Y,
        "Crs",
        3,
        BLACK,
        LIGHTGREY,
        BUTTON_X + (1 * (BUTTON_W + BUTTON_X)),
        BUTTON_Y,
        "Log",
        3,
        BLACK,
        LIGHTGREY,
        BUTTON_X + (2 * (BUTTON_W + BUTTON_X)),
        BUTTON_Y,
        "Mem",
        3,
        BLACK,
        LIGHTGREY,
        BUTTON_X + (3 * (BUTTON_W + BUTTON_X)),
        BUTTON_Y,
};
#endif
// ----- software timer
unsigned long Timer2 = 1000000; //500000L;                         // 500mS loop ... used when sending data to to Processing
unsigned long Stop2 = 0;

bool on = true;
byte pin = 22;

/*
  * Setting for Serial interrupt
  */
//*** flag data on the listener port is ready
volatile bool listenerDataReady = false;

/*** function check if a string is a number
*/
boolean isNumeric(char *value)
{
  boolean result = true;
  int i = 0;
  while (value[i] != '\0' && result && i < FIELD_BUFFER)
  {
    result = (isDigit(value[i]) || value[i] == '.' || value[i] == '-');
    i++;
  }
  return result;
}
//*** ISR to set listerDataReady flag
void listenerReady()
{
  listenerDataReady = true;
}

/*
debugWrite() <--provides basic debug info from other tasks
takes a String as input parameter
*/
void debugWrite(String debugMsg)
{
#ifdef DEBUG
  if (debugMsg.length() > 1)
    Serial.println(debugMsg);
  else
    Serial.print(debugMsg);
#else
#ifdef DISPLAY_ATTACHED
  int str_len = debugMsg.length();
  char charMsg[str_len];
  debugMsg.toCharArray(charMsg, str_len);
  screen_println(charMsg, 2, flag_colour, BLACK, false);

#endif
#endif
}

/*
   Class definitions go here
*/

/*
  Purpose:  Helper class stacking NMEA data as a part of the multiplexer application
            - Pushin and popping NMEAData structure on the stack for buffer purposes
 */
class NMEAStack
{
public:
  NMEAStack();              // Constructor with the size of the stack
  int push(NMEAData _nmea); // put an NMEAData struct on the stack and returns the lastIndex or -1
  NMEAData pop();           // get an NMEAData struct from the stack and decreases the lastIndex
  int getIndex();           // returns the position of the next free postion in the stack

private:
  NMEAData stack[STACKSIZE]; // the array containg the structs
  int lastIndex = 0;         // an index pointng to the first free psotiion in the stack
};

NMEAStack::NMEAStack()
{
  this->lastIndex = 0;
  for (int i = 0; i < STACKSIZE; i++)
  {
    for (int j = 0; j < MAX_NMEA_FIELDS; j++)
    {
      stack[i].fields[j] = "";
    }
    stack[i].nrOfFields = 0;
    stack[i].sentence = "";
  }
}

int NMEAStack::push(NMEAData _nmea)
{
#ifdef DEBUG
  debugWrite("Pushing on index:" + String(this->lastIndex));
#endif
  if (this->lastIndex < STACKSIZE)
  {
    stack[this->lastIndex++] = _nmea;
    return this->lastIndex;
  }
  else
  {
    this->lastIndex = STACKSIZE;
    return -1; // of stack is full
  }
}

NMEAData NMEAStack::pop()
{
  NMEAData nmeaOut;
  nmeaOut.sentence = "";
  if (this->lastIndex > 0)
  {
    this->lastIndex--;
    nmeaOut = stack[this->lastIndex];
  }
#ifdef DEBUG
  debugWrite("Popped from index: " + String(lastIndex));
#endif

  return nmeaOut;
}

int NMEAStack::getIndex()
{
  return this->lastIndex;
}

/*
    Purpose:  An NMEA0183 parser to convert old to new version NMEA sentences
            - Reading NMEA0183 v1.5 data without a checksum,
            - Filtering out current heading data causing incorrect course in fo in navigation app
              i.e. HDG, HDM and VHW messages
            
          
  NOTE:     NMEA encoding conventions in short
            An NMEA sentence consists of a start delimiter, followed by a comma-separated sequence
            of fields, followed by the character '*' (ASCII 42), the checksum and an end-of-line marker.
            i.e. <start delimiter><field 0>,<field 1>,,,<field n>*<checksum><end-of-linemarker>
            The start delimiter is either $ or !. <field 0> contains the tag and the remaining fields
            the values. The tag is normaly a 5 character wide identifier where the 1st 2 characters
            identify the talker ID and the last 3 identify the sentence ID.
            Maximum sentence length, including the $ and <CR><LF> is 82 bytes.

  Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_nmea_0183_physical_protocol_layer
  */
class NMEAParser
{
public:
  NMEAParser(NMEAStack *_ptrNMEAStack);

  void parseNMEASentence(String nmeaIn); // parse an NMEA sentence with each part stored in the array

  unsigned long getCounter(); //return nr of sentences parsed since switched on

private:
  NMEAStack *ptrNMEAStack;
  String NMEAFilter = NMEA_SPECIALTY;
  NMEAData nmeaData; // self explaining
  String nmeaSentence = "";
  void reset();                            // clears the nmeaData struct;
  String checksum(String str);             //calculate the checksum for str
  NMEAData nmeaSpecialty(NMEAData nmeaIn); // special treatment function
  unsigned long counter = 0;
};

// ***
// *** NMEAParser Constructor
// *** input parameters:
// *** reference to the debugger object
NMEAParser::NMEAParser(NMEAStack *_ptrNMEAStack)
    : ptrNMEAStack(_ptrNMEAStack)
{
  //*** initialize the NMEAData struct.
  reset();
}

/*
 * Clear the nmeaData attribute
 */
void NMEAParser::reset()
{
  nmeaData.nrOfFields = 0;
  nmeaData.sentence = "";
  for (int i = 0; i < MAX_NMEA_FIELDS; i++)
  {
    nmeaData.fields[i] = "";
  }
  current_color = WHITE;
}

/*

*/
NMEAData NMEAParser::nmeaSpecialty(NMEAData nmeaIn)
{
  String filter = NMEA_SPECIALTY;

  NMEAData nmeaOut; //= nmeaIn;
#ifdef DEBUG
  debugWrite(" Specialty found... for filter" + filter);
#endif
  if (filter.indexOf(nmeaIn.fields[0]) > -1)
  {
    /* In my on-board Robertson data network some sentences
       are not NMEA0183 compliant. So these sentences need
       to be converted to compliant sentences
    */
    //*** $IIDBK is not NMEA0183 compliant and needs conversion
    //*** Since DBK/DBS sentences are obsolete DPT is used
    if (nmeaIn.fields[0] == _DBK)
    {
#ifdef DEBUG
      debugWrite("Found " + String(_DBK));
#endif
      // a typical non standard DBK message I receive is
      // $IIDBK,A,0017.6,f,,,,
      // Char A can also be a V if invalid and shoul be removed
      // All fields after the tag shift 1 position to the left
      // Since we modify the sentence we'll also put our talker ID in place

      //*** below code is for DPT since TZ iBoat does not use DBT
      nmeaOut.fields[0] = "$EPDPT";
      if (nmeaIn.fields[3] == "f")
      {
        //depth in feet need to be converted
        float ft = nmeaIn.fields[2].toFloat();
        nmeaOut.fields[1] = String(ft * FTM, 1);
      }
      else
        nmeaOut.fields[1] = nmeaIn.fields[2];
      nmeaOut.fields[2] = "0.0";
      nmeaOut.nrOfFields = 3;
      for (int i = 0; i < nmeaOut.nrOfFields; i++)
      {
#ifdef DEBUG
        debugWrite("Field[" + String(i) + "] = " + nmeaOut.fields[i]);
#endif
        if (i > 0)
          nmeaOut.sentence += ",";
        nmeaOut.sentence += nmeaOut.fields[i];
      }
      nmeaOut.sentence += checksum(nmeaOut.sentence);

#ifdef DEBUG
      debugWrite(" Modified to:" + nmeaOut.sentence);
#endif
      return nmeaOut;
    }

    //*** current Battery info is in a non NMEA0183 format
    //*** i.e. $PSTOB,13.2,V
    //*** will be converted to $AOXDR,U,13.2,V,BATT,*CS
    if (nmeaIn.fields[0] == _TOB)
    {
      reset();
      float batt = (nmeaIn.fields[1]).toFloat() + BATTERY_OFFSET;
      nmeaOut.nrOfFields = 5;
      nmeaOut.fields[0] = "$EPXDR";
      nmeaOut.fields[1] = "U";              // the transducer unit
      nmeaOut.fields[2] = String(batt, 1);  // the actual measurement value
      nmeaOut.fields[3] = nmeaIn.fields[2]; // unit of measure
      nmeaOut.fields[3].toUpperCase();
      nmeaOut.fields[4] = "BATT";
      for (int i = 0; i < nmeaOut.nrOfFields; i++)
      {
#ifdef DEBUG
        debugWrite("Field[" + String(i) + "] = " + nmeaOut.fields[i]);
#endif
        if (i > 0)
          nmeaOut.sentence += ",";
        nmeaOut.sentence += nmeaOut.fields[i];
      }
      nmeaOut.sentence += checksum(nmeaOut.sentence);
      return nmeaOut;
    }
  }
  return nmeaOut;
}

// calculate checksum function (thanks to https://mechinations.wordpress.com)
String NMEAParser::checksum(String str)
{
  byte cs = 0;
  for (unsigned int n = 1; n < str.length(); n++)
  {
    if (str[n] != '$' || str[n] != '!' || str[n] != '*')
    {
      cs ^= str[n];
    }
  }

  if (cs < 0x10)
    return "*0" + String(cs, HEX);
  else
    return "*" + String(cs, HEX);
}

/*
   parse an NMEA sentence into into an NMEAData structure.
*/
void NMEAParser::parseNMEASentence(String nmeaStr)
{
  reset();
  int currentIndex = 0;
  int lastIndex = -1;
  int sentenceLength = nmeaStr.length();

//*** check for a valid NMEA sentence
#ifdef DEBUG
  debugWrite(" In te loop to parse for " + String(sentenceLength) + " chars");
#endif
  if (nmeaStr[0] == '$' || nmeaStr[0] == '!' || nmeaStr[0] == '~')
  {

    //*** parse the fields from the NMEA string
    //*** keeping in mind that indeOf() can return -1 if not found!
    currentIndex = nmeaStr.indexOf(',', 0);
    while (lastIndex < sentenceLength)
    {

      //*** remember to sepatrate fields with the ',' character
      //*** but do not end with one!
      if (lastIndex > 0)
        nmeaData.sentence += ',';

      //*** we want the data without the ',' in fields array
      if (currentIndex - lastIndex > 1) // check for an empty field
      {
        nmeaData.fields[nmeaData.nrOfFields] = nmeaStr.substring(lastIndex + 1, currentIndex);
        nmeaData.sentence += nmeaData.fields[nmeaData.nrOfFields];
      }
      else
        nmeaData.fields[nmeaData.nrOfFields] = "0";
      nmeaData.nrOfFields++;
      lastIndex = currentIndex; // searching from next char of ',' !
      currentIndex = nmeaStr.indexOf(',', lastIndex + 1);
      //*** check if we found the last seperator
      if (currentIndex < 0)
      {
        //*** and make sure we parse the last part of the string too!
        currentIndex = sentenceLength;
      }
    }
    if (NMEAFilter.indexOf(nmeaData.fields[0]) > -1)
    {
      nmeaData = nmeaSpecialty(nmeaData);
    }
    else if (nmeaData.sentence.indexOf('*') < 1) //Check for checksum in sentence
    {
      nmeaData.sentence += checksum(nmeaData.sentence);
    }
#ifdef DEBUG
    debugWrite("Parsed : " + nmeaData.sentence);
#endif
    nmeaData.sentence += NMEA_TERMINATOR;
#ifdef DEBUG
    debugWrite("Parsed & terminated: " + nmeaData.sentence);
#endif
    ptrNMEAStack->push(nmeaData); //push the struct to the stack for later use; i.e. buffer it
    counter++;                    // for every sentence pushed the counter increments
  }

  return;
}

unsigned long NMEAParser::getCounter()
{
  return counter;
}

/***********************************************************************************
   Global variables go here
*/
NMEAStack NmeaStack;
NMEAParser NmeaParser(&NmeaStack);
NMEAData NmeaData;

/*
  Initialize the NMEA Talker port and baudrate
  on RX/TX port 2 to the multiplexer
*/
void initializeTalker()
{
  nmeaSerialOut.begin(TALKER_RATE, SWSERIAL_8N1, 22, TALKER_PORT, true);
#ifdef DEBUG
  debugWrite("Talker initialized...");
#endif
}

/*** Converts and adjusts the incomming values to usable values for the HMI display 
 * and concatenates these values in one string so it can be send in one command to the 
 * Nextion HMI in timed intervals of 50ms.
 * This is due the fact that a timer in the HMI checks on new data and refreshes the 
 * display. So no need to send more data than you can chew!
 * A refresh of 20x per second is more then sufficient.
 * The string is formatted like:
 * <Sentence ID1>=<Value1>#....<Sentence IDn>=<Value_n>#
 * Sentence ID = 3 chars i.e. SOG, COG etc
 * Value can be an integer or float with 1 decimal and max 5 char long incl. delimiter
 * i.e. SOG=6.4#COG=213.2#BAT=12.5#AWA=37#AWS=15.7#
 * The order is not applicable, so can be random
 */
void displayData()
{
  char _BITVAL[255] = {0};

  // if cog is a number
  if (isNumeric(nb_COG))
  {
    strcat(_BITVAL, "COG=");
    strcat(_BITVAL, nb_COG);
    strcat(_BITVAL, "#");
  }

  //set awa if is a number
  if (isNumeric(nb_AWA))
  {
    strcat(_BITVAL, "AWA=");
    strcat(_BITVAL, nb_AWA);
    strcat(_BITVAL, "#");
  }

  //set sog if is a number
  if (isNumeric(nb_SOG))
  {
    strcat(_BITVAL, "SOG=");
    strcat(_BITVAL, nb_SOG);
    strcat(_BITVAL, "#");
  }

  // set aws is is a number
  if (isNumeric(nb_AWS))
  {
    strcat(_BITVAL, "AWS=");
    strcat(_BITVAL, nb_AWS);
    strcat(_BITVAL, "#");
  }

  // set BATT is is a number
  if (isNumeric(nb_BAT))
  {
    strcat(_BITVAL, "BAT=");
    strcat(_BITVAL, nb_BAT);
    strcat(_BITVAL, "#");
  }
  // set dpt is is a number
  if (isNumeric(nb_DPT))
  {
    strcat(_BITVAL, "DPT=");
    strcat(_BITVAL, nb_DPT);
    strcat(_BITVAL, "#");
  }
  // Calculate TWS from AWA and SOG as described Starpath TrueWind by, David Burch, 2000
  // TWS= SQRT( SOG^2*AWS^2 + (2*SOG*AWA*COS(AWA/180)))
  double sog, awa, aws, tws = 0.0;
  sog = atof(nb_SOG);
  awa = atof(nb_AWA);
  aws = atof(nb_AWS);
  tws = sqrt(sog * sog + aws * aws - (2 * sog * aws * cos((double)awa * PI / 180)));
  sprintf(nb_TWS, "%.1f", tws);
  strcat(_BITVAL, "TWS=");
  strcat(_BITVAL, nb_TWS);
  strcat(_BITVAL, "#");
  //*** Nextion display timer max speed is 50ms
  // so no need to send faster than 50ms otherwise
  // flooding the serialbuffer
  if (millis() - tmr1 > NEXTION_SND_DELAY)
  {
    tmr1 = millis();
#ifdef NEXTION_ATTACHED

    if (strcmp(oldVal, _BITVAL) != 0)
    {
      /*
      dbSerial.print("Preparing NMEA data: clearing buffer:");
      sendCommand("code_c");                     // clear the previous databuffer if present
      recvRetCommandFinished(NEXTION_RCV_DELAY); // always wait for a reply from the HMI!
*/
      strcpy(oldVal, _BITVAL);

      /*nexSerial.print("winddisplay.nmea.txt=");
      nexSerial.print(_BITVAL);
      nexSerial.write(0xFF);
      nexSerial.write(0xFF);
      nexSerial.write(0xFF);
      */
      dbSerial.print("Sending NMEA data: ");
      nmeaTxt.setText(_BITVAL);
      dbSerial.println(_BITVAL);
    }

#endif

    newData = false;
  }
}

/*
 * Start reading converted NNMEA sentences from the stack
 * and write them to Serial Port 2 to send them to the 
 * external NMEA device.
 * Update the display with the value(s) send
 */
byte startTalking()
{
  NMEAData nmeaOut;

#ifdef NEXTION_ATTACHED
  double tmpVal = 0.0;
#endif

  //*** for all  NMEAData opjects on the stack
  //*** NOTE; the stack has a buffer of NMEA_BUFFER_SIZE objects
  //***       normaly only 1 or 2 should be on the stack
  //***       if the stack is full your timing is out of control

  if (NmeaStack.getIndex() > 0)
  {
    nmeaOut = NmeaStack.pop();
    //outStr =nmeaOut.sentence;

    for (int i = 0; i < (int)nmeaOut.sentence.length(); i++)
    {
      //outChar[i]=outStr[i];
      nmeaSerialOut.write(nmeaOut.sentence[i]);
    }

#ifdef DEBUG
    debugWrite(" Sending :" + nmeaOut.sentence);
#endif
  }
#ifdef NEXTION_ATTACHED
  // check which screens is active and update with data
  // switch (active_menu_button)
  // {

  // case SPEED:
  // speeds are checked for values <100; Higher is non existant
  if (nmeaOut.fields[0] == _RMC)
  {
    memcpy(nb_SOG, &nmeaOut.fields[7], FIELD_BUFFER - 1);

    /* tmpVal = nmeaOut.fields[7].toDouble();
      if (tmpVal < 100)
        update_display(tmpVal, screen_units[SPEED], "SOG", Q1); */
  }
  if (nmeaOut.fields[0] == _VHW)
  {
    memcpy(nb_STW, &nmeaOut.fields[5], FIELD_BUFFER - 1);
    // tmpVal = nmeaOut.fields[5].toDouble();
    // if (tmpVal < 100)
    //   update_display(tmpVal, screen_units[SPEED], "STW", Q2);
  }
  if (nmeaOut.fields[0] == _VWR)
  {
    memcpy(nb_AWS, &nmeaOut.fields[3], FIELD_BUFFER - 1);
    memcpy(nb_AWA, &nmeaOut.fields[1], FIELD_BUFFER - 1);
    if (nmeaOut.fields[2] == "L")
    {
      //update_display(tmpVal, screen_units[DEGL], "AWA", Q4);
      memmove(nb_AWA + 1, nb_AWA, FIELD_BUFFER - 2);
      nb_AWA[0] = '-';
    }
    // tmpVal = nmeaOut.fields[3].toDouble();
    // if (tmpVal < 100)
    //   update_display(tmpVal, screen_units[SPEED], "AWS", Q3);
    // tmpVal = nmeaOut.fields[1].toDouble();
    //char tmpChr[(nmeaOut.fields[2].length())];
    //(nmeaOut.fields[2]).toCharArray(tmpChr,nmeaOut.fields[2].length(),0);
    // if (tmpVal < 360 && nmeaOut.fields[2] == "R")
    //   update_display(tmpVal, screen_units[DEGR], "AWA", Q4);
    // else if (tmpVal < 360 && nmeaOut.fields[2] == "L")
    //   update_display(tmpVal, screen_units[DEGL], "AWA", Q4);
  }
  //   break;
  // case CRS:
  if (nmeaOut.fields[0] == _RMC)
  {
    memcpy(nb_COG, &nmeaOut.fields[8], FIELD_BUFFER - 1);
    // tmpVal = nmeaOut.fields[8].toDouble();
    // if (tmpVal < 360)
    //   update_display(tmpVal, screen_units[DEG], "TRU", Q1);
  }
  if (nmeaOut.fields[0] == _hDG)
  {
    memcpy(nb_HDG, &nmeaOut.fields[1], FIELD_BUFFER - 1);
    // tmpVal = nmeaOut.fields[1].toDouble();
    // if (tmpVal < 360)
    //   update_display(tmpVal, screen_units[DEG], "MAG", Q2);
  }
  if (nmeaOut.fields[0] == _dPT)
  {
    memcpy(nb_DPT, &nmeaOut.fields[1], FIELD_BUFFER - 1);
    // tmpVal = nmeaOut.fields[1].toDouble();
    // update_display(tmpVal, screen_units[MTRS], "DPT", Q3);
  }
  if (nmeaOut.fields[0] == _VLW)
  {
    memcpy(nb_TRP, &nmeaOut.fields[3], FIELD_BUFFER - 1);
    // tmpVal = nmeaOut.fields[3].toDouble();
    // update_display(tmpVal, screen_units[DIST], "TRP", Q4);
    //   }

    //   break;
    // case LOG:
    // Voltage an Temperature are checked <100; Higher is non exsitant.
    if (nmeaOut.fields[0] == _xDR)
    {
      if (nmeaOut.fields[4] == "BATT")
      {
        memcpy(nb_BAT, &nmeaOut.fields[2], FIELD_BUFFER - 1);
        // tmpVal = nmeaOut.fields[2].toDouble();
        // if (tmpVal < 100)
        //   update_display(tmpVal, screen_units[VOLT], "BAT", Q1);
      }
    }
    if (nmeaOut.fields[0] == _MTW)
    {
      memcpy(nb_WTR, &nmeaOut.fields[1], FIELD_BUFFER - 1);
      // tmpVal = nmeaOut.fields[1].toDouble();
      // if (tmpVal < 100)
      //   update_display(tmpVal, screen_units[TEMP], "WTR", Q2);
    }
    if (nmeaOut.fields[0] == _VLW)
    {
      memcpy(nb_LOG, &nmeaOut.fields[1], FIELD_BUFFER - 1);
      // tmpVal = nmeaOut.fields[1].toDouble();
      // update_display(tmpVal, screen_units[DIST], "LOG", Q3);
    }
    //  if (nmeaOut.fields[0] == _VLW)
    //   {
    //     memcpy(nb_TRP, &nmeaOut.fields[3], FIELD_BUFFER - 1);
    //     // tmpVal = nmeaOut.fields[3].toDouble();
    //     // update_display(tmpVal, screen_units[DIST], "TRP", Q4);
    // //   }
    // //   break;
    // // case MEM:
    // // default:
    //   // if ((micros() - Stop2) > Timer2)
    //   // {
    //   //   Stop2 = micros(); // + Timer2;                                    // Reset timer

    //   //   tmpVal = getFreeSram();
    //   //   update_display(tmpVal, "Byte", "FREE", Q1);

    //   //   tmpVal = 0;
    //   //   update_display(tmpVal, "V.", PROGRAM_VERSION, Q2);

    //   //   tmpVal = NmeaStack.getIndex();
    //   //   update_display(tmpVal, " ", "STACK", Q3);

    //   //   tmpVal = NmeaParser.getCounter();
    //   //   update_display(tmpVal, "nr", "MSG", Q4);
    // }
    /*
      if( show_flag){
        // One time instruction for logging when LOG button pressed
        debugWrite( "Connect a cable to the serial port on" ); 
        debugWrite("115200 Baud!");
        debugWrite( String(PROGRAM_NAME)+" "+String(PROGRAM_VERSION));
        debugWrite("Free SRAM:"+ String(getFreeSram()));
        show_flag = false;
      }
      */

    Serial.print(nmeaOut.sentence);

    // break;
  }
#endif

  return 1;
}

/**********************************************************************************
  Purpose:  Helper class reading NMEA data from the serial port as a part of the multiplexer application
            - Reading NMEA0183 v1.5 data without a checksum,
*/

/*
Cleasr the inputbuffer by reading until empty, since Serial.flush does not this anymore
*/
void clearNMEAInputBuffer()
{

  while (Serial1.available() > 0)
  {
    Serial1.read();
  }
}

/*
* Initializes UART 2 for incomming NMEA0183 data from the Robertson network
*/
void initializeListener()
{

  Serial1.begin(LISTENER_RATE, SERIAL_8N1, LISTENER_RX, LISTENER_TX, true);
  //clearNMEAInputBuffer();
#ifdef DEBUG
  debugWrite("Listener initialized...");
#endif
}

/*
  Decode the incomming character and test if it is valid NMEA data.
  If true than put it the NMEA buffer and call NMEAParser object
  to process incomming and complete MNEA sentence
*/
void decodeNMEAInput(char cIn)
{
  switch (cIn)
  {
  case '~':
    // reserved by NMEA
  case '!':
    //for AIS info
  case '$':
    // for general NMEA info
    nmeaStatus = RECEIVING;
    nmeaIndex = 0;
    break;
  case '*':
    if (nmeaStatus == RECEIVING)
    {
      nmeaStatus = CHECKSUMMING;
    }
    break;
  case '\n':
  case '\r':
    // in old v1.5 version, NMEA Data may not be checksummed!
    if (nmeaStatus == RECEIVING || nmeaStatus == CHECKSUMMING)
    {
      nmeaDataReady = true;
      nmeaStatus = TERMINATING;
    }
    else
      nmeaStatus = INVALID;

    break;
  }
  switch (nmeaStatus)
  {
  case INVALID:
    // do nothing
    nmeaIndex = 0;
    nmeaDataReady = false;
    break;
  case RECEIVING:
  case CHECKSUMMING:
    nmeaBuffer[nmeaIndex] = cIn;
    nmeaIndex++;
    break;
  case TERMINATING:

    nmeaStatus = INVALID;
    if (nmeaDataReady)
    {
      nmeaDataReady = false;

      // Clear the remaining buffer content with '\0'
      for (int y = nmeaIndex + 1; y < NMEA_BUFFER_SIZE + 1; y++)
      {
        nmeaBuffer[y] = '\0';
      }
#ifdef DEBUG
      debugWrite(nmeaBuffer);
#endif
      NmeaParser.parseNMEASentence(nmeaBuffer);

      //clear the NMEAbuffer with 0
      memset(nmeaBuffer, 0, NMEA_BUFFER_SIZE + 1);
      nmeaIndex = 0;
    }

    break;
  }
}

/*
 * Start listeneing for incomming NNMEA sentences
 */
void startListening()
{
#ifdef DEBUG
  //debugWrite("Listening....");
#endif

  while (Serial1.available() > 0 && nmeaStatus != TERMINATING)
  {
    decodeNMEAInput(Serial1.read());
  }
}

#ifdef TEST

String NmeaStream[10] = {
    "$IIVWR,151,R,02.4,N,,,,",
    "$IIMTW,12.2,C",
    "!AIVDM,1,1,,A,13aL<mhP000J9:PN?<jf4?vLP88B,0*2B",
    "$IIDBK,A,0014.4,f,,,,",
    "$IIVLW,1149.1,N,001.07,N",
    "$GPGLL,5251.3091,N,00541.8037,E,151314.000,A,D*5B",
    "$GPRMC,095218.000,A,5251.5621,N,00540.8482,E,4.25,201.77,120420,,,D*6D",
    "$PSTOB,13.0,v",
    "$IIVWR,151,R,02.3,N,,,,",
    "$IIVHW,,,000,M,01.57,N,,"};

int softIndex = 0;
long softTimerOld = 0;
long softTimerNow;

void runSoftGenerator()
{
  softTimerNow = millis();
  //if( softTimerNow - softTimerOld >250 ){
  if (on)
  {
    digitalWrite(pin, LOW);
    on = false;
    //debugWrite("BLINKER: OFF");
    // If the LED is off, turn it on and remember the state.
  }
  else
  {
    digitalWrite(pin, HIGH);
    on = true;
    //Send output to Serial Monitor via debugger
    //debugWrite("BLINKER: ON");
  }

  if (softIndex < 10)
  {

    NmeaParser.parseNMEASentence(NmeaStream[softIndex++]);

    delay(200);
  }
  else
    softIndex = 0;

  // }
  // softTimerOld=softTimerNow;
}

#endif

void setup()
{
  // put your setup code here, to run once:
  //Serial.begin(SAMPLERATE);
#ifdef NEXTION_ATTACHED
  if (nexInit())
  {
    dbSerial.println("Initialisation succesful....");
  }
  else
  {
    dbSerial.println("Initialisation failed...");
    dbSerial.println("Resetting Nextion...");
    sendCommand("rest");
    delay(3000);
  }

  delay(150);
  dbSerial.print(" Writing version to splash: ");
  versionTxt.setText(PROGRAM_VERSION);
  delay(5000);
  dbSerial.print("Switcing to page 1: ");
  sendCommand("page 1");
  recvRetCommandFinished(NEXTION_RCV_DELAY);

  // restet the HMI o default 0 values
  memcpy(nb_AWA, "--.-", 5);
  memcpy(nb_COG, "---.-", 6);
  memcpy(nb_SOG, "--.-", 5);
  memcpy(nb_AWS, "--.-", 5);
  memcpy(nb_DPT, "--.-", 5);
  memcpy(nb_BAT, "--.-", 5);
  displayData();
#endif

  //Serial.begin(115200);
  initializeListener();
  initializeTalker();

  //Serial1.begin(4800, SERIAL_8N1, 16, 17, true);
  nmeaSerialOut.begin(38400, SWSERIAL_8N1, 22, TALKER_PORT, true);
}

void loop()
{
  // put your main code here, to run repeatedly:

#ifdef TEST
  runSoftGenerator();
#endif

  startListening();

#ifdef DISPLAY_ATTACHED
  buttonPressed();
#endif

  startTalking();
  /* while (Serial1.available())
  {
    Serial.print(char(Serial1.read()));
  } */
  displayData();
}
