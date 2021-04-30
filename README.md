# Yazz_NMEAtor_ESP32
Project:  Yazz_NMEAtor_ESP32.cpp, Copyright 2020, Roy Wassili
  Contact:  waps61 @gmail.com
  URL:      https://www.hackster.io/waps61
  VERSION:  1.0
  Date:     10-10-2020
  Last
  Update:   30-04-2021 V1.0
            Released version
            30-04-2021 V0.13
            Tested version for input and output with all parameters to display
            29-04-2021 V0.12
            Beta version with display communication to Nextion NX4832K035
            Not all parameters are show yet
            26-04-2021 V0.11
            Preparation for display implementation, compiled version and runnable
            26-04-2021 V0.10
            First runnable MVP reading NMEA data,converting and sending NMEA0183 data
            18-04-2021 V0.02
            Fixed a bug in the Depth calculation in the Arduino V1.04 that was inherited 
            in this version. Aruino version updated to V1.05
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
