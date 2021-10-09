/*
 * Decodes the telemetry data from the Taranis R9M module S.port
 *       
 * Connect R9M module GND to Arduino GND
 *         R9M TX via INVERTER MODULE to Arduino pin 0 (rx)
 *         or use HC05 bluetooth master module at Arduino,  with an INVERTER and HC06 bluetooth slave at Taranis
 *                HC05 GND - Arduino GND, HC05 VCC - Arduino 5V, HC05 TX - Arduino pin 0 (rx)
 *         
 * Other than START_FRAME and CRC all data bytes are sent LSBYTE first e.g. So APP_ID is sent 00 then 08 as are the 4 GPS data bytes
 * Frame byte order  START_FRAME = 0, SENSOR_ID = 1, DATA_FRAME = 2, APP_ID_BYTE_1 = 3, APP_ID_BYTE_2 = 4,
 * DATA_BYTE_1 = 5, DATA_BYTE_2 = 6, DATA_BYTE_3 = 7, DATA_BYTE_4 = 8, CRC = 9 
 * 
 * Sensor IDs ID1 = 0x00,  ID2 = 0xA1,  ID3 = 0x22,  ID4 = 0x83,  ID5 = 0xE4,  ID6 = 0x45,  ID7 = 0xC6,
                    ID8 = 0x67,  ID9 = 0x48,  ID10 = 0xE9, ID11 = 0x6A, ID12 = 0xCB, ID13 = 0xAC, ID14 = 0x0D,
                    ID15 = 0x8E, ID16 = 0x2F, ID17 = 0xD0, ID18 = 0x71, ID19 = 0xF2, ID20 = 0x53, ID21 = 0x34,
                    ID22 = 0x95, ID23 = 0x16, ID24 = 0xB7, ID25 = 0x98, ID26 = 0x39, ID27 = 0xBA, ID28 = 0x1b, ID_IGNORE = 0XFF
 * 
 * 
 * Telemetry frames FRSKY_TELEMETRY_START_FRAME 0x7E, FRSKY_SENSOR_DATA_FRAME 0x10, FRSKY_STUFFING 0x7D
 * 
 * GPS Frame GPS_DEFAULT_ID ID4, GPS_DATA_COUNT 7, GPS_LAT_LON_DATA_ID 0x0800, GPS_ALT_DATA_ID 0x0820,
 *           GPS_SPEED_DATA_ID 0x0830, GPS_COG_DATA_ID 0x0840, GPS_DATE_TIME_DATA_ID 0x0850
 *           
 * FCS frame FCS_DEFAULT_ID ID3, FCS_DATA_COUNT 2,  FCS_CURR_DATA_ID 0x0200, FCS_VOLT_DATA_ID 0x0210, 
 *
 *AP_IDs RXBatt 0xF104, RSSI 0xF101, VSpd 0x0110, Hdg 0x0840, 0450 0x0450, AccX 0x0700, AccY 0x0710, AccZ 0x0720, Tmp1 0x0400, 
 *       Tmp2 0x0410, Galt 0x0820, A4(cell) 0x0910, GSpd 0x0830, GLatLon 0x 0800, VFAS(battery) 0x0210, Curr 0x0200, Alt 0x0100,
 *       Fuel 0x0600, 0420 0x0420, date/time 0x0850
*/

/*-----( Import needed libraries )-----*/

#include <FrSkyR9MSportDecoder.h>


/*-----( Declare Constants and Pin Numbers )-----*/

/*-----( Declare objects )-----*/

FrSkyR9MSportDecoder decode;

/*-----( Declare Variables )-----*/


//***initial setup***

void setup() 
{
//Comms at 76500 
 
decode.begin();

}
//----end setup----

void loop()  

  {  
  decode.plainReadData();//get telemetry
 
  decode.displayLatLon();//decode lat/lon
  Serial.print("GPS latitude ");
  Serial.println(decode.decodedLat,7);
  Serial.print("GPS longitude ");
  Serial.println(decode.decodedLon,7);
  
  decode.displayGpsAltitude();//decode altitude
  Serial.print("GPS altitude(m) ");
  Serial.println(decode.decodedGpsAltitude,0);
  
  decode.displayAltitude();
  Serial.print("Altitude(m) ");
  Serial.println(decode.decodedAltitude,0);

  decode.displayGpsSpeed();
  Serial.print("GPS speed(m/s) ");
  Serial.println(decode.decodedGpsSpeed,0);

  decode.displayGpsCog();
  Serial.print("Course over ground(degrees) ");
  Serial.println(decode.decodedCog);

  decode.displayCellVolts();
  Serial.print("Cell voltage ");
  Serial.println(decode.decodedCellVolts);

  decode.displayRxBattVolts();
  Serial.print("RX battery voltage ");
  Serial.println(decode.decodedRxVolts);

  decode.displayVertSpeed();
  Serial.print("Vertical speed(m/s) ");
  Serial.println(decode.decodedVSpeed,0);

  decode.displayBattVolts();
  Serial.print("Battery voltage ");
  Serial.println(decode.decodedBattVolts);

  decode.displayCurrent();
  Serial.print("Battery current(amps) ");
  Serial.println(decode.decodedCurrent,1);

  decode.displayRssi();
  Serial.print("RSSI(db) ");
  Serial.println(decode.decodedRssi,0);

  decode.displayDistanceToHome();
  Serial.print("Distance to home(m) ");
  Serial.println(decode.decodedDistanceToHome,0);
  
  Serial.println("************************************");
 
  } 

//--Loop Ends---
