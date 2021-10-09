/*
 * Author: Brian Lambert
 *
 * Decodes GPS data from the Taranis R9M module
 * 
 * For the initial proof of concept connect 
 *         USB/TTL GND to Arduino GND
 *         USB/TTL TX via INVERTER MODULE to Arduino pin 0 (rx)        
 *         Use Realterm to capture data from R9M and then transfered data to Arduino for initial testing purposes.
 *         
 * Otherwise-        
 * Connect R9M module GND to Arduino GND
 *         R9M TX via INVERTER MODULE to Arduino pin 0 (rx)
 *         or use HC05 bluetooth master module at Arduino,  with an INVERTER and HC06 slave at Taranis
 *                HC05 GND - Arduino GND, HC05 VCC - Arduino 5V, HCo5 TX - Arduino pin 0 (rx)
 *         
 * Other than SART_FRAME and CRC all data bytes are sent LSBYTE first e.g. So ID1 is sent 00 08 as are the 4 GPS data bytes
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

#include "Arduino.h"
#include "FrSkyR9MSportDecoder.h"




FrSkyR9MSportDecoder::FrSkyR9MSportDecoder()
{
  
}

//Start up everything
void FrSkyR9MSportDecoder::begin()
{
  // start the serial communication with the host computer
    Serial.begin(57600);
    Serial.println(F("Comms with Arduino started at 57600 -- waiting for telemetry..."));
	printThis = false;//used in beginTest()
}

//Start up everything for testing, prints details
void FrSkyR9MSportDecoder::beginTest()
{
  // start the serial communication with the host computer
    Serial.begin(57600);
    Serial.println(F("Comms with Arduino started at 57600"));
	Serial.println(F("If sensor not displayed it may not be available in telemetry"));
	printThis = true;
}


 
//************display distance to home***********

void FrSkyR9MSportDecoder::displayDistanceToHome()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printDistanceToHome(frame);
		    }
      }
  }

//************display date/time***********

void FrSkyR9MSportDecoder::displayDateTime()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printDateTime(frame);
		  }
      }
  }

//************display altitude***********

void FrSkyR9MSportDecoder::displayAltitude()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printAltitude(frame);
		    }
      }
  }

//************display current***********

void FrSkyR9MSportDecoder::displayCurrent()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printCurrent(frame);
		    }
      }
  }

//************display battery voltage***********

void FrSkyR9MSportDecoder::displayBattVolts()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printBattVolts(frame);
		    }
      }
  }

//************display vertical speed***********

void FrSkyR9MSportDecoder::displayVertSpeed()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printVertSpeed(frame);
		    }
      }
  }

//************display RSSI***********

void FrSkyR9MSportDecoder::displayRssi()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printRssi(frame);
		    }
      }
  }

//************display RX battery voltage***********

void FrSkyR9MSportDecoder::displayRxBattVolts()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printRxBattVolts(frame);
		    }
      }
  }

//************display cell voltage***********

void FrSkyR9MSportDecoder::displayCellVolts()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printCellVolts(frame);
		    }
      }
  }

//************display GPS course over ground***********

void FrSkyR9MSportDecoder::displayGpsCog()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printGpsCog(frame);
		    }
      }
  }


//************display GPS speed***********

void FrSkyR9MSportDecoder::displayGpsSpeed()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printGpsSpeed(frame);
		    }
      }
  }


//************display GPS altitude***********

void FrSkyR9MSportDecoder::displayGpsAltitude()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printGpsAltitude(frame);
		    }
      }
  }


//************display lat/lon***********

void FrSkyR9MSportDecoder::displayLatLon()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
		  if (crcCheck(frame))// if crc check ok
			{
				printLatLonData(frame);
		    }
      }
  }

//***********plain read data**************

void FrSkyR9MSportDecoder::plainReadData()
  {
	  resetDecodedData();//reset all decoded data variables
    int gotData;
    int frame = 0;
    int dataCell = 0;
    while(frame <= FRAMES_TO_READ)
      {
        while(dataCell <= BYTES_PER_FRAME)
          { 
            if (Serial.available() > 0)//serial available
              {
                gotData = Serial.read();
                if(gotData == START_FRAME)
                    {
                      dataCell = 0;//reset dataCell pointer to start
                      frame++;//next frame
                      hexData[frame][dataCell] = gotData;
                    }
                else
                  {
                    dataCell++;
                    hexData[frame][dataCell] = gotData;
                  }
            }
            if(frame==FRAMES_TO_READ){break;}
          }
          dataCell = 0;
          if(frame==FRAMES_TO_READ){break;}
    }          
  }

//********print GPS lat/lon data**********

void FrSkyR9MSportDecoder::printLatLonData(int frame)
  {
    if((hexData[frame][3] == 0x00) && (hexData[frame][4] == 0x08))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeLatLon(combineHexNumbers());
      }
  }

//********print GPS altitude data**********

  void FrSkyR9MSportDecoder::printGpsAltitude(int frame)
  {
    if((hexData[frame][3] == 0x20) && (hexData[frame][4] == 0x08))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeGpsAltitude(combineHexNumbers());
      }
  }

//********print GPS speed data**********

  void FrSkyR9MSportDecoder::printGpsSpeed(int frame)
  {
    if((hexData[frame][3] == 0x30) && (hexData[frame][4] == 0x08))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeGpsSpeed(combineHexNumbers());
      }
  }

  //********print GPS speed data**********

  void FrSkyR9MSportDecoder::printGpsCog(int frame)
  {
    if((hexData[frame][3] == 0x40) && (hexData[frame][4] == 0x08))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeGpsCog(combineHexNumbers());
      }
  }

  //********print cell voltage data**********

  void FrSkyR9MSportDecoder::printCellVolts(int frame)
  {
    if((hexData[frame][3] == 0x10) && (hexData[frame][4] == 0x09))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeCellVolts(combineHexNumbers());
      }
  }

  //********print cell voltage data**********

  void FrSkyR9MSportDecoder::printRxBattVolts(int frame)
  {
    if((hexData[frame][3] == 0x04) && (hexData[frame][4] == 0xF1))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeRxBattVolts(combineHexNumbers());
      }
  }

  //********print cell voltage data**********

  void FrSkyR9MSportDecoder::printRssi(int frame)
  {
    if((hexData[frame][3] == 0x01) && (hexData[frame][4] == 0xF1))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeRssi(combineHexNumbers());
      }
  }

//********print vertical speed data**********

  void FrSkyR9MSportDecoder::printVertSpeed(int frame)
  {
    if((hexData[frame][3] == 0x10) && (hexData[frame][4] == 0x01))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeVertSpeed(combineHexNumbers());
      }
  }

  //********print battery voltage data**********

  void FrSkyR9MSportDecoder::printBattVolts(int frame)
  {
    if((hexData[frame][3] == 0x10) && (hexData[frame][4] == 0x02))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeBattVolts(combineHexNumbers());
      }
  }

  //********print current**********

  void FrSkyR9MSportDecoder::printCurrent(int frame)
  {
    if((hexData[frame][3] == 0x00) && (hexData[frame][4] == 0x02))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeCurrent(combineHexNumbers());
      }
  }

  //********print altitude**********

  void FrSkyR9MSportDecoder::printAltitude(int frame)
  {
    if((hexData[frame][3] == 0x00) && (hexData[frame][4] == 0x01))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeAltitude(combineHexNumbers());
      }
  }

  //********print distance to home**********

  void FrSkyR9MSportDecoder::printDistanceToHome(int frame)
  {
    if((hexData[frame][3] == 0x20) && (hexData[frame][4] == 0x04))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeDistanceToHome(combineHexNumbers());
      }
  }

  //********print date/time**********

  void FrSkyR9MSportDecoder::FrSkyR9MSportDecoder::printDateTime(int frame)
  {
    
    if((hexData[frame][3] == 0x50) && (hexData[frame][4] == 0x08))
      { 
        num1 = hexData[frame][8];
        num2 = hexData[frame][7];
        num3 = hexData[frame][6];
        num4 = hexData[frame][5];
        //Serial.println(combineHexNumbers(),HEX);
        decodeDateTime(combineHexNumbers());
      }
  }


//************combine 4 HEX numbers into one***********
//****HEX numbers must be in long num1 to num4. num1 is MSB

  long FrSkyR9MSportDecoder::combineHexNumbers()
    {
      long num5;
      num5=(num1 << 8)|(num2);
      num5 = (num5 << 8)|(num3);
      num5 = (num5 << 8)|(num4);
      return num5;
    }
  

//*****print contents of read buffer USED FOR TESTING**********

void FrSkyR9MSportDecoder::printHexData()
  {
    for(int frame = 0; frame < FRAMES_TO_READ; frame++)
      {
        for(int cell =0; cell < BYTES_PER_FRAME; cell++)
          {
            if(hexData[frame][cell] == START_FRAME)
              {
                Serial.println();
              }
            if(hexData[frame][cell] <= 0x0F)
                  {
            Serial.print("0");
             }
            Serial.print(hexData[frame][cell],HEX);
            Serial.print(" ");   
        }     
      }
  }


//*****read HEX data from RealTerm and print USED FOR TESTING*******

void FrSkyR9MSportDecoder::readRealTerm()
    {
      unsigned int n = 0;
      while(n<BYTES_PER_FRAME)//buffer not full
        {
          if (Serial.available() > 0)//serial available
            {
              if(firstPart)//first part of HEX
                {
                  str[0] = Serial.read();// read the incoming byte:
                  firstPart = false;
                }
              else //second part
                {
                  str[1] = Serial.read();
                  firstPart = true;
                  ret = strtoul(str, &ptr, 16);//change two chars to HEX
                  // say what you got:
                  if(ret == START_FRAME)
                        {
                          Serial.println();
                        }
                   if(ret < 0x10)// print a 0 before some HEX bytes
                        {
                          Serial.print("0");
                        }
                      Serial.print(ret,HEX);
                      Serial.print(" ");
                  n++;
                }
            }
          }
    }

    //******decode GPS lat/Lon**************

    void FrSkyR9MSportDecoder::decodeLatLon(long data)
      {
        float lat = 0.0;
        float lon = 0.0;
        float latLonData = (data & 0x3FFFFFFF) / 10000.0 / 60.0;
        if((data & 0x40000000) > 0) latLonData = -latLonData;                 // is negative?
        if((data & 0x80000000) == 0) lat = latLonData; else lon = latLonData; // is latitude?
          if(lat>0)
            {
              if(printThis)
				{
				Serial.print("Latitude ");
				Serial.println(lat,7);
			  }
				decodedLat = lat;
            }
          if(lon!=0)
            {
              if(printThis)
				{
				Serial.print("Longitude ");
				Serial.println(lon,7);
				}
			  decodedLon = lon;
            }
      }

      //****decode GPS altitude*************
      
      void FrSkyR9MSportDecoder::decodeGpsAltitude(long data)
        {
          float altitude = ((int32_t)data) / 100.0;
          if(printThis)
		  {
		  Serial.print("GPS altitude ");
          Serial.print(altitude);
          Serial.println(" m");
		  }
		  decodedGpsAltitude = altitude;
        }
        
//************decode GPS ground speed*************

        void FrSkyR9MSportDecoder::decodeGpsSpeed(long gpsSpeed)
        {
          float speed = gpsSpeed / 1944.0; // Convert knots to m/s
          if(printThis)
		  {
		  Serial.print("Speed ");
          Serial.print(speed);
          Serial.println("m/s");
		  }
		  decodedGpsSpeed = speed;
        }
        
//*************decode GPS bearing ************

        void FrSkyR9MSportDecoder::decodeGpsCog(long gpsCog)
        {
          float cog = gpsCog / 100.0;
          if(printThis)
		  {
		  Serial.print("Course over ground ");
          Serial.print(cog);
          Serial.println(" degrees");
		  }
		  decodedCog = cog;
        }

//*************decode cell voltage ************

        void FrSkyR9MSportDecoder::decodeCellVolts(long cVolts)
        {
          float volts = cVolts / 100.0;
          if(printThis)
		  {
		  Serial.print("Battery cell voltage ");
          Serial.print(volts);
          Serial.println(" volts");
		  }
		  decodedCellVolts = volts;
        }

//*************decode Rx battery voltage ************

        void FrSkyR9MSportDecoder::decodeRxBattVolts(long rxVolts)
        {
          float volts = rxVolts / 19.2; //empirical value
          if(printThis)
		  {
		  Serial.print("RX battery voltage ");
          Serial.print(volts);
          Serial.println(" volts");
		  }
		  decodedRxVolts = volts;
        }

//*************decode RSSI ************

        void FrSkyR9MSportDecoder::decodeRssi(long rawRssi)
        {
          float rssi = rawRssi / 1.01;
          if(printThis)
		  {
		  Serial.print("RSSI ");
          Serial.print(rssi,0);
          Serial.println(" db");
		  }
		  decodedRssi = rssi;
        }

//*************decode Vspeed ************

        void FrSkyR9MSportDecoder::decodeVertSpeed(long rawSpeed)
        {
          float vSpeed = rawSpeed / 1944.0; // Convert knots to m/s
          if(printThis)
		  {
		  Serial.print("Vertical speed ");
          Serial.print(vSpeed);
          Serial.println(" m/s");
		  }
		  decodedVSpeed = vSpeed;
        }

//*************decode battery voltage ************

        void FrSkyR9MSportDecoder::decodeBattVolts(long rawBattVolts)
        {
          float battVolts = rawBattVolts / 100.0; 
          if(printThis)
		  {
		  Serial.print("Battery voltage ");
          Serial.print(battVolts);
          Serial.println(" volts");
		  }
		  decodedBattVolts = battVolts;
        }

//*************decode current ************

        void FrSkyR9MSportDecoder::decodeCurrent(long rawCurrent)
        {
          float current = rawCurrent / 10.0; 
          if(printThis)
		  {
		  Serial.print("Battery current ");
          Serial.print(current);
          Serial.println(" amps");
		  }
		  decodedCurrent = current;
        }

//*************decode altitude ************

        void FrSkyR9MSportDecoder::decodeAltitude(long rawAltitude)
        {
          float altitude = rawAltitude / 100.0; 
          if(printThis)
		  {
		  Serial.print("Altitude ");
          Serial.print(altitude);
          Serial.println(" m");
		  }
		  decodedAltitude = altitude;
        }

//*************decode distance to home ************

        void FrSkyR9MSportDecoder::decodeDistanceToHome(long rawDistance)
        {
          float distance = rawDistance / 100.00; 
          if(printThis)
		  {
		  Serial.print("Distance to home ");
          Serial.print(distance);
          Serial.println(" m");
		  }
		  decodedDistanceToHome = distance;
        }

//*************decode date/time ************

        void FrSkyR9MSportDecoder::decodeDateTime(long rawDateTime)
        {
          uint8_t year;
          uint8_t month;
          uint8_t day;
          uint8_t hour;
          uint8_t minute;
          uint8_t second;
          if((rawDateTime & 0xFF) > 0) //iis a date
            {
              rawDateTime >>= 8; day = rawDateTime & 0xFF;
              rawDateTime >>= 8; month = rawDateTime & 0xFF;
              rawDateTime >>= 8; year = rawDateTime & 0xFF; 
            }
          else //is time
            {
              rawDateTime >>= 8; second = rawDateTime & 0xFF;
              rawDateTime >>= 8; minute = rawDateTime & 0xFF;
              rawDateTime >>= 8; hour = rawDateTime & 0xFF;
            }
		  if(printThis)
		  {
          Serial.print("Date/time ");
          Serial.print(day);
          Serial.print(" ");
          Serial.print(month);
          Serial.print(" ");
          Serial.print(year);
          Serial.print("  ");
          Serial.print(hour);
          Serial.print(" :");
          Serial.print(minute);
          Serial.print(":");
          Serial.println(second);
		  }
		  }

//*************reset decoded data variables ************

        void FrSkyR9MSportDecoder::resetDecodedData()
        {
          decodedLat = 0.0;
		  decodedLon = 0.0;
		  decodedGpsAltitude = 0.0;
		  decodedGpsSpeed = 0.0;
		  decodedCog = 0.0;
		  decodedCellVolts = 0.0;
		  decodedRxVolts = 0.0;
		  decodedRssi = 0.0;
		  decodedVSpeed = 0.0;
		  decodedBattVolts = 0.0;
		  decodedCurrent = 0.0;
		  decodedAltitude = 0.0;
        }

//*************CRC checking ************

        bool FrSkyR9MSportDecoder::crcCheck(int frame)
        {
			int checksum = 0;
			for(int p = 2; p < 9; p++)
				{
					checksum += hexData[frame][p];
					checksum += checksum >> 8;
					checksum &= 0x00FF;
				}
		  checksum = (0xFF - checksum);
			if(checksum == hexData[frame][9])
				{
					return true;
				}
			else
			{
				return false;
			}
		}

//*********************************************************************
//***********Tracker stuff*******************************************

//****start tracker************

	void FrSkyR9MSportDecoder::beginTracker()//store tracker's location data
	{
		
		trackerLat = decodedLat;
		trackerLon = decodedLon;
		trackerBearing = decodedCog;
		
	}

	//*********get distance to aircraft************
	float FrSkyR9MSportDecoder::getDistance(float lat1,float long1,float lat2,float long2)
{
	float d = 0.0;
	float theta1 = lat1*TORADS;
	float theta2 = lat2*TORADS;
	float deltaTheta = (lat2-lat1)*TORADS;
	float deltaLanda = (long2-long1)*TORADS;
	float a = sin(deltaTheta/2)*sin(deltaTheta/2) + cos(theta1)*cos(theta2)*sin(deltaLanda/2)*sin(deltaLanda/2);
	float c1 = 2*atan2(sqrt(a), sqrt(1-a));
	return d = R*c1;
}


//**********get bearing to aircraft**************

	float FrSkyR9MSportDecoder::getBearing(float lat1,float long1,float lat2,float long2)
{
	
	float theta1 = lat1*TORADS;
	float theta2 = lat2*TORADS;
	float landa1 = long1*TORADS;
	float landa2 = long2*TORADS;
	float y = sin(landa2 - landa1)*cos(theta2);
	float x = cos(theta1)*sin(theta2) - sin(theta1)*cos(theta2)*cos(landa2-landa1);
	float brng = atan2(y,x)*TODEGS;
	return fmod((brng + 360.0),360.0);

}

//***********get quad location**********
	void FrSkyR9MSportDecoder::getQuadLocation()
	{
		if((decodedLat != 0.0) && (decodedLon != 0.0))//if no lat/lon keep last value
			{
				distanceToQuad = getDistance(trackerLat,trackerLon,decodedLat,decodedLon);
				bearingToQuad = getBearing(trackerLat,trackerLon,decodedLat,decodedLon);
				angularHeightToQuad = getAngularHeightToQuad();
			}
	}

//***********get vertical angle tracker to quad*****
	float FrSkyR9MSportDecoder::getAngularHeightToQuad()
	{
		return atan(decodedAltitude/distanceToQuad)*TODEGS;
			
	}

