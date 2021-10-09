/*
 * Decodes the telemetry data from the Taranis R9M module S.port
 * Drives a 3D printed tracker see Youtube for details of use and construction
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
 *       
 *Use mapping in subrourine testServoRangeError()to correct horizontal and vertical servos for range error
 *Then set SERVO_HORZ_MIN, SERVO_HORZ_MAX, SERVO_VERT_MIN, SERVO_VERT_MAX to the values determined by mapping adjustments.
 */

/*-----( Import needed libraries )-----*/

#include <FrSkyR9MSportDecoder.h>
#include <Servo.h>

/*-----( Declare Constants and Pin Numbers )-----*/
#define DEBUG 0 //for print statements on set DEBUG to 1
#if(DEBUG ==1)
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define  debugArg(x,y) Serial.print(x,y)
#define debuglnArg(x,y) Serial.println(x,y)
#else
#define debug(x)
#define debugln(x)
#define debugArg(x,y))
#define debuglnArg(x,y)
#endif

#define SERVO_HORZ_PIN 11
#define SERVO_VERT_PIN 12

//set mapping values to correct servo range error
//In my case 0 degrees required corresponded to 12 degrees input to servo
//and 180 degrees corresponded to 153 degrees input to servo.
#define MAP_0_DEGREES_HORZ_CLOCK 12
#define MAP_180_DEGREES_HORZ_CLOCK 153
#define MAP_0_DEGREES_HORZ_ANTICLOCK 12
#define MAP_180_DEGREES_HORZ_ANTICLOCK 162
#define MAP_90_DEGREES_VERT_CLOCK 85
#define MAP_180_DEGREES_VERT_CLOCK 210
#define MAP_90_DEGREES_VERT_ANTICLOCK 85
#define MAP_180_DEGREES_VERT_ANTICLOCK 210
//
#define SERVO_HORZ_HOME 90
#define SERVO_HORZ_MIN 12 //use values in MAP_0_DEGREES_HORZ_CLOCK
#define SERVO_HORZ_MAX 153 //use values in MAP_180_DEGREES_HORZ_CLOCK or ANTICLOCK whichever is the smaller
#define SERVO_VERT_HOME 90
#define SERVO_VERT_MIN 85 //use values in MAP_90_DEGREES_VERT_CLOCK
#define SERVO_VERT_MAX 210 //use values in MAP_180_DEGREES_VERT_CLOCK
#define SERVO_DELAY 10
#define SERVO_SLOW_DELAY 30
#define ROTATE 180 //changes servo rotation from anticlock to clock
/*-----( Declare objects )-----*/

FrSkyR9MSportDecoder decode;
Servo servoHorz;
Servo servoVert;

/*-----( Declare Variables )-----*/

float servoAngleToQuad = 90.0;
float oldServoAngleToQuad = 90.0;
float oldAngularHeightToQuad = 90.0;
float newAngularHeightToQuad = 90.0;

//***initial setup***

void setup() 
{
//Comms at 76500 
servoHorz.attach(SERVO_HORZ_PIN);
servoVert.attach(SERVO_VERT_PIN);
//testServos();
testServoRangeError();//moves horz servo through 90,0,90,180,90,0,90 degrees, vert servo through 90,180,90 degrees
                       // adjust mapping in subroutine to correct for servo error.

decode.begin();//start telemetry decoder

//get tracker location details
decode.plainReadData();//get telemetry
decode.displayLatLon();//decode lat/lon
decode.displayAltitude();//decode altitude
decode.displayGpsCog();//decode bearing
decode.beginTracker();//start tracker
debug(F("Tracker latitude "));
debuglnArg(decode.trackerLat,7);
debug(F("Tracker longitude "));
debuglnArg(decode.trackerLon,7);
debug(F("Tracker bearing "));
debuglnArg(decode.trackerBearing,7);


}
//----end setup----

void loop()  

  {  
  debugln(F("**********************"));
  decode.plainReadData();//get telemetry
  decode.displayLatLon();//decode lat/lon
  decode.displayAltitude();//decode altitude
  decode.getQuadLocation();//get bearing of quad relative to tracker
  debug(F("Bearing to quad from tracker(degrees) "));
  debugln(decode.bearingToQuad);
  debug(F("Distance to quad from tracker(m) "));
  debugln(decode.distanceToQuad);
  debug(F("Altitude of quad(m) "));
  debugln(decode.decodedAltitude);
  debug(F("Angular height to quad from tracker(degrees) "));
  debugln(decode.angularHeightToQuad);
  driveTracker();//move servos to track quad
  } 

//--Loop Ends---

//************Functions*************

//***********track quad driver**********

void driveTracker()// drive servos
  {
    //do horizontal tracking stuff
    servoAngleToQuad = getServoHorzAngle();
    if(decode.distanceToQuad <= 3.0)// if quad near tracker set servoHorz to home;
      {
        servoAngleToQuad = SERVO_HORZ_HOME;
      }
    debug(F("Tracker bearing "));
    debuglnArg(decode.trackerBearing,7);
    debug(F("Bearing to quad from tracker) "));
    debugln(decode.bearingToQuad);
    debug(F("servoHorz angle to quad "));
    debugln(servoAngleToQuad);
    if(servoAngleToQuad > SERVO_HORZ_MAX)
      {
        servoAngleToQuad = SERVO_HORZ_MAX;
      }
    if(servoAngleToQuad < SERVO_HORZ_MIN)
      {
        servoAngleToQuad = SERVO_HORZ_MIN;
      }
    servoHorzMove(oldServoAngleToQuad, servoAngleToQuad, SERVO_DELAY);
    debug(F("modified servoHorz angle to quad "));
    debugln(servoAngleToQuad);
    oldServoAngleToQuad = servoAngleToQuad;

    //vertical tracking stuff
    newAngularHeightToQuad = SERVO_VERT_HOME + decode.angularHeightToQuad;
    if(decode.distanceToQuad <= 3.0)// if quad near tracker set servo angular height to SERVO_VERT_HOME;
      {
        newAngularHeightToQuad = SERVO_VERT_HOME;
      }
    if(newAngularHeightToQuad > SERVO_VERT_MAX)
      {
        newAngularHeightToQuad = SERVO_VERT_MAX;
      }
    if(newAngularHeightToQuad < SERVO_VERT_MIN)
      {
        newAngularHeightToQuad = SERVO_VERT_MIN;
      }
    debug(F("Quad altitude "));
    debugln(decode.decodedAltitude);
    debug(F("distance to quad "));
    debugln(decode.distanceToQuad);
    debug(F("ServoVert angle to quad "));  
    debugln(newAngularHeightToQuad);
    debugln();
    servoVertMove(oldAngularHeightToQuad, newAngularHeightToQuad, SERVO_DELAY);
    oldAngularHeightToQuad = newAngularHeightToQuad;
  }

//***********servos to home position***********
 
void servosToHome()
  {
    servoHorzMove(servoHorz.read(),SERVO_HORZ_HOME, SERVO_SLOW_DELAY);
    delay(SERVO_DELAY);
    servoVertMove(servoVert.read(),SERVO_VERT_HOME, SERVO_SLOW_DELAY);
    delay(SERVO_DELAY);
  }

  //***********servo horz move************

  void servoHorzMove(int from, int to, int servoDelay)
    {
     if(to != from)// do nothing of to and from are the same
     {
      if(from < to)
        {
          from = correctHorzRangeClock(from);
          to = correctHorzRangeClock(to);
          Serial.print("clockwise ");
          Serial.println(to);
          Serial.println(from);
        }
      else
        {
          from = correctHorzRangeAnticlock(from);
          to = correctHorzRangeAnticlock(to);
          Serial.print("anticlock ");
          Serial.println(to);
          Serial.println(from);
        }
      if(to > SERVO_HORZ_MAX)
      {
        to = SERVO_HORZ_MAX;
      }
    if(to < SERVO_HORZ_MIN)
      {
        to = SERVO_VERT_MIN;
      }
      if(from > SERVO_HORZ_MAX)
      {
        from = SERVO_HORZ_MAX;
      }
    if(from < SERVO_HORZ_MIN)
      {
        from = SERVO_VERT_MIN;
      }
      from = ROTATE - from;//correct for anticlock rotation and servo arm position error
      to = ROTATE - to;
      if(from < to)
        {
          for(int n = from; n <= to; n++)
            {
              servoHorz.write(n);
              delay(servoDelay);
            }
        }
      else
        {
          for(int n = from; n >= to; n--)
            {
              servoHorz.write(n);
              delay(servoDelay);
            }
        }
     }//exit if to = from   
    }



//***********servo vert move***********

    void servoVertMove(int from, int to, int servoDelay)
    {
     if(to != from)//only move if needed
     {
      if(from < to)
        {
          from = correctVertRangeClock(from);
          to = correctVertRangeClock(to);
        }
      else
        {
          from = correctVertRangeAnticlock(from);
          to = correctVertRangeAnticlock(to);
        }
      if(to > SERVO_VERT_MAX)
      {
        to = SERVO_VERT_MAX;
      }
    if(to < SERVO_VERT_MIN)
      {
        to = SERVO_VERT_MIN;
      }
      if(from > SERVO_VERT_MAX)
      {
        from = SERVO_VERT_MAX;
      }
    if(from < SERVO_VERT_MIN)
      {
        from = SERVO_VERT_MIN;
      }
      from = ROTATE - from;//correct for servo arm position
      to = ROTATE - to;
      if(from < to)
        {
          for(int n = from; n <= to; n++)
            {
              servoVert.write(n);
              delay(servoDelay);
            }
        }
      else
        {
          for(int n = from; n >= to; n--)
            {
              servoVert.write(n);
              delay(servoDelay);
            }
        }
     }
    }

    //************test servos***********

    void testServos()
      {
        servoHorzMove(servoHorz.read(),SERVO_HORZ_MIN, SERVO_SLOW_DELAY );
        servoHorzMove(SERVO_HORZ_MIN,SERVO_HORZ_MAX, SERVO_SLOW_DELAY );
        servoHorzMove(SERVO_HORZ_MAX,SERVO_HORZ_HOME, SERVO_SLOW_DELAY );
        servoVertMove(servoVert.read(),SERVO_VERT_MIN, SERVO_SLOW_DELAY );
        servoVertMove(SERVO_VERT_MIN,SERVO_VERT_MAX, SERVO_SLOW_DELAY );
        servoVertMove(SERVO_VERT_MAX,SERVO_VERT_HOME, SERVO_SLOW_DELAY );
      }

    //*******calculates servoHorz angle for quad direction*******

    int getServoHorzAngle()
      {
        int quadrant = 0;;
        int angle = 90;
        //get quadrant that tracker is using
        if((decode.trackerBearing >= 0) && (decode.trackerBearing <=90))
            {
              quadrant = 1;
            }
        else if((decode.trackerBearing > 90) && (decode.trackerBearing <=180))
            {
              quadrant = 2;
            }
        else if((decode.trackerBearing > 180) && (decode.trackerBearing <=270))
            {
              quadrant = 3;
            }
        else if((decode.trackerBearing > 270) && (decode.trackerBearing <360))
            {
              quadrant = 4;
            }
            
        switch(quadrant)// find quads quadrant and calculate angle for servo
          {
            case 1:
                if((decode.bearingToQuad >= 0) && (decode.bearingToQuad <= decode.trackerBearing + 90))//get quad quadrant
                  {
                    angle = SERVO_HORZ_HOME + decode.bearingToQuad - decode.trackerBearing;
                    debugln(F("Tracker quadrant 1, Quad bearing quadrant 1"));
                   
                  }
                else if((decode.bearingToQuad < 360) && (decode.bearingToQuad >= decode.trackerBearing +270))
                  {
                    angle = SERVO_HORZ_HOME + decode.bearingToQuad - decode.trackerBearing - 360;
                    debugln(F("Tracker quadrant 1, Quad bearing quadrant 4"));
                  }
                break;
            case 2:
                if((decode.bearingToQuad >= decode.trackerBearing - 90) && (decode.bearingToQuad <= decode.trackerBearing + 90))//get quad quadrant
                  {
                    angle = SERVO_HORZ_HOME + decode.bearingToQuad - decode.trackerBearing;
                    debugln(F("Tracker quadrant 2, Quad bearing quadrant 2 or 3"));
                  }
                break;
            case 3:
                if((decode.bearingToQuad >= decode.trackerBearing - 90) && (decode.bearingToQuad <= decode.trackerBearing + 90))//get quad quadrant
                  {
                    angle = SERVO_HORZ_HOME + decode.bearingToQuad - decode.trackerBearing;
                    debugln(F("Tracker quadrant 3, Quad bearing quadrant 2 or 3"));
                  }
                break;
            case 4:
                if((decode.bearingToQuad >= decode.trackerBearing - 90) && (decode.bearingToQuad < 360))//get quad quadrant
                  {
                    angle = SERVO_HORZ_HOME + decode.bearingToQuad - decode.trackerBearing;
                    debugln(F("Tracker quadrant 4, Quad bearing quadrant 4"));
                  }
                  else if((decode.bearingToQuad >= 0) && (decode.bearingToQuad <= decode.trackerBearing + 90 - 360))//get quad quadrant
                  {
                    angle = SERVO_HORZ_HOME + decode.bearingToQuad - decode.trackerBearing + 360;
                    debugln(F("Tracker quadrant 4, Quad bearing quadrant 1"));
                  }
                  break;
             case 0:
                debugln(F("calculate servo angle--error"));
                break;
          }
          return angle;
      }

      //******test******

      //***********test****************
  void doTest1()
  {
    decode.trackerLat = 53.8736437;//start from here
    decode.trackerLon = -1.7354171;
    decode.trackerBearing = 0.00;//tracker pointing north
    decode.decodedAltitude = 0.0;
    decode.decodedLat = decode.trackerLat;
    decode.decodedLon = decode.trackerLon;
    debugln();
    for(int n = 0; n <= 50; n++)//fly north and gain height
    {
      decode.decodedLat = decode.decodedLat + 0.00020;
      //decode.decodedLon = decode.decodedLon + 0.00020;
      decode.decodedAltitude = decode.decodedAltitude + 20;
      decode.getQuadLocation();
      driveTracker();
      //printTest();
      //delay(250);
    }
    debugln();
    for(int n = 0; n <= 50; n++)//fly west and gain height
    {
      //decode.decodedLat = decode.decodedLat - 0.00020;
      decode.decodedLon = decode.decodedLon - 0.00020;
      decode.decodedAltitude = decode.decodedAltitude + 20;
      decode.getQuadLocation();
      driveTracker();
      //printTest();
      //delay(250);
    }
    debugln();
    for(int n = 0; n <= 100; n++)//fly east at same height
    {
      //decode.decodedLat = decode.decodedLat - 0.00020;
      decode.decodedLon = decode.decodedLon + 0.00020;
      //decode.decodedAltitude = decode.decodedAltitude + 1;
      decode.getQuadLocation();
      driveTracker();
      //printTest();
      //delay(250);
    }
    debugln();
    for(int n = 0; n <= 50; n++)//fly west at same height
    {
      //decode.decodedLat = decode.decodedLat + 0.00020;
      decode.decodedLon = decode.decodedLon - 0.00020;
      //decode.decodedAltitude = decode.decodedAltitude + 1;
      decode.getQuadLocation();
      driveTracker();
      //printTest();
      //delay(250);
    }
    debugln();
    for(int n = 0; n <= 50; n++)//come home
    {
      decode.decodedLat = decode.decodedLat - 0.00020;
      //decode.decodedLon = decode.decodedLon + 0.00020;
      decode.decodedAltitude = decode.decodedAltitude - 20;
      decode.getQuadLocation();
      driveTracker();
      //printTest();
      //delay(250);
    }
    //servoHorzMove(servoAngleToQuad,SERVO_HORZ_HOME );//home servos
    //servoVertMove(decode.angularHeightToQuad,SERVO_VERT_HOME );
    servosToHome();
  }
//**********print test***********
void printTest()
  {
    debug(F("Quad distance "));
    debugln(decode.distanceToQuad);
    debug(F("Quad altitude "));
    debugln(decode.decodedAltitude);
    debug(F("Quad bearing "));
    debugln(decode.bearingToQuad);
    debug(F("Quad angular height "));
    debugln(decode.angularHeightToQuad);
  }

//*****correct Horz servo range error*****

  int correctHorzRangeClock(int angle)
    {
      return map(angle,0,180, MAP_0_DEGREES_HORZ_CLOCK, MAP_180_DEGREES_HORZ_CLOCK);
    }

  int correctHorzRangeAnticlock(int angle)
    {
      return map(angle,0,180, MAP_0_DEGREES_HORZ_ANTICLOCK, MAP_180_DEGREES_HORZ_ANTICLOCK);
    }


//*****correct Vert servo range error*****

  int correctVertRangeClock(int angle)
    {
      return map(angle,90,180, MAP_90_DEGREES_VERT_CLOCK, MAP_180_DEGREES_VERT_CLOCK);
    }

  int correctVertRangeAnticlock(int angle)
    {
      return map(angle,90,180, MAP_90_DEGREES_VERT_ANTICLOCK, MAP_180_DEGREES_VERT_ANTICLOCK);
    }
    
  //*****test horz angular error***

  void testServoRangeError()
    {
      servoHorzMove(90, 0,SERVO_SLOW_DELAY);
      delay(500);
      servoHorzMove(0, 90,SERVO_SLOW_DELAY);
      delay(500);
      servoHorzMove(90, 180,SERVO_SLOW_DELAY);
      delay(500);
      servoHorzMove(180, 90,SERVO_SLOW_DELAY);
      delay(500);
      servoHorzMove(90, 0,SERVO_SLOW_DELAY);
      delay(500);
      servoHorzMove(0, 90,SERVO_SLOW_DELAY);
      servoVertMove(90, 180,SERVO_SLOW_DELAY);
      delay(500);
      servoVertMove(180, 90,SERVO_SLOW_DELAY);
      delay(500);
      servoVertMove(90, 95,SERVO_SLOW_DELAY);
      delay(500);
    }
  
