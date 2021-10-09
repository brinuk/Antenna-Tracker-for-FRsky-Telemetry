 /* Author: Brian Lambert */

#ifndef FrSkyR9MSportDecoder_h
#define FrSkyR9MSportDecoder_h

/*-----( Import needed libraries )-----*/
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif



/*-----( Declare Constants and Pin Numbers )-----*/

#define BYTES_PER_FRAME 10
#define FRAMES_TO_READ 40
#define START_FRAME 0x7E
#define R 6371000.00 //radius of the earth
#define PI 3.14159265
#define TORADS PI/180.0
#define TODEGS 180.0/PI

class FrSkyR9MSportDecoder
{
/*-----( Declare Variables and functions )-----*/
  public:
	FrSkyR9MSportDecoder();
	void displayDistanceToHome();
	void displayDateTime();
	void displayAltitude();
	void displayCurrent();
	void displayBattVolts();
	void displayVertSpeed();
	void displayRssi();
	void displayRxBattVolts();
	void displayCellVolts();
	void displayGpsCog();
	void displayGpsSpeed();
	void displayGpsAltitude();
	void displayLatLon();
	void plainReadData();
	void begin();
	void beginTest();
	void printHexData();
	float decodedLat;
	float decodedLon;
	float decodedAltitude;
	float decodedCurrent;
	float decodedBattVolts;
	float decodedVSpeed;
	float decodedRssi;
	float decodedRxVolts;
	float decodedCellVolts;
	float decodedCog;
	float decodedGpsSpeed;
	float decodedGpsAltitude;
	float decodedDistanceToHome;
	void beginTracker();
	float trackerLat;
	float trackerLon;
	float trackerBearing;
	float distanceToQuad;
	float bearingToQuad;
	void getQuadLocation();
	float getAngularHeightToQuad();
	float angularHeightToQuad;
	
	
	

  private:
	bool firstPart = true;
	char str[1];
	char *ptr;
	long ret;
	byte hexData[FRAMES_TO_READ][BYTES_PER_FRAME];
	long num1;
	long num2;
	long num3;
	long num4;
	bool printThis;
	String printData;
	void printLatLonData(int frame);
	void printGpsAltitude(int frame);
	void printGpsSpeed(int frame);
	void printGpsCog(int frame);
	void printCellVolts(int frame);
	void printRxBattVolts(int frame);
	void printRssi(int frame);
	void printVertSpeed(int frame);
	void printBattVolts(int frame);
	void printCurrent(int frame);
	void printAltitude(int frame);
	void printDateTime(int frame);
	void printDistanceToHome(int frame);
	long combineHexNumbers();
	void readRealTerm();
	void decodeLatLon(long data);
	void decodeGpsAltitude(long data);
	void decodeGpsSpeed(long gpsSpeed);
	void decodeGpsCog(long gpsCog);
	void decodeCellVolts(long cVolts);
	void decodeRxBattVolts(long rxVolts);
	void decodeRssi(long rawRssi);
	void decodeVertSpeed(long rawSpeed);
	void decodeBattVolts(long rawBattVolts);
	void decodeCurrent(long rawCurrent);
	void decodeAltitude(long rawAltitude);
	void decodeDateTime(long rawDateTime);
	void decodeDistanceToHome(long rawDistance);
	void resetDecodedData();
	bool crcCheck(int frame);
	float getDistance(float lat1,float long1,float lat2,float long2);
	float getBearing(float lat1,float long1,float lat2,float long2);
	
	
	
	
};

#endif