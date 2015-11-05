/*
Version 0.1
NazaToFrsky
based on: 
NAZADECODER from http://www.rcgroups.com/forums/showthread.php?t=1995704 Added some snipets from Airmamaf (Bagaosd https://code.google.com/p/bagaosd/wiki/NazaConnector)
and
APM MavLink to FrSky X8R S.Port converter from http://diydrones.ning.com/forum/topics/amp-to-frsky-x8r-sport-converter?commentId=705844%3AComment%3A1539864&xg_source=activity

NOTE: Some Parts of the code are NOT for commercial use (NAZADECODER)!!

*/

#include "FrSkySPort.h"
#include "NazaDecoderLib.h"
#include "config.h"
#include "ADC.h"

ADC *adc = new ADC();

#define A38 38

int ledpin = 13;
int timer = 0;

//----------------------FrSkySPort------------------------------
// Message #24  GPS_RAW_INT 
uint8_t    gps_fixtype = 0;                  //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    sat_visible = 0;           // numbers of visible satelites
// FrSky Taranis uses the first recieved lat/long as homeposition. 
double    latitude = 0;              // 585522540;
double    longitude = 0;            // 162344467;
double    gps_altitude = 0;        // 1000 = 1m
int32_t longitude2 =0;
int32_t latitude2 =0;
int32_t gps_altitude2 = 0;
// Message #74 VFR_HUD 
uint32_t  groundspeed = 0;
uint32_t  heading = 0;
// FrSky Taranis uses the first recieved value after 'PowerOn' or  'Telemetry Reset'  as zero altitude
int32_t    bar_altitude = 0;    // 100 = 1m
int32_t    climb_rate=0;        // 100= 1m/s
// These are special for FrSky
int32_t     gps_status = 0;     // (ap_sat_visible * 10) + ap_fixtype                                             // ex. 83 = 8 sattelites visible, 3D lock 

uint32_t internal_temp=0;   // internal temperature in C * 100

IntervalTimer timer0;

void setup()
{
  ConsoleSerial.begin(9600);
  FrSkySPort_Init();
  NazaSerial.begin(115200);
  pinMode(ledpin, OUTPUT);

  pinMode(A38, INPUT);
    pinMode(A2, INPUT);

  adc->setReference(ADC_REF_1V2);
  adc->setAveraging(32); // set number of averages
  adc->setResolution(16); // set bits of resolution

  timer0.begin(read_temp, 200000 ); // 200ms in us
}

void read_temp()
{
  
  // https://forum.pjrc.com/threads/23580-Teensy-3-Temperature-Calculation
  int a2d = adc->analogRead(38);
  internal_temp = (181964504 - 69971 * a2d) >> 12;
  internal_temp = 25 - (a2d - 38700)/-35.7;;
}

void loop()
{                                                  
  // Process Naza Informations
  if(NazaSerial.available())
  {
    uint8_t decodedMessage = NazaDecoder.decode(NazaSerial.read());
    switch (decodedMessage)
    {
      case NAZA_MESSAGE_GPS:
        gps_fixtype = NazaDecoder.getFixType();                               // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix  
        sat_visible =  NazaDecoder.getNumSat();          // numbers of visible satelites
        gps_status = (sat_visible*10) + gps_fixtype; 
        if(gps_fixtype >= 3)  {
          latitude = NazaDecoder.getLat();
          longitude = NazaDecoder.getLon();
          gps_altitude = NazaDecoder.getAlt();    // 1m =1000
          groundspeed = NazaDecoder.getSpeed();
          climb_rate = NazaDecoder.getClimbSpeed();
        }       
        break;
      case NAZA_MESSAGE_COMPASS:
        heading=NazaDecoder.getHeading();
        break;
    }
  } else {
     read_temp();
  }
  FrSkySPort_Process();               // Process Informations and send it to FrSky

  // DBEUG
if((timer == DEBUG_Timer) && DEBUG)
{
 if(gps_fixtype >= 3 )
 {
  ConsoleSerial.print("Lat: "); ConsoleSerial.print(latitude, 7);
  ConsoleSerial.print(", Lon: "); ConsoleSerial.print(longitude, 7);
  ConsoleSerial.print(", Alt: "); ConsoleSerial.print(gps_altitude, 7);
  ConsoleSerial.print(", Fix: "); ConsoleSerial.print(gps_fixtype);
  ConsoleSerial.print(", Sat: "); ConsoleSerial.println(sat_visible); 
  ConsoleSerial.print("Speed: "); ConsoleSerial.println(groundspeed);
  ConsoleSerial.print("climb_rate: "); ConsoleSerial.println(climb_rate);
 }
 ConsoleSerial.print("Heading: "); ConsoleSerial.println(NazaDecoder.getHeading(), 2);  
 timer=0;
}
else
{
  timer++;
}
} 

void timer0_callback(void) {

    //digitalWriteFast(ledPin+1, HIGH);

    adc->startSingleRead(A38, ADC_0); // also: startSingleDifferential, analogSynchronizedRead, analogSynchronizedReadDifferential

    //digitalWriteFast(ledPin+1, LOW);
    //digitalWriteFast(ledPin+1, !digitalReadFast(ledPin+1));
}

/*
void adc0_isr() {

    uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel

    // add value to correct buffer
    if(pin==A38) {
        
        internal_temp=adc->readSingle();
        
    } else { // clear interrupt anyway
        adc->readSingle();
    }

    // restore ADC config if it was in use before being interrupted by the analog timer
    if (adc->adc0->adcWasInUse) {
        // restore ADC config, and restart conversion
        adc->adc0->loadConfig(&adc->adc0->adc_config);
        // avoid a conversion started by this isr to repeat itself
        adc->adc0->adcWasInUse = false;
    }


    //digitalWriteFast(ledPin+2, !digitalReadFast(ledPin+2));

}*/

