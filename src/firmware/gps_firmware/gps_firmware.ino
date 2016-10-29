/*
  Create a new folder named "Adafruit_GPS" in Arduino/Library then clone the library
  GPS Library at: https://github.com/adafruit/Adafruit_GPS
  Full guide: https://learn.adafruit.com/adafruit-ultimate-gps/arduino-wiring
  In Addition, download compass libraries: https://learn.adafruit.com/adafruit-hmc5883l-breakout-triple-axis-magnetometer-compass-sensor/wiring-and-test
  -Nick Wu
*/
//Data format: D(lat),(long),(fix),(x),(y),(z),(headingDegrees)
// ^ things within brackets are variables
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_GPS.h>

//CHANGE THIS CONSTANT
const float DECLINATION_ANGLE = 0.2793; //Find declination here: http://www.magnetic-declination.com/
#define GPSECHO  false

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

boolean usingInterrupt = false; //keeps track of using interrupt, off by default!
void useInterrupt(boolean);

void setup() {
  Serial.begin(115200);
  gps_setup();
  //compass_setup();
  Serial.println("Setup finished");
}

void loop() {
  gps_check_new_data();
  sensors_event_t event;
  Serial.println("reading");
  //mag.getEvent(&event);
  char input = Serial.read();
  if (input == 'I') {
    send_gps(false);
  }
  else if (input == 'D') {
    send_gps(true);
    send_compass(event);
  }
  else {
    Serial.println("-1");
  }
  Serial.read();
  Serial.flush();
}
void compass_setup() {
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    while (1) {
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      delay(2000);
    }
  }
  sensor_t sensor;
  mag.getSensor(&sensor);
}
void send_compass(sensors_event_t event) {
  //HEADING INFO
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  heading += DECLINATION_ANGLE;
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;
  float headingDegrees = heading * 180 / M_PI;
  //C(x),(y),(z)
  //Serial.print(","); Serial.print(event.magnetic.x);
  //Serial.print(","); Serial.print(event.magnetic.y);
  //Serial.print(","); Serial.print(event.magnetic.z);
  Serial.print(","); 
  Serial.print(headingDegrees);Serial.println(",");
  //Serial.print(",");
}
void gps_setup() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  //GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
}

void gps_check_new_data() {
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
}
void send_gps(boolean data) {
  //G(long),(lat),(fix)
  if (data) {
    Serial.print("GPS ");
    Serial.print("D,"); Serial.print(GPS.latitudeDegrees, 6);
    Serial.print(","); Serial.print(GPS.longitudeDegrees, 6);
    Serial.print(","); Serial.print((int)GPS.fix);
  } else {
    Serial.println("GPS");
  }
}
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
