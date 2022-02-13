#include <SPI.h>
#include "CN0398.h"
#include "Communication.h"

void setup() {
  Serial.begin(9600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3); //CPHA = CPOL = 1    MODE = 3
  delay(1000);

  pinMode(CS_PIN, OUTPUT);

   CN0398_init();

#if(USE_RTD_CALIBRATION == YES)
   CN0398_calibration(RTD_CHANNEL);
   Serial.println(F("RTD channel calibration completed!"));
#else
   Serial.println(F("Calibration for RTD channel is disabled."));
#endif

#if(USE_TH_CALIBRATION == YES)
   CN0398_calibration(TH_CHANNEL);
   Serial.println(F("TC channel calibration completed!"));
#else
   Serial.println(F("Calibration for TC channel is disabled."));
#endif
   
}

void loop() {
        
        delay(DISPLAY_REFRESH);
        CN0398_set_data();
        Serial.println("test");
        CN0398_display_data();
}
