#include <SPI.h>
#include "CN0398.h"
#include "Communication.h"

void setup() {
  Serial.begin(9600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3); //CPHA = CPOL = 1    MODE = 3

  pinMode(CS_PIN, OUTPUT);

  CN0398_setup();
  CN0398_init();
  delay(500);
  Serial.println(F("Initialization complete!\n"));

  Serial.println(F("Do you want to perform pH calibration [y/N]?"));
  char response = Serial.read();
  Serial.println();

  if (response == 'y' || response == 'Y') {
    CN0398_calibrate_ph();
  } else {
    CN0398_set_use_nernst(true);
    Serial.println("Do you want to load default calibration?[y/N]. If not[N], the Nernst equation will be used.");
    response = Serial.read();
    if (response == 'y' || response == 'Y') {
      CN0398_set_use_nernst(false);
    }
  }
  Serial.println();
}

void loop() {

  delay(DISPLAY_REFRESH);
  CN0398_set_data();
  CN0398_display_data();
}
