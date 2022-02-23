/**
*   @file     CN0398.cpp
*   @brief    Application file for the CN0398 project
*   @author   Analog Devices Inc.
*
********************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
********************************************************************************/
#include "CN0398.h"
#include "Arduino.h"

#include "AD7124.h"

#include "Communication.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define RREF (5000.0)
#define TEMP_GAIN (16.0)
#define PT1000_RESISTANCE_TO_TEMP(x) ((x - 1000.0) / (0.385))
#define _2_23 (1 << 23)

#define ms_delay (1)

int32_t adcValue[3];
float temperature, pH, voltage[2], moisture;

bool use_nernst = false;
float offset_voltage;
float default_calibration_ph[2][2] = {{4, 0.169534}, {10, -0.134135}};
float calibration_ph[2][2];
uint8_t solution0, solution1;

void CN0398_calibrate_ph_pt0(float temperature) {
    CN0398_set_digital_output(CHANNEL_P2, true);
    int32_t data = CN0398_read_channel(PH_CHANNEL);

    float volt = CN0398_data_to_voltage_bipolar(data, 1, 3.3);

    if (temperature < 0) {
        calibration_ph[0][0] = ph_temp_lut[solution0][0];
    } else {
        for (uint8_t i = 1; i < NUMBER_OF_TEMPERATURE_ENTRIES; i++) {
            if (temperature > ph_temperatures[i - 1] &&
                temperature <= ph_temperatures[i]) {
                calibration_ph[0][0] = ph_temp_lut[solution0][i];
                break;
            }
        }
    }

    calibration_ph[0][1] = volt;
    CN0398_set_digital_output(CHANNEL_P2, false);

    printf("\tCalibration solution1 pH = %.3f with sensor voltage of %fV\n",
           calibration_ph[0][0],
           volt);
    printf("\n");
}

void CN0398_calibrate_ph_pt1(float temperature) {
    CN0398_set_digital_output(CHANNEL_P2, true);
    int32_t data = CN0398_read_channel(PH_CHANNEL);

    float volt = CN0398_data_to_voltage_bipolar(data, 1, 3.3);

    if (temperature < 0) {
        calibration_ph[1][0] = ph_temp_lut[solution1][0];
    } else {
        for (uint8_t i = 1; i < NUMBER_OF_TEMPERATURE_ENTRIES; i++) {
            if (temperature > ph_temperatures[i - 1] &&
                temperature <= ph_temperatures[i]) {
                calibration_ph[1][0] = ph_temp_lut[solution1][i];
                break;
            }
        }
    }

    calibration_ph[1][1] = volt;
    CN0398_set_digital_output(CHANNEL_P2, false);

    printf("\tCalibration solution2 pH = %.3f with sensor voltage of %fV\n",
           calibration_ph[1][0],
           volt);
    printf("\n");
}

void CN0398_calibrate_ph_offset() {
    CN0398_set_digital_output(CHANNEL_P2, true);

    int32_t data = CN0398_read_channel(PH_CHANNEL);

    float volt     = CN0398_data_to_voltage_bipolar(data, 1, 3.3);
    offset_voltage = volt;
    printf("\tOffset voltage is %fV\n", volt);
    printf("\n");

    CN0398_set_digital_output(CHANNEL_P2, false);
}

float CN0398_read_rtd() {
    float temperature = 0;

    int32_t data;

    adcValue[RTD_CHANNEL] = data = CN0398_read_channel(RTD_CHANNEL);

    float resistance = ((data - _2_23) * RREF) / (TEMP_GAIN * _2_23);

#ifdef USE_LINEAR_TEMP_EQ
    temperature = PT100_RESISTANCE_TO_TEMP(resistance);
#else

#define A (3.9083 * pow(10, -3))
#define B (-5.775 * pow(10, -7))
    if (resistance < R0)
        temperature = -242.02 + 2.228 * resistance +
                      (2.5859 * pow(10, -3)) * pow(resistance, 2) -
                      (48260.0 * pow(10, -6)) * pow(resistance, 3) -
                      (2.8183 * pow(10, -3)) * pow(resistance, 4) +
                      (1.5243 * pow(10, -10)) * pow(resistance, 5);
    else
        temperature =
        ((-A + sqrt(double(pow(A, 2) - 4 * B * (1 - resistance / R0)))) /
         (2 * B));
#endif
    return temperature;
}

float CN0398_read_ph(float temperature) {
    float ph = 0;

#ifdef PH_SENSOR_PRESENT
    int32_t data;

    CN0398_set_digital_output(CHANNEL_P2, true);

    adcValue[PH_CHANNEL] = data = CN0398_read_channel(PH_CHANNEL);

    float volt = voltage[PH_CHANNEL - 1] =
    CN0398_data_to_voltage_bipolar(data, 1, 3.3);

    if (use_nernst) {
        ph = PH_ISO - ((volt - ZERO_POINT_TOLERANCE) /
                       ((2.303 * AVOGADRO * (temperature + KELVIN_OFFSET)) /
                        FARADAY_CONSTANT));
    } else {
        float m = (calibration_ph[1][0] - calibration_ph[0][0]) /
                  (calibration_ph[1][1] - calibration_ph[0][1]);
        ph = m * (volt - calibration_ph[1][1] - offset_voltage) +
             calibration_ph[1][0];
    }

    CN0398_set_digital_output(CHANNEL_P2, false);

#endif
    return ph;
}

float CN0398_read_moisture() {
    float moisture = 0;
#ifdef MOISTURE_SENSOR_PRESENT

    digitalWrite(ADP7118_PIN, HIGH);
    CN0398_set_digital_output(CHANNEL_P3, true);

    delay(SENSOR_SETTLING_TIME);
    int32_t data = adcValue[MOISTURE_CHANNEL] =
    CN0398_read_channel(MOISTURE_CHANNEL);

    digitalWrite(ADP7118_PIN, LOW);

    float volt = voltage[MOISTURE_CHANNEL - 1] =
    CN0398_data_to_voltage_bipolar(data, 1, 3.3);

#ifdef USE_MANUFACTURER_MOISTURE_EQ
    if (volt <= 1.1) {
        moisture = 10 * volt - 1;
    } else if (volt > 1.1 && volt <= 1.3) {
        moisture = 25 * volt - 17.5;
    } else if (volt > 1.3 && volt <= 1.82) {
        moisture = 48.08 * volt - 47.5;
    } else if (volt > 1.82) {
        moisture = 26.32 * volt - 7.89;
    }
#else
    moisture = -1.18467 + 21.5371 * volt - 110.996 * (pow(volt, 2)) +
               397.025 * (pow(volt, 3)) - 666.986 * (pow(volt, 4)) +
               569.236 * (pow(volt, 5)) - 246.005 * (pow(volt, 6)) +
               49.4867 * (pow(volt, 7)) - 3.37077 * (pow(volt, 8));
#endif
    if (moisture > 100) moisture = 100;
    if (moisture < 0) moisture   = 0;
#endif

    CN0398_set_digital_output(CHANNEL_P3, false);

    return moisture;
}

void CN0398_set_digital_output(uint8_t p, bool state) {
    enum ad7124_registers regNr = static_cast<enum ad7124_registers>(
    AD7124_Channel_0); // Select _ADC_Control register
    uint32_t setValue = AD7124_ReadDeviceRegister(regNr);
    if (state)
        setValue |= ((AD7124_8_IO_CTRL1_REG_GPIO_DAT1) << p);
    else
        setValue &= (~(AD7124_8_IO_CTRL1_REG_GPIO_DAT1 << p));
    AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC
    delay(ms_delay);
}

int32_t CN0398_read_channel(int ch) {
    int32_t data;

    CN0398_enable_channel(ch);

    convFlag = 1;
    digitalWrite(CS_PIN, LOW);
    CN0398_start_single_conversion();

    if (AD7124_WaitForConvReady(10000) == -3) {
        Serial.println("TIMEOUT");
        return 0;
    }

    delay(100);

    AD7124_ReadData(&data);
    convFlag = 0;
    digitalWrite(CS_PIN, HIGH);

    CN0398_disable_channel(ch);
    return data;
}

float CN0398_data_to_voltage_bipolar(uint32_t data, uint8_t gain, float VREF) {
    data = data - _2_23;
    return ((data / ((_2_23 - 1) / 2)) - 1) * (VREF / gain);
}

float CN0398_data_to_voltage(uint32_t data, uint8_t gain, float VREF) {
    data = data - _2_23;
    return (data / ((_2_23 - 1) / 2)) * (VREF / gain);
}

void CN0398_enable_channel(int channel) {
    enum ad7124_registers regNr = static_cast<enum ad7124_registers>(
    AD7124_Channel_0 + channel); // Select _ADC_Control register
    uint32_t setValue = AD7124_ReadDeviceRegister(regNr);
    setValue |= (uint32_t) AD7124_CH_MAP_REG_CH_ENABLE; // Enable channel0
    setValue &= 0xFFFF;
    AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC
    delay(ms_delay);
}

void CN0398_disable_channel(int channel) {
    enum ad7124_registers regNr = static_cast<enum ad7124_registers>(
    AD7124_Channel_0 + channel); // Select _ADC_Control register
    uint32_t setValue = AD7124_ReadDeviceRegister(regNr);
    setValue &= (~(uint32_t) AD7124_CH_MAP_REG_CH_ENABLE); // Enable channel0
    setValue &= 0xFFFF;
    AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC
    delay(ms_delay);
}

void CN0398_enable_current_source0(int current_source_channel) {
    enum ad7124_registers regNr = AD7124_IOCon1; // Select _ADC_Control register
    uint32_t setValue           = AD7124_ReadDeviceRegister(regNr);
    setValue &= ~(AD7124_IO_CTRL1_REG_IOUT_CH0(0xF));
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH0(2 * current_source_channel +
                                             1); // set IOUT0 current
    setValue &= 0xFFFFFF;
    AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC
    delay(ms_delay);
}

void CN0398_enable_current_source1(int current_source_channel) {
    enum ad7124_registers regNr = AD7124_IOCon1; // Select _ADC_Control register
    uint32_t setValue           = AD7124_ReadDeviceRegister(regNr);
    setValue &= ~(AD7124_IO_CTRL1_REG_IOUT_CH1(0xF));
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH1(2 * current_source_channel +
                                             1); // set IOUT0 current
    setValue &= 0xFFFFFF;
    AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC
    delay(ms_delay);
}

void CN0398_start_single_conversion() {
    enum ad7124_registers regNr =
    AD7124_ADC_Control;                        // Select _ADC_Control register
    AD7124_WriteDeviceRegister(regNr, 0x0584); // Write data to _ADC
}

void CN0398_reset() {
    AD7124_Reset();
    Serial.println(F("Reseted AD7124\n"));
}

void CN0398_setup() {
    AD7124_Setup();
}

void CN0398_init() {
    pinMode(ADP7118_PIN, OUTPUT);
    digitalWrite(ADP7118_PIN, LOW);

    uint32_t setValue;
    int i;
    enum ad7124_registers regNr;

    // Set Config_0 0x19
    regNr    = AD7124_Config_0; // Select Config_0 register
    setValue = AD7124_ReadDeviceRegister(regNr);
    setValue |= AD7124_CFG_REG_BIPOLAR;    // Select bipolar operation
    setValue |= AD7124_CFG_REG_BURNOUT(0); // Burnout current source off
    setValue |= AD7124_CFG_REG_REF_BUFP;
    setValue |= AD7124_CFG_REG_REF_BUFM;
    setValue |= AD7124_CFG_REG_AIN_BUFP;
    setValue |= AD7124_CFG_REG_AINN_BUFM;
    setValue |= AD7124_CFG_REG_REF_SEL(1); // Select REFIN2(+)/REFIN2(-)
    setValue |= AD7124_CFG_REG_PGA(4);
    setValue &= 0xFFFF;
    AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC

    // Set Config_1
    regNr    = AD7124_Config_1;
    setValue = AD7124_ReadDeviceRegister(regNr);
    setValue |= AD7124_CFG_REG_BIPOLAR;    // Select bipolar operation
    setValue |= AD7124_CFG_REG_BURNOUT(0); // Burnout current source off
    setValue |= AD7124_CFG_REG_REF_BUFP;
    setValue |= AD7124_CFG_REG_REF_BUFM;
    setValue |= AD7124_CFG_REG_AIN_BUFP;
    setValue |= AD7124_CFG_REG_AINN_BUFM;
    setValue |= AD7124_CFG_REG_REF_SEL(0);
    setValue |= AD7124_CFG_REG_PGA(0);
    setValue &= 0xFFFF;
    AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC

    int ainp_map[]                    = {9, 6, 8};
    int ainm_map[]                    = {10, 7, 19};
    enum ad7124_registers registers[] = {
    static_cast<enum ad7124_registers>(AD7124_Channel_0),
    static_cast<enum ad7124_registers>(AD7124_Channel_1),
    static_cast<enum ad7124_registers>(AD7124_Channel_2)};
    for (i = 0; i < 3; i++) {
        regNr    = registers[i];
        setValue = 0;
        setValue |= AD7124_CH_MAP_REG_SETUP(
        i > 0); // Select setup0 if i == 0 and setup1 otherwise
        setValue |= AD7124_CH_MAP_REG_AINP(ainp_map[i]);
        setValue |= AD7124_CH_MAP_REG_AINM(ainm_map[i]);
        setValue &= (~(uint32_t) AD7124_CH_MAP_REG_CH_ENABLE); // Disable
                                                               // channel
        setValue &= 0xFFFF;
        AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC
        delay(ms_delay);
    }

    // Set IO_Control_1 0x03
    regNr    = AD7124_IOCon1; // Select IO_Control_1 register
    setValue = 0;
    setValue |=
    AD7124_8_IO_CTRL1_REG_GPIO_CTRL2; // enable AIN3 as digital output
    setValue |=
    AD7124_8_IO_CTRL1_REG_GPIO_CTRL3; // enable AIN4 as digital output
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH0(11); // source ain11
    setValue |= AD7124_IO_CTRL1_REG_IOUT_CH1(12); // source ain12
    setValue |= AD7124_IO_CTRL1_REG_IOUT0(0x4);   // set IOUT0 current to 500uA
    setValue |= AD7124_IO_CTRL1_REG_IOUT1(0x4);   // set IOUT1 current to 500uA
    setValue &= 0xFFFFFF;
    AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC

    regNr    = AD7124_IOCon2; // Select IO_Control_2 register
    setValue = 0;
    setValue |=
    AD7124_8_IO_CTRL2_REG_GPIO_VBIAS7; // enable bias voltage on AIN7
    setValue &= 0xFFFFFF;
    AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC

    // Set _ADC_Control 0x01
    regNr    = AD7124_ADC_Control; // Select _ADC_Control register
    setValue = AD7124_ReadDeviceRegister(regNr);
    setValue |= AD7124_ADC_CTRL_REG_DATA_STATUS; // set data status bit in order
                                                 // to check on which channel
                                                 // the conversion is
    setValue |= AD7124_ADC_CTRL_REG_REF_EN;
    setValue |= AD7124_ADC_CTRL_REG_POWER_MODE(0); // set low power mode
    setValue &= 0xFFFF;
    AD7124_WriteDeviceRegister(regNr, setValue); // Write data to _ADC
    delay(ms_delay);
}

void CN0398_set_data(void) {
    temperature = CN0398_read_rtd();
    pH          = CN0398_read_ph(temperature);
    moisture    = CN0398_read_moisture();
}
void CN0398_display_data(void) {
    Serial.print(F("Temperature = "));
    Serial.print(temperature);
    Serial.println(F("°C"));

    Serial.print(F("pH = "));
    Serial.print(pH);
    Serial.println();

    Serial.print(F("Moisture = "));
    Serial.print(moisture);
    Serial.println(F("%"));
    Serial.println("");
    Serial.println("");
}

void CN0398_calibrate_ph(void) {
    Serial.println(F("Do you want to calibrate offset voltage [y/N]?"));
    char response = Serial.read();
    Serial.println();

    if (response == 'y' || response == 'Y') {
        Serial.println(F(
        "Calibration step 0. Short the pH probe and press any key to "
        "calibrate."));
        response = Serial.read();
        CN0398_calibrate_ph_offset();
    }
    CN0398_print_calibration_solutions();

    bool response_ok = false;
    while (!response_ok) {
        Serial.println(
        F("Input calibration solution used for first step [1-9][a-e]:"));
        response = Serial.read();
        if (isDigit(response)) {
            response_ok = true;
            solution0   = response - '0';
        } else if (response >= 'A' && response <= 'E') {
            response_ok = true;
            solution0   = response - 'A' + 10;
        } else if (response >= 'a' && response <= 'e') {
            response_ok = true;
            solution0   = response - 'a' + 10;
        } else {
            response_ok = false;
        }
    }
    Serial.print(F("\t"));
    Serial.print(solutions[solution0]);
    Serial.print(F(" solution selected. Solution pH at 25°C = "));
    Serial.println(ph_temp_lut[solution0][11], 3);
    Serial.println();

    float temperature = CN0398_read_rtd();
    Serial.println(F(
    "Calibration step 1. Place pH probe in first calibration solution and "
    "press any key to start calibration."));
    response = Serial.read();
    CN0398_calibrate_ph_pt0(temperature);

    response_ok = false;
    while (!response_ok) {
        Serial.println(
        F("Input calibration solution used for second step [1-9][a-e]:"));
        response = Serial.read();
        if (isDigit(response)) {
            response_ok = true;
            solution1   = response - '0';
        } else if (response >= 'A' && response <= 'E') {
            response_ok = true;
            solution1   = response - 'A' + 10;
        } else if (response >= 'a' && response <= 'e') {
            response_ok = true;
            solution1   = response - 'a' + 10;
        } else {
            response_ok = false;
        }
    }
    Serial.print(F("\t"));
    Serial.print(solutions[solution1]);
    Serial.print(F(" solution selected. Solution pH at 25°C = "));
    Serial.println(ph_temp_lut[solution1][11], 3);
    Serial.println();

    Serial.println(F(
    "Calibration step 2. Place pH probe in second calibration solution and "
    "press any key to start calibration."));
    response = Serial.read();
    CN0398_calibrate_ph_pt1(temperature);
    Serial.println();
    Serial.println();
}

void CN0398_print_calibration_solutions(void) {
    Serial.println(
    F("Calibration solutions available for two point calibration:"));
    for (int i = 0; i < NUMBER_OF_SOLUTIONS; ++i) {
        Serial.print(i, HEX);
        Serial.println(solutions[i]);
    }
    Serial.println();
}

void CN0398_set_use_nernst(bool state) {
    use_nernst = state;
}
