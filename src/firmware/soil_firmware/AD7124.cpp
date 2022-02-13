/**
*   @file     AD7124.cpp
*   @brief    Source file for AD7124 ADC
*   @author   Analog Devices Inc.
*
* For support please go to:
* Github: https://github.com/analogdevicesinc/mbed-adi
* Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
* Product: http://www.analog.com/ad7124
* More: https://wiki.analog.com/resources/tools-software/mbed-drivers-all

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

#include <stdint.h>

#include "Communication.h"
#include "AD7124.h"

/*! Array holding the info for the ad7124 registers - address, initial value,
    size and access type. */
    ad7124_st_reg ad7124_regs[57] = {
        {0x00, 0x00,   1, 2}, /* AD7124_Status */
        {0x01, 0x0000, 2, 1}, /* AD7124_ADC_Control */
        {0x02, 0x0000, 3, 2}, /* AD7124_Data */
        {0x03, 0x000000, 3, 1}, /* AD7124_IOCon1 */
        {0x04, 0x0000, 2, 1}, /* AD7124_IOCon2 */
        {0x05, 0x12,   1, 2}, /* AD7124_ID */
        {0x06, 0x0000, 3, 2}, /* AD7124_Error */
        {0x07, 0x000040, 3, 1}, /* AD7124_Error_En */
        {0x08, 0x00,   1, 2}, /* AD7124_Mclk_Count */
        {0x09, 0x8001, 2, 1}, /* AD7124_Channel_0 */
        {0x0A, 0x0001, 2, 1}, /* AD7124_Channel_1 */
        {0x0B, 0x0001, 2, 1}, /* AD7124_Channel_2 */
        {0x0C, 0x0001, 2, 1}, /* AD7124_Channel_3 */
        {0x0D, 0x0001, 2, 1}, /* AD7124_Channel_4 */
        {0x0E, 0x0001, 2, 1}, /* AD7124_Channel_5 */
        {0x0F, 0x0001, 2, 1}, /* AD7124_Channel_6 */
        {0x10, 0x0001, 2, 1}, /* AD7124_Channel_7 */
        {0x11, 0x0001, 2, 1}, /* AD7124_Channel_8 */
        {0x12, 0x0001, 2, 1}, /* AD7124_Channel_9 */
        {0x13, 0x0001, 2, 1}, /* AD7124_Channel_10 */
        {0x14, 0x0001, 2, 1}, /* AD7124_Channel_11 */
        {0x15, 0x0001, 2, 1}, /* AD7124_Channel_12 */
        {0x16, 0x0001, 2, 1}, /* AD7124_Channel_13 */
        {0x17, 0x0001, 2, 1}, /* AD7124_Channel_14 */
        {0x18, 0x0001, 2, 1}, /* AD7124_Channel_15 */
        {0x19, 0x0860, 2, 1}, /* AD7124_Config_0 */
        {0x1A, 0x0860, 2, 1}, /* AD7124_Config_1 */
        {0x1B, 0x0860, 2, 1}, /* AD7124_Config_2 */
        {0x1C, 0x0860, 2, 1}, /* AD7124_Config_3 */
        {0x1D, 0x0860, 2, 1}, /* AD7124_Config_4 */
        {0x1E, 0x0860, 2, 1}, /* AD7124_Config_5 */
        {0x1F, 0x0860, 2, 1}, /* AD7124_Config_6 */
        {0x20, 0x0860, 2, 1}, /* AD7124_Config_7 */
        {0x21, 0x060180, 3, 1}, /* AD7124_Filter_0 */
        {0x22, 0x060180, 3, 1}, /* AD7124_Filter_1 */
        {0x23, 0x060180, 3, 1}, /* AD7124_Filter_2 */
        {0x24, 0x060180, 3, 1}, /* AD7124_Filter_3 */
        {0x25, 0x060180, 3, 1}, /* AD7124_Filter_4 */
        {0x26, 0x060180, 3, 1}, /* AD7124_Filter_5 */
        {0x27, 0x060180, 3, 1}, /* AD7124_Filter_6 */
        {0x28, 0x060180, 3, 1}, /* AD7124_Filter_7 */
        {0x29, 0x800000, 3, 1}, /* AD7124_Offset_0 */
        {0x2A, 0x800000, 3, 1}, /* AD7124_Offset_1 */
        {0x2B, 0x800000, 3, 1}, /* AD7124_Offset_2 */
        {0x2C, 0x800000, 3, 1}, /* AD7124_Offset_3 */
        {0x2D, 0x800000, 3, 1}, /* AD7124_Offset_4 */
        {0x2E, 0x800000, 3, 1}, /* AD7124_Offset_5 */
        {0x2F, 0x800000, 3, 1}, /* AD7124_Offset_6 */
        {0x30, 0x800000, 3, 1}, /* AD7124_Offset_7 */
        {0x31, 0x500000, 3, 1}, /* AD7124_Gain_0 */
        {0x32, 0x500000, 3, 1}, /* AD7124_Gain_1 */
        {0x33, 0x500000, 3, 1}, /* AD7124_Gain_2 */
        {0x34, 0x500000, 3, 1}, /* AD7124_Gain_3 */
        {0x35, 0x500000, 3, 1}, /* AD7124_Gain_4 */
        {0x36, 0x500000, 3, 1}, /* AD7124_Gain_5 */
        {0x37, 0x500000, 3, 1}, /* AD7124_Gain_6 */
        {0x38, 0x500000, 3, 1}, /* AD7124_Gain_7 */
    };

ad7124_st_reg *regs = ad7124_regs; // reg map 38 bytes ?
uint8_t useCRC; //
int check_ready; // ?
int spi_rdy_poll_cnt; // timer ?

const static uint8_t _SPI_MODE = 0x03;
const static uint8_t _RESET = 0xFF;
const static uint8_t _DUMMY_BYTE = 0xFF;
const static uint16_t _READ_FLAG = 0x4000;
const static uint8_t _DELAY_TIMING = 0x02;
	
/**
 * @brief AD7790 constructor, sets CS pin and SPI format
 * @param CS - (optional)chip select of the AD7790
 * @param MOSI - (optional)pin of the SPI interface
 * @param MISO - (optional)pin of the SPI interface
 * @param SCK  - (optional)pin of the SPI interface
 */
void AD7124_Init()
{
    regs = ad7124_regs;
    check_ready = 0;
    useCRC = AD7124_DISABLE_CRC;
    spi_rdy_poll_cnt = 25000;
}

/***************************************************************************//**
* @brief Reads the value of the specified register without checking if the
*        device is ready to accept user requests.
*
* @param device - The handler of the instance of the driver.
* @param pReg - Pointer to the register structure holding info about the
*               register to be read. The read value is stored inside the
*               register structure.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_NoCheckReadRegister(ad7124_st_reg* pReg)
{
    int32_t ret       = 0;
    uint8_t _buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t i         = 0;
    uint8_t check8    = 0;
    uint8_t msgBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};


    check8 = useCRC;

    /* Build the Command word */
    _buffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
                AD7124_COMM_REG_RA(pReg->addr);

    /* Read data from the device */
    ret = AD7124_SPI_Read(_buffer,
                   ((useCRC != AD7124_DISABLE_CRC) ? pReg->_size + 1
                    : pReg->_size) + 1);
    if(ret < 0)
        return ret;

    /* Check the CRC */
    if(check8 == AD7124_USE_CRC) {
        msgBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
                    AD7124_COMM_REG_RA(pReg->addr);
        for(i = 1; i < pReg->_size + 2; ++i) {
            msgBuf[i] = _buffer[i];
        }
        check8 = AD7124_ComputeCRC8(msgBuf, pReg->_size + 2);
    }

    if(check8 != 0) {
        /* ReadRegister checksum failed. */
        return COMM_ERR;
    }

    /* Build the result */
    pReg->value = 0;
    for(i = 1; i < pReg->_size + 1; i++) {
        pReg->value <<= 8;
        pReg->value += _buffer[i];
    }

    return ret;
}

/***************************************************************************//**
* @brief Writes the value of the specified register without checking if the
*        device is ready to accept user requests.
*
* @param device - The handler of the instance of the driver.
* @param reg - Register structure holding info about the register to be written
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_NoCheckWriteRegister(ad7124_st_reg reg)
{
    int32_t ret      = 0;
    int32_t regValue = 0;
    uint8_t wrBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t i        = 0;
    uint8_t crc8     = 0;


    /* Build the Command word */
    wrBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR |
               AD7124_COMM_REG_RA(reg.addr);

    /* Fill the write buffer */
    regValue = reg.value;
    for(i = 0; i < reg._size; i++) {
        wrBuf[reg._size - i] = regValue & 0xFF;
        regValue >>= 8;
    }

    /* Compute the CRC */
    if(useCRC != AD7124_DISABLE_CRC) {
        crc8 = AD7124_ComputeCRC8(wrBuf, reg._size + 1);
        wrBuf[reg._size + 1] = crc8;
    }

    /* Write data to the device */
    ret = AD7124_SPI_Write(wrBuf,
                    (useCRC != AD7124_DISABLE_CRC) ? reg._size + 2
                    : reg._size + 1);

    return ret;
}

/***************************************************************************//**
* @brief Reads the value of the specified register only when the device is ready
*        to accept user requests. If the device ready flag is deactivated the
*        read operation will be executed without checking the device state.
*
* @param device - The handler of the instance of the driver.
* @param pReg - Pointer to the register structure holding info about the
*               register to be read. The read value is stored inside the
*               register structure.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_ReadRegister(ad7124_st_reg* pReg)
{
    int32_t ret;

    if (pReg->addr != ERR_REG && check_ready) {
        ret = AD7124_WaitForSpiReady(spi_rdy_poll_cnt);
        if (ret < 0)
            return ret;
    }
    ret = AD7124_NoCheckReadRegister(pReg);

    return ret;
}

/***************************************************************************//**
* @brief Writes the value of the specified register only when the device is
*        ready to accept user requests. If the device ready flag is deactivated
*        the write operation will be executed without checking the device state.
*
* @param device - The handler of the instance of the driver.
* @param reg - Register structure holding info about the register to be written
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WriteRegister(ad7124_st_reg pReg)
{
    int32_t ret;

    if (check_ready) {
        ret = AD7124_WaitForSpiReady(spi_rdy_poll_cnt);
        if (ret < 0)
            return ret;
    }
    ret = AD7124_NoCheckWriteRegister(pReg);

    return ret;
}

/***************************************************************************//**
* @brief Reads and returns the value of a device register. The read value is
*        also stored in software register list of the device.
*
* @param device - The handler of the instance of the driver.
* @param reg - Which register to read from.
* @param pError - Pointer to the location where to store the error code if an
*                 error occurs. Stores 0 for success or negative error code.
*                 Does not store anything if pErorr = NULL;
*
* @return Returns the value read from the specified register.
*******************************************************************************/
uint32_t AD7124_ReadDeviceRegister(enum ad7124_registers reg)
{
    AD7124_ReadRegister(&regs[reg]);
    return (regs[reg].value);
}

/***************************************************************************//**
* @brief Writes the specified value to a device register. The value to be
*        written is also stored in the software register list of the device.
*
* @param device - The handler of the instance of the driver.
* @param reg - Which register to write to.
* @param value - The value to be written to the reigster of the device.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WriteDeviceRegister(enum ad7124_registers reg, uint32_t value)
{
    regs[reg].value = value;
    return(AD7124_WriteRegister(regs[reg]));
}

/***************************************************************************//**
* @brief Resets the device.
*
* @param device - The handler of the instance of the driver.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_Reset()
{
    int32_t ret = 0;
    uint8_t wrBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    ret = AD7124_SPI_Write( wrBuf, 8);

    delay(200);

    return ret;
}

/***************************************************************************//**
* @brief Waits until the device can accept read and write user actions.
*
* @param device - The handler of the instance of the driver.
* @param timeout - Count representing the number of polls to be done until the
*                  function returns.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WaitForSpiReady(uint32_t timeout)
{
    int32_t ret;
    int8_t _ready = 0;

    while(!_ready && --timeout) {
        /* Read the value of the Error Register */
        ret = AD7124_ReadRegister(&regs[AD7124_Error]);
        if(ret < 0)
            return ret;

        /* Check the SPI IGNORE Error bit in the Error Register */
        _ready = (regs[AD7124_Error].value &
                 AD7124_ERR_REG_SPI_IGNORE_ERR) == 0;
    }

    return timeout ? 0 : 3;
}

/***************************************************************************//**
* @brief Waits until a new conversion result is available.
*
* @param device - The handler of the instance of the driver.
* @param timeout - Count representing the number of polls to be done until the
*                  function returns if no new data is available.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WaitForConvReady(uint32_t timeout)
{
    int32_t ret;
    int8_t _ready = 0;

    while(!_ready && --timeout) {
        /* Read the value of the Status Register */
        ret = AD7124_ReadRegister(&regs[AD7124_Status]);
        if(ret < 0)
            return ret;

        /* Check the RDY bit in the Status Register */
        _ready = (regs[AD7124_Status].value &
                 AD7124_STATUS_REG_RDY) == 0;
    }

    return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
* @brief Reads the conversion result from the device.
*
* @param device - The handler of the instance of the driver.
* @param pData - Pointer to store the read data.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_ReadData( int32_t* pData)
{
    int32_t ret       = 0;
    uint8_t _buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t i         = 0;
    ad7124_st_reg *pReg;

    if( !pData)
        return INVALID_VAL;

    pReg = &regs[AD7124_Data];

    /* Build the Command word */
    _buffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
                AD7124_COMM_REG_RA(pReg->addr);

     SPI_Read(1, _buffer, pReg->_size + 1);


    if(ret < 0)
        return ret;

    /* Build the result */
    *pData = 0;
    for(i = 1; i < pReg->_size + 1; i++) {
        *pData <<= 8;
        *pData += _buffer[i];
    }
    return ret;
}

/***************************************************************************//**
* @brief Computes the CRC checksum for a data buffer.
*
* @param pBuf - Data buffer
* @param bufSize - Data buffer size in bytes
*
* @return Returns the computed CRC checksum.
*******************************************************************************/
uint8_t AD7124_ComputeCRC8(uint8_t * pBuf, uint8_t bufSize)
{
    uint8_t i   = 0;
    uint8_t crc = 0;

    while(bufSize) {
        for(i = 0x80; i != 0; i >>= 1) {
            if(((crc & 0x80) != 0) != ((*pBuf & i) != 0)) { /* MSB of CRC register XOR input Bit from Data */
                crc <<= 1;
                crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
            } else {
                crc <<= 1;
            }
        }
        pBuf++;
        bufSize--;
    }
    return crc;
}


/***************************************************************************//**
* @brief Updates the device SPI interface settings.
*
* @param device - The handler of the instance of the driver.
*
* @return None.
*******************************************************************************/
void AD7124_UpdateDevSpiSettings()
{

    if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_IGNORE_ERR_EN) {
       // check_ready = 1;
    } else {
        check_ready = 0;
    }
}

/***************************************************************************//**
* @brief Initializes the AD7124.
*
* @param device - The handler of the instance of the driver.
* @param slave_select - The Slave Chip Select Id to be passed to the SPI calls.
* @param regs - The list of registers of the device (initialized or not) to be
*               added to the instance of the driver.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_Setup()
{
    int32_t ret;
    enum ad7124_registers regNr;

    spi_rdy_poll_cnt = 25000;

    /*  Reset the device interface.*/
    ret = AD7124_Reset();
    if (ret < 0)
        return ret;

    check_ready = 0;

    /* Initialize registers AD7124_ADC_Control through AD7124_Filter_7. */
    for(regNr = AD7124_Status; (regNr < AD7124_Offset_0) && !(ret < 0);regNr = static_cast<ad7124_registers>(regNr + 1)) {
        if (regs[regNr].rw == AD7124_RW) {
            ret = AD7124_WriteRegister(regs[regNr]);
            if (ret < 0)
                break;
        }

        /* Get CRC State and device SPI interface settings */
        if (regNr == AD7124_Error_En) {
            AD7124_UpdateDevSpiSettings();
        }
    }

    return ret;
}

uint8_t AD7124_SPI_Read(uint8_t *data, uint8_t bytes_number)
{
	//to change for arduino
   if(convFlag == 0)
      SPI_Read(0, data, bytes_number);
   else
      SPI_Read(1, data, bytes_number);

    return bytes_number;
}

uint8_t AD7124_SPI_Write(uint8_t *data, uint8_t bytes_number)
{
	//to change for arduino
   if(convFlag == 0)
      SPI_Write(0, data, bytes_number);
   else
      SPI_Write(1, data, bytes_number);

   return bytes_number;

}
