/**
 *
 * @file ads124s08.c
 * @author Matt Gaidica
 * @brief  ADS124S08 Low level routines using TI Drivers
 *
 * @copyright Copyright (C) 2019-22 Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "ADS124S06.h"

const SPISettings ADS124S06_DEFAULT_SPI_SETTINGS(1000000, MSBFIRST, SPI_MODE1);

/* Internal register map array (to recall current configuration) */
static uint8_t registerMap[NUM_REGISTERS];

ADS124S06::ADS124S06(uint8_t csPin, SPISettings settings) : _csPin(csPin), _spiSettings(settings) {}

void ADS124S06::begin()
{
    pinMode(_csPin, OUTPUT);
    deselect();
    SPI.begin(); // Initialize the SPI bus
}

void ADS124S06::end()
{
    // SPI.end();
}

void ADS124S06::select()
{
    SPI.beginTransaction(_spiSettings); // Start the SPI transaction with the provided settings
    digitalWrite(_csPin, LOW);
}

void ADS124S06::deselect()
{
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction(); // End the SPI transaction
}

uint8_t ADS124S06::getRegisterValue(uint8_t address)
{
    assert(address < NUM_REGISTERS);
    return registerMap[address];
}

bool ADS124S06::adcStartupRoutine()
{
    uint8_t initRegisterMap[NUM_REGISTERS] = {0};
    uint8_t status, i;

    // Provide additional delay time for power supply settling
    delayMicroseconds(DELAY_2p2MS);

    // Toggle reset to assure default register settings.
    resetADC();

    // Must wait 4096 tCLK after reset
    delayMicroseconds(DELAY_4096TCLK);

    status = readSingleRegister(REG_ADDR_STATUS);
    if ((status & ADS_nRDY_MASK))
    {
        return false; // Device not ready
    }

    // Ensure internal register array is initialized
    restoreRegisterDefaults();

    // Configure initial device register settings here
    writeSingleRegister(REG_ADDR_STATUS, 0x00); // Reset POR event

    // Create temporary array based on desired configuration
    for (i = 0; i < NUM_REGISTERS; i++)
    {
        initRegisterMap[i] = registerMap[i];
    }

    // Read back all registers, except for status register.
    readMultipleRegisters(REG_ADDR_ID, NUM_REGISTERS);
    for (i = REG_ADDR_STATUS; i < REG_ADDR_SYS - REG_ADDR_STATUS + 1; i++)
    {
        if (i == REG_ADDR_STATUS)
            continue;
        if (initRegisterMap[i] != registerMap[i])
            return false;
    }
    return true;
}

uint8_t ADS124S06::readSingleRegister(uint8_t address)
{
    /* Initialize arrays */
    uint8_t DataTx[COMMAND_LENGTH + 1] = {OPCODE_RREG | (address & OPCODE_RWREG_MASK), 0, 0};
    uint8_t DataRx[COMMAND_LENGTH + 1] = {0, 0, 0};

    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    /* Build TX array and send it */
    spiSendReceiveArrays(DataTx, DataRx, COMMAND_LENGTH + 1);

    /* Update register array and return read result */
    registerMap[address] = DataRx[COMMAND_LENGTH];
    return DataRx[COMMAND_LENGTH];
}

void ADS124S06::readMultipleRegisters(uint8_t startAddress, uint8_t count)
{
    uint8_t DataTx[COMMAND_LENGTH + NUM_REGISTERS] = {0};
    uint8_t DataRx[COMMAND_LENGTH + NUM_REGISTERS] = {0};
    uint8_t i;

    /* Check that the register address and count are in range */
    assert(startAddress + count <= NUM_REGISTERS);

    // Read register data bytes
    DataTx[0] = OPCODE_RREG | (startAddress & OPCODE_RWREG_MASK);
    DataTx[1] = count - 1;

    spiSendReceiveArrays(DataTx, DataRx, COMMAND_LENGTH + count);

    for (i = 0; i < count; i++)
    {
        // Store received register data into internal registerMap copy
        registerMap[i + startAddress] = DataRx[COMMAND_LENGTH + i];
    }
}

void ADS124S06::writeSingleRegister(uint8_t address, uint8_t data)
{
    /* Initialize arrays */
    uint8_t DataTx[COMMAND_LENGTH + 1] = {OPCODE_WREG | (address & OPCODE_RWREG_MASK), 0, data};
    uint8_t DataRx[COMMAND_LENGTH + 1] = {0};

    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    /* Build TX array and send it */
    spiSendReceiveArrays(DataTx, DataRx, COMMAND_LENGTH + 1);

    /* Update register array */
    registerMap[address] = DataTx[COMMAND_LENGTH];
}

void ADS124S06::writeMultipleRegisters(uint8_t startAddress, uint8_t count, uint8_t regData[])
{
    uint8_t DataTx[COMMAND_LENGTH + NUM_REGISTERS] = {0};
    uint8_t DataRx[COMMAND_LENGTH + NUM_REGISTERS] = {0};
    uint8_t i, j = 0;

    /* Check that the register address and count are in range */
    assert(startAddress + count <= NUM_REGISTERS);

    /* Check that regData is not a NULL pointer */
    assert(regData);

    DataTx[0] = OPCODE_WREG | (startAddress & OPCODE_RWREG_MASK);
    DataTx[1] = count - 1;
    for (i = startAddress; i < startAddress + count; i++)
    {
        DataTx[COMMAND_LENGTH + j++] = regData[i];
        registerMap[i] = regData[i];
    }

    // SPI communication
    spiSendReceiveArrays(DataTx, DataRx, COMMAND_LENGTH + count);
}

void ADS124S06::sendCommand(uint8_t op_code)
{
    /* Assert if this function is used to send any of the following commands */
    assert(OPCODE_RREG != op_code); /* Use "readSingleRegister()"  or "readMultipleRegisters()"  */
    assert(OPCODE_WREG != op_code); /* Use "writeSingleRegister()" or "writeMultipleRegisters()" */

    /* SPI communication */
    spiSendReceiveByte(op_code);

    // Check for RESET command
    if (OPCODE_RESET == op_code)
    {
        // Must wait 4096 tCLK after reset
        delayMicroseconds(DELAY_4096TCLK);

        /* Update register array to keep software in sync with device */
        restoreRegisterDefaults();
    }
}

void ADS124S06::startConversions()
{
    // Wakeup device if in POWERDOWN
    sendWakeup();
    sendSTART();
}

void ADS124S06::stopConversions()
{
    /* Stop continuous conversions */
    sendSTOP();
}

void ADS124S06::resetADC()
{
    sendRESET();

    // Must wait 4096 tCLK after reset
    delayMicroseconds(DELAY_4096TCLK);

    // Update the local copy to stay in sync
    restoreRegisterDefaults();
}

int32_t ADS124S06::readConvertedData(uint8_t status[], readMode mode)
{
    uint8_t DataTx[RDATA_COMMAND_LENGTH + STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH] = {0}; // Initialize all array elements to 0
    uint8_t DataRx[RDATA_COMMAND_LENGTH + STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH] = {0};
    uint8_t byteLength;
    uint8_t dataPosition;
    uint8_t byte_options;
    uint8_t data[5];
    bool status_byte_enabled = 0;
    int32_t signByte, upperByte, middleByte, lowerByte;

    // Status Byte is sent if SENDSTAT bit of SYS register is set
    byte_options = IS_SENDSTAT_SET << 1 | IS_CRC_SET;
    switch (byte_options)
    {
    case 0: // No STATUS and no CRC
        byteLength = DATA_LENGTH;
        dataPosition = 0;
        break;
    case 1: // No STATUS and CRC
        byteLength = DATA_LENGTH + CRC_LENGTH;
        dataPosition = 0;
        break;
    case 2: // STATUS and no CRC
        byteLength = STATUS_LENGTH + DATA_LENGTH;
        dataPosition = 1;
        status_byte_enabled = 1;
        break;
    case 3: // STATUS and CRC
        byteLength = STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH;
        dataPosition = 1;
        status_byte_enabled = 1;
        break;
    }

    if (mode == COMMAND)
    {
        DataTx[0] = OPCODE_RDATA;
        byteLength += 1;
        dataPosition += 1;
    }
    spiSendReceiveArrays(DataTx, DataRx, byteLength);

    // Parse returned SPI data
    /* Check if STATUS byte is enabled and if we have a valid "status" memory pointer */
    if (status_byte_enabled && status)
    {
        status[0] = DataRx[dataPosition - 1];
    }

    /* Return the 32-bit sign-extended conversion result */
    if (DataRx[dataPosition] & 0x80u)
    {
        signByte = 0xFF000000;
    }
    else
    {
        signByte = 0x00000000;
    }

    if (IS_CRC_SET)
    {
        if (IS_SENDSTAT_SET)
        {
            data[0] = DataRx[dataPosition - 1]; // status
            data[1] = DataRx[dataPosition];     // msb
            data[2] = DataRx[dataPosition + 1]; // mid
            data[3] = DataRx[dataPosition + 2]; // lsb
            data[4] = DataRx[dataPosition + 3]; // crc
            bool error = (bool)getCRC(data, 5, CRC_INITIAL_SEED);

            if (error)
            {
                // if error, report and handle the error
                while (1)
                    ;
            }
        }
        else
        {
            data[0] = DataRx[dataPosition];     // msb
            data[1] = DataRx[dataPosition + 1]; // mid
            data[2] = DataRx[dataPosition + 2]; // lsb
            data[3] = DataRx[dataPosition + 3]; // crc
            bool error = (bool)getCRC(data, 4, CRC_INITIAL_SEED);

            if (error)
            {
                // if error, report and handle the error
                while (1)
                    ;
            }
        }
    }
    upperByte = ((int32_t)DataRx[dataPosition] & 0xFF) << 16;
    middleByte = ((int32_t)DataRx[dataPosition + 1] & 0xFF) << 8;
    lowerByte = ((int32_t)DataRx[dataPosition + 2] & 0xFF);

    return (signByte + upperByte + middleByte + lowerByte);
}

void ADS124S06::restoreRegisterDefaults()
{
    /* Default register settings */
    registerMap[REG_ADDR_ID] = ID_DEFAULT;
    registerMap[REG_ADDR_STATUS] = STATUS_DEFAULT;
    registerMap[REG_ADDR_INPMUX] = INPMUX_DEFAULT;
    registerMap[REG_ADDR_PGA] = PGA_DEFAULT;
    registerMap[REG_ADDR_DATARATE] = DATARATE_DEFAULT;
    registerMap[REG_ADDR_REF] = REF_DEFAULT;
    registerMap[REG_ADDR_IDACMAG] = IDACMAG_DEFAULT;
    registerMap[REG_ADDR_IDACMUX] = IDACMUX_DEFAULT;
    registerMap[REG_ADDR_VBIAS] = VBIAS_DEFAULT;
    registerMap[REG_ADDR_SYS] = SYS_DEFAULT;
    registerMap[REG_ADDR_OFCAL0] = OFCAL0_DEFAULT;
    registerMap[REG_ADDR_OFCAL1] = OFCAL1_DEFAULT;
    registerMap[REG_ADDR_OFCAL2] = OFCAL2_DEFAULT;
    registerMap[REG_ADDR_FSCAL0] = FSCAL0_DEFAULT;
    registerMap[REG_ADDR_FSCAL1] = FSCAL1_DEFAULT;
    registerMap[REG_ADDR_FSCAL2] = FSCAL2_DEFAULT;
    registerMap[REG_ADDR_GPIODAT] = GPIODAT_DEFAULT;
    registerMap[REG_ADDR_GPIOCON] = GPIOCON_DEFAULT;
}

void ADS124S06::sendWakeup()
{
    uint8_t dataTx = OPCODE_WAKEUP;

    // Wakeup device
    sendCommand(dataTx);
}

void ADS124S06::sendSTART()
{
    uint8_t dataTx = OPCODE_START;

    // Send START Command
    sendCommand(dataTx);
}

void ADS124S06::sendSTOP()
{
    uint8_t dataTx = OPCODE_STOP;

    // Send STOP Command
    sendCommand(dataTx);
}

void ADS124S06::sendRESET()
{
    uint8_t dataTx = OPCODE_RESET;

    // Send RESET command
    sendCommand(dataTx);
}

void ADS124S06::spiSendReceiveArrays(uint8_t DataTx[], uint8_t DataRx[], uint8_t byteLength)
{
    select();

    for (uint8_t i = 0; i < byteLength; i++)
    {
        DataRx[i] = SPI.transfer(DataTx[i]);
    }

    deselect();
}

uint8_t ADS124S06::spiSendReceiveByte(uint8_t dataTx)
{
    uint8_t dataRx = 0;

    select();

    dataRx = SPI.transfer(dataTx);

    deselect();

    return dataRx;
}

// NOT IMPLEMENTED, DNU CRC
bool getCRC(uint8_t data[], uint8_t length, uint8_t seed)
{
    return false;
}