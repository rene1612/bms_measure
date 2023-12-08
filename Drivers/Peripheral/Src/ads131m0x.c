/**
 * \copyright Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
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

#include "ads131m0x.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "string.h"
#include "gpio.h"
#include <stdlib.h>
#include "can.h"



//****************************************************************************
//
// Internal variables
//
//****************************************************************************
_adcConfM* adcConfM = NULL;
//_ADS131M08_ch ch;
#define SPI_DEFAULT_TIMEOUT 100U

// Array of SPI word lengths
const static uint8_t        wlength_byte_values[] = {2, 3, 4, 4};

extern DMA_HandleTypeDef hdma_spi1_rx;
uint8_t ads131m08_task_scheduler;


//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************
//
uint8_t     		buildSPIarray(const uint16_t opcodeArray[], uint8_t numberOpcodes, uint8_t byteArray[]);
uint16_t    		enforce_selected_device_modes(uint16_t data);
uint8_t     		getWordByteLength(void);
uint16_t 			getRegisterValue(uint8_t address);
HAL_StatusTypeDef 	ADS131M08_send_command(uint16_t cmd, uint16_t addr, uint16_t regs);
HAL_StatusTypeDef 	ADS131M08_recv_response(uint16_t cmd, uint16_t addr, uint16_t regs, _m08SendFlg sendFlg);

void				restoreRegisterDefaults(void);
uint8_t 			ADS131M08_wait_drdy_int(const uint32_t timeout_10ms);
HAL_StatusTypeDef 	ADS131M08_startup();
float 				ADS131M08_convert_to_mVolt(int32_t reg);
float 				ADS131M08_convert_to_mAmp(int32_t reg);
void 				delay_us(uint32_t microseconds);

void 				toggleRESET(void);
void 				ADS131M08_control_cs_signal(_signalState onOff);
GPIO_PinState		ADS131M08_read_cs_signal();


//*****************************************************************************
//
//! Getter function to access registerMap array from outside of this module.
//!
//! \fn uint16_t getRegisterValue(uint8_t address)
//!
//! NOTE: The internal registerMap arrays stores the last know register value,
//! since the last read or write operation to that register. This function
//! does not communicate with the device to retrieve the current register value.
//! For the most up-to-date register data or retrieving the value of a hardware
//! controlled register, it is recommend to use readSingleRegister() to read the
//! current register value.
//!
//! \return unsigned 16-bit register value.
//
//*****************************************************************************
uint16_t getRegisterValue(uint8_t address)
{
    assert(address < NUM_REGISTERS);
    return adcConfM->sr.mp[address];
}


//*****************************************************************************
//
//! EXTI  External Interrupt ISR Handler CallBackFun
//!
//! \fn void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//!
//! NOTE: is called on falling edge of drdx-Pin, when new adc-data are availabe
//!
//! \return none
//
//*****************************************************************************
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == MCU_DRDY_Pin) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	//ADS131M08_receive_data();
    	ads131m08_task_scheduler |= ADS131M08_PARSE_NEW_DATA;
    	main_task_scheduler |= PROCESS_ADS131M08;
    }
}


//*****************************************************************************
//
//!
//!
//! \fn void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
//!
//! NOTE: Callback function for rx-compleation
//!
//! \return none
//
//*****************************************************************************
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
    // RX Done .. Do Something ...
	ADS131M08_control_cs_signal(SET_SIGNAL);
}






//*****************************************************************************
//
//! Worker-"Thread" for adc
//!
//! \fn uint8_t	process_ADS131M08(void)
//!
//! NOTE:
//!
//! \return ads131m08_task_scheduler
//!			0 if all work are done
//!			>0 if some work is left over
//
//*****************************************************************************
uint8_t	process_ADS131M08(void)
{

	if (ads131m08_task_scheduler & ADS131M08_SEND_AGGR_DATA)
	{
		ads131m08_task_scheduler &= ~ADS131M08_SEND_AGGR_DATA;
		can_task_scheduler |= PROCESS_CAN_SEND_NEW_ADC_DATA;
    	main_task_scheduler |= PROCESS_CAN;
	}

	if (ads131m08_task_scheduler & ADS131M08_PARSE_NEW_DATA)
	{

	    if (adcConfM->Lock == DATA_UNLOCKED)
	    {
			ADS131M08_receive_data();
	    	adcConfM->Lock = DATA_LOCKED;
			ADS131M08_parse_adc_data();
	    	adcConfM->Lock = DATA_UNLOCKED;
			ads131m08_task_scheduler &= ~ADS131M08_PARSE_NEW_DATA;
	    }
	}

	return ads131m08_task_scheduler;
}


//*****************************************************************************
//
//! Offset Callibration for adc
//!
//! \fn uint8_t	ADS131M08_offset_callibration(_ADS131M08_ch ch, int32_t offset)
//!
//! NOTE:
//!
//! \return ads131m08_task_scheduler
//!			0 on Error
//!			1 on Success
//*****************************************************************************
uint8_t	ADS131M08_offset_callibration(_ADS131M08_ch ch, int32_t offset)
{
	uint8_t addr;
	uint16_t regs;
	uint32_t data;

	if (ch >= NUMB_ADC_CH)
		return 0;

//	if (offset & 0x80000000)
//		data = (0xFFFFFF - (offset & 0x007FFFFF)) | 0x800000;
//	else
//		data = offset & 0x007FFFFF;

	data = offset & 0x00FFFFFF;

	addr = CH0_OCAL_MSB_ADDRESS + ch*5;

	regs = (uint16_t)((data & 0xFFFF00) >> 8);
	writeSingleRegister(addr, regs);

	addr = CH0_OCAL_LSB_ADDRESS + ch*5;
	regs = (uint16_t)((data & 0x0000FF) << 8);
	writeSingleRegister(addr, regs);

	return 1;
}

//*****************************************************************************
//
//! Gain Callibration for adc
//!
//! \fn uint8_t	ADS131M08_gain_callibration(_ADS131M08_ch ch, uint32_t gain)
//!
//! NOTE:
//!
//! \return ads131m08_task_scheduler
//!			0 on Error
//!			1 on Success
//*****************************************************************************
uint8_t	ADS131M08_gain_callibration(_ADS131M08_ch ch, uint32_t gain)
{
	uint8_t addr, regs;

	if (ch >= NUMB_ADC_CH)
		return 0;

	addr = CH0_GCAL_MSB_ADDRESS + ch*5;
	regs = (uint16_t)((gain & 0x00FFFF00) >> 8);
	writeSingleRegister(addr, regs);

	addr = CH0_GCAL_LSB_ADDRESS + ch*5;
	regs = (uint16_t)((gain & 0x000000FF) << 8);
	writeSingleRegister(addr, regs);

	return 1;
}


//*****************************************************************************
//
//! ADC-init function
//!
//! \fn HAL_StatusTypeDef ADS131M08_init(SPI_HandleTypeDef* hspi)
//!
//! NOTE:
//!
//! \return HAL_StatusTypeDef
//!			HAL_EROR on Error
//!			HAL_OK on Success
//*****************************************************************************
HAL_StatusTypeDef ADS131M08_init(SPI_HandleTypeDef* hspi)
{
    HAL_StatusTypeDef result = HAL_OK;
    uint8_t i;

	adcConfM  = (_adcConfM*)malloc(sizeof(_adcConfM));
	memset(adcConfM, 0, sizeof(_adcConfM));
	adcConfM->stat            = ADS131M08_INIT;
	adcConfM->hspi            = hspi;
	adcConfM->nReset.port     = MCU_SYNC_RESET_GPIO_Port;
	adcConfM->nReset.pin      = MCU_SYNC_RESET_Pin;
	adcConfM->cs.port         = GPIOA;
	adcConfM->cs.pin          = SPI1_CS_Pin;
	adcConfM->nDrdy.port      = MCU_DRDY_GPIO_Port;
	adcConfM->nDrdy.pin       = MCU_DRDY_Pin;
	adcConfM->bufLen          = M08_WORDS_IN_FRAME * M08_WORD_LENGTH;      // 9 Words x 24 Bits
	adcConfM->rxBuf           = (uint8_t*)malloc(adcConfM->bufLen);
	memset(adcConfM->rxBuf, 0, adcConfM->bufLen);
	adcConfM->txBuf           = (uint8_t*)malloc(adcConfM->bufLen);
	memset(adcConfM->txBuf, 0, adcConfM->bufLen);
	//adcConfM->ch = 0;
	//adcConfM->Lock = DATA_UNLOCKED;

	for(i=0; i<CHANNEL_COUNT; i++ )
	{
		adcConfM->chData[i].average = 0;
		adcConfM->chData[i].average_counter = 0;
		switch (i)
		{
			case ADC_CH1:
			case ADC_CH2:
			case ADC_CH3:
			case ADC_CH4:
			case ADC_CH6:
				adcConfM->chData[i].measure_type=CURRENT_MA_FLOAT;
				break;
			case ADC_CH5:
				adcConfM->chData[i].measure_type=VOLTAGE_MV_FLOAT;
				break;
			default:
				break;
		}
	}

	ads131m08_task_scheduler = ADS131M08_NO_TASK;

    result = ADS131M08_startup();

    if(result != HAL_OK)
    {
    	adcConfM->stat = ADS131M08_INIT_FAIL;
    }

    //enable DRDY-falling edge int to start continous DMA-read
    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    return result;
}


//*****************************************************************************
//
//! Example start up sequence for the ADS131M0x.
//!
//! \fn HAL_StatusTypeDef ADS131M08_startup(void)
//!
//! Before calling this function, the device must be powered,
//! the SPI/GPIO pins of the MCU must have already been configured,
//! and (if applicable) the external clock source should be provided to CLKIN.
//!
//! \return None.
//
//*****************************************************************************
HAL_StatusTypeDef ADS131M08_startup()

{
	int i=0;

    // Reset Sequence
    HAL_GPIO_WritePin(adcConfM->cs.port, adcConfM->cs.pin, GPIO_PIN_SET);

	/* (OPTIONAL) Provide additional delay time for power supply settling */
	HAL_Delay(50);

	/* (OPTIONAL) Toggle nRESET pin to ensure default register settings. */
	/* NOTE: This also ensures that the device registers are unlocked.	 */
	toggleRESET();

    /* (REQUIRED) Initialize internal 'registerMap' array with device default settings */
	restoreRegisterDefaults();

    /* (OPTIONAL) Validate first response word when beginning SPI communication: (0xFF20 | CHANCNT) */
	uint16_t response = sendCommand(OPCODE_NULL);

	/* (OPTIONAL) Define your initial register settings here */
    writeSingleRegister(CLOCK_ADDRESS, (CLOCK_DEFAULT & ~CLOCK_OSR_MASK) | CLOCK_OSR_16384);

    /* (REQUIRED) Configure MODE register settings
     * NOTE: This function call is required here for this particular code implementation to work.
     * This function will enforce the MODE register settings as selected in the 'ads131m0x.h' header file.
     */
    writeSingleRegister(MODE_ADDRESS, MODE_DEFAULT);

    /* (OPTIONAL) Read back all registers */
    //ADS131M08_offset_callibration(ADC_CH4, 0);

    //ADS131M08_offset_callibration(ADC_CH5, 0);

    /* (OPTIONAL) Read back all registers */
    // Wakeup device


    // Ignore the first 5 conversion results to allow for the
    // output buffers to fill-up and the SINC3 filter to settle
    for(i = 0; i < 5; i++)
    {
//        ADS131M08_wait_drdy_int(100);
        delay_us(5);
        ADS131M08_control_cs_signal(RESET_SIGNAL);
        HAL_SPI_Receive(adcConfM->hspi, adcConfM->rxBuf, adcConfM->bufLen, SPI_DEFAULT_TIMEOUT);
        ADS131M08_control_cs_signal(SET_SIGNAL);
    }

    return HAL_OK;
	/* (OPTIONAL) Check STATUS register for faults */
}

//*****************************************************************************
//
//! Toggles the "nSYNC/nRESET" pin to trigger a reset
//! (LOW, delay 2 ms, then HIGH).
//!
//! \fn void toggleRESET(void)
//!
//! \return None.
//
//*****************************************************************************
void toggleRESET(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    HAL_GPIO_WritePin(adcConfM->nReset.port, adcConfM->nReset.pin, GPIO_PIN_RESET);

    // Minimum /RESET pulse width (tSRLRST) equals 2,048 CLKIN periods (1 ms @ 2.048 MHz)
    HAL_Delay(2);

    HAL_GPIO_WritePin(adcConfM->nReset.port, adcConfM->nReset.pin, GPIO_PIN_SET);
    // tREGACQ delay before communicating with the device again
    delay_us(5);
}



//*****************************************************************************
//
//! Reads the contents of a single register at the specified address.
//!
//! \fn uint16_t readSingleRegister(uint8_t address)
//!
//! \param address is the 8-bit address of the register to read.
//!
//! \return Returns the 8-bit register read result.
//
//*****************************************************************************
uint16_t readSingleRegister(uint8_t address)
{
	/* Check that the register address is in range */
	assert(address < NUM_REGISTERS);

    adcConfM->regAddr = address;

// Build TX and RX byte array
#ifdef ENABLE_CRC_IN
	adcConfM->txBuf[8] = 0;      // 2 words, up to 4 bytes each = 8 bytes maximum
	adcConfM->rxBuf[8] = 0;
#else
	adcConfM->txBuf[4] = 0;      // 1 word, up to 4 bytes long = 4 bytes maximum
	adcConfM->rxBuf[4] = 0;
#endif
    uint16_t opcode = OPCODE_RREG | (((uint16_t) address) << 7);
    uint8_t numberOfBytes = buildSPIarray(&opcode, 1, adcConfM->txBuf);

    /* Set the nCS pin LOW */
    ADS131M08_control_cs_signal(RESET_SIGNAL);

	// [FRAME 1] Send RREG command
    HAL_SPI_TransmitReceive(adcConfM->hspi, adcConfM->txBuf, adcConfM->rxBuf, numberOfBytes, SPI_DEFAULT_TIMEOUT);

    /* Set the nCS pin HIGH */
    ADS131M08_control_cs_signal(SET_SIGNAL);

	// [FRAME 2] Send NULL command to retrieve the register data
    adcConfM->sr.mp[address] = sendCommand(OPCODE_NULL);

	return adcConfM->sr.mp[address];
}



//*****************************************************************************
//
//! Writes data to a single register.
//!
//! \fn void writeSingleRegister(uint8_t address, uint16_t data)
//!
//! \param address is the address of the register to write to.
//! \param data is the value to write.
//!
//! This command will be ignored if device registers are locked.
//!
//! \return None.
//
//*****************************************************************************
uint8_t writeSingleRegister(uint8_t address, uint16_t data)
{
	uint16_t updated_reg = 0;

    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    adcConfM->regAddr = address;

    // (OPTIONAL) Enforce certain register field values when
    // writing to the MODE register to fix the operation mode
    if (MODE_ADDRESS == address)
    {
        data = enforce_selected_device_modes(data);
    }

    // Build TX and RX byte array
#ifdef ENABLE_CRC_IN
    adcConfM->txBuf[12] = 0;     // 3 words, up to 4 bytes each = 12 bytes maximum
    adcConfM->rxBuf[12] = 0;
#else
    adcConfM->txBuf[8] = 0;      // 2 words, up to 4 bytes long = 8 bytes maximum
    adcConfM->rxBuf[8] = 0;
#endif
    uint16_t opcodes[2];
    opcodes[0] = OPCODE_WREG | (((uint16_t) address) << 7);
    opcodes[1] = data;
    uint8_t numberOfBytes = buildSPIarray(&opcodes[0], 2, adcConfM->txBuf);

    /* Set the nCS pin LOW */
    ADS131M08_control_cs_signal(RESET_SIGNAL);

    // Send command
    HAL_SPI_TransmitReceive(adcConfM->hspi, adcConfM->txBuf, adcConfM->rxBuf, numberOfBytes, SPI_DEFAULT_TIMEOUT);

    /* Set the nCS pin HIGH */
    ADS131M08_control_cs_signal(SET_SIGNAL);


    // (RECOMMENDED) Read back register to confirm register write was successful
    updated_reg = readSingleRegister(address);

    if (data == updated_reg)
    {
        // Update internal array
        adcConfM->sr.mp[address] = data;
        return 1;
    }
    else
    	return 0;

    // NOTE: Enabling the CRC words in the SPI command will NOT prevent an invalid W
}


//*****************************************************************************
//
//! Sends the specified SPI command to the ADC (NULL, STANDBY, or WAKEUP).
//!
//! \fn uint16_t sendCommand(uint16_t opcode)
//!
//! \param opcode SPI command byte.
//!
//! NOTE: Other commands have their own dedicated functions to support
//! additional functionality.
//!
//! \return ADC response byte (typically the STATUS byte).
//
//*****************************************************************************
uint16_t sendCommand(uint16_t opcode)
{
    /* Assert if this function is used to send any of the following opcodes */
    assert(OPCODE_RREG != opcode);      /* Use "readSingleRegister()"   */
    assert(OPCODE_WREG != opcode);      /* Use "writeSingleRegister()"  */
    assert(OPCODE_LOCK != opcode);      /* Use "lockRegisters()"        */
    assert(OPCODE_UNLOCK != opcode);    /* Use "unlockRegisters()"      */
    assert(OPCODE_RESET != opcode);     /* Use "resetDevice()"          */

    // Build TX and RX byte array
#ifdef ENABLE_CRC_IN
	adcConfM->txBuf[8] = 0;      // 2 words, up to 4 bytes each = 8 bytes maximum
	adcConfM->rxBuf[8] = 0;
#else
	adcConfM->txBuf[4] = 0;      // 1 word, up to 4 bytes long = 4 bytes maximum
	adcConfM->rxBuf[4] = 0;
#endif
    uint8_t numberOfBytes = buildSPIarray(&opcode, 1, adcConfM->txBuf);

    /* Set the nCS pin LOW */
    ADS131M08_control_cs_signal(RESET_SIGNAL);

    HAL_SPI_TransmitReceive(adcConfM->hspi, adcConfM->txBuf, adcConfM->rxBuf, numberOfBytes, SPI_DEFAULT_TIMEOUT);

    /* Set the nCS pin HIGH */
    ADS131M08_control_cs_signal(SET_SIGNAL);

    // Combine response bytes and return as a 16-bit word
    uint16_t adcResponse = combineBytes(adcConfM->rxBuf[0], adcConfM->rxBuf[1]);
    return adcResponse;
}

////*****************************************************************************
////
////! Reads ADC data.
////!
////! \fn bool readData(adc_channel_data *DataStruct)
////!
////! \param *DataStruct points to an adc_channel_data type-defined structure/
////!
////! NOTE: Should be called after /DRDY goes low, and not during a /DRDY falling edge!
////!
////! \return Returns true if the CRC-OUT of the data read detects an error.
////
////*****************************************************************************
//bool readData(adc_channel_data *DataStruct)
//{
//    int i;
//    uint8_t crcTx[4]                        = { 0 };
//    uint8_t dataRx[4]                       = { 0 };
//    uint8_t bytesPerWord                    = getWordByteLength();
//
//#ifdef ENABLE_CRC_IN
//    // Build CRC word (only if "RX_CRC_EN" register bit is enabled)
//    uint16_t crcWordIn = calculateCRC(&DataTx[0], bytesPerWord * 2, 0xFFFF);
//    crcTx[0] = upperByte(crcWordIn);
//    crcTx[1] = lowerByte(crcWordIn);
//#endif
//
//    /* Set the nCS pin LOW */
//    setCS(LOW);
//
//    // Send NULL word, receive response word
//    for (i = 0; i < bytesPerWord; i++)
//    {
//        dataRx[i] = spiSendReceiveByte(0x00);
//    }
//    DataStruct->response = combineBytes(dataRx[0], dataRx[1]);
//
//    // (OPTIONAL) Do something with the response (STATUS) word.
//    // ...Here we only use the response for calculating the CRC-OUT
//    //uint16_t crcWord = calculateCRC(&dataRx[0], bytesPerWord, 0xFFFF);
//
//    // (OPTIONAL) Ignore CRC error checking
//    uint16_t crcWord = 0;
//
//    // Send 2nd word, receive channel 1 data
//    for (i = 0; i < bytesPerWord; i++)
//    {
//        dataRx[i] = spiSendReceiveByte(crcTx[i]);
//    }
//    DataStruct->channel0 = signExtend(&dataRx[0]);
//    //crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);
//
//#if (CHANNEL_COUNT > 1)
//
//    // Send 3rd word, receive channel 2 data
//    for (i = 0; i < bytesPerWord; i++)
//    {
//        dataRx[i] = spiSendReceiveByte(0x00);
//    }
//    DataStruct->channel1 = signExtend(&dataRx[0]);
//    //crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);
//
//#endif
//#if (CHANNEL_COUNT > 2)
//
//    // Send 4th word, receive channel 3 data
//    for (i = 0; i < bytesPerWord; i++)
//    {
//        dataRx[i] = spiSendReceiveByte(0x00);
//    }
//    DataStruct->channel2 = signExtend(&dataRx[0]);
//    //crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);
//
//#endif
//#if (CHANNEL_COUNT > 3)
//
//    // Send 5th word, receive channel 4 data
//    for (i = 0; i < bytesPerWord; i++)
//    {
//        dataRx[i] = spiSendReceiveByte(0x00);
//    }
//    DataStruct->channel3 = signExtend(&dataRx[0]);
//    //crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);
//
//#endif
//#if (CHANNEL_COUNT > 4)
//
//    // Send 6th word, receive channel 5 data
//    for (i = 0; i < bytesPerWord; i++)
//    {
//        dataRx[i] = spiSendReceiveByte(0x00);
//    }
//    DataStruct->channel4 = signExtend(&dataRx[0]);
//    //crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);
//
//#endif
//#if (CHANNEL_COUNT > 5)
//
//    // Send 7th word, receive channel 6 data
//    for (i = 0; i < bytesPerWord; i++)
//    {
//        dataRx[i] = spiSendReceiveByte(0x00);
//    }
//    DataStruct->channel5 = signExtend(&dataRx[0]);
//    //crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);
//
//#endif
//#if (CHANNEL_COUNT > 6)
//
//    // Send 8th word, receive channel 7 data
//    for (i = 0; i < bytesPerWord; i++)
//    {
//        dataRx[i] = spiSendReceiveByte(0x00);
//    }
//    DataStruct->channel6 = signExtend(&dataRx[0]);
//    //crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);
//
//#endif
//#if (CHANNEL_COUNT > 7)
//
//    // Send 9th word, receive channel 8 data
//    for (i = 0; i < bytesPerWord; i++)
//    {
//        dataRx[i] = spiSendReceiveByte(0x00);
//    }
//    DataStruct->channel7 = signExtend(&dataRx[0]);
//    //crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);
//
//#endif
//
//    // Send the next word, receive CRC data
//    for (i = 0; i < bytesPerWord; i++)
//    {
//        dataRx[i] = spiSendReceiveByte(0x00);
//    }
//    DataStruct->crc = combineBytes(dataRx[0], dataRx[1]);
//
//    /* NOTE: If we continue calculating the CRC with a matching CRC, the result should be zero.
//     * Any non-zero result will indicate a mismatch.
//     */
//    //crcWord = calculateCRC(&dataRx[0], bytesPerWord, crcWord);
//
//    /* Set the nCS pin HIGH */
//    setCS(HIGH);
//
//    // Returns true when a CRC error occurs
//    return ((bool) crcWord);
//}

//*****************************************************************************
//
//! Sends the specified SPI command to the ADC (NULL, STANDBY, or WAKEUP).
//!
//! \fn uint16_t sendCommand(uint16_t opcode)
//!
//! \param opcode SPI command byte.
//!
//! NOTE: Other commands have their own dedicated functions to support
//! additional functionality.
//!
//! \return ADC response byte (typically the STATUS byte).
//
//*****************************************************************************

//HAL_StatusTypeDef ADS131M08_send_command(uint16_t cmd, uint16_t addr, uint16_t regs)
//{
//    HAL_StatusTypeDef result = HAL_OK;
//    _m08SendFlg sendFlg = SEND_SYSTEM_CMD;
//    GPIO_PinState pinStat = GPIO_PIN_RESET;
//
//    switch(cmd)
//    {
//        case ADS131M08_CMD_NULL:
//        case ADS131M08_CMD_RESET:
//        case ADS131M08_CMD_STANDBY:
//        case ADS131M08_CMD_WAKEUP:
//        case ADS131M08_CMD_LOCK:
//        case ADS131M08_CMD_UNLOCK:
//            adcConfM->txBuf[0] = ADS131M08_UPPER(cmd);
//            adcConfM->txBuf[1] = ADS131M08_LOWER(cmd);
//            adcConfM->txBuf[2] = 0;
//            adcConfM->txBuf[3] = 0;
//            adcConfM->command = cmd;
//            adcConfM->regAddr = 0;
//
//            sendFlg = SEND_SYSTEM_CMD;
//            break;
//
//        case ADS131M08_CMD_RREG:    // ADS131A04_CMD_RREGS
//            adcConfM->txBuf[0] = ADS131M08_UPPER(ADS131M08_REG_COMMAND(cmd,addr));
//            adcConfM->txBuf[1] = ADS131M08_LOWER(ADS131M08_REG_COMMAND(cmd,addr));
//            adcConfM->txBuf[2] = 0;
//            adcConfM->txBuf[3] = 0;
//            adcConfM->command = cmd;
//            adcConfM->regAddr = addr;
//            adcConfM->regs = regs;
//
//            sendFlg = SEND_REGISTER_CMD;
//            break;
//        case ADS131M08_CMD_WREG:
//            adcConfM->txBuf[0] = ADS131M08_UPPER(ADS131M08_REG_COMMAND(cmd,addr));
//            adcConfM->txBuf[1] = ADS131M08_LOWER(ADS131M08_REG_COMMAND(cmd,addr));
//            adcConfM->txBuf[2] = ADS131M08_UPPER(regs);
//            adcConfM->txBuf[3] = ADS131M08_LOWER(regs);
//            adcConfM->command = cmd;
//            adcConfM->regAddr = addr;
//            adcConfM->regs = regs;
//            adcConfM->sr.mp[addr] = regs;
//
//            sendFlg = SEND_REGISTER_CMD;
//            break;
//        default:
//            break;
//    }
//
//    // Check availablity to send data
//
//    while(GPIO_PIN_RESET == pinStat)
//    {
//        pinStat = ADS131M08_read_cs_signal();
//    }
//    ADS131M08_control_cs_signal(RESET_SIGNAL);
//
//    result = HAL_SPI_TransmitReceive(adcConfM->hspi, adcConfM->txBuf, adcConfM->rxBuf, adcConfM->bufLen, SPI_DEFAULT_TIMEOUT);
//    ADS131M08_control_cs_signal(SET_SIGNAL);
//
//    adcConfM->response = combineBytes(adcConfM->rxBuf[0], adcConfM->rxBuf[1]);
//
//    if(sendFlg == SEND_REGISTER_CMD)
//    {
//        adcConfM->sr.mp[adcConfM->regAddr] = adcConfM->response;
//    }
//
//    return result;
//}
//
//
//
//
//HAL_StatusTypeDef ADS131M08_recv_response(uint16_t cmd, uint16_t addr, uint16_t regs, _m08SendFlg sendFlg)
//{
//
//    //uint16_t temp;
//    // Initialize buffer to send NULL command
//    HAL_StatusTypeDef result = HAL_OK;
//
//    memset(adcConfM->txBuf, 0, adcConfM->bufLen);
//
//    // Check availablity to send data
//    while(GPIO_PIN_RESET == ADS131M08_read_cs_signal())
//    {}
//    ADS131M08_control_cs_signal(RESET_SIGNAL);
//
//
//    if(sendFlg == SEND_SYSTEM_CMD)
//    {
//    	   if(HAL_OK != HAL_SPI_TransmitReceive(adcConfM->hspi, adcConfM->txBuf, adcConfM->rxBuf, adcConfM->bufLen, SPI_DEFAULT_TIMEOUT))
//    	    {
//    	        return HAL_ERROR;
//    	    }
//        adcConfM->response = combineBytes(adcConfM->rxBuf[0], adcConfM->rxBuf[1]);
//
//        if(cmd != adcConfM->response)
//        {
//            return HAL_ERROR;
//        }
//    }
//    else if(sendFlg == SEND_REGISTER_CMD)
//    {
////        temp = ADS131M08_REG_COMMAND(cmd,addr);
////        confirmBuf[0] = ADS131M08_UPPER(temp);
////
////        if(confirmBuf[0] != adcConfM->rxBuf[0])
////        {
////            return HAL_ERROR;
////        }
////        else
////        {
//    //	 memset(adcConfM->rxBuf, 0, adcConfM->bufLen);
//
//        result = HAL_SPI_TransmitReceive(adcConfM->hspi, adcConfM->txBuf, adcConfM->rxBuf, adcConfM->bufLen, SPI_DEFAULT_TIMEOUT);
////        memset(adcConfM->rxBuf, 0, adcConfM->bufLen);
//       // result = HAL_SPI_Receive(adcConfM->hspi, adcConfM->rxBuf, adcConfM->bufLen, SPI_DEFAULT_TIMEOUT);
//            adcConfM->sr.mp[adcConfM->regAddr] = combineBytes(adcConfM->rxBuf[0], adcConfM->rxBuf[1]);
////        }
////
////        if(regs != adcConfM->sr.mp[adcConfM->regAddr])
////        {
////            return HAL_ERROR;
////        }
//    }
//    else
//    {
//        // Do Nothing
//    }
//    ADS131M08_control_cs_signal(SET_SIGNAL);
//    return HAL_OK;
//
//}




void ADS131M08_control_cs_signal(_signalState onOff)
{
    if(onOff == SET_SIGNAL)
    {
            HAL_GPIO_WritePin(adcConfM->cs.port, adcConfM->cs.pin, GPIO_PIN_SET);

    }
    else if(onOff == RESET_SIGNAL)
    {
            HAL_GPIO_WritePin(adcConfM->cs.port, adcConfM->cs.pin, GPIO_PIN_RESET);
    }
    else
    {
        // Do Nothing
    }
}


GPIO_PinState ADS131M08_read_cs_signal()
{
	return HAL_GPIO_ReadPin(adcConfM->cs.port, adcConfM->cs.pin);
}

//GPIO_PinState ADS131M08_read_drdy_signal()
//{
//	return HAL_GPIO_ReadPin(adcConfM->nDrdy.port, adcConfM->nDrdy.pin);
//}

//*****************************************************************************
//
//! Calculates the 16-bit CRC for the selected CRC polynomial.
//!
//! \fn uint16_t ADS131M08_calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue)
//!
//! \param dataBytes[] pointer to first element in the data byte array
//! \param numberBytes number of bytes to be used in CRC calculation
//! \param initialValue the seed value (or partial crc calculation), use 0xFFFF when beginning a new CRC computation
//!
//! NOTE: This calculation is shown as an example and is not optimized for speed.
//!
//! \return 16-bit calculated CRC word
//
//*****************************************************************************
#ifdef ENABLE_CRC_IN
uint16_t ADS131M08_calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue)
{
	/* Check that "dataBytes" is not a null pointer */
	assert(dataBytes != 0x00);

	int         bitIndex, byteIndex;
	bool        dataMSb;						/* Most significant bit of data byte */
	bool        crcMSb;						    /* Most significant bit of crc byte  */
	uint8_t     bytesPerWord = wlength_byte_values[WLENGTH];

	/*
     * Initial value of crc register
     * NOTE: The ADS131M0x defaults to 0xFFFF,
     * but can be set at function call to continue an on-going calculation
     */
    uint16_t crc = initialValue;

    #ifdef CRC_CCITT
    /* CCITT CRC polynomial = x^16 + x^12 + x^5 + 1 */
    const uint16_t poly = 0x1021;
    #endif

    #ifdef CRC_ANSI
    /* ANSI CRC polynomial = x^16 + x^15 + x^2 + 1 */
    const uint16_t poly = 0x8005;
    #endif

    //
    // CRC algorithm
    //

    // Loop through all bytes in the dataBytes[] array
	for (byteIndex = 0; byteIndex < numberBytes; byteIndex++)
	{
	    // Point to MSb in byte
	    bitIndex = 0x80u;

	    // Loop through all bits in the current byte
	    while (bitIndex > 0)
	    {
	        // Check MSB's of data and crc
	        dataMSb = (bool) (dataBytes[byteIndex] & bitIndex);
	        crcMSb  = (bool) (crc & 0x8000u);

	        crc <<= 1;              /* Left shift CRC register */

	        // Check if XOR operation of MSBs results in additional XOR operations
	        if (dataMSb ^ crcMSb)
	        {
	            crc ^= poly;        /* XOR crc with polynomial */
	        }

	        /* Shift MSb pointer to the next data bit */
	        bitIndex >>= 1;
	    }
	}

	return crc;
}
#endif


//*****************************************************************************
//
//! Updates the registerMap[] array to its default values.
//!
//! \fn void restoreRegisterDefaults(void)
//!
//! NOTES:
//! - If the MCU keeps a copy of the ADS131M0x register settings in memory,
//! then it is important to ensure that these values remain in sync with the
//! actual hardware settings. In order to help facilitate this, this function
//! should be called after powering up or resetting the device (either by
//! hardware pin control or SPI software command).
//!
//! - Reading back all of the registers after resetting the device can
//! accomplish the same result; however, this might be problematic if the
//! device was previously in CRC mode or the WLENGTH was modified, since
//! resetting the device exits these modes. If the MCU is not aware of this
//! mode change, then read register commands will return invalid data due to
//! the expectation of data appearing in a different byte position.
//!
//! \return None.
//
//*****************************************************************************
void restoreRegisterDefaults(void)
{
	 adcConfM->sr.mp[ID_ADDRESS]             =   0x00;               /* NOTE: This a read-only register */
	 adcConfM->sr.mp[STATUS_ADDRESS]         =   STATUS_DEFAULT;
	 adcConfM->sr.mp[MODE_ADDRESS]           =   MODE_DEFAULT;
	 adcConfM->sr.mp[CLOCK_ADDRESS]          =   CLOCK_DEFAULT;
	 adcConfM->sr.mp[GAIN1_ADDRESS]          =   GAIN1_DEFAULT;
	 adcConfM->sr.mp[GAIN2_ADDRESS]          =   GAIN2_DEFAULT;
	 adcConfM->sr.mp[CFG_ADDRESS]            =   CFG_DEFAULT;
	 adcConfM->sr.mp[THRSHLD_MSB_ADDRESS]    =   THRSHLD_MSB_DEFAULT;
	 adcConfM->sr.mp[THRSHLD_LSB_ADDRESS]    =   THRSHLD_LSB_DEFAULT;
	 adcConfM->sr.mp[CH0_CFG_ADDRESS]        =   CH0_CFG_DEFAULT;
	 adcConfM->sr.mp[CH0_OCAL_MSB_ADDRESS]   =   CH0_OCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH0_OCAL_LSB_ADDRESS]   =   CH0_OCAL_LSB_DEFAULT;
	 adcConfM->sr.mp[CH0_GCAL_MSB_ADDRESS]   =   CH0_GCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH0_GCAL_LSB_ADDRESS]   =   CH0_GCAL_LSB_DEFAULT;
#if (CHANNEL_COUNT > 1)
	 adcConfM->sr.mp[CH1_CFG_ADDRESS]        =   CH1_CFG_DEFAULT;
	 adcConfM->sr.mp[CH1_OCAL_MSB_ADDRESS]   =   CH1_OCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH1_OCAL_LSB_ADDRESS]   =   CH1_OCAL_LSB_DEFAULT;
	 adcConfM->sr.mp[CH1_GCAL_MSB_ADDRESS]   =   CH1_GCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH1_GCAL_LSB_ADDRESS]   =   CH1_GCAL_LSB_DEFAULT;
#endif
#if (CHANNEL_COUNT > 2)
	 adcConfM->sr.mp[CH2_CFG_ADDRESS]        =   CH2_CFG_DEFAULT;
	 adcConfM->sr.mp[CH2_OCAL_MSB_ADDRESS]   =   CH2_OCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH2_OCAL_LSB_ADDRESS]   =   CH2_OCAL_LSB_DEFAULT;
	 adcConfM->sr.mp[CH2_GCAL_MSB_ADDRESS]   =   CH2_GCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH2_GCAL_LSB_ADDRESS]   =   CH2_GCAL_LSB_DEFAULT;
#endif
#if (CHANNEL_COUNT > 3)
	 adcConfM->sr.mp[CH3_CFG_ADDRESS]        =   CH3_CFG_DEFAULT;
	 adcConfM->sr.mp[CH3_OCAL_MSB_ADDRESS]   =   CH3_OCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH3_OCAL_LSB_ADDRESS]   =   CH3_OCAL_LSB_DEFAULT;
	 adcConfM->sr.mp[CH3_GCAL_MSB_ADDRESS]   =   CH3_GCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH3_GCAL_LSB_ADDRESS]   =   CH3_GCAL_LSB_DEFAULT;
#endif
#if (CHANNEL_COUNT > 4)
	 adcConfM->sr.mp[CH4_CFG_ADDRESS]        =   CH4_CFG_DEFAULT;
	 adcConfM->sr.mp[CH4_OCAL_MSB_ADDRESS]   =   CH4_OCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH4_OCAL_LSB_ADDRESS]   =   CH4_OCAL_LSB_DEFAULT;
	 adcConfM->sr.mp[CH4_GCAL_MSB_ADDRESS]   =   CH4_GCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH4_GCAL_LSB_ADDRESS]   =   CH4_GCAL_LSB_DEFAULT;
#endif
#if (CHANNEL_COUNT > 5)
	 adcConfM->sr.mp[CH5_CFG_ADDRESS]        =   CH5_CFG_DEFAULT;
	 adcConfM->sr.mp[CH5_OCAL_MSB_ADDRESS]   =   CH5_OCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH5_OCAL_LSB_ADDRESS]   =   CH5_OCAL_LSB_DEFAULT;
	 adcConfM->sr.mp[CH5_GCAL_MSB_ADDRESS]   =   CH5_GCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH5_GCAL_LSB_ADDRESS]   =   CH5_GCAL_LSB_DEFAULT;
#endif
#if (CHANNEL_COUNT > 6)
	 adcConfM->sr.mp[CH6_CFG_ADDRESS]        =   CH6_CFG_DEFAULT;
	 adcConfM->sr.mp[CH6_OCAL_MSB_ADDRESS]   =   CH6_OCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH6_OCAL_LSB_ADDRESS]   =   CH6_OCAL_LSB_DEFAULT;
	 adcConfM->sr.mp[CH6_GCAL_MSB_ADDRESS]   =   CH6_GCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH6_GCAL_LSB_ADDRESS]   =   CH6_GCAL_LSB_DEFAULT;
#endif
#if (CHANNEL_COUNT > 7)
	 adcConfM->sr.mp[CH7_CFG_ADDRESS]        =   CH7_CFG_DEFAULT;
	 adcConfM->sr.mp[CH7_OCAL_MSB_ADDRESS]   =   CH7_OCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH7_OCAL_LSB_ADDRESS]   =   CH7_OCAL_LSB_DEFAULT;
	 adcConfM->sr.mp[CH7_GCAL_MSB_ADDRESS]   =   CH7_GCAL_MSB_DEFAULT;
	 adcConfM->sr.mp[CH7_GCAL_LSB_ADDRESS]   =   CH7_GCAL_LSB_DEFAULT;
#endif
	 adcConfM->sr.mp[REGMAP_CRC_ADDRESS]     =   REGMAP_CRC_DEFAULT;
}



//****************************************************************************
//
// Helper functions
//
//****************************************************************************

HAL_StatusTypeDef ADS131M08_receive_data()
{
	HAL_StatusTypeDef result=HAL_OK;

	// Wait until cs pin is reset to send data
	if(GPIO_PIN_RESET == ADS131M08_read_cs_signal())
	{
		return HAL_ERROR;
	}

    ADS131M08_control_cs_signal(RESET_SIGNAL);
    //result = HAL_SPI_Receive_IT(adcConfM->hspi, adcConfM->rxBuf, adcConfM->bufLen);
    result = HAL_SPI_Receive(adcConfM->hspi, adcConfM->rxBuf, adcConfM->bufLen, 50);

    if (result != HAL_OK)
    {
        ADS131M08_control_cs_signal(SET_SIGNAL);
    	return result;
    }

    ADS131M08_control_cs_signal(SET_SIGNAL);
    return HAL_OK;
}


//void ADS131M08_receive_data()
//{
//    // Wait until cs pin is reset to send data
//    while(GPIO_PIN_RESET == ADS131M08_read_cs_signal()) {}
//
//    ADS131M08_control_cs_signal(RESET_SIGNAL);
//    HAL_SPI_Receive(adcConfM->hspi, adcConfM->rxBuf, adcConfM->bufLen, 50);
//    ADS131M08_control_cs_signal(SET_SIGNAL);
//}



//*****************************************************************************
//
//! Takes a 16-bit word and returns the most-significant byte.
//!
//! \fn uint8_t upperByte(uint16_t uint16_Word)
//!
//! \param temp_word is the original 16-bit word.
//!
//! \return 8-bit most-significant byte.
//
//*****************************************************************************
uint8_t upperByte(uint16_t uint16_Word)
{
    uint8_t msByte;
    msByte = (uint8_t) ((uint16_Word >> 8) & 0x00FF);

    return msByte;
}



//*****************************************************************************
//
//! Takes a 16-bit word and returns the least-significant byte.
//!
//! \fn uint8_t lowerByte(uint16_t uint16_Word)
//!
//! \param temp_word is the original 16-bit word.
//!
//! \return 8-bit least-significant byte.
//
//*****************************************************************************
uint8_t lowerByte(uint16_t uint16_Word)
{
    uint8_t lsByte;
    lsByte = (uint8_t) (uint16_Word & 0x00FF);

    return lsByte;
}

uint32_t ADS131M08_convert_adc_data(const uint8_t* dataBuf)
{
    uint32_t upperByte;
    uint32_t middleByte;
    uint32_t lowerByte;

    // The output data extends to 32 bits with eight zeroes(0b00000000, 1Byte) added to the least significant bits when using the 32-bit device word length setting, datasheet 38p
    upperByte    = ((uint32_t) dataBuf[0] << 16);
    middleByte   = ((uint32_t) dataBuf[1] << 8);
    lowerByte  = ((uint32_t) dataBuf[2] << 0);

    return (upperByte | middleByte | lowerByte);
}


void ADS131M08_parse_adc_data()
{
    uint8_t index;

    adcConfM->response = combineBytes(adcConfM->rxBuf[0], adcConfM->rxBuf[1]);

	for(adcConfM->ch = ADC_CH1, index = 1; adcConfM->ch < NUMB_ADC_CH; adcConfM->ch++, index++)
	{
		if ( adcConfM->chData[adcConfM->ch].average_counter < CHANNEL_OVERSAMPLING )
		{
			adcConfM->chData[adcConfM->ch].r = ADS131M08_convert_adc_data(&adcConfM->rxBuf[index * M08_WORD_LENGTH]);

			if ( adcConfM->chData[adcConfM->ch].r & 0x800000)
			{
				adcConfM->chData[adcConfM->ch].average -= ((0xFFFFFF - adcConfM->chData[adcConfM->ch].r) +1);
			}
			else
			{
				adcConfM->chData[adcConfM->ch].average += adcConfM->chData[adcConfM->ch].r;
			}

			adcConfM->chData[adcConfM->ch].average_counter ++;
		}
		else
		{
			adcConfM->chData[adcConfM->ch].average /= CHANNEL_OVERSAMPLING;
			adcConfM->chData[adcConfM->ch].average_counter = 0;

			switch (adcConfM->chData[adcConfM->ch].measure_type )
			{
				case CURRENT_MA_FLOAT:
					 adcConfM->chData[adcConfM->ch].v = ADS131M08_convert_to_mAmp(adcConfM->chData[adcConfM->ch].average);
					 break;

				case VOLTAGE_MV_FLOAT:
					 adcConfM->chData[adcConfM->ch].v = ADS131M08_convert_to_mVolt(adcConfM->chData[adcConfM->ch].average);
					break;

				default:
					break;
			}
			//Do the things that need to be done
			if (adcConfM->ch >= NUMB_ADC_CH-1)
			{
				adcConfM->ch = 0;
				ads131m08_task_scheduler |= ADS131M08_SEND_AGGR_DATA;
				main_task_scheduler |= PROCESS_ADS131M08;
				return;
			}
		}
	}
}


float ADS131M08_convert_to_mVolt(int32_t reg)
{
    const float unitFS = 345.0f / 8388607.0f; // unit: mV (if unit is V, calculated value is out of 'float' range)

    // convert register to mVolt
    return (unitFS * (float)reg);
}



float ADS131M08_convert_to_mAmp(int32_t reg)
{
    const float unitFS = 47000.0f / 8388607.0f; // unit: mV (if unit is V, calculated value is out of 'float' range)

    // convert register to mVolt
    return (float)((float)reg * unitFS);
    //return (((float)reg * 4.8f)/8388607.0f );
}

//*****************************************************************************
//
//! Takes two 8-bit words and returns a concatenated 16-bit word.
//!
//! \fn uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte)
//!
//! \param upperByte is the 8-bit value that will become the MSB of the 16-bit word.
//! \param lowerByte is the 8-bit value that will become the LSB of the 16-bit word.
//!
//! \return concatenated 16-bit word.
//
//*****************************************************************************
uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte)
{
    uint16_t combinedValue;
    combinedValue = ((uint16_t) upperByte << 8) | ((uint16_t) lowerByte);

    return combinedValue;
}


//*****************************************************************************
//
//! Combines ADC data bytes into a single signed 32-bit word.
//!
//! \fn int32_t combineDataBytes(const uint8_t dataBytes[])
//!
//! \param dataBytes is a pointer to uint8_t[] where the first element is the MSB.
//!
//! \return Returns the signed-extend 32-bit result.
//
//*****************************************************************************
int32_t signExtend(const uint8_t dataBytes[])
{

#ifdef WORD_LENGTH_24BIT

    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t middleByte  = ((int32_t) dataBytes[1] << 16);
    int32_t lowerByte   = ((int32_t) dataBytes[2] << 8);

    return (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);     // Right-shift of signed data maintains signed bit

#elif defined WORD_LENGTH_32BIT_SIGN_EXTEND

    int32_t signByte    = ((int32_t) dataBytes[0] << 24);
    int32_t upperByte   = ((int32_t) dataBytes[1] << 16);
    int32_t middleByte  = ((int32_t) dataBytes[2] << 8);
    int32_t lowerByte   = ((int32_t) dataBytes[3] << 0);

    return (signByte | upperByte | middleByte | lowerByte);

#elif defined WORD_LENGTH_32BIT_ZERO_PADDED

    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t middleByte  = ((int32_t) dataBytes[1] << 16);
    int32_t lowerByte   = ((int32_t) dataBytes[2] << 8);

    return (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);     // Right-shift of signed data maintains signed bit

#elif defined WORD_LENGTH_16BIT_TRUNCATED

    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t lowerByte   = ((int32_t) dataBytes[1] << 16);

    return (((int32_t) (upperByte | lowerByte)) >> 16);                 // Right-shift of signed data maintains signed bit

#endif
}



//****************************************************************************
//
// Internal functions
//
//****************************************************************************


//*****************************************************************************
//
//! Builds SPI TX data arrays according to number of opcodes provided and
//! currently programmed device word length.
//!
//! \fn uint8_t buildSPIarray(const uint16_t opcodeArray[], uint8_t numberOpcodes, uint8_t byteArray[])
//!
//! \param opcodeArray[] pointer to an array of 16-bit opcodes to use in the SPI command.
//! \param numberOpcodes the number of opcodes provided in opcodeArray[].
//! \param byteArray[] pointer to an array of 8-bit SPI bytes to send to the device.
//!
//! NOTE: The calling function must ensure it reserves sufficient memory for byteArray[]!
//!
//! \return number of bytes added to byteArray[].
//
//*****************************************************************************
uint8_t buildSPIarray(const uint16_t opcodeArray[], uint8_t numberOpcodes, uint8_t byteArray[])
{
    /*
     * Frame size = opcode word(s) + optional CRC word
     * Number of bytes per word = 2, 3, or 4
     * Total bytes = bytes per word * number of words
     */
    uint8_t numberWords     = numberOpcodes + (SPI_CRC_ENABLED ? 1 : 0);
    uint8_t bytesPerWord    = getWordByteLength();
    uint8_t numberOfBytes   = numberWords * bytesPerWord;

    int i;
    for (i = 0; i < numberOpcodes; i++)
    {
        // NOTE: Be careful not to accidentally overflow the array here.
        // The array and opcodes are defined in the calling function, so
        // we are trusting that no mistakes were made in the calling function!
        byteArray[(i*bytesPerWord) + 0] = upperByte(opcodeArray[i]);
        byteArray[(i*bytesPerWord) + 1] = lowerByte(opcodeArray[i]);
    }

#ifdef ENABLE_CRC_IN
    // Calculate CRC and put it into TX array
    uint16_t crcWord = calculateCRC(&byteArray[0], numberOfBytes, 0xFFFF);
    byteArray[(i*bytesPerWord) + 0] = upperByte(crcWord);
    byteArray[(i*bytesPerWord) + 1] = lowerByte(crcWord);
#endif

    return numberOfBytes;
}



//*****************************************************************************
//
//! Modifies MODE register data to maintain device operation according to
//! preselected mode(s) (RX_CRC_EN, WLENGTH, etc.).
//!
//! \fn uint16_t enforce_selected_device_mode(uint16_t data)
//!
//! \param data uint16_t register data.
//!
//! \return uint16_t modified register data.
//
//*****************************************************************************
uint16_t enforce_selected_device_modes(uint16_t data)
{


    ///////////////////////////////////////////////////////////////////////////
    // Enforce RX_CRC_EN setting

#ifdef ENABLE_CRC_IN
    // When writing to the MODE register, ensure RX_CRC_EN bit is ALWAYS set
    data |= MODE_RX_CRC_EN_ENABLED;
#else
    // When writing to the MODE register, ensure RX_CRC_EN bit is NEVER set
    data &= ~MODE_RX_CRC_EN_ENABLED;
#endif // ENABLE_CRC_IN


    ///////////////////////////////////////////////////////////////////////////
    // Enforce WLENGH setting

#ifdef WORD_LENGTH_24BIT
    // When writing to the MODE register, ensure WLENGTH bits are ALWAYS set to 01b
    data = (data & ~MODE_WLENGTH_MASK) | MODE_WLENGTH_24BIT;
#elif defined WORD_LENGTH_32BIT_SIGN_EXTEND
    // When writing to the MODE register, ensure WLENGH bits are ALWAYS set to 11b
    data = (data & ~MODE_WLENGTH_MASK) | MODE_WLENGTH_32BIT_MSB_SIGN_EXT;
#elif defined WORD_LENGTH_32BIT_ZERO_PADDED
    // When writing to the MODE register, ensure WLENGH bits are ALWAYS set to 10b
    data = (data & ~MODE_WLENGTH_MASK) | MODE_WLENGTH_32BIT_LSB_ZEROES;
#elif defined WORD_LENGTH_16BIT_TRUNCATED
    // When writing to the MODE register, ensure WLENGH bits are ALWAYS set to 00b
    data = (data & ~MODE_WLENGTH_MASK) | MODE_WLENGTH_16BIT;
#endif


    ///////////////////////////////////////////////////////////////////////////
    // Enforce DRDY_FMT setting

#ifdef DRDY_FMT_PULSE
    // When writing to the MODE register, ensure DRDY_FMT bit is ALWAYS set
    data = (data & ~MODE_DRDY_FMT_MASK) | MODE_DRDY_FMT_NEG_PULSE_FIXED_WIDTH;
#else
    // When writing to the MODE register, ensure DRDY_FMT bit is NEVER set
    data = (data & ~MODE_DRDY_FMT_MASK) | MODE_DRDY_FMT_LOGIC_LOW;
#endif


    ///////////////////////////////////////////////////////////////////////////
    // Enforce CRC_TYPE setting

#ifdef CRC_CCITT
    // When writing to the MODE register, ensure CRC_TYPE bit is NEVER set
    data = (data & ~STATUS_CRC_TYPE_MASK) | STATUS_CRC_TYPE_16BIT_CCITT;
#elif defined CRC_ANSI
    // When writing to the MODE register, ensure CRC_TYPE bit is ALWAYS set
    data = (data & ~STATUS_CRC_TYPE_MASK) | STATUS_CRC_TYPE_16BIT_ANSI;
#endif

    // Return modified register data
    return data;
}



//*****************************************************************************
//
//! Returns the ADS131M0x configured word length used for SPI communication.
//!
//! \fn uint8_t getWordByteLength(void)
//!
//! NOTE: It is important that the MODE register value stored in registerMap[]
//! remains in sync with the device. If these values get out of sync then SPI
//! communication may fail!
//!
//! \return SPI word byte length (2, 3, or 4)
//
//*****************************************************************************
uint8_t getWordByteLength(void)
{
    return wlength_byte_values[WLENGTH];
}



void delay_us(uint32_t microseconds)
{
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = microseconds * (HAL_RCC_GetHCLKFreq() / 1000000);

    while ((HAL_GetTick() - tickstart) < wait);
}

