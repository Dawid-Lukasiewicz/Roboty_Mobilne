#include "pn532.h"
#include <stdbool.h>

#define SPI_TIMEOUT 10
#define _SPI_STATREAD                   0x02
#define _SPI_DATAWRITE                  0x01
#define _SPI_DATAREAD                   0x03
#define _SPI_READY                      0x01
#define PN532_FRAME_MAX_LENGTH              255
#define PN532_DEFAULT_TIMEOUT               1000


const uint8_t PN532_ACK[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
const uint8_t PN532_FRAME_START[] = {0x00, 0x00, 0xFF};

void PN532_Init(PN532_Typedef* pn532, SPI_HandleTypeDef* hspi, GPIO_TypeDef* SS_Port, uint16_t SS_Pin)
{
	pn532->hspi = hspi;
	pn532->SS_Port = SS_Port;
	pn532->SS_Pin = SS_Pin;
	PN532_Wakeup(pn532);
}

void PN532_SpiRw(PN532_Typedef* pn532, uint8_t* data, uint8_t count)
{
	HAL_GPIO_WritePin(pn532->SS_Port, pn532->SS_Pin, 0);
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(pn532->hspi, data, data, count, SPI_TIMEOUT);
	HAL_Delay(1);
	HAL_GPIO_WritePin(pn532->SS_Port, pn532->SS_Pin, 1);
}


int PN532_ReadData(PN532_Typedef* pn532, uint8_t* data, uint16_t count)
{
    uint8_t frame[count + 1];
    frame[0] = _SPI_DATAREAD;
    HAL_Delay(5);
    PN532_SpiRw(pn532, frame, count + 1);
    for (uint8_t i = 0; i < count; i++)
        data[i] = frame[i + 1];
    return PN532_STATUS_OK;
}


int PN532_WriteData(PN532_Typedef* pn532, uint8_t *data, uint16_t count)
{
    uint8_t frame[count + 1];
    frame[0] = _SPI_DATAWRITE;
    for (uint8_t i = 0; i < count; i++)
        frame[i + 1] = data[i];
    PN532_SpiRw(pn532, frame, count + 1);
    return PN532_STATUS_OK;
}

int PN532_Wakeup(PN532_Typedef* pn532)
{
    // Send any special commands/data to wake up PN532
    uint8_t data[] = {0x00};
    HAL_Delay(1000);
	HAL_GPIO_WritePin(pn532->SS_Port, pn532->SS_Pin, 0);
    HAL_Delay(2); // T_osc_start
    PN532_SpiRw(pn532, data, 1);
    HAL_Delay(1000);
    return PN532_STATUS_OK;
}

bool PN532_WaitReady(PN532_Typedef* pn532, uint32_t timeout)
{
    uint8_t status[] = {_SPI_STATREAD, 0x00};
    uint32_t tickstart = HAL_GetTick();
    while (HAL_GetTick() - tickstart < timeout)
    {
        HAL_Delay(10);
        PN532_SpiRw(pn532, status, sizeof(status));
        if (status[1] == _SPI_READY)
            return true;
         else
            HAL_Delay(5);
    }
    return false;
}


int PN532_WriteFrame(PN532_Typedef* pn532, uint8_t* data, uint16_t length)
{
    if (length > PN532_FRAME_MAX_LENGTH || length < 1)
        return PN532_STATUS_ERROR; // Data must be array of 1 to 255 bytes.

    // Build frame to send as:
    // - Preamble (0x00)
    // - Start code  (0x00, 0xFF)
    // - Command length (1 byte)
    // - Command length checksum
    // - Command bytes
    // - Checksum
    // - Postamble (0x00)

    uint8_t frame[PN532_FRAME_MAX_LENGTH + 7];
    uint8_t checksum = 0;
    frame[0] = PN532_PREAMBLE;
    frame[1] = PN532_STARTCODE1;
    frame[2] = PN532_STARTCODE2;
    for (uint8_t i = 0; i < 3; i++)
        checksum += frame[i];
    frame[3] = length & 0xFF;
    frame[4] = (~length + 1) & 0xFF;
    for (uint8_t i = 0; i < length; i++)
    {
        frame[5 + i] = data[i];
        checksum += data[i];
    }
    frame[length + 5] = ~checksum & 0xFF;
    frame[length + 6] = PN532_POSTAMBLE;
    if (PN532_WriteData(pn532, frame, length + 7) != PN532_STATUS_OK)
        return PN532_STATUS_ERROR;

    return PN532_STATUS_OK;
}


int PN532_ReadFrame(PN532_Typedef* pn532, uint8_t* response, uint16_t length)
{
    uint8_t buff[PN532_FRAME_MAX_LENGTH + 7];
    uint8_t checksum = 0;
    // Read frame with expected length of data.
    PN532_ReadData(pn532, buff, length + 7);
    // Swallow all the 0x00 values that preceed 0xFF.
    uint8_t offset = 0;
    while (buff[offset] == 0x00)
    {
        offset += 1;
        if (offset >= length + 8){
            printf("Response frame preamble does not contain 0x00FF!");
            return PN532_STATUS_ERROR;
        }
    }

    if (buff[offset] != 0xFF)
    {
        printf("Response frame preamble does not contain 0x00FF!");
        return PN532_STATUS_ERROR;
    }
    offset += 1;
    if (offset >= length + 8)
    {
        printf("Response contains no data!");
        return PN532_STATUS_ERROR;
    }
    // Check length & length checksum match.
    uint8_t frame_len = buff[offset];
    if (((frame_len + buff[offset+1]) & 0xFF) != 0)
    {
        printf("Response length checksum did not match length!");
        return PN532_STATUS_ERROR;
    }
    // Check frame checksum value matches bytes.
    for (uint8_t i = 0; i < frame_len + 1; i++)
        checksum += buff[offset + 2 + i];

    checksum &= 0xFF;
    if (checksum != 0)
    {
        printf("Response checksum did not match expected checksum");
        return PN532_STATUS_ERROR;
    }
    // Return frame data.
    for (uint8_t i = 0; i < frame_len; i++) {
        response[i] = buff[offset + 2 + i];
    }
    return frame_len;
}

/**
  * @brief: Send specified command to the PN532 and expect up to response_length.
  *     Will wait up to timeout seconds for a response and read a bytearray into
  *     response buffer.
  * @param pn532: PN532 handler
  * @param command: command to send
  * @param response: buffer returned
  * @param response_length: expected response length
  * @param params: can optionally specify an array of bytes to send as parameters
  *     to the function call, or NULL if there is no need to send parameters.
  * @param params_length: length of the argument params
  * @param timeout: timout of systick
  * @retval: Returns the length of response or -1 if error.
  */
int PN532_CallFunction(
    PN532_Typedef* pn532,
    uint8_t command,
    uint8_t* response,
    uint16_t response_length,
    uint8_t* params,
    uint16_t params_length,
    uint32_t timeout
){
    // Build frame data with command and parameters.
    uint8_t buff[PN532_FRAME_MAX_LENGTH];
    buff[0] = PN532_HOSTTOPN532;
    buff[1] = command & 0xFF;
    for (uint8_t i = 0; i < params_length; i++)
        buff[2 + i] = params[i];

    // Send frame and wait for response.
    if (PN532_WriteFrame(pn532, buff, params_length + 2) != PN532_STATUS_OK)
    {
        PN532_Wakeup(pn532);
        printf("Trying to wakeup");
        return PN532_STATUS_ERROR;
    }

    if (!PN532_WaitReady(pn532, timeout))
        return PN532_STATUS_ERROR;

    // Verify ACK response and wait to be ready for function response.
    PN532_ReadData(pn532, buff, sizeof(PN532_ACK));
    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++)
    {
        if (PN532_ACK[i] != buff[i]) {
            printf("Did not receive expected ACK from PN532!");
            return PN532_STATUS_ERROR;
        }
    }

    if (!PN532_WaitReady(pn532, timeout))
        return PN532_STATUS_ERROR;

    // Read response bytes.
    int frame_len = PN532_ReadFrame(pn532, buff, response_length + 2);

    // Check that response is for the called function.
    if (! ((buff[0] == PN532_PN532TOHOST) && (buff[1] == (command+1))))
    {
        printf("Received unexpected command response!");
        return PN532_STATUS_ERROR;
    }
    // Return response data.
    for (uint8_t i = 0; i < response_length; i++)
    {
        response[i] = buff[i + 2];
    }
    // The the number of bytes read
    return frame_len - 2;
}

int PN532_SendCommand(PN532_Typedef* pn532, uint8_t command, uint8_t* params, uint16_t params_length, uint32_t timeout)
{
	// Build frame data with command and parameters.
	uint8_t buff[PN532_FRAME_MAX_LENGTH];
	buff[0] = PN532_HOSTTOPN532;
	buff[1] = command & 0xFF;
	for (uint8_t i = 0; i < params_length; i++)
		buff[2 + i] = params[i];

	// Send frame and wait for response.
	if (PN532_WriteFrame(pn532, buff, params_length + 2) != PN532_STATUS_OK)
	{
		PN532_Wakeup(pn532);
		printf("Trying to wakeup");
		return PN532_STATUS_ERROR;
	}

	if (!PN532_WaitReady(pn532, timeout))
		return PN532_STATUS_ERROR;

    // Verify ACK response and wait to be ready for function response.
    PN532_ReadData(pn532, buff, sizeof(PN532_ACK));
    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++)
    {
        if (PN532_ACK[i] != buff[i]) {
            printf("Did not receive expected ACK from PN532!");
            return PN532_STATUS_ERROR;
        }
    }

	return PN532_STATUS_OK;
}

int PN532_GetResponse(PN532_Typedef* pn532, uint8_t* response, uint16_t response_length, uint32_t timeout)
{
	uint8_t buff[PN532_FRAME_MAX_LENGTH];
    if (!PN532_WaitReady(pn532, timeout))
        return PN532_STATUS_ERROR;

    // Read response bytes.
    int frame_len = PN532_ReadFrame(pn532, buff, response_length + 2);

    // Return response data.
    for (uint8_t i = 0; i < response_length; i++)
    {
        response[i] = buff[i + 2];
    }
    // The the number of bytes read
    return frame_len - 2;
}

/**
  * @brief: Call PN532 GetFirmwareVersion function and return a buff with the IC,
  *  Ver, Rev, and Support values.
  */
int PN532_GetFirmwareVersion(PN532_Typedef* pn532, uint8_t* version)
{
    // length of version: 4
    if (PN532_CallFunction(pn532, PN532_COMMAND_GETFIRMWAREVERSION,
                           version, 4, NULL, 0, 500) == PN532_STATUS_ERROR)
    {
        printf("Failed to detect the PN532");
        return PN532_STATUS_ERROR;
    }
    return PN532_STATUS_OK;
}

/**
  * @brief: Configure the PN532 to read MiFare cards.
  */
int PN532_SamConfiguration(PN532_Typedef* pn532)
{
    // - 0x01, normal mode
    // - 0x14, timeout 50ms * 20 = 1 second
    // - 0x01, use IRQ pin
    uint8_t params[] = {0x01, 0x14, 0x01};
    PN532_CallFunction(pn532, PN532_COMMAND_SAMCONFIGURATION,
                       NULL, 0, params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return PN532_STATUS_OK;
}

int PN532_StartPassiveTargetIDDetection(PN532_Typedef* pn532, uint8_t card_baud, uint32_t timeout)
{
	uint8_t params[] = {0x01, card_baud};
	return PN532_SendCommand(pn532, PN532_COMMAND_INLISTPASSIVETARGET, params, sizeof(params), timeout);
}

int PN532_ReadDetectedID(PN532_Typedef* pn532, uint8_t* response, uint32_t timeout)
{
	uint8_t buff[19];

	 PN532_GetResponse(pn532, buff, sizeof(buff), timeout);

    if (buff[0] != 0x01)
        return PN532_STATUS_ERROR;

    if (buff[5] > 7)
        printf("Found card with unexpectedly long UID!");

    for (uint8_t i = 0; i < buff[5]; i++)
        response[i] = buff[6 + i];

    return buff[5];
}



