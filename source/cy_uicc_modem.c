/******************************************************************************
* File Name:   cy_uicc_modem.c
*
* Description: This file implements functions to communicate with the UICC of
*              a modem
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <ctype.h>
#include <stdlib.h>

#include "cy_uicc_modem.h"
#include "cy_misc_types.h"
#include "cy_debug.h"
#include "cy_memtrack.h"

#include "cy_string.h"
#include "cy_byte_array.h"
#include "cyabs_rtos.h"


/*-- Local Definitions -------------------------------------------------*/

#define FLUSH_READ_TIMEOUT_MSEC       10
#define INTER_WORD_READ_TIMEOUT_MSEC  3000

#define PORT_NAME_PREFIX        ""
#define MAX_AT_RESP_LEN         256

#define MAX_BUF_LEN             600
#define RSP_DATA_SIZE           600

#define SEND_COMMAND_IN_PARTS   0 // was 1
#define DEFAULT_MODEM           "Default Modem"


/*-- Local Functions -------------------------------------------------*/

static bool startsWith( const uint8_t *buffer_p,
                        size_t bufferSize,
                        const uint8_t *pattern_p,
                        size_t patternSize)
{
    if (bufferSize >= patternSize) {
        return (memcmp(pattern_p, buffer_p, patternSize) == 0);
    }
    return false;
}

static bool endsWith( const uint8_t *buffer_p,
                      size_t bufferSize,
                      const uint8_t *pattern_p,
                      size_t patternSize)
{
    if (bufferSize >= patternSize) {
        return (memcmp(pattern_p, &buffer_p[bufferSize - patternSize], patternSize) == 0);
    }
    return false;
}

// remove trailing \r and \n
// Note: function will modify text_p
static char* removeTrailingCrLf(char* text_p,
                                size_t *numResponseBytes_p)
{
    char *temp = NULL;

    if (text_p != NULL) {
        size_t len = strlen(text_p);
        if (len > 0) {
            temp = &text_p[len - 1];

            while ((*temp == '\r') || (*temp == '\n')) {
                *temp = '\0';
                temp--;
                (*numResponseBytes_p)--;

                if (temp == text_p) {
                    break;
                }
            }
        }
        temp = text_p;
    }
    return temp;
}

static void ExtractResponse(_inout_ uint8_t *responseBuf_p,
                            _inout_ size_t *numResponseBytes_p)
{
    const char ERROR_STR[] = AT_RSP_ERROR_END;
    const size_t ERROR_STR_LEN = strlen(ERROR_STR);
    const char OK_STR[] = AT_RSP_OK_END;
    const size_t OK_STR_LEN = strlen(OK_STR);
    const char BEGIN_STR[] = AT_RSP_START;
    const size_t BEGIN_STR_LEN = strlen(BEGIN_STR);
    size_t numBytesRead = *numResponseBytes_p;

    if (endsWith( (const uint8_t*)responseBuf_p,
                  numBytesRead,
                  (const uint8_t*)OK_STR, OK_STR_LEN)) {

        if (startsWith( (const uint8_t*)responseBuf_p,
                        numBytesRead,
                        (const uint8_t*)BEGIN_STR, BEGIN_STR_LEN)) {

            numBytesRead -= (BEGIN_STR_LEN + OK_STR_LEN);
            memmove(responseBuf_p, &responseBuf_p[BEGIN_STR_LEN], numBytesRead);
            responseBuf_p[numBytesRead] = '\0';

            removeTrailingCrLf( (char*)responseBuf_p,
                                &numBytesRead);

            //DEBUG_PRINT(("%s [%d]: success, responseBuf_p=%s\n", __FUNCTION__, __LINE__, responseBuf_p));

            *numResponseBytes_p = numBytesRead;
        } else {
            DEBUG_PRINT(("%s [%d]: unexpected response\n", __FUNCTION__, __LINE__));
        }
    } else if (endsWith((const uint8_t*)responseBuf_p,
                        numBytesRead,
                        (const uint8_t*)ERROR_STR, ERROR_STR_LEN)) {
        DEBUG_PRINT(("%s [%d]: error\n", __FUNCTION__, __LINE__));
    }
}

static bool Receive(Modem_Handle_t handle,
                    uint8_t *responseBuf_p,
                    size_t maxResponseBufSize,
                    size_t *numResponseBytes_p)
{
#define RECEIVE_MAX_TRIES       2
#define RECEIVE_WAIT_DURATION   50  // ms

    size_t tries = RECEIVE_MAX_TRIES;    // timeOut = tries x waitDuration
    size_t numBytesRead;

    while (tries) {
        numBytesRead = Modem_ReadResponse(handle,
                                          INTER_WORD_READ_TIMEOUT_MSEC,
                                          responseBuf_p,
                                          maxResponseBufSize - 1); /* reserve a space for null terminator */

        if (numBytesRead > 0) {
            //DEBUG_PRINT(("%s [%d]: %s\n", __FUNCTION__, __LINE__, (char*)responseBuf_p));
            break;
        } else {
            cy_rtos_delay_milliseconds(RECEIVE_WAIT_DURATION);
        }
        tries--;
    }

    responseBuf_p[numBytesRead] = '\0';

    *numResponseBytes_p = numBytesRead;
    return (tries != 0);

#undef RECEIVE_WAIT_DURATION
#undef RECEIVE_MAX_TRIES
}

static bool ReceiveAll( Modem_Handle_t handle,
                        uint8_t *responseBuf_p,
                        size_t maxResponseBufSize,
                        size_t *numResponseBytes_p)
{
    bool retValue;

    *numResponseBytes_p = 0;

    //while (true) {
    do {
        size_t numBytesRead = 0;
        retValue = Receive( handle,
                            &responseBuf_p[*numResponseBytes_p],
                            maxResponseBufSize - (*numResponseBytes_p),
                            &numBytesRead);

        if (!retValue || (numBytesRead == 0)) {
            DEBUG_PRINT(("%s [%d]: retValue=%d, numBytesRead=%lu\n",
                         __FUNCTION__, __LINE__, retValue, (unsigned long)numBytesRead));
            break;
        }

        //DEBUG_PRINT(("%s [%d]: numBytesRead=%lu\n",
        //  __FUNCTION__, __LINE__, (unsigned long)numBytesRead));
        //print_bytes("responseBuf_p: ", responseBuf_p, numBytesRead);

        /*retValue =*/ ExtractResponse( (uint8_t*)responseBuf_p,
                                        &numBytesRead);

        //DEBUG_PRINT(("%s [%d]: numBytesRead=%lu\n",
        //  __FUNCTION__, __LINE__, (unsigned long)numBytesRead));
        //print_bytes("responseBuf_p: ", responseBuf_p, numBytesRead);

        *numResponseBytes_p += numBytesRead;
        /*
        if (retValue) {
          DEBUG_PRINT(("%s [%d]: break\n", __FUNCTION__, __LINE__));
          break;
        } */

    } while (false);

    if (*numResponseBytes_p > 0) {
        //if (*numResponseBytes_p >= 0) {
        DEBUG_PRINT(("%s [%d]: %s\n", __FUNCTION__, __LINE__, (char*)responseBuf_p));
        //print_bytes("responseBuf_p: ", responseBuf_p, *numResponseBytes_p);

        return true;
    }

    return false;
}

#if (SEND_COMMAND_IN_PARTS == 1)
static bool SendByteArrayAsString(_in_ Modem_Handle_t handle,
                                  _in_ const uint8_t *array_p,
                                  _in_ size_t arraySize)
{
    bool returnValue = true;
    size_t i;

    ReturnAssert(handle != NULL, false);
    ReturnAssert(array_p != NULL, false);

    for (i = 0; i < arraySize; i++) {
        char tempBuf[3];

        SNPRINTF(tempBuf, sizeof(tempBuf), "%02X", array_p[i]);
        tempBuf[2] = '\0';  /* make sure it's null-terminated */

        /* if (i >= 60) {
          DEBUG_PRINT(("%s [%d]: %d of %d: %s\n",
              __FUNCTION__, __LINE__, i, arraySize, tempBuf));
        } */

        returnValue = Modem_WriteCommand( handle,
                                          (const uint8_t*)tempBuf,
                                          strlen(tempBuf));
        if (!returnValue) {
            DEBUG_PRINT(("%s [%d]: failed at %d of %d: %s\n",
                         __FUNCTION__, __LINE__, i, arraySize, tempBuf));
        }

        ReturnAssert(returnValue, false);
    }

    return returnValue;
}
#endif


/*-- Public Functions -------------------------------------------------*/

UICC_Result_t Modem_SendApduCommandHelper(_in_ Modem_Handle_t handle,
        _in_ const uint8_t *commandBuf_p,
        _in_ size_t commandBufSize,
        _out_ uint8_t *responseBuf_p,
        _out_ size_t *responseBufSize_p)
{
#define RECEIVE_MAX_TRIES       2
#define RECEIVE_WAIT_DURATION   50  // ms

#define AT_CSIM_CMD_PADDED_LEN  16
#define AT_CSIM_RESP_PADDED_LEN 10

    bool returnValue;

#if (SEND_COMMAND_IN_PARTS == 1)
    char strBuffer[AT_CSIM_CMD_PADDED_LEN];
#else
    size_t tempLen;
    char strBuffer[MAX_BUF_LEN + AT_CSIM_CMD_PADDED_LEN];
#endif

    size_t retries = RECEIVE_MAX_TRIES;
    const char expectedResponse[] = AT_RSP_CSIM;

    ReturnAssert(handle != INVALID_HANDLE, UICC_ARGUMENT_ERROR);
    ReturnAssert(commandBuf_p != NULL, UICC_ARGUMENT_ERROR);
    ReturnAssert(responseBuf_p != NULL, UICC_ARGUMENT_ERROR);
    ReturnAssert(responseBufSize_p != NULL, UICC_ARGUMENT_ERROR);

#if (SEND_COMMAND_IN_PARTS == 1)
    SNPRINTF(strBuffer, sizeof(strBuffer), AT_CMD_CSIM_START,
             (unsigned)commandBufSize * 2);
    returnValue = Modem_WriteCommand( handle,
                                      (const uint8_t*)strBuffer,
                                      strlen(strBuffer));
    ReturnAssert(returnValue, UICC_UNDEFINED_ERROR);

    returnValue = SendByteArrayAsString(handle,
                                        commandBuf_p,
                                        commandBufSize);
    ReturnAssert(returnValue, UICC_UNDEFINED_ERROR);

    SNPRINTF(strBuffer, sizeof(strBuffer), AT_CMD_CSIM_END);
    returnValue = Modem_WriteCommand( handle,
                                      (const uint8_t*)strBuffer,
                                      strlen(strBuffer));
    ReturnAssert(returnValue, UICC_UNDEFINED_ERROR);

#else
    ReturnAssert((2 * commandBufSize + AT_CSIM_CMD_PADDED_LEN) < sizeof(strBuffer), UICC_ARGUMENT_ERROR);

    SNPRINTF(strBuffer, sizeof(strBuffer), AT_CMD_CSIM_START, 2 * commandBufSize);
    tempLen = strlen(strBuffer);

    returnValue = ByteArrayToString(commandBuf_p,
                                    commandBufSize,
                                    &strBuffer[tempLen],
                                    sizeof(strBuffer) - tempLen);
    ReturnAssert(returnValue, UICC_UNDEFINED_ERROR);

    //strcat(strBuffer, "\"\r\n");
    strcat(strBuffer, AT_CMD_CSIM_END);

    DEBUG_PRINT(("len=%d, strBuffer=%s\n", strlen(strBuffer), strBuffer));

    returnValue = Modem_WriteCommand( handle,
                                      (const uint8_t*)strBuffer,
                                      strlen(strBuffer));
    ReturnAssert(returnValue, UICC_UNDEFINED_ERROR);
#endif

    while (retries > 0) {
        const char CME_ERROR[] = AT_RSP_CME_ERROR;
        const size_t CME_ERROR_LEN = strlen(CME_ERROR);

        size_t i = 0;
        size_t numBytesRead = 0;
        char *test_p;

#if (SEND_COMMAND_IN_PARTS == 1)
        uint8_t *recvBuf_p = responseBuf_p;
        size_t maxResponseBufSize = *responseBufSize_p;
#else
        uint8_t *recvBuf_p = (uint8_t*)strBuffer;
        size_t maxResponseBufSize = sizeof(strBuffer);
#endif

        returnValue = ReceiveAll( handle,
                                  recvBuf_p,
                                  maxResponseBufSize,
                                  &numBytesRead);

        if (!returnValue) {
            DEBUG_PRINT(("%s [%d]: ReceiveAll failed\n", __FUNCTION__, __LINE__));
            //return UICC_UNDEFINED_ERROR;
        }

        if ((numBytesRead > CME_ERROR_LEN) &&
                (strstr((char*)recvBuf_p, CME_ERROR) != NULL)) {
            DEBUG_PRINT(("%s [%d]: %s received\n", __FUNCTION__, __LINE__, CME_ERROR));
            return UICC_UNDEFINED_ERROR;
        }

        /* expected response: +CSIM: N,"xx..."  */
        if ((numBytesRead > strlen(expectedResponse)) &&
                ((test_p = strstr((char*)recvBuf_p, expectedResponse)) != NULL)) {
            size_t numItems;
            /*size_t*/ unsigned int len;

            i = (uint8_t*)test_p - recvBuf_p;
            //DEBUG_PRINT(("i=%u\n", (unsigned)i));

            i += strlen(expectedResponse);
            DEBUG_ASSERT(i < numBytesRead);

            /* get the length of hex data (between the quotes) */
            numItems = sscanf((const char*)&recvBuf_p[i], "%u", &len);

            /* hex string must have even-number length */
            ReturnAssert(len % 2 == 0, UICC_UNDEFINED_ERROR);

            if (numItems > 0) {
                bool found = false;
                size_t startOfData = 10;  /* estimated len of: +CSIM: N," */

                /* must find start quote in response */
                for (; i < numBytesRead; i++) {
                    if (recvBuf_p[i] == '\"') {
                        found = true;
                        break;
                    }
                }

                if (found) {
                    //DEBUG_PRINT(("i=%u, recvBuf_p[i]=%c\n", (unsigned)i, recvBuf_p[i]));
                    test_p = (char*)&recvBuf_p[i];
                }

                if (!found) {
                    DEBUG_PRINT(("%s [%d]: cannot find start quote\n", __FUNCTION__, __LINE__));

                    Modem_FlushAll(handle);

                    /* tell the caller to supply a buffer large enough to store the end quote, OK and CrLf's */
                    return MAKE_ULONG((startOfData + len + AT_CSIM_RESP_PADDED_LEN), UICC_TOO_SMALL_BUFFER);
                }

                /* advance to start of data */
                i++;
                startOfData = i;
                test_p = (char*)&recvBuf_p[i];

                found = false;

                /* must find end quote in response */
                for (; i < numBytesRead; i++) {
                    if (recvBuf_p[i] == '\"') {
                        /* replace end quote with null terminator */
                        recvBuf_p[i] = '\0';
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    DEBUG_PRINT(("%s [%d]: cannot find end quote\n", __FUNCTION__, __LINE__));

                    Modem_FlushAll(handle);

                    /* tell the caller to supply a buffer large enough to store the end quote, OK and CrLf's */
                    return MAKE_ULONG((startOfData + len + AT_CSIM_RESP_PADDED_LEN), UICC_TOO_SMALL_BUFFER);
                }

                /* assert that the hex string len is indeed correct */
                DEBUG_ASSERT(len == (unsigned)strlen(test_p));

                if (*responseBufSize_p < (len /2)) {
                    /* tell the caller to supply a buffer large enough to store the byte array */
                    return MAKE_ULONG(len/2, UICC_TOO_SMALL_BUFFER);
                }

                returnValue = StringToByteArray(test_p,
                                                responseBuf_p,
                                                *responseBufSize_p);

                /* return byte array length, which is half the hex string length */
                *responseBufSize_p = len / 2;
            }

            return UICC_NO_ERROR;
        } else {
            cy_rtos_delay_milliseconds(RECEIVE_WAIT_DURATION);
        }

        retries--;
    }

    return UICC_UNDEFINED_ERROR;

#undef RECEIVE_WAIT_DURATION
#undef RECEIVE_MAX_TRIES
#undef AT_CSIM_PADDED_LEN
#undef AT_CSIM_RESP_PADDED_LEN
}


int Modem_Init( _in_  Modem_Handle_t handle,
                _out_ char *modelName_p,
                _in_  size_t modelNameBufSize,
                _out_ char *imei_p,
                _in_  size_t imeiBufSize)
{
#define MAX_INIT_TRIES    1
#define READ_IMEI         0 /* 1:enable; 0=disable */

    int tries;        /* Number of tries so far */

    DEBUG_PRINT(("%s [%d]\n", __FUNCTION__, __LINE__));

    for (tries = 0; tries < MAX_INIT_TRIES; tries++) {

        if (Modem_SendATCommand(handle,
                                AT_CMD_ECHO_OFF,
                                NULL,
                                0)) {
        }

        if (Modem_SendATCommand(handle,
                                AT_CMD_MODEL,
                                modelName_p,
                                modelNameBufSize)) {

            if (strstr(modelName_p, AT_RSP_ERROR) != NULL) {
                /* if we receive an ERROR msg, the modem is present,
                   but we need to close & reopen */
                return RESULT_MODEM_RETRY;
            }
        }

#if READ_IMEI
        if (Modem_SendATCommand(handle,
                                AT_CMD_IMEI,
                                imei_p,
                                imeiBufSize)) {

            if (strstr(imei_p, AT_RSP_ERROR) != NULL) {
                /* if we receive an ERROR msg, the modem is present,
                   but we need to close & reopen */
                return RESULT_MODEM_RETRY;
            }
        }

        if ((strlen(modelName_p) > 0) &&
                (strlen(imei_p) > 0)) {
            return RESULT_MODEM_OK;
        }

#else
        DEBUG_PRINT(("modelName_p = %s\n", modelName_p));
        if (strlen(modelName_p) > 0) {
            return RESULT_MODEM_OK;
        }

        // assume there is always a modem present
        strcpy(modelName_p, DEFAULT_MODEM);
        return RESULT_MODEM_OK;
#endif
    }

    return RESULT_MODEM_ABSENT;
}


bool Modem_IsFound(
    _in_ Modem_List_t *listModems_p,
    _in_ const char *modelName_p,
    _in_ const char *imei_p)
{
    ReturnAssert(listModems_p != NULL, false);
    ReturnAssert(modelName_p != NULL, false);
    ReturnAssert(imei_p != NULL, false);

    if (listModems_p->modems != NULL) {
        size_t i;
        for (i = 0; i < listModems_p->numModems; i++) {
            if ((strcmp(modelName_p, listModems_p->modems[i].modelName_p) == 0) &&
                    (strcmp(imei_p, listModems_p->modems[i].imei_p) == 0)) {
                return true;
            }
        }
    }

    return false;
}


int Modem_Probe(_in_  const char *portName_p,
                _out_ char *modelName_p,
                _in_  size_t modelNameBufSize,
                _out_ char *imei_p,
                _in_  size_t imeiBufSize)
{
#define MAX_RETRIES   1

    int result = RESULT_MODEM_ABSENT;
    size_t tries = 0;

    ReturnAssert(portName_p != NULL, RESULT_MODEM_ABSENT);
    ReturnAssert(modelName_p != NULL, RESULT_MODEM_ABSENT);
    ReturnAssert(imei_p != NULL, RESULT_MODEM_ABSENT);

    //DEBUG_PRINT(("%s [%d]: %s\n", __FUNCTION__, __LINE__, portName_p));

    while (true) {
        Modem_Handle_t handle;
        handle = Modem_Open(portName_p);

        if (handle != INVALID_HANDLE) {
            Modem_FlushAll(handle);

            result = Modem_Init(handle,
                                modelName_p,
                                modelNameBufSize,
                                imei_p,
                                imeiBufSize);

            Modem_Close(handle);

            if ((result != RESULT_MODEM_RETRY) ||
                    (tries >= MAX_RETRIES)) {
                break;
            }
        } else {
            DEBUG_PRINT(("%s [%d]: Unable to open %s - check access permission\n",
                         __FUNCTION__, __LINE__, portName_p));
            break;
        }

        tries++;
    }

    return result;

#undef MAX_RETRIES
}


void Modem_MakePortName(_in_  const char* rawName_p,
                        _in_  size_t rawNameLen,
                        _out_ char* serialportName_p,
                        _in_  size_t maxSerialPortBufSize)
{
    size_t prefixLen;

    VoidAssert(rawName_p != NULL);
    VoidAssert(serialportName_p != NULL);

    prefixLen = strlen(PORT_NAME_PREFIX);

    if ((prefixLen + rawNameLen) < maxSerialPortBufSize) {
        if (prefixLen > 0) {
            strcpy(serialportName_p, PORT_NAME_PREFIX);
        }
        memcpy(&serialportName_p[prefixLen], rawName_p, rawNameLen);
        serialportName_p[prefixLen + rawNameLen] = '\0';
    }
}


/* (Win32)
   terminalName:    "SIM_2 SimTech HS-USB AT Port 9001 (COM27)"
   serialPortName:  "\\\\.\\COM27"

   (Ubuntu)
   terminalName:    "SIM_1 SIMCOM_SIM7600G-H (/dev/ttyUSB2)"
   serialPortName:  "/dev/ttyUSB2"
*/
UICC_Result_t Modem_ExtractPortName(_in_  const char* terminalName_p,
                                    _out_ char* serialportName_p,
                                    _in_  size_t maxSerialPortBufSize)
{
    size_t tempLen;

    ReturnAssert(terminalName_p != NULL, UICC_ARGUMENT_ERROR);
    ReturnAssert(serialportName_p != NULL, UICC_ARGUMENT_ERROR);

    tempLen = strlen(terminalName_p);
    if (terminalName_p[tempLen - 1] == ')') {
        char *temp_p = (char*)&terminalName_p[tempLen - 2];
        while (temp_p != terminalName_p) {
            if (*temp_p == '(') {
                break;
            }
            --temp_p;
        }

        if (*temp_p == '(') {
            temp_p++;

            Modem_MakePortName( temp_p,
                                strlen(temp_p) - 1,   /* subtract 1 for ')' */
                                serialportName_p,
                                maxSerialPortBufSize);

            return UICC_NO_ERROR;
        }
    }

    return UICC_UNDEFINED_ERROR;
}


void Modem_FreeModemList(_in_ Modem_List_t *listModems_p)
{
    if (listModems_p != NULL) {
        if (listModems_p->modems != NULL) {
            size_t i;

            for (i = 0; i < listModems_p->numModems; i++) {
                CY_MEMTRACK_FREE(listModems_p->modems[i].portName_p);
                CY_MEMTRACK_FREE(listModems_p->modems[i].modelName_p);
                CY_MEMTRACK_FREE(listModems_p->modems[i].imei_p);
            }

            CY_MEMTRACK_FREE(listModems_p->modems);
        }
    }
    CY_MEMTRACK_FREE(listModems_p);
}


UICC_Result_t Modem_SimTransReceive(_in_ Modem_Handle_t handle,
                                    _in_ const UICC_Buffer_t *PpsCommand,
                                    _out_ UICC_Buffer_t *PpsResponse)
{
#define USE_TEMP_RESPONSE_BUFFER     0
#define MIN_RESPONSE_BUF_SIZE       21

    UICC_Result_t retStatus = UICC_UNDEFINED_ERROR;

    size_t ui4RspLength;
    uint16_t ui2StatusWord = 0x0000;
    uint8_t ui1Sw1 = 0x00;
    uint8_t ui1Sw2 = 0x00;

    ReturnAssert(handle != INVALID_HANDLE, UICC_ARGUMENT_ERROR);
    ReturnAssert(PpsCommand != NULL, UICC_ARGUMENT_ERROR);
    ReturnAssert(PpsCommand->p != NULL, UICC_ARGUMENT_ERROR);
    ReturnAssert(PpsResponse != NULL, UICC_ARGUMENT_ERROR);
    ReturnAssert(PpsResponse->p != NULL, UICC_ARGUMENT_ERROR);

    if (PpsResponse->allocated < MIN_RESPONSE_BUF_SIZE) {
        return MAKE_ULONG(MIN_RESPONSE_BUF_SIZE, UICC_TOO_SMALL_BUFFER);
    }

    do {
        uint8_t firstCmdByte = PpsCommand->p[0];
        ui4RspLength = PpsResponse->allocated;

        //DEBUG_PRINT(("\nRemainingResponseLength=%lu\n",
        //  (unsigned long)PpsResponse->allocated));

        /* Send the command to the target card and receive the response for a command */
        if (UICC_NO_ERROR != (retStatus = Modem_SendApduCommandHelper(
                                              handle,
                                              PpsCommand->p,
                                              PpsCommand->len,
                                              PpsResponse->p,
                                              &ui4RspLength))) {
            break;
        }

        PpsResponse->len = (uint16_t)ui4RspLength;

        /* The response length should be at least the length of the status word(SW) which is 2 bytes */
        if (PpsResponse->len < SIZE_OF_SW ) {
            break;
        }

        READU16(&PpsResponse->p[PpsResponse->len - SIZE_OF_SW], ui2StatusWord);
        ui1Sw1 = HIBYTE(ui2StatusWord);
        ui1Sw2 = LOBYTE(ui2StatusWord);

        if (ui1Sw1 == 0x61) {
            UICC_Result_t ui2TempRetStatus = UICC_NO_ERROR;

            print_bytes("<-- ", PpsResponse->p, PpsResponse->len);

            do {
                /* temporary buffer to store the APDU command */
                //uint8_t ui1CmdBuffer[] = {PpsCommand->p[0], 0xC0, 0x00, 0x00, ui1Sw2};
                uint8_t ui1CmdBuffer[] = {firstCmdByte, 0xC0, 0x00, 0x00, ui1Sw2};
                uint8_t *responseBuf_p;

#if (USE_TEMP_RESPONSE_BUFFER == 1)
                /* temporary buffer to store the APDU response */
                uint8_t ui1RspBuffer[RSP_DATA_SIZE];
                responseBuf_p = ui1RspBuffer;
                ui4RspLength = sizeof(ui1RspBuffer);

                /* discard the previous status word */
                PpsResponse->len -= SIZE_OF_SW;

#else

                /* discard the previous status word */
                PpsResponse->len -= SIZE_OF_SW;

                /* append next response to the previous one (overwriting the previous status word) */
                responseBuf_p = &PpsResponse->p[PpsResponse->len];
                ui4RspLength = PpsResponse->allocated - PpsResponse->len;

#endif
                print_bytes("--> ", ui1CmdBuffer, sizeof(ui1CmdBuffer));

                //DEBUG_PRINT(("\nRemainingResponseLength=%lu\n",
                //  (unsigned long)(PpsResponse->allocated - PpsResponse->len)));

                if (UICC_NO_ERROR != (ui2TempRetStatus = Modem_SendApduCommandHelper(
                                          handle,
                                          ui1CmdBuffer,
                                          sizeof(ui1CmdBuffer),
                                          responseBuf_p,
                                          &ui4RspLength))) {
                    if (LOWORD(ui2TempRetStatus) == UICC_TOO_SMALL_BUFFER) {
                        ui2TempRetStatus = MAKE_ULONG((PpsResponse->len + HIWORD(ui2TempRetStatus)),
                                                      UICC_TOO_SMALL_BUFFER);
                    }
                    break;
                }

                print_bytes("<-- ", responseBuf_p, ui4RspLength);

#if (USE_TEMP_RESPONSE_BUFFER == 1)
                /* ensure there is sufficient remaining response buffer */
                if ((PpsResponse->allocated - PpsResponse->len) < (uint16_t)ui4RspLength) {
                    //DEBUG_PRINT(("\nRemainingResponseLength=%d, ui4RspLength=%d\n",
                    //  PpsResponse->allocated - PpsResponse->len, ui4RspLength));

                    DEBUG_ASSERT((PpsResponse->allocated - PpsResponse->len) >= (uint16_t)ui4RspLength);

                    ui2TempRetStatus = UICC_UNDEFINED_ERROR;
                    break;
                }

                /* append the temporary response to the previous response (override the previous status word) */
                memcpy( &PpsResponse->p[PpsResponse->len],
                        responseBuf_p,
                        ui4RspLength);
#endif

                PpsResponse->len += ui4RspLength;

                READU16(&PpsResponse->p[PpsResponse->len - SIZE_OF_SW], ui2StatusWord);
                ui1Sw1 = HIBYTE(ui2StatusWord);
                ui1Sw2 = LOBYTE(ui2StatusWord);

                if (ui1Sw1 != 0x61) {
                    //DEBUG_PRINT(("\nRemainingResponseLength=%lu\n",
                    //  (unsigned long)(PpsResponse->allocated - PpsResponse->len)));
                }

            } while (ui1Sw1 == 0x61);

            if (ui2TempRetStatus != UICC_NO_ERROR) {
                retStatus = ui2TempRetStatus;
                break;
            }
        } else {
            DEBUG_PRINT(("ui2StatusWord=%04X\n", ui2StatusWord));

            //DEBUG_PRINT(("\nRemainingResponseLength=%lu\n",
            //  (unsigned long)(PpsResponse->allocated - PpsResponse->len)));
        }

        if (ui2StatusWord != ISO_STATUS_OK) {
            break;
        }

        retStatus = UICC_NO_ERROR;

    } while(false);

    return retStatus;

#undef USE_TEMP_RESPONSE_BUFFER
#undef MIN_RESPONSE_BUF_SIZE
}

void Modem_Reset(_in_ Modem_Handle_t handle)
{
#ifdef AT_CMD_RESET

    char buffer[MAX_AT_RESP_LEN] = "";  /* Input buffer */
    bool result;

    //DEBUG_PRINT(("%s [%d]: %s\n",
    //    __FUNCTION__, __LINE__, atCmd));

    result = Modem_SendATCommand( handle,
                                  AT_CMD_RESET,
                                  buffer,
                                  sizeof(buffer));
    (void)result;

    DEBUG_PRINT(("%s [%d]: %s\n",
                 __FUNCTION__, __LINE__, buffer));

#endif
}

static size_t GetATCmdLength(_in_  const char *commandString)
{
    /* if the command string is 'echoed' in the response,
     * we need to skip over it. To do that, we need to know
     * how many chars are present in the command string,
     * excluding the chars: ?, \r and \n */
    size_t nCmdChars = strlen(commandString);
    ReturnAssert(nCmdChars > 0, 0);

    /* ignore ?, \r or \n at the end of the command string */
    char *cmdPtr = (char *)&commandString[nCmdChars - 1];
    while (cmdPtr != commandString) {
        if ((*cmdPtr == '?') || (*cmdPtr == '\r') || (*cmdPtr == '\n')) {
            cmdPtr--;
        }
        else {
            break;
        }
    }
    nCmdChars = (size_t)(cmdPtr - commandString) + 1;
    return nCmdChars;
}

bool Modem_SendATCommand( _in_  Modem_Handle_t handle,
                          _in_  const char *commandString,
                          _out_ char *responseStr_p,
                          _in_  size_t maxResponseBufSize)
{
    bool retValue;
    size_t nbytes = 0;       /* Number of bytes read */
    char buffer[MAX_AT_RESP_LEN];

    DEBUG_PRINT(("%s [%d]: %s\n", __FUNCTION__, __LINE__, commandString));

    retValue = Modem_WriteCommand(handle,
                                  (const uint8_t*)commandString,
                                  strlen(commandString));

    if (retValue) {
        size_t temp;

        memset(buffer, 0, sizeof(buffer));

        /* read characters into response buffer until we get a CR or NL */
        while ((temp = Modem_ReadResponse(handle,
                                          INTER_WORD_READ_TIMEOUT_MSEC,
                                          (uint8_t*)&buffer[nbytes],
                                          sizeof(buffer) - nbytes)) > 0) {
            //DEBUG_PRINT(("%s [%d]: temp=%d\n", __FUNCTION__, __LINE__, temp));

            nbytes += temp;

            if (buffer[nbytes - 1] == '\r' || buffer[nbytes - 1] == '\n') {
                /* nul terminate the string and see if we got an OK response */
                buffer[nbytes - 1] = '\0';

                //DEBUG_PRINT(("%s [%d]: found CRLF\n", __FUNCTION__, __LINE__));
                //DEBUG_PRINT(("(%s)\n", buffer));
                break;
            }
        }

        if (nbytes > 0) {

            if (strstr(buffer, AT_RSP_OK) != NULL) {
                char *ptr;
                char *start;

                size_t nCmdLen = GetATCmdLength(commandString);

                /* look for start of response string */
                ptr = buffer;

                if (memcmp(buffer, commandString, nCmdLen) == 0) {
                    /* command string is 'echoed' in the response, skip over it */
                    ptr += nCmdLen;
                }

                start = ptr = left_trim(ptr);

                /* look for end of response string */
                right_trim(ptr);

                DEBUG_PRINT(("%s\n", start));

                /* if caller wants the response */
                if (responseStr_p != NULL) {
                    STRNCPY_APPEND_NULL(responseStr_p, start, maxResponseBufSize - 1);
                }
            }
            else {
                DEBUG_PRINT(("%s\n", buffer));

                /* if caller wants the response */
                if (responseStr_p != NULL) {
                    STRNCPY_APPEND_NULL(responseStr_p, buffer, maxResponseBufSize - 1);
                }
            }
        }

        return (nbytes > 0);
    }

    return retValue;
}

/* read until there is no more data */
void Modem_FlushAll(_in_ Modem_Handle_t handle)
{
#define MAX_FLUSH_TRIES     50
#define MAX_FLUSH_BUF_SIZE  20

    uint8_t recvBuf[MAX_FLUSH_BUF_SIZE];
    size_t tries = MAX_FLUSH_TRIES;

    //DEBUG_PRINT(("%s [%d]: Flush start\n", __FUNCTION__, __LINE__));

    while (tries > 0) {
        size_t numBytesRead;

        numBytesRead = Modem_ReadResponse(handle,
                                          FLUSH_READ_TIMEOUT_MSEC,
                                          recvBuf,
                                          sizeof(recvBuf));

        if (numBytesRead == 0) {
            break;
        } else {
            //print_bytes("data: ", recvBuf, numBytesRead);
        }

        tries--;
    }

    //DEBUG_PRINT(("%s [%d]: Flush done\n", __FUNCTION__, __LINE__));

#undef MAX_FLUSH_TRIES
#undef MAX_FLUSH_BUF_SIZE
}
