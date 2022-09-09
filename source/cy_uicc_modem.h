/******************************************************************************
* File Name:   cy_uicc_modem_h
*
* Description: This file is the public interface of cy_uicc_modem.c
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

#ifndef SOURCE_CY_UICC_MODEM_H_
#define SOURCE_CY_UICC_MODEM_H_

#include <stdbool.h>
#include "cy_uicc_common_data.h"
#include "cy_misc_types.h"
#include "cy_atmodem.h"

#ifdef __cplusplus
extern "C" {
#endif


/*-- Public Definitions -------------------------------------------------*/

#define INVALID_HANDLE            NULL

#define RESULT_MODEM_OK           0
#define RESULT_MODEM_ABSENT       -1
#define RESULT_MODEM_RETRY        -2

#define MAX_SERIAL_PORT_NAME_LEN  16
#define MAX_MODEM_MODEL_LEN       60
#define MAX_MODEM_IMEI_LEN        32 //16

#define DEFAULT_MODEM_SERIAL_PORT "SIM1"
#define DEFAULT_MODEM_BAUD_RATE   115200
#define MAX_MODEM_BAUD_RATE       PPP_MAX_MODEM_BAUD_RATE

#define INTER_WORD_READ_TIMEOUT_MSEC  3000
#define MAX_AT_RESP_LEN               256

typedef bool (*cb_modem_data_read)(void *buffer, size_t size, void *context);

typedef struct {
    cb_modem_data_read read_callback;
    void *read_callback_ctx;
    Modem_Handle_t handle;
} Modem_ReadCallback_t;


/*-- Public Functions -------------------------------------------------*/

bool Modem_IsFound(
    _in_ Modem_List_t *listModems_p,
    _in_ const char *modelName_p,
    _in_ const char *imei_p);

int Modem_Probe(_in_  const char *portName_p,
                _out_ char *modelName_p,
                _in_  size_t modelNameBufSize,
                _out_ char *imei_p,
                _in_  size_t imeiBufSize);

void Modem_MakePortName(_in_  const char* rawName_p,
                        _in_  size_t rawNameLen,
                        _out_ char* serialportName_p,
                        _in_  size_t maxSerialPortBufSize);

UICC_Result_t Modem_ExtractPortName(_in_  const char* terminalName_p,
                                    _out_ char* serialportName_p,
                                    _in_  size_t maxSerialPortBufSize);

UICC_Result_t Modem_GetModemList(
    _out_ Modem_List_t **pListModems_p);

void Modem_FreeModemList(_in_ Modem_List_t *listModems_p);

Modem_Handle_t Modem_Open(_in_  const char* serialPortName_p);

void Modem_Close(_in_  Modem_Handle_t handle);

UICC_Result_t Modem_SimTransReceive(_in_ Modem_Handle_t handle,
                                    _in_ const UICC_Buffer_t *PpsCommand,
                                    _out_ UICC_Buffer_t *PpsResponse);

int Modem_Init( _in_  Modem_Handle_t handle,
                _out_ char *modelName_p,
                _in_  size_t modelNameBufSize,
                _out_ char *imei_p,
                _in_  size_t imeiBufSize);

bool Modem_Uart_Init(void);

bool Modem_Uart_Deinit(void);

void Modem_Reset(_in_ Modem_Handle_t handle);

UICC_Result_t Modem_SendApduCommandHelper(_in_ Modem_Handle_t handle,
        _in_ const uint8_t *commandBuf_p,
        _in_ size_t commandBufSize,
        _out_ uint8_t *responseBuf_p,
        _out_ size_t *responseBufSize_p);

bool Modem_SendATCommand( _in_  Modem_Handle_t handle,
                          _in_  const char *commandString,
                          _out_ char *responseBuf_p,
                          _in_  size_t maxResponseBufSize);

bool Modem_SendATCommandEx(_in_  Modem_Handle_t handle,
                           _in_  const char *commandString,
                           _out_ char *responseBuf_p,
                           _in_  size_t maxResponseBufSize,
                           _in_  size_t waitTimeBeforeReadMsec,
                           _in_  size_t timeoutReadMsec,
                           _in_  bool keepOk);

void Modem_FlushAll(_in_ Modem_Handle_t handle);

bool Modem_WriteCommand(_in_ Modem_Handle_t handle,
                        _in_ const uint8_t *commandBuf_p,
                        _in_ size_t commandBufSize);

size_t Modem_ReadResponse(_in_  Modem_Handle_t handle,
                          _in_  uint32_t readTimeoutMsec,
                          _out_ uint8_t *responseBuf_p,
                          _in_  size_t maxResponseBufSize);

bool Modem_InstallReadCallback(_in_ Modem_Handle_t handle,
                               _in_ void *data_p);

void Modem_DeleteReadCallback(_in_ Modem_Handle_t handle);

void Modem_EnableReadCallback(_in_ Modem_Handle_t handle);

void Modem_DisableReadCallback(_in_ Modem_Handle_t handle);

bool Modem_SetBaudRate( _in_ Modem_Handle_t handle,
                        _in_ uint32_t baudrate);

bool Modem_IsFlagFirstRead( _in_ Modem_Handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_CY_UICC_MODEM_H_*/
