/******************************************************************************
* File Name:   fake_uicc_modem_uart.c
*
* Description: This file implements a fake modem.
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

#include "variant_config.h"  /* for VARIANT_MODEM */

#if (VARIANT_MODEM == FAKE_VARIANT)

#include "cy_debug.h"
#include "cy_memtrack.h"
#include "cy_uicc_modem.h"


/*-- Public Functions -------------------------------------------------*/

bool Modem_Uart_Init(void)
{
    return true;
}

bool Modem_Uart_Deinit(void)
{
    return true;
}

UICC_Result_t Modem_GetModemList(_out_ Modem_List_t **pListModems_p)
{
    UICC_Result_t result = UICC_NO_ERROR;
    Modem_List_t *output_p = NULL;

    ReturnAssert(pListModems_p != NULL, UICC_ARGUMENT_ERROR);

    output_p = (Modem_List_t *)CY_MEMTRACK_MALLOC(sizeof(*output_p));
    ReturnAssert(output_p != NULL, UICC_NOT_ENOUGH_MEMORY);

    memset(output_p, 0, sizeof(*output_p));

    *pListModems_p = output_p;

    return result;
}


Modem_Handle_t Modem_Open(_in_ const char* serialPortName_p)
{
    return (Modem_Handle_t)1;
}


void Modem_Close(_in_ Modem_Handle_t handle)
{
    (void)handle;
}


bool Modem_WriteCommand(_in_ Modem_Handle_t handle,
                        _in_ const uint8_t *commandBuf_p,
                        _in_ size_t commandBufSize)
{
    return true;
}

size_t Modem_ReadResponse(_in_  Modem_Handle_t handle,
                          _in_  uint32_t readTimeoutMsec,
                          _out_ uint8_t *responseBuf_p,
                          _in_  size_t maxResponseBufSize)
{
    return 0;
}


bool Modem_InstallReadCallback(_in_  Modem_Handle_t handle,
                               _in_  void *data_p)
{
    return true;
}


void Modem_DeleteReadCallback(_in_  Modem_Handle_t handle)
{

}


void Modem_EnableReadCallback(_in_  Modem_Handle_t handle)
{
}


void Modem_DisableReadCallback(_in_  Modem_Handle_t handle)
{
}

bool Modem_SetBaudRate( _in_  Modem_Handle_t handle,
                        _in_  uint32_t baudrate)
{
    return true;
}

bool Modem_IsFlagFirstRead( _in_ Modem_Handle_t handle)
{
    return true;
}

#endif /* VARIANT_MODEM */
