/******************************************************************************
* File Name:   cy_uicc_modem_uart.c
*
* Description: This file implements an UART modem that provides access to its
*              UICC
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

#if (VARIANT_MODEM == HW_VARIANT)

#include <stdlib.h>
#include "feature_config.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "cy_debug.h"
#include "cy_memtrack.h"
#include "cy_string.h"
#include "cy_byte_array.h"

#include "cy_uicc_modem.h"
#include "cyabs_rtos.h"


/*-- Local Definitions -------------------------------------------------*/

#define ONLY_ENUMERATE_ONE_MODEM      0   // (1:one modem only; 0:can have multiple modems)

#define INTER_BYTE_READ_TIMEOUT_MSEC      10
#define ISR_INTER_BYTE_READ_TIMEOUT_MSEC  1

// on Cortex M3 and M4: a smaller number indicate higher interrupt priority
// 0=highest, 7=lowest
// We are using configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, because our Uart ISR
// calls FreeRTOS API (ending in FromISR)
#define INT_PRIORITY                  6 // 1 to 4 (are too high)

#define UART_READ_TASK_STACK_SIZE     4096

#define UART_READ_TASK_PRIORITY       CY_RTOS_PRIORITY_ABOVENORMAL

#define UART_READ_IRQ_EVENT_TYPE          CYHAL_UART_IRQ_RX_NOT_EMPTY
#define MAX_UART_READ_TRIES               1000
#define MAX_UART_READ_CONSECUTIVE_FAILS   3

#define USE_CIRCULAR_BUFFER             1 // 1:enable, 0:disable

#if USE_CIRCULAR_BUFFER
#if (MAX_MODEM_BAUD_RATE <= 230400)
    #define CIRCULAR_RX_BUFFER_SIZE     8192 // 4096 failed on MURATA

#else
    #define CIRCULAR_RX_BUFFER_SIZE     16384
#endif

#else
#if (MAX_MODEM_BAUD_RATE <= 230400)
    /* use this block if MAX_MODEM_BAUD_RATE <= 230400 */
    #define UART_RX_BUF_SIZE              2     //was 1
    #define UART_RX_DATA_QUEUE_SIZE       5120  //was 10240

#elif (MAX_MODEM_BAUD_RATE <= 921600)
    /* use this block if MAX_MODEM_BAUD_RATE <= 921600 */
    #define UART_RX_BUF_SIZE              2     //was 1
    #define UART_RX_DATA_QUEUE_SIZE       10240 //was 20480

#else
    /* use this block if MAX_MODEM_BAUD_RATE > 921600 */
    #define UART_RX_BUF_SIZE              80
    #define UART_RX_DATA_QUEUE_SIZE       512
#endif
#endif


typedef struct {
    // identifier
    uint32_t magic;

    // hardware-related
    const char* portName_p;
    cyhal_gpio_t rx;
    cyhal_gpio_t tx;
    uint32_t baudrate;

    // instance members
    cyhal_uart_t *uart_obj_p;
    int handleRefCount;     /* increment on ModemOpen; decrement on ModemClose */
    bool uart_irq_enabled;
    bool tx_done;
    bool isFlagFirstRead;

#if USE_CIRCULAR_BUFFER
    uint8_t  rxdata[CIRCULAR_RX_BUFFER_SIZE];
    uint16_t rxdata_head_index;
    uint16_t rxdata_tail_index;
    cy_semaphore_t rxdata_recv_sem;

#else
    cy_queue_t rx_queue;
#endif

    cy_thread_t rx_task_handle;

} ModemUart_t;

#if (USE_CIRCULAR_BUFFER == 0)
typedef struct {
    uint8_t buf[UART_RX_BUF_SIZE];
    uint16_t  length;
} cy_uart_rx_data_message_t;
#endif


/*-- Local Data -------------------------------------------------*/

static const char *TAG = "uart";
static const uint32_t MODEM_HANDLE_IDENT = 0xFEDC1234;

static ModemUart_t s_modemUartList[] = {
    {
        // identifier
        .magic = MODEM_HANDLE_IDENT,

        // hardware-related
        .portName_p = DEFAULT_MODEM_SERIAL_PORT,
        .rx = PPP_MODEM_UART_RX,
        .tx = PPP_MODEM_UART_TX,
        .baudrate = DEFAULT_MODEM_BAUD_RATE,

        // instance members
        .uart_obj_p = NULL,
        .handleRefCount = 0,
        .uart_irq_enabled = false,
        .tx_done = false,
        .isFlagFirstRead = true,

#if USE_CIRCULAR_BUFFER
        .rxdata_head_index = 0,
        .rxdata_tail_index = 0,
        .rxdata_recv_sem = NULL,
#else
        .rx_queue = NULL,
#endif
        .rx_task_handle = NULL,
    },
};

static bool s_modemUartListInitialized = false;
static cy_mutex_t s_modemUartListMutex;  /* mutex protects the s_modemUartList */


/*-- Local Functions -------------------------------------------------*/

/* Event handler callback function */
static void uart_event_handler( void *handler_arg,
                                cyhal_uart_event_t event)
{
    ModemUart_t* modemUart_p = (ModemUart_t*)handler_arg;

    VoidAssert(handler_arg != INVALID_HANDLE);
    VoidAssert(modemUart_p->magic == MODEM_HANDLE_IDENT);

    cyhal_uart_t *uart_obj_p = modemUart_p->uart_obj_p;
    VoidAssert(uart_obj_p != NULL);

    if ((event & CYHAL_UART_IRQ_TX_DONE) == CYHAL_UART_IRQ_TX_DONE) {
        modemUart_p->tx_done = true;

    } else if ((event & UART_READ_IRQ_EVENT_TYPE) == UART_READ_IRQ_EVENT_TYPE) {

#if USE_CIRCULAR_BUFFER

        // read 1 byte at a time, no time-out
        if (cyhal_uart_getc(uart_obj_p,
                            &modemUart_p->rxdata[modemUart_p->rxdata_head_index],
                            0) == CY_RSLT_SUCCESS) {

            modemUart_p->rxdata_head_index++;
            modemUart_p->rxdata_head_index %= CIRCULAR_RX_BUFFER_SIZE;

            if (modemUart_p->rxdata_head_index == modemUart_p->rxdata_tail_index) {
                // if buffer overrun occurs, assert
                // then try increasing CIRCULAR_RX_BUFFER_SIZE
                DEBUG_ASSERT(modemUart_p->rxdata_head_index != modemUart_p->rxdata_tail_index);
            }
            cy_rtos_set_semaphore(&modemUart_p->rxdata_recv_sem, true);
        }

#else
        /* Data is received in the RX FIFO */
        cy_uart_rx_data_message_t message;
        cy_rslt_t result;

        message.length = 0;

        size_t tries = 0;
        size_t consecutive_fails = 0;

#if 1
        while (message.length < sizeof(message.buf)) {
#else
        while (cyhal_uart_readable(uart_obj_p) &&
                (message.length < sizeof(message.buf))) {
#endif

            size_t temp = sizeof(message.buf) - message.length;

            // this call reads 1 to 4 bytes at a time
            if (cyhal_uart_read(uart_obj_p,
                                (void*)&message.buf[message.length],
                                &temp) == CY_RSLT_SUCCESS) {
                message.length += (uint16_t)temp;
            }

            if (temp == 0) {
                consecutive_fails++;
            } else {
                consecutive_fails = 0;
            }

            if (consecutive_fails >= MAX_UART_READ_CONSECUTIVE_FAILS) {
                break;
            }

            if (tries > MAX_UART_READ_TRIES) {
                break;
            }

            tries++;
        }

        if (message.length > 0) {
            result = cy_rtos_put_queue( &modemUart_p->rx_queue,
                                        &message,
                                        0,
                                        true);

            if (result != CY_RSLT_SUCCESS) {
                size_t num_items = 0;
                cy_rtos_count_queue(&modemUart_p->rx_queue,
                                    &num_items);
                DEBUG_ASSERT(0);
            }
        }
#endif
    }
}


static void uart_irq_helper(Modem_Handle_t handle,
                            bool operation)
{
    ModemUart_t* modemUart_p = (ModemUart_t*)handle;

    VoidAssert(handle != INVALID_HANDLE);
    VoidAssert(modemUart_p->magic == MODEM_HANDLE_IDENT);

    cyhal_uart_t *uart_obj_p = modemUart_p->uart_obj_p;
    VoidAssert(uart_obj_p != NULL);

    /* The UART callback handler registration */
    cyhal_uart_register_callback( uart_obj_p,
                                  uart_event_handler,
                                  handle);

    /* Enable required UART events */
    cyhal_uart_enable_event(uart_obj_p,
                            (cyhal_uart_event_t)(
                                UART_READ_IRQ_EVENT_TYPE
                                | CYHAL_UART_IRQ_TX_DONE
                                /*| CYHAL_UART_IRQ_TX_ERROR
                                  | CYHAL_UART_IRQ_RX_DONE
                                  | CYHAL_UART_IRQ_RX_ERROR*/),
                            INT_PRIORITY,
                            operation);

    modemUart_p->uart_irq_enabled = operation;
}

static cy_rslt_t my_cy_uart_init( cyhal_uart_t *uart_obj_p,
                                  cyhal_gpio_t tx,
                                  cyhal_gpio_t rx,
                                  uint32_t baudrate)
{
#define RX_BUFFER_SIZE    0

#if (RX_BUFFER_SIZE > 0)
    static uint8_t s_rx_buffer[RX_BUFFER_SIZE];
#endif

    const cyhal_uart_cfg_t uart_config = {
        .data_bits = 8,
        .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,

#if (RX_BUFFER_SIZE > 0)
        .rx_buffer = s_rx_buffer,
        .rx_buffer_size = sizeof(s_rx_buffer),
#else
        .rx_buffer = NULL,
        .rx_buffer_size = 0,
#endif
    };

    cy_rslt_t result = cyhal_uart_init( uart_obj_p,
                                        tx,
                                        rx,
                                        NULL,
                                        &uart_config);

    if (result == CY_RSLT_SUCCESS) {
        CY_LOGD(TAG, "%s [%d]: baudrate = %lu",
                __FUNCTION__, __LINE__, baudrate);

        result = cyhal_uart_set_baud( uart_obj_p,
                                      baudrate,
                                      NULL);
    }

    return result;
}


static int my_uart_write_async( ModemUart_t *modemUart_p,
                                const uint8_t *ptr,
                                int len)
{
    cy_rslt_t res;

    ReturnAssert(modemUart_p != NULL, 0);
    ReturnAssert(modemUart_p->uart_obj_p != NULL, 0);

    if (ptr != NULL) {
        modemUart_p->tx_done = false;
        res = cyhal_uart_write_async(modemUart_p->uart_obj_p,
                                     (void *)ptr,
                                     (size_t)len);

        if (res == CY_RSLT_SUCCESS) {
            // wait for Tx Done
            while(!modemUart_p->tx_done) {
                //CY_LOGD(TAG, "%s [%d]", __FUNCTION__, __LINE__);
            };

            return len;
        }
    }
    return 0;
}

static int my_uart_write( ModemUart_t *modemUart_p,
                          const uint8_t *ptr,
                          int len)
{
#define USE_UART_WRITE    1

    ReturnAssert(modemUart_p != NULL, 0);
    ReturnAssert(modemUart_p->uart_obj_p != NULL, 0);

#if (USE_UART_WRITE)
    cy_rslt_t res;
    size_t tx_length = len;
    size_t written = 0;

    if (ptr != NULL) {
        while (tx_length) {
            res = cyhal_uart_write(modemUart_p->uart_obj_p,
                                   (void *)&ptr[written],
                                   &tx_length);

            if (res == CY_RSLT_SUCCESS) {
                //return (int)tx_length;
                written += tx_length;
                tx_length = len - written;

            } else {
                break;
            }
        }
    }
    return written;

#else
    int nChars = 0;

    if (ptr != NULL) {
        for (/* Empty */; nChars < len; ++nChars) {
            /* blocking call */
            cyhal_uart_putc(modemUart_p->uart_obj_p,
                            (uint32_t)*ptr);
            ++ptr;
        }
    }
    return (nChars);
#endif

#undef USE_UART_WRITE
}

static size_t my_uart_read( cyhal_uart_t *uart_obj_p,
                            uint32_t readTimeoutMsec,
                            uint8_t *ptr,
                            size_t len)
{
#define USE_UART_READ    0

    //CY_LOGD(TAG, "%s [%d] len=%lu ptr=0x%08x", __FUNCTION__, __LINE__, len, ptr);
    ReturnAssert(ptr != NULL, 0);

#if (USE_UART_READ)
    cy_rslt_t res;
    size_t rx_length = len;

    if (ptr != NULL) {
        res = cyhal_uart_read(uart_obj_p,
                              (void *)ptr,
                              &rx_length);

        if (res == CY_RSLT_SUCCESS) {
            return rx_length;
        }
    }
    return 0;

#else
    size_t nChars = 0;
    uint32_t timeoutMsec = readTimeoutMsec;

    if (ptr != NULL) {
        for(/* Empty */; nChars < len; ++ptr) {
            uint8_t value;
            if (cyhal_uart_getc(uart_obj_p,
                                &value,
                                timeoutMsec) != CY_RSLT_SUCCESS) {
                break;
            }
            timeoutMsec = INTER_BYTE_READ_TIMEOUT_MSEC;

            *ptr = value;
            ++nChars;
        }
    }
    return (nChars);
#endif

#undef USE_UART_READ
}


static void uart_read_data_task(cy_thread_arg_t arg)
{
    Modem_ReadCallback_t *data_p = (Modem_ReadCallback_t*) arg;
    ModemUart_t* modemUart_p;

#if (USE_CIRCULAR_BUFFER == 0)
    uint16_t max_msg_length = 0;
    size_t max_queue_items = 0;
#endif

    CY_LOGD(TAG, "%s [%d]", __FUNCTION__, __LINE__);
    VoidAssert(data_p != NULL);

    modemUart_p = (ModemUart_t*)data_p->handle;

    VoidAssert(data_p->handle != INVALID_HANDLE);
    VoidAssert(modemUart_p->magic == MODEM_HANDLE_IDENT);

    while (true) {

#if USE_CIRCULAR_BUFFER
        while (cy_rtos_get_semaphore( &modemUart_p->rxdata_recv_sem,
                                      CY_RTOS_NEVER_TIMEOUT,
                                      false) != CY_RSLT_SUCCESS) {
            CY_LOGD(TAG, "%s [%d]: rxdata_recv_sem - timeout! repeat", __FUNCTION__, __LINE__);
        }

        size_t size = 0;
        if (modemUart_p->rxdata_head_index > modemUart_p->rxdata_tail_index) {
            size = modemUart_p->rxdata_head_index - modemUart_p->rxdata_tail_index;
        }
        else if (modemUart_p->rxdata_head_index < modemUart_p->rxdata_tail_index) {
            size = CIRCULAR_RX_BUFFER_SIZE - modemUart_p->rxdata_tail_index;
        }

        //CY_LOGD(TAG, "%s [%d]: size = %u",
        //        __FUNCTION__, __LINE__, size);

        if (size > 0) {
            DEBUG_ASSERT(data_p != NULL);
            DEBUG_ASSERT(data_p->read_callback != NULL);
            DEBUG_ASSERT(data_p->read_callback_ctx != NULL);

            data_p->read_callback(&modemUart_p->rxdata[modemUart_p->rxdata_tail_index],
                                  size,
                                  data_p->read_callback_ctx);

            modemUart_p->rxdata_tail_index = (modemUart_p->rxdata_tail_index + size) % CIRCULAR_RX_BUFFER_SIZE;
        }

#else
        cy_uart_rx_data_message_t message;
        while (cy_rtos_get_queue( &modemUart_p->rx_queue,
                                  &message,
                                  CY_RTOS_NEVER_TIMEOUT,
                                  false) != CY_RSLT_SUCCESS) {
            CY_LOGD(TAG, "%s [%d]: rx_queue - timeout! repeat", __FUNCTION__, __LINE__);
        }

        if (message.length > 0) {

            bool do_log = false;

            size_t num_items = 0;
            cy_rtos_count_queue(&modemUart_p->rx_queue,
                                &num_items);

            if (num_items > max_queue_items) {
                max_queue_items = num_items;
                do_log = true;
            }

            if (message.length > max_msg_length) {
                max_msg_length = message.length;
                do_log = true;
            }
#if 1
            (void)do_log; // unused

#else
            if (do_log) {
                CY_LOGD(TAG, "%s [%d]: received=%u, queue: %u items",
                        __FUNCTION__, __LINE__, message.length, num_items);
            }
#endif

            if (data_p != NULL) {
                //CY_LOGD(TAG, "%s [%d]: message.length = %u",
                //                 __FUNCTION__, __LINE__, message.length);

                // assume data is PPP data meant for the LWIP stack
                /* print_bytes("buf: ", message.buf, message.length); */

                DEBUG_ASSERT(data_p->read_callback != NULL);
                DEBUG_ASSERT(data_p->read_callback_ctx != NULL);

                data_p->read_callback(message.buf,
                                      message.length,
                                      data_p->read_callback_ctx);
            } else {
                DEBUG_ASSERT(data_p != NULL);
            }
        } else {
            DEBUG_ASSERT(message.length > 0);
        }
#endif
    }
}


/*-- Public Functions -------------------------------------------------*/

bool Modem_Uart_Init(void)
{
    cy_rslt_t result = cy_rtos_init_mutex(&s_modemUartListMutex);
    ReturnAssert(result == CY_RSLT_SUCCESS, false);

    s_modemUartListInitialized = true;
    return true;
}

bool Modem_Uart_Deinit(void)
{
    if (s_modemUartListInitialized) {
        cy_rtos_deinit_mutex(&s_modemUartListMutex);
        s_modemUartListInitialized = false;
    }

    return true;
}

UICC_Result_t Modem_GetModemList(_out_ Modem_List_t **pListModems_p)
{
    UICC_Result_t result = UICC_NO_ERROR;
    Modem_List_t *output_p = NULL;
    size_t i;

    ReturnAssert(pListModems_p != NULL, UICC_ARGUMENT_ERROR);

    /* CY_LOGD(TAG, "%s [%d]", __FUNCTION__, __LINE__); */

    output_p = (Modem_List_t *)CY_MEMTRACK_MALLOC(sizeof(*output_p));
    ReturnAssert(output_p != NULL, UICC_NOT_ENOUGH_MEMORY);

    memset(output_p, 0, sizeof(*output_p));

    for (i = 0; i < ARRAY_SIZE(s_modemUartList); i++) {
        int res;
        char modelName[MAX_MODEM_MODEL_LEN] = "";
        char imei[MAX_MODEM_IMEI_LEN] = "";
        const char *portName_p = s_modemUartList[i].portName_p;

        res = Modem_Probe(portName_p,
                          modelName,
                          sizeof(modelName),
                          imei,
                          sizeof(imei));

        if ((res == RESULT_MODEM_OK) &&
                (!Modem_IsFound(output_p, modelName, imei))) {
            size_t nIndex = output_p->numModems;
            Modem_Info_t *temp;

            temp = (Modem_Info_t *)CY_MEMTRACK_REALLOC(output_p->modems,
                                           (nIndex + 1) * sizeof(*temp));

            if (temp != NULL) {
                output_p->modems = temp;

                output_p->modems[nIndex].portName_p = CY_MEMTRACK_MALLOC(strlen(portName_p) + 1);

                if (output_p->modems[nIndex].portName_p != NULL) {
                    strcpy(output_p->modems[nIndex].portName_p, portName_p);
                }

                output_p->modems[nIndex].modelName_p = CY_MEMTRACK_MALLOC(strlen(modelName) + 1);

                if (output_p->modems[nIndex].modelName_p != NULL) {
                    strcpy(output_p->modems[nIndex].modelName_p, modelName);
                }

                output_p->modems[nIndex].imei_p = CY_MEMTRACK_MALLOC(strlen(imei) + 1);

                if (output_p->modems[nIndex].imei_p != NULL) {
                    strcpy(output_p->modems[nIndex].imei_p, imei);
                }

                output_p->numModems++;

#if (ONLY_ENUMERATE_ONE_MODEM == 1)
                break;
#endif
            } else {
                CY_LOGD(TAG, "%s [%d]: realloc failed", __FUNCTION__, __LINE__);
                result = UICC_NOT_ENOUGH_MEMORY;
                break;
            }
        }
    }

    *pListModems_p = output_p;

    return result;
}


Modem_Handle_t Modem_Open(_in_ const char* serialPortName_p)
{
    cy_rslt_t result;
    size_t i;
    cyhal_uart_t *uart_obj_p = INVALID_HANDLE;
    bool found = false;

    ReturnAssert(s_modemUartListInitialized, INVALID_HANDLE);   // has Modem_Uart_Init being invoked?
    ReturnAssert(serialPortName_p != NULL, INVALID_HANDLE);

    /* CY_LOGD(TAG, "%s [%d]: %s", __FUNCTION__, __LINE__, serialPortName_p); */

    for (i = 0; i < ARRAY_SIZE(s_modemUartList); i++) {
        if (strcmp(serialPortName_p, s_modemUartList[i].portName_p) == 0) {
            found = true;
            break;
        }
    }

    ReturnAssert(found, INVALID_HANDLE);
    ReturnAssert(s_modemUartList[i].magic == MODEM_HANDLE_IDENT, INVALID_HANDLE);

    if (s_modemUartList[i].uart_obj_p == NULL) {
        // generate a new handle
        uart_obj_p = (cyhal_uart_t*)CY_MEMTRACK_MALLOC(sizeof(*uart_obj_p));
        ReturnAssert(uart_obj_p != NULL, INVALID_HANDLE);

        result = my_cy_uart_init( uart_obj_p,
                                  s_modemUartList[i].tx,
                                  s_modemUartList[i].rx,
                                  s_modemUartList[i].baudrate);

        if (result != CY_RSLT_SUCCESS) {
            CY_MEMTRACK_FREE(uart_obj_p);
            return INVALID_HANDLE;
        }

    } else {
        DEBUG_ASSERT(s_modemUartList[i].handleRefCount > 0);
    }

    // store the new handle or increment the ref count of an existing handle
    if (cy_rtos_get_mutex(&s_modemUartListMutex, CY_RTOS_NEVER_TIMEOUT) == CY_RSLT_SUCCESS) {
        if (s_modemUartList[i].uart_obj_p == NULL) {
            s_modemUartList[i].uart_obj_p = uart_obj_p;
            s_modemUartList[i].handleRefCount = 1;

        } else {
            s_modemUartList[i].handleRefCount++;
        }

        cy_rtos_set_mutex(&s_modemUartListMutex);
    }

    DEBUG_ASSERT(s_modemUartList[i].uart_obj_p != NULL);

    return (Modem_Handle_t)&s_modemUartList[i];
}


void Modem_Close(_in_ Modem_Handle_t handle)
{
    ModemUart_t* modemUart_p = (ModemUart_t*)handle;
    /* CY_LOGD(TAG, "%s [%d] handle=%p", __FUNCTION__, __LINE__, handle); */

    VoidAssert(s_modemUartListInitialized);   // has Modem_Uart_Init being invoked?
    VoidAssert(handle != INVALID_HANDLE);

    VoidAssert(modemUart_p->magic == MODEM_HANDLE_IDENT);
    VoidAssert(modemUart_p->handleRefCount > 0);

    // decrement the ref counter and if it reaches 0, free the handle
    if (cy_rtos_get_mutex(&s_modemUartListMutex, CY_RTOS_NEVER_TIMEOUT) == CY_RSLT_SUCCESS) {
        modemUart_p->handleRefCount--;

        if (modemUart_p->handleRefCount == 0) {
            VoidAssert(modemUart_p->uart_obj_p != NULL);

            cyhal_uart_free(modemUart_p->uart_obj_p);
            CY_MEMTRACK_FREE(modemUart_p->uart_obj_p);

            modemUart_p->uart_obj_p = NULL;
        }

        cy_rtos_set_mutex(&s_modemUartListMutex);
    }
}


bool Modem_WriteCommand(_in_ Modem_Handle_t handle,
                        _in_ const uint8_t *commandBuf_p,
                        _in_ size_t commandBufSize)
{
    size_t bytesWritten;
    ModemUart_t* modemUart_p = (ModemUart_t*)handle;

    ReturnAssert(commandBuf_p != NULL, false);
    ReturnAssert(handle != INVALID_HANDLE, false);

    // don't throw assertion here, as a PPP-disconnect may cause
    // tcpip stack to send remaining data. Just ignore write
    // attempts if modemUart_p has already been deleted
    //
    //ReturnAssert(modemUart_p->magic == MODEM_HANDLE_IDENT, false);
    //ReturnAssert(uart_obj_p != NULL, false);

    if (modemUart_p->magic != MODEM_HANDLE_IDENT) {
        CY_LOGD(TAG, "%s [%d] missing magic identifier, write aborted",
                         __FUNCTION__, __LINE__);
        return false;
    }

    cyhal_uart_t *uart_obj_p = modemUart_p->uart_obj_p;
    if (uart_obj_p == NULL) {
        CY_LOGD(TAG, "%s [%d] uart_obj_p is NULL, write aborted",
                         __FUNCTION__, __LINE__);
        return false;
    }

    /* CY_LOGD(TAG, "%s [%d] handle=%p commandBufSize=%u",
                     __FUNCTION__, __LINE__, handle, commandBufSize); */

    //print_bytes("commandBuf_p->p: ", commandBuf_p, commandBufSize);

    if (modemUart_p->uart_irq_enabled) {
        bytesWritten = my_uart_write_async( modemUart_p,
                                            commandBuf_p,
                                            commandBufSize);
    } else {
        bytesWritten = my_uart_write( modemUart_p,
                                      commandBuf_p,
                                      commandBufSize);
    }

    /* CY_LOGD(TAG, "%s [%d] bytesWritten=%u",
                     __FUNCTION__, __LINE__, bytesWritten); */

    if (commandBufSize != bytesWritten) {
        CY_LOGE(TAG, "%s [%d] ERROR! commandBufSize=%u, bytesWritten=%u",
                __FUNCTION__, __LINE__, commandBufSize, bytesWritten);
    }

    return (commandBufSize == bytesWritten);
}

size_t Modem_ReadResponse(_in_  Modem_Handle_t handle,
                          _in_  uint32_t readTimeoutMsec,
                          _out_ uint8_t *responseBuf_p,
                          _in_  size_t maxResponseBufSize)
{
    size_t bytesRead;
    ModemUart_t* modemUart_p = (ModemUart_t*)handle;

    ReturnAssert(handle != INVALID_HANDLE, 0);
    ReturnAssert(modemUart_p->magic == MODEM_HANDLE_IDENT, false);
    ReturnAssert(responseBuf_p != NULL, 0);

    cyhal_uart_t *uart_obj_p = modemUart_p->uart_obj_p;
    ReturnAssert(uart_obj_p != NULL, false);

    bytesRead = my_uart_read( uart_obj_p,
                              readTimeoutMsec,
                              responseBuf_p,
                              maxResponseBufSize);

    /* CY_LOGD(TAG, "%s [%d] bytesRead=%u",
                     __FUNCTION__, __LINE__, bytesRead); */
    return bytesRead;
}


bool Modem_InstallReadCallback(_in_  Modem_Handle_t handle,
                               _in_  void *data_p)
{
    cy_rslt_t result;
    CY_LOGD(TAG, "%s [%d]", __FUNCTION__, __LINE__);

    ModemUart_t* modemUart_p = (ModemUart_t*)handle;

    ReturnAssert(handle != INVALID_HANDLE, false);
    ReturnAssert(modemUart_p->magic == MODEM_HANDLE_IDENT, false);

#if USE_CIRCULAR_BUFFER
    memset(modemUart_p->rxdata, 0, sizeof(modemUart_p->rxdata));
    modemUart_p->rxdata_head_index = 0;
    modemUart_p->rxdata_tail_index = 0;

    result = cy_rtos_init_semaphore(&modemUart_p->rxdata_recv_sem, 1, 0);
    ReturnAssert(result == CY_RSLT_SUCCESS, false);
    ReturnAssert(modemUart_p->rxdata_recv_sem != NULL, false);

#else
    ReturnAssert(modemUart_p->rx_queue == NULL, false);

    result = cy_rtos_init_queue(&modemUart_p->rx_queue,
                                UART_RX_DATA_QUEUE_SIZE,
                                sizeof(cy_uart_rx_data_message_t));
    ReturnAssert(result == CY_RSLT_SUCCESS, false);
    ReturnAssert(modemUart_p->rx_queue != NULL, false);
#endif

    result = cy_rtos_create_thread( &modemUart_p->rx_task_handle,
                                    uart_read_data_task,
                                    modemUart_p->portName_p,
                                    NULL,
                                    UART_READ_TASK_STACK_SIZE,
                                    UART_READ_TASK_PRIORITY,
                                    (cy_thread_arg_t) data_p  // argument
                                  );

    if (CY_RSLT_SUCCESS != result) {
        DEBUG_PRINT(("cy_rtos_create_thread %s failed: (0x%lx)",
                     modemUart_p->portName_p, result));
    }

    ReturnAssert(modemUart_p->rx_task_handle != NULL, false);
    return true;
}


void Modem_DeleteReadCallback(_in_  Modem_Handle_t handle)
{
    ModemUart_t* modemUart_p = (ModemUart_t*)handle;

    VoidAssert(handle != INVALID_HANDLE);
    VoidAssert(modemUart_p->magic == MODEM_HANDLE_IDENT);

    CY_LOGD(TAG, "%s [%d]", __FUNCTION__, __LINE__);

    if (modemUart_p->rx_task_handle != NULL) {
        if (CY_RSLT_SUCCESS != cy_rtos_terminate_thread(&modemUart_p->rx_task_handle)) {
            DEBUG_PRINT(("cy_rtos_terminate_thread %s failed\n", modemUart_p->portName_p));
        }
        modemUart_p->rx_task_handle = NULL;
    }

#if (USE_CIRCULAR_BUFFER)
    if (modemUart_p->rxdata_recv_sem != NULL) {
        cy_rtos_deinit_semaphore(&modemUart_p->rxdata_recv_sem);
        modemUart_p->rxdata_recv_sem = NULL;
    }

#else
    if (modemUart_p->rx_queue != NULL) {
        cy_rtos_deinit_queue(&modemUart_p->rx_queue);
        modemUart_p->rx_queue = NULL;
    }
#endif
}


void Modem_EnableReadCallback(_in_  Modem_Handle_t handle)
{
    CY_LOGD(TAG, "%s [%d]", __FUNCTION__, __LINE__);
    uart_irq_helper(handle, true);
}


void Modem_DisableReadCallback(_in_  Modem_Handle_t handle)
{
    CY_LOGD(TAG, "%s [%d]", __FUNCTION__, __LINE__);
    uart_irq_helper(handle, false);
}

bool Modem_SetBaudRate( _in_  Modem_Handle_t handle,
                        _in_  uint32_t baudrate)
{
    cy_rslt_t result;
    uint32_t actualbaud;
    ModemUart_t* modemUart_p = (ModemUart_t*)handle;

    ReturnAssert(handle != INVALID_HANDLE, false);
    ReturnAssert(modemUart_p->magic == MODEM_HANDLE_IDENT, false);

    cyhal_uart_t *uart_obj_p = modemUart_p->uart_obj_p;
    ReturnAssert(uart_obj_p != NULL, false);

    CY_LOGD(TAG, "%s [%d]: baudrate = %lu",
            __FUNCTION__, __LINE__, baudrate);

    result = cyhal_uart_set_baud( uart_obj_p,
                                  baudrate,
                                  &actualbaud);
    CY_LOGD(TAG, "%s [%d]: actualbaud = %lu",
            __FUNCTION__, __LINE__, actualbaud);

    return (result == CY_RSLT_SUCCESS);
}

bool Modem_IsFlagFirstRead( _in_ Modem_Handle_t handle)
{
    bool result;
    ModemUart_t* modemUart_p = (ModemUart_t*)handle;

    ReturnAssert(handle != INVALID_HANDLE, false);
    ReturnAssert(modemUart_p->magic == MODEM_HANDLE_IDENT, false);

    // read the flag
    result = modemUart_p->isFlagFirstRead;

    // clear the flag so subsequent reads will return false
    modemUart_p->isFlagFirstRead = false;

    return result;
}

#endif /* VARIANT_MODEM */
