/******************************************************************************
* File Name:   cy_modem.c
*
* Description: This file implements the modem functions.
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

#include "feature_config.h"
#include "gps_config.h"

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include "cy_modem.h"
#include "cy_memtrack.h"
#include "cy_debug.h"
#include "cy_string.h"

#include "cyhal.h"


/*-- Local Definitions -------------------------------------------------*/

#define CY_MODEM_RESULT_OK          AT_RSP_OK       //"OK"
#define CY_MODEM_RESULT_ERROR       AT_RSP_ERROR    //"ERROR"

#define LINE_BUFFER_SIZE            512

#ifdef PPP_MODEM_IO_REF
#define MODEM_IO_REF                PPP_MODEM_IO_REF
#endif

#define MODEM_POWER_KEY             PPP_MODEM_POWER_KEY

#if (PPP_MODEM_POWER_METHOD == PPP_SIMPLE_SWITCH_METHOD)
#define MODEM_POWER_ON                      PPP_MODEM_POWER_KEY_ON_LEVEL
#define MODEM_POWER_OFF                     PPP_MODEM_POWER_KEY_OFF_LEVEL
#define POWER_OFF_PULSE_WIDTH_MSEC          3000

// 20 to 30sec works; 5 to 12.5sec failed; 15sec ok on 1 modem, but failed on WaveShare dongle
#define MODEM_POWER_ON_WAIT_INTERVAL_MSEC   30000

#elif (PPP_MODEM_POWER_METHOD == PPP_POWER_STEP_METHOD)
#define MODEM_POWER_ON                      PPP_MODEM_POWER_KEY_ON_LEVEL
#define MODEM_POWER_OFF                     PPP_MODEM_POWER_KEY_OFF_LEVEL
#define POWER_ON_PULSE_WIDTH_MSEC           500
#define POWER_OFF_PULSE_WIDTH_MSEC          2500

#define MODEM_POWER_ON_WAIT_INTERVAL_MSEC   10000 //was 50000
#define FIRST_POWER_ON_DELAY_1_MSEC         2000
#define FIRST_POWER_ON_DELAY_2_MSEC         18000
#endif


#define GPS_READY_MAX_RETRIES               10
#define WAIT_FOR_GPS_READY_MSEC             1000
#define SWITCH_PPP_TO_CMD_MODE_DELAY_MSEC   500 // was 1000


/*-- Local Data -------------------------------------------------*/
static const char *TAG = "modem";


/*-- Local Functions -------------------------------------------------*/

// e.g.  +COPS: 0,0,"Singtel Tata",7
static bool extract_operator_name(const char* input_p,
                                  char *buf_p,
                                  size_t bufSize)
{
    ReturnAssert(input_p != NULL, false);
    ReturnAssert(buf_p != NULL, false);

    const char ch = '"';
    char *start_p;
    char *end_p = NULL;

    start_p = strchr(input_p, ch);
    if (start_p != NULL) {
        end_p = strchr(start_p + 1, ch);
    }

    if (end_p != NULL) {
        *end_p = '\0';
        start_p++;
        SNPRINTF(buf_p, bufSize, "%s", start_p);
        CY_LOGD(TAG, "OperatorName: %s", buf_p);
        return (strlen(buf_p) > 0);
    }

    return false;
}

#ifdef AT_RSP_UE_INFO_PATTERN_LTE
// e.g. +CPSI: LTE,Online,525-01,0x0309,187230312,184,EUTRAN-BAND3,1300,5,5,-148,-862,-8
static bool extract_ue_system_info( const char* input_p,
                                    char *buf_p,
                                    size_t bufSize)
{
    ReturnAssert(input_p != NULL, false);
    ReturnAssert(buf_p != NULL, false);

    char* ptr;
    const char systemMode[] = AT_RSP_UE_INFO_PATTERN_LTE;
    if ((ptr = strstr(input_p, systemMode)) == NULL) {
        CY_LOGE(TAG, "%s [%d]: cannot find mode: %s",
                __FUNCTION__, __LINE__, systemMode);
        return false;
    }

    ptr += strlen(systemMode);

    char *start_p = NULL;
    int comma_count = 5;
    int i;

    for (i = 0; i < comma_count; i++) {
        const char ch = ',';

        ptr = strchr(ptr + 1, ch);
        if (ptr == NULL) {
            return false;
        }

        if (i == (comma_count - 2)) {
            start_p = ptr;
        }
    }

    if ((start_p == NULL) || (ptr == NULL)) {
        return false;
    }

    *ptr = '\0';
    start_p++;

    SNPRINTF(buf_p, bufSize, "%s,%s", systemMode, start_p);
    CY_LOGD(TAG, "UE SystemInfo: %s", buf_p);
    return (strlen(buf_p) > 0);
}
#endif


#ifdef AT_RSP_GET_GPS_INFO
// Input:  +CGPSINFO: 0119.378145,N,10352.147261,E,160621,024410.0,20.4,0.0,
// Output:            0119.378145,N,10352.147261,E
static bool extract_gps_location( const char* input_p,
                                  char *buf_p,
                                  size_t bufSize)
{
    ReturnAssert(input_p != NULL, false);
    ReturnAssert(buf_p != NULL, false);

    char* ptr;
    const char marker[] = AT_RSP_GET_GPS_INFO;

    if ((ptr = strstr(input_p, marker)) == NULL) {
        CY_LOGE(TAG, "%s [%d]: cannot find marker: %s",
                __FUNCTION__, __LINE__, marker);
        return false;
    }

    ptr += strlen(marker);

    char *start_p = ptr;
    int comma_count = 4;
    int i;

    for (i = 0; i < comma_count; i++) {
        const char ch = ',';

        ptr = strchr(ptr + 1, ch);
        if (ptr == NULL) {
            return false;
        }
    }

    if (ptr == NULL) {
        return false;
    }

    *ptr = '\0';

    if (strlen(start_p) > GPS_INFO_MIN_LEN) {
        SNPRINTF(buf_p, bufSize, "%s", start_p);
        CY_LOGD(TAG, "GPS: %s", buf_p);
        return true;
    }
    return false;
}
#endif


static Modem_Handle_t modem_open(void)
{
    return Modem_Open(DEFAULT_MODEM_SERIAL_PORT);
}


static bool wait_for_modem_ready( cy_modem_t *modem_p,
                                  long duration)
{
#define READ_TIMEOUT_MSEC     1000

    long stamp = 0;
    size_t len;

    CY_LOGD(TAG, "%s", __FUNCTION__);

    while (stamp < duration) {
        memset(modem_p->line_buffer_p, 0, modem_p->line_buffer_size);
        len = Modem_ReadResponse( modem_p->handle,
                                  READ_TIMEOUT_MSEC,
                                  (uint8_t*)modem_p->line_buffer_p,
                                  modem_p->line_buffer_size);

        if (len > 0) {
            CY_LOGD(TAG, "%5ld: [len=%d] %s", stamp, len, modem_p->line_buffer_p);

#ifdef AT_RSP_READY
            if (strstr(modem_p->line_buffer_p, AT_RSP_READY) != NULL) {
                return true;
            }
#endif
        }

        stamp += READ_TIMEOUT_MSEC;

#ifdef PPP_SEND_AT_DURING_WAIT_FOR_MODEM_READY
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_READY_CHECK,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, AT_RSP_OK) != NULL) {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                return true;
            }
        }
#endif

    }

    if (Modem_SendATCommand(modem_p->handle,
                            AT_CMD_READY_CHECK,
                            modem_p->line_buffer_p,
                            modem_p->line_buffer_size)) {
        if (strstr(modem_p->line_buffer_p, AT_RSP_OK) != NULL) {
            CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            return true;
        }
    }

    return false;

#undef READ_TIMEOUT_MSEC
}

#if (PPP_MODEM_POWER_METHOD == PPP_POWER_STEP_METHOD)
static void do_modem_power_on_pulse(void)
{
    cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_OFF);

    cy_rtos_delay_milliseconds(POWER_ON_PULSE_WIDTH_MSEC);

    cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_ON);
}
#endif

static bool modem_power_on(cy_modem_t *modem_p)
{
    ReturnAssert(modem_p != NULL, false);

    if (!modem_p->is_powered_on) {
        CY_LOGD(TAG, "%s", __FUNCTION__);

#if (PPP_MODEM_POWER_METHOD == PPP_SIMPLE_SWITCH_METHOD)
        cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_ON);

#elif (PPP_MODEM_POWER_METHOD == PPP_POWER_STEP_METHOD)
        bool value = cyhal_gpio_read(MODEM_POWER_KEY);
        if (value) {
            cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_ON);
        } else {
            // do a pulse
            do_modem_power_on_pulse();
        }
#endif

        modem_p->is_powered_on = true;

        return wait_for_modem_ready(modem_p,
                                    MODEM_POWER_ON_WAIT_INTERVAL_MSEC);
    }
    return false;
}

static void modem_power_off(cy_modem_t *modem_p)
{
    VoidAssert(modem_p != NULL);

    if (modem_p->is_powered_on) {
        CY_LOGD(TAG, "%s", __FUNCTION__);

#if (PPP_MODEM_POWER_METHOD == PPP_SIMPLE_SWITCH_METHOD)
        cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_OFF);

#elif (PPP_MODEM_POWER_METHOD == PPP_POWER_STEP_METHOD)
        bool value = cyhal_gpio_read(MODEM_POWER_KEY);
        if (value) {
            // do a pulse
            cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_ON);
            cy_rtos_delay_milliseconds(POWER_OFF_PULSE_WIDTH_MSEC);
            cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_OFF);

        } else {
            // do 2 pulses
            cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_OFF);
            cy_rtos_delay_milliseconds(POWER_ON_PULSE_WIDTH_MSEC);
            cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_ON);
            cy_rtos_delay_milliseconds(POWER_OFF_PULSE_WIDTH_MSEC);
            cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_OFF);
        }
#endif

        modem_p->is_powered_on = false;
    }
}

#if 0
static void modem_reset(void)
{
    modem_power_off();
    modem_power_on(MODEM_POWER_ON_WAIT_INTERVAL_MSEC);
}
#endif


static bool modem_set_max_baud_rate(cy_modem_t *modem_p,
                                    uint32_t baudrate)
{
    bool result = false;
    ReturnAssert(modem_p != NULL, false);
    CY_LOGD(TAG, "%s [%d]: baudrate = %lu",
            __FUNCTION__, __LINE__, baudrate);

    SNPRINTF( modem_p->line_buffer_p,
              modem_p->line_buffer_size,
              AT_CMD_SET_BAUD_RATE "=%lu\r",
              baudrate);

    if (Modem_SendATCommand(modem_p->handle,
                            modem_p->line_buffer_p, /* cmd */
                            modem_p->line_buffer_p, /* response */
                            modem_p->line_buffer_size)) {

        if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
            CY_LOGE(TAG, "%s [%d]: pdp context failed", __FUNCTION__, __LINE__);
        } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
            CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            result = true;
        }
    }

    if (result) {
        result = (bool)Modem_SetBaudRate(modem_p->handle, baudrate);
    }

    return result;
}


static bool cy_modem_power_button_init(void)
{
    cy_rslt_t result;

#ifdef MODEM_IO_REF
    /* Initialize the GPIO for modem IO REF */
    result = cyhal_gpio_init( MODEM_IO_REF,
                              CYHAL_GPIO_DIR_OUTPUT,
                              CYHAL_GPIO_DRIVE_STRONG,
                              MODEM_POWER_ON);  // default is high

    ReturnAssert((result == CY_RSLT_SUCCESS), false);
#endif

    /* Initialize the GPIO for modem power */
    result = cyhal_gpio_init( MODEM_POWER_KEY,
                              CYHAL_GPIO_DIR_OUTPUT,
                              CYHAL_GPIO_DRIVE_STRONG,
                              MODEM_POWER_OFF);

    return (result == CY_RSLT_SUCCESS)? true : false;
}


/*-- Public Functions -------------------------------------------------*/

cy_modem_t *cy_modem_new(bool connect_ppp)
{
    cy_modem_t *modem_p = CY_MEMTRACK_CALLOC(1, sizeof(*modem_p));
    ReturnAssert(modem_p != NULL, NULL);

    do {
        modem_p->handle = modem_open();
        if (modem_p->handle == INVALID_HANDLE) {
            CY_LOGE(TAG, "%s [%d]: modem_open failed", __FUNCTION__, __LINE__);
            break;
        }

        modem_p->read_cb_p = CY_MEMTRACK_CALLOC(1, sizeof(*modem_p->read_cb_p));
        if (modem_p->read_cb_p == NULL) {
            CY_LOGE(TAG, "%s [%d]: calloc failed", __FUNCTION__, __LINE__);
            break;
        }
        modem_p->read_cb_p->handle = modem_p->handle;

        modem_p->line_buffer_size = LINE_BUFFER_SIZE;
        modem_p->line_buffer_p = CY_MEMTRACK_CALLOC(1, modem_p->line_buffer_size);
        if (modem_p->line_buffer_p == NULL) {
            CY_LOGE(TAG, "%s [%d]: calloc failed", __FUNCTION__, __LINE__);
            break;
        }

        modem_p->mode = CY_MODEM_COMMAND_MODE;
        modem_p->is_powered_on = false;
        modem_p->is_data_connected = false;

        if (connect_ppp) {
            if (!Modem_InstallReadCallback(modem_p->handle, modem_p->read_cb_p)) {
                CY_LOGE(TAG, "%s [%d]: Modem_InstallReadCallback failed",
                        __FUNCTION__, __LINE__);
                break;
            }

            Modem_DisableReadCallback(modem_p->handle);
        }

        return modem_p;
    } while (false);

    cy_modem_delete(modem_p);
    return NULL;
}

void cy_modem_delete(cy_modem_t *modem_p)
{
    if (modem_p != NULL) {
        cy_modem_powerdown(modem_p);

        Modem_DeleteReadCallback(modem_p->handle);
        Modem_Close(modem_p->handle);

        CY_MEMTRACK_FREE(modem_p->line_buffer_p);
        CY_MEMTRACK_FREE(modem_p->read_cb_p);
        CY_MEMTRACK_FREE(modem_p);
    }
}

bool cy_modem_init(void)
{
    bool result = Modem_Uart_Init();
    ReturnAssert(result, result);

    result = cy_modem_power_button_init();
    ReturnAssert(result, result);

    return true;
}

bool cy_modem_deinit(void)
{
    bool result = Modem_Uart_Deinit();
    ReturnAssert(result, result);

    return true;
}

bool cy_modem_powerup(cy_modem_t *modem_p)
{
    do {
        if (Modem_IsFlagFirstRead(modem_p->handle)) {
            CY_LOGD(TAG, "First Power On");
            //s_cleanPowerOn = false;

#if (PPP_MODEM_POWER_METHOD == PPP_SIMPLE_SWITCH_METHOD)
            // in case the modem is already running PPP, stop it first
            CY_LOGD(TAG, "Stop PPP");
            modem_stop_ppp(modem_p);

            // then turn off the modem
            CY_LOGD(TAG, "Power off modem");
            cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_OFF);
            cy_rtos_delay_milliseconds(POWER_OFF_PULSE_WIDTH_MSEC);

#elif (PPP_MODEM_POWER_METHOD == PPP_POWER_STEP_METHOD)
            // in case the modem is already running PPP, stop it first
            CY_LOGD(TAG, "Stop PPP");
            modem_stop_ppp(modem_p);

            // then turn off the modem
            int i;
            for (i = 0; i < 2; i++) {
                CY_LOGD(TAG, "Power step %d", i);
                cy_modem_powerdown(modem_p);
                do_modem_power_on_pulse();
            }

            cy_rtos_delay_milliseconds(FIRST_POWER_ON_DELAY_1_MSEC);

            cy_rtos_delay_milliseconds(FIRST_POWER_ON_DELAY_2_MSEC);
#endif
        }


        if (!modem_power_on(modem_p)) {
            CY_LOGE(TAG, "%s [%d]: modem_power_on failed", __FUNCTION__, __LINE__);
            break;
        }

        if (!modem_set_max_baud_rate(modem_p, MAX_MODEM_BAUD_RATE)) {
            CY_LOGE(TAG, "%s [%d]: modem_set_max_baud_rate failed", __FUNCTION__, __LINE__);
            break;
        }

        // ATZ
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_RESTORE_USER_SETTINGS,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: ATZ", __FUNCTION__, __LINE__);
                break;
            } else {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // ATQ0 V1 E1 S0=0 &C1 &D2 +FCLASS=0
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_DEFINE_USER_SETTINGS,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: ATQ0 V1 E1 S0=0 &C1 &D2 +FCLASS=0", __FUNCTION__, __LINE__);
                break;
            } else {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // ATE0
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_ECHO_OFF,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: ATE0", __FUNCTION__, __LINE__);
                break;
            } else {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // IMSI number
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_IMSI,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: IMSI", __FUNCTION__, __LINE__);
                break;
            } else {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // set flow control: none
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_SET_FLOW_CONTROL,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error: flow control", __FUNCTION__, __LINE__);
                break;
            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // store profile
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_SAVE_USER_SETTINGS,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error: store profile", __FUNCTION__, __LINE__);
                break;
            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // query SIM card status
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_QUERY_SIM_CARD_STATUS,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Sim Card error", __FUNCTION__, __LINE__);
                break;
            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

#ifdef AT_CMD_QUERY_HOTSWAP_LEVEL
        // query SIM card Hot Swap Level
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_QUERY_HOTSWAP_LEVEL,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
#endif

#ifdef AT_CMD_QUERY_HOTSWAP_LEVEL
        // enable SIM card Hot Swap
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_SET_HOTSWAP_ON,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
#endif

        //cy_modem_update_gps_location(modem_p);

        return true;

    } while (false);

    return false;
}

bool cy_modem_powerdown(cy_modem_t *modem_p)
{
#ifdef AT_CMD_POWER_OFF_MODEM
    if (modem_p->handle != INVALID_HANDLE) {
        /* Power down module */
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_POWER_OFF_MODEM "\r",
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error: power down", __FUNCTION__, __LINE__);
            } else {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
    }
#endif

    modem_power_off(modem_p);

    return true;
}


const char* cy_modem_get_operator_name(cy_modem_t *modem_p)
{
    ReturnAssert(modem_p != NULL, "");
    return modem_p->operatorName;
}

#ifdef AT_CMD_GPS_SESSION_START
bool cy_modem_update_gps_location(cy_modem_t *modem_p)
{
    bool result = false;
    bool is_mode_changed = false;
    char location[GPS_INFO_MAX_LEN + 1] = "";
    int i;

    if (modem_p->mode == CY_MODEM_PPP_MODE) {
        is_mode_changed = cy_modem_change_mode(modem_p,
                                               CY_MODEM_COMMAND_MODE);
    }

    // GPS On
    if (Modem_SendATCommand(modem_p->handle,
                            AT_CMD_GPS_SESSION_START,
                            modem_p->line_buffer_p,
                            modem_p->line_buffer_size)) {
        if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
            CY_LOGE(TAG, "%s [%d]: Error turning on GPS", __FUNCTION__, __LINE__);
        } else {
            CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            result = true;
        }
    }

    if (result) {
        result = false;

        for (i = 0; i < GPS_READY_MAX_RETRIES; i++) {
            cy_rtos_delay_milliseconds(WAIT_FOR_GPS_READY_MSEC);

            // CGPSINFO
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_GET_GPS_INFO,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error reading: CGPSINFO", __FUNCTION__, __LINE__);
                    break;
                } else {
                    CY_LOGD(TAG, "%s", modem_p->line_buffer_p);

                    if (extract_gps_location( modem_p->line_buffer_p,
                                              location,
                                              sizeof(location))) {
                        // success
                        result = true;
                        break;
                    }
                }
            }
        }
    }

    if (result) {
#if (HAVE_FLASH_EEPROM == 1)
        CY_LOGD(TAG, "save updated GPS info into eeprom");
        result = flash_eeprom_set_gps_location(location);
#endif

        // GPS Off
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_GPS_SESSION_STOP,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error turning off GPS", __FUNCTION__, __LINE__);
            } else {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
    }

    if (is_mode_changed) {
        // restore the previous mode
        cy_modem_change_mode(modem_p,
                             CY_MODEM_PPP_MODE);
    }

    return result;
}
#endif

const char* cy_modem_get_ue_system_info(cy_modem_t *modem_p)
{
    ReturnAssert(modem_p != NULL, "");
    return modem_p->ueSystemInfo;
}

bool cy_modem_change_mode( cy_modem_t *modem_p,
                           cy_modem_mode_t new_mode)
{
    bool result = false;

    ReturnAssert(modem_p != NULL, false);
    CY_LOGD(TAG, "%s [%d]: new_mode=%d", __FUNCTION__, __LINE__, new_mode);

    switch (new_mode) {
    case CY_MODEM_PPP_MODE:
        if (modem_p->is_data_connected) {
            // resume a previous data connection
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_SWITCH_CMD_TO_DATA_MODE,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: enter ppp mode failed", __FUNCTION__, __LINE__);
                } else {
                    CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                    result = true;
                }
            }
        } else {
            // start a new data connection
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_DIAL,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: enter ppp mode failed", __FUNCTION__, __LINE__);
                } else {
                    CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                    result = true;
                }
            }
        }

        modem_p->mode = CY_MODEM_PPP_MODE;
        Modem_EnableReadCallback(modem_p->handle);
        break;

    case CY_MODEM_COMMAND_MODE:
        Modem_DisableReadCallback(modem_p->handle);
        Modem_FlushAll(modem_p->handle);

        if (modem_p->mode == CY_MODEM_PPP_MODE) {
            // quiet period before sending "+++"
            cy_rtos_delay_milliseconds(SWITCH_PPP_TO_CMD_MODE_DELAY_MSEC);
        }

        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_SWITCH_DATA_TO_CMD_MODE,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: exit ppp mode failed", __FUNCTION__, __LINE__);
            } else {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                result = true;
            }
        }

        modem_p->mode = CY_MODEM_COMMAND_MODE;
        break;

    default:
        break;
    }

    return result;
}

bool modem_start_ppp(cy_modem_t *modem_p,
                     const char *apn_p)
{
    do {
        bool result;

        // query GSM network
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_QUERY_GSM_NETWORK,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Failed to register to GSM network", __FUNCTION__, __LINE__);
                break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // query GPRS network
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_QUERY_GPRS_NETWORK,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Failed to register to GPRS network", __FUNCTION__, __LINE__);
                break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // operator name
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_OPERATOR_SELECTION,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: operator name", __FUNCTION__, __LINE__);
                break;

            } else {
                CY_LOGD(TAG, "[%d] %s", __LINE__, modem_p->line_buffer_p);

                // e.g.  +COPS: 0,0,"Singtel Tata",7
                extract_operator_name(modem_p->line_buffer_p,
                                      modem_p->operatorName,
                                      sizeof(modem_p->operatorName));
            }
        }

        // signal quality
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_QUERY_SIGNAL_QUALITY,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: signal quality", __FUNCTION__, __LINE__);
                break;

            } else {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

#ifdef AT_CMD_QUERY_UE_INFO
        // UE Info
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_QUERY_UE_INFO,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: UE Info", __FUNCTION__, __LINE__);
                break;

            } else {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);

                // e.g. +CPSI: NO SERVICE,Online
                if (strstr(modem_p->line_buffer_p, AT_RSP_UE_INFO_PATTERN_FAILED) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: No Service", __FUNCTION__, __LINE__);
                    break;
                } else {
                    // e.g.  +CPSI: LTE,Online,525-01,0x0309,187230312,184,EUTRAN-BAND3,1300,5,5,-1488
                    extract_ue_system_info( modem_p->line_buffer_p,
                                            modem_p->ueSystemInfo,
                                            sizeof(modem_p->ueSystemInfo));
                }
            }
        }
#endif

        /* Set PDP Context (APN) */
        if (strlen(apn_p) > 0) {
            SNPRINTF( modem_p->line_buffer_p,
                      modem_p->line_buffer_size,
                      AT_CMD_SET_PDP_CONTEXT "=%u,\"%s\",\"%s\"\r",
                      1,
                      "IP",
                      apn_p);
        } else {
            SNPRINTF( modem_p->line_buffer_p,
                      modem_p->line_buffer_size,
                      AT_CMD_SET_PDP_CONTEXT "=%u,\"%s\",\r",
                      1,
                      "IP");
        }

        if (Modem_SendATCommand(modem_p->handle,
                                modem_p->line_buffer_p, /* cmd */
                                modem_p->line_buffer_p, /* response */
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: pdp context failed", __FUNCTION__, __LINE__);
                break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        /* Enter PPP mode */
        result = cy_modem_change_mode(modem_p, CY_MODEM_PPP_MODE);
        if (result) {
            modem_p->is_data_connected = true;
        }

        return result;

    } while (false);

    return false;
}

bool modem_stop_ppp(cy_modem_t *modem_p)
{
    bool result;
    result = cy_modem_change_mode(modem_p, CY_MODEM_COMMAND_MODE);

    /* Hang up */
    if (Modem_SendATCommand(modem_p->handle,
                            AT_CMD_HALT_PPP_DAEMON "\r",
                            modem_p->line_buffer_p,
                            modem_p->line_buffer_size)) {
        if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
            CY_LOGE(TAG, "%s [%d]: hang up failed", __FUNCTION__, __LINE__);
        } else {
            CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            result = true;
        }
        modem_p->is_data_connected = false;
    }
    return result;
}
