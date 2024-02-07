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

#if (FEATURE_FLASH_EEPROM == ENABLE_FEATURE)
#include "flash_eeprom.h"
#endif

/*-- Local Definitions -------------------------------------------------*/

#define CY_MODEM_RESULT_OK          AT_RSP_OK       //"OK"
#define CY_MODEM_RESULT_ERROR       AT_RSP_ERROR    //"ERROR"
#define CY_MODEM_RESULT_ABORTED     AT_RSP_ABORTED  //"ABORTED"

#define LINE_BUFFER_SIZE            512

#ifdef PPP_MODEM_IO_REF
#define MODEM_IO_REF                PPP_MODEM_IO_REF
#endif

#ifdef PPP_MODEM_RTS
#define MODEM_RTS                   PPP_MODEM_RTS
#endif

#define MODEM_POWER_KEY             PPP_MODEM_POWER_KEY
#define MODEM_POWER_ON              PPP_MODEM_POWER_KEY_ON_LEVEL
#define MODEM_POWER_OFF             PPP_MODEM_POWER_KEY_OFF_LEVEL

#if (PPP_MODEM_POWER_METHOD == PPP_SIMPLE_SWITCH_METHOD)
#define POWER_OFF_PULSE_WIDTH_MSEC          3000

// 20 to 30sec works; 5 to 12.5sec failed; 15sec ok on 1 modem, but failed on WaveShare dongle
#define MODEM_POWER_ON_WAIT_INTERVAL_MSEC   30000

#elif (PPP_MODEM_POWER_METHOD == PPP_POWER_STEP_METHOD)
#define POWER_ON_PULSE_WIDTH_MSEC           500
#define POWER_OFF_PULSE_WIDTH_MSEC          2500

#define MODEM_POWER_ON_WAIT_INTERVAL_MSEC   10000
#define FIRST_POWER_ON_DELAY_MSEC           20000

#elif (PPP_MODEM_POWER_METHOD == PPP_PULSE_SWITCH_METHOD)
#define POWER_ON_PULSE_WIDTH_MSEC           100
#define POWER_OFF_PULSE_WIDTH_MSEC          3000
#define MODEM_POWER_ON_WAIT_INTERVAL_MSEC   5000

#endif

#define MAX_QUERY_SIM_CARD_STATUS_RETRIES   20
#define QUERY_SIM_CARD_STATUS_INTERVAL_MSEC 1000

#define MAX_GPS_READY_RETRIES               10
#define WAIT_FOR_GPS_READY_MSEC             1000
#define SWITCH_PPP_TO_CMD_MODE_DELAY_MSEC   2000

#define MAX_QUERY_OPERATOR_SELECTION_RETRIES    100
#define QUERY_OPERATOR_SELECTION_INTERVAL_MSEC  1000

#define MAX_QUERY_NETWORK_REGISTRATION_RETRIES      10
#define QUERY_NETWORK_REGISTRATION_INTERVAL_MSEC    3000

#define MAX_QUERY_PACKET_DOMAIN_RETRIES     100
#define QUERY_PACKET_DOMAIN_INTERVAL_MSEC   1000
#define ATTACH_PACKET_DOMAIN_SERVICE        1

#define MAX_QUERY_SIGNAL_QUALITY_RETRIES    100
#define QUERY_SIGNAL_QUALITY_INTERVAL_MSEC  1000
#define SIGNAL_QUALITY_RSSI_UNKNOWN         99

#define MAX_SET_MAX_BAUD_RATE_RETRIES       5
#define SET_MAX_BAUD_RATE_INTERVAL_MSEC     1000

#define MAX_SET_PDP_CONTEXT_RETRIES         5
#define SET_PDP_CONTEXT_INTERVAL_MSEC       3000

#define NETWORK_REGISTRATION_STATUS_NOT_REG_NOT_TRYING    0
#define NETWORK_REGISTRATION_STATUS_REG_HOME              1
#define NETWORK_REGISTRATION_STATUS_TRYING                2
#define NETWORK_REGISTRATION_STATUS_REG_DENIED            3
#define NETWORK_REGISTRATION_STATUS_UNKNOWN               4
#define NETWORK_REGISTRATION_STATUS_REG_ROAM              5
#define NETWORK_REGISTRATION_STATUS_REG_SMS_HOME          6
#define NETWORK_REGISTRATION_STATUS_REG_SMS_ROAM          7
#define NETWORK_REGISTRATION_STATUS_EMERGENCY_ONLY        8
#define NETWORK_REGISTRATION_STATUS_REG_CSFB_HOME         9
#define NETWORK_REGISTRATION_STATUS_REG_CSFB_ROAM         10

#define RADIO_ACCESS_TECH_GSM               0
#define RADIO_ACCESS_TECH_GSM_COMPACT       1
#define RADIO_ACCESS_TECH_UTRAN             2
#define RADIO_ACCESS_TECH_GSM_GPRS_EDGE     3
#define RADIO_ACCESS_TECH_UTRAN_HSDPA       4
#define RADIO_ACCESS_TECH_UTRAN_HSUPAT      5
#define RADIO_ACCESS_TECH_UTRAN_HSDPA_HSUPA 6
#define RADIO_ACCESS_TECH_E_UTRAN           7
#define RADIO_ACCESS_TECH_EC_GSM_IOT        8
#define RADIO_ACCESS_TECH_E_UTRAN_NB_S1     9

// Modem_SendATCommandEx timeout values
#define UE_INFO_WAIT_TIME_BEFORE_READ_MSEC  6000
#define UE_INFO_READ_TIMEOUT_MSEC           10000
#define ICCID_WAIT_TIME_BEFORE_READ_MSEC    1000
#define ICCID_READ_TIMEOUT_MSEC             3000

// Quectel BG96 - do once so as to enable BIP
#define READ_BG96_CFG_CMD       0
#define WRITE_BG96_CFG_CMD      0


/*-- Local Data -------------------------------------------------*/
static const char *TAG = "modem";

static char s_operatorName[CY_MODEM_OPERATOR_NAME_MAX_LEN] = "";
static char s_ueSystemInfo[CY_MODEM_UE_SYSTEM_MAX_LEN] = "";
static char s_iccid[CY_MODEM_ICCID_MAX_LEN] = "";
static int  s_rssi = SIGNAL_QUALITY_RSSI_UNKNOWN;

/*-- Local Functions -------------------------------------------------*/

// e.g.  +ICCID: 8931070422315955905\r\n\r\nOK
// e.g.   +CCID: 8931070522316030820\r\n\r\nOK
static bool extract_iccid(const char* input_p,
                          char *buf_p,
                          size_t bufSize)
{
  ReturnAssert(input_p != NULL, false);
  ReturnAssert(buf_p != NULL, false);

  char *start_p;
  char *end_p = NULL;

  //CY_LOGD(TAG, "%s [%d] input_p = %s", __FUNCTION__, __LINE__, input_p);

  // look for colon or space
  start_p = strchr(input_p, ':');
  if (start_p == NULL) {
    start_p = strchr(input_p, ' ');
  }

  if (start_p != NULL) {
    // look for \r or \n
    end_p = strchr(start_p + 1, '\r');
    if (end_p == NULL) {
      end_p = strchr(start_p + 1, '\n');
    }
  }
  else {
    CY_LOGD(TAG, "%s [%d] start_p is NULL", __FUNCTION__, __LINE__);
  }

  if (end_p != NULL) {
    *end_p = '\0';
    start_p++;

    // remove leading space
    start_p = left_trim(start_p);

    SNPRINTF(buf_p, bufSize, "%s", start_p);
    CY_LOGD(TAG, "ICCID: %s", buf_p);
    return (strlen(buf_p) > 0);
  }
  else {
    CY_LOGD(TAG, "%s [%d] end_p is NULL", __FUNCTION__, __LINE__);
  }

  return false;
}

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

static const char* str_network_registration_status(int status)
{
    switch(status) {
    case NETWORK_REGISTRATION_STATUS_NOT_REG_NOT_TRYING:
        return "Not registered - not trying";

    case NETWORK_REGISTRATION_STATUS_REG_HOME:
        return "Registered - home network";

    case NETWORK_REGISTRATION_STATUS_TRYING:
        return "Trying to register";

    case NETWORK_REGISTRATION_STATUS_REG_DENIED:
        return "Registration denied";

    case NETWORK_REGISTRATION_STATUS_UNKNOWN:
        return "Unknown";

    case NETWORK_REGISTRATION_STATUS_REG_ROAM:
        return "Registered - roaming";

    case NETWORK_REGISTRATION_STATUS_REG_SMS_HOME:
        return "Registered - SMS only, home network";

    case NETWORK_REGISTRATION_STATUS_REG_SMS_ROAM:
        return "Registered - SMS only, roaming";

    case NETWORK_REGISTRATION_STATUS_EMERGENCY_ONLY:
        return "Attached for emergency bearer services only";

    case NETWORK_REGISTRATION_STATUS_REG_CSFB_HOME:
        return "Registered - CSFB not preferred, home network";

    case NETWORK_REGISTRATION_STATUS_REG_CSFB_ROAM:
        return "Registered - CSFB not preferred, roaming";

    default:
        break;
    }
    return "Invalid";
}

static const char* str_radio_access_technology(int tech)
{
    switch(tech) {
    case RADIO_ACCESS_TECH_GSM:
        return "GSM; 2G";

    case RADIO_ACCESS_TECH_GSM_COMPACT:
        return "GSM Compact";

    case RADIO_ACCESS_TECH_UTRAN:
        return "UTRAN; 3G";

    case RADIO_ACCESS_TECH_GSM_GPRS_EDGE:
        return "GSM/GPRS with EDGE availability; 2.75G";

    case RADIO_ACCESS_TECH_UTRAN_HSDPA:
        return "UTRAN with HSDPA availability; 3.5G";

    case RADIO_ACCESS_TECH_UTRAN_HSUPAT:
        return "UTRAN with HSUPA availability; 3.5G";

    case RADIO_ACCESS_TECH_UTRAN_HSDPA_HSUPA:
        return "UTRAN with HSDPA and HSUPA availability; 3.5G";

    case RADIO_ACCESS_TECH_E_UTRAN:
        return "E-UTRAN; LTE";

    case RADIO_ACCESS_TECH_EC_GSM_IOT:
        return "EC-GSM-IoT (A/Gb mode); LTE Cat M1";

    case RADIO_ACCESS_TECH_E_UTRAN_NB_S1:
        return "E-UTRAN (NB-S1 mode); LTE Cat NB1";

    default:
        break;
    }
    return "Invalid";
}

// Warning: line_buffer_p may be altered
static bool parse_network_registration_response(char *line_buffer_p)
{
    char *first_p;
    ReturnAssert(line_buffer_p != NULL, false);

    //CY_LOGD(TAG, "%s [%d] %s", __FUNCTION__, __LINE__, line_buffer_p);

    // get the field after the first separator
    if ((first_p = strstr(line_buffer_p, ",")) != NULL) {
        first_p++;

        char *second_p;
        if ((second_p = strstr(first_p, ",")) != NULL) {
            *second_p = '\0';
        }

        int status;
        status = atoi(first_p);
        CY_LOGD(TAG, "Status (stat) = %d - %s",
                status, str_network_registration_status(status));
        second_p++;

        char *third_p;
        if ((third_p = strstr(second_p, ",")) != NULL) {
            *third_p = '\0';

            CY_LOGD(TAG, "Location Area (lac) = %s", second_p);
            third_p++;

            char *fourth_p;
            if ((fourth_p = strstr(third_p, ",")) != NULL) {
                *fourth_p = '\0';

                CY_LOGD(TAG, "Cell ID (ci) = %s", third_p);
                fourth_p++;

                char *fifth_p;
                if ((fifth_p = strstr(fourth_p, ",")) != NULL) {
                    *fifth_p = '\0';
                    fifth_p++;
                }

                if (*fourth_p != '\0') {
                    int tech;
                    tech = atoi(fourth_p);
                    CY_LOGD(TAG, "Access Tech (AcT) = %d - %s",
                            tech, str_radio_access_technology(tech));
                }

                if (fifth_p != NULL) {
                    if (*fifth_p != '\0') {
                        CY_LOGD(TAG, "Routing Area (rac) = %s", fifth_p);
                    }
                }
            }
        }

        if ((status == NETWORK_REGISTRATION_STATUS_REG_HOME) ||
            (status == NETWORK_REGISTRATION_STATUS_REG_ROAM)) {
            return true;
        }
    }

    return false;
}

// Warning: line_buffer_p may be altered
static bool parse_attach_packet_domain_response(char *line_buffer_p)
{
    ReturnAssert(line_buffer_p != NULL, false);

    //CY_LOGD(TAG, "%s [%d] %s", __FUNCTION__, __LINE__, line_buffer_p);

    char *space_p;

    if ((space_p = strstr(line_buffer_p, " ")) != NULL) {
        char *crlf_p;

        space_p++;

        // get the CR LF after the first space separator
        if ((crlf_p = strstr(space_p, "\r\n")) != NULL) {

            int value;
            *crlf_p = '\0';

            value = atoi(space_p);
            CY_LOGD(TAG, "value = %d", value);

            if (value == ATTACH_PACKET_DOMAIN_SERVICE) {
                return true;
            }
        }
    }

    return false;
}

// Warning: line_buffer_p may be altered
static bool parse_signal_quality_response(char *line_buffer_p, int *rssi_p)
{
    char *comma_p;
    ReturnAssert(line_buffer_p != NULL, false);

    //CY_LOGD(TAG, "%s [%d] %s", __FUNCTION__, __LINE__, line_buffer_p);

    // get the field before the first separator
    if ((comma_p = strstr(line_buffer_p, ",")) != NULL) {
        char *space_p;

        if ((space_p = strstr(line_buffer_p, " ")) != NULL) {
            space_p++;

            if (space_p < comma_p) {
                int rssi;
                *comma_p = '\0';

                rssi = atoi(space_p);
                CY_LOGD(TAG, "rssi = %d", rssi);

                if (rssi < SIGNAL_QUALITY_RSSI_UNKNOWN) {
                    // caller wants the result
                    if (rssi_p != NULL) {
                        *rssi_p = rssi;
                    }
                    return true;
                }
            }
        }
    }

    return false;
}

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
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
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
            //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
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
        if (value != MODEM_POWER_ON) {
            cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_ON);
        } else {
            // do a pulse
            do_modem_power_on_pulse();
        }

#elif (PPP_MODEM_POWER_METHOD == PPP_PULSE_SWITCH_METHOD)
        cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_ON);
        cy_rtos_delay_milliseconds(POWER_ON_PULSE_WIDTH_MSEC);

        cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_OFF);
        cy_rtos_delay_milliseconds(POWER_ON_PULSE_WIDTH_MSEC);

        cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_ON);
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

#if ((PPP_MODEM_POWER_METHOD == PPP_SIMPLE_SWITCH_METHOD) || \
     (PPP_MODEM_POWER_METHOD == PPP_PULSE_SWITCH_METHOD))

        cyhal_gpio_write(MODEM_POWER_KEY, MODEM_POWER_OFF);
        cy_rtos_delay_milliseconds(POWER_OFF_PULSE_WIDTH_MSEC);

#elif (PPP_MODEM_POWER_METHOD == PPP_POWER_STEP_METHOD)

        bool value = cyhal_gpio_read(MODEM_POWER_KEY);
        if (value == MODEM_POWER_OFF) {
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

#define BAUD_RATE_CMD_BUF_SIZE      80

static bool modem_set_max_baud_rate(cy_modem_t *modem_p,
                                    uint32_t baudrate)
{
    bool result = false;
    int i;
    char temp_cmd_buffer[BAUD_RATE_CMD_BUF_SIZE] = "";

    ReturnAssert(modem_p != NULL, false);
    CY_LOGD(TAG, "%s [%d]: baudrate = %lu",
            __FUNCTION__, __LINE__, baudrate);

    SNPRINTF( temp_cmd_buffer,
              sizeof(temp_cmd_buffer),
              AT_CMD_SET_BAUD_RATE "=%lu\r",
              baudrate);

    for (i = 0; i < MAX_SET_MAX_BAUD_RATE_RETRIES; i++) {
        if (Modem_SendATCommand(modem_p->handle,
                                temp_cmd_buffer,
                                modem_p->line_buffer_p, /* response */
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: set max baud rate failed", __FUNCTION__, __LINE__);
            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                result = true;
            }
        }

        if (result) {
            result = (bool)Modem_SetBaudRate(modem_p->handle, baudrate);
            break;
        }
        else {
            cy_rtos_delay_milliseconds(SET_MAX_BAUD_RATE_INTERVAL_MSEC);
        }
    }

    return result;
}
#undef BAUD_RATE_CMD_BUF_SIZE

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

#ifdef MODEM_RTS
    /* Initialize the GPIO for modem RTS */
    result = cyhal_gpio_init( MODEM_RTS,  //    ATMODEM_HW_PIN_UART_RTS
                              CYHAL_GPIO_DIR_OUTPUT,
                              CYHAL_GPIO_DRIVE_STRONG,
                              MODEM_POWER_OFF);  // default is low

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

    cy_modem_delete(modem_p, true);
    return NULL;
}

void cy_modem_delete(cy_modem_t *modem_p, bool power_off_modem)
{
    if (modem_p != NULL) {
        if (power_off_modem) {
            cy_modem_powerdown(modem_p);
        }

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

bool cy_modem_powerup(cy_modem_t *modem_p, bool connect_ppp)
{
    do {
        if (connect_ppp) {
            if (Modem_IsFlagFirstRead(modem_p->handle)) {
                CY_LOGD(TAG, "First Power On");

                // assume the modem may be already running (due to a previous session)
                modem_p->is_powered_on = true;

                // in case the modem is already running PPP, stop it first
                CY_LOGD(TAG, "Stop PPP");
                modem_stop_ppp(modem_p);

                // then turn off the modem
                CY_LOGD(TAG, "Power off modem");

#if ((PPP_MODEM_POWER_METHOD == PPP_SIMPLE_SWITCH_METHOD) || \
     (PPP_MODEM_POWER_METHOD == PPP_PULSE_SWITCH_METHOD))

                cy_modem_powerdown(modem_p);

#elif (PPP_MODEM_POWER_METHOD == PPP_POWER_STEP_METHOD)

                int i;
                for (i = 0; i < 2; i++) {
                    CY_LOGD(TAG, "Power step %d", i);
                    cy_modem_powerdown(modem_p);
                    do_modem_power_on_pulse();
                }

                // combined the 2 delays
                cy_rtos_delay_milliseconds(FIRST_POWER_ON_DELAY_MSEC);
#endif
            }
        }

        if (!modem_power_on(modem_p)) {
            CY_LOGE(TAG, "%s [%d]: modem_power_on failed", __FUNCTION__, __LINE__);
            break;
        }

        if (connect_ppp) {
            int i;

            // set echo: off
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_ECHO_OFF,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error reading: ATE0", __FUNCTION__, __LINE__);
                    break;
                } else {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }

            // set flow control: none
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_SET_FLOW_CONTROL_NONE,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error: flow control", __FUNCTION__, __LINE__);
                    // continue upon error //break;
                } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }

            // set baud rate
            if (!modem_set_max_baud_rate(modem_p, MAX_MODEM_BAUD_RATE)) {
                CY_LOGE(TAG, "%s [%d]: modem_set_max_baud_rate failed", __FUNCTION__, __LINE__);
                break;
            }

            // set error msg format: verbose
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_SET_ERROR_MSG_FORMAT_VERBOSE,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error: msg format", __FUNCTION__, __LINE__);
                    break;
                } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }

            // set TA response format: verbose
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_SET_TA_RESPONSE_FORMAT_VERBOSE,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error: TA response format", __FUNCTION__, __LINE__);
                    break;
                } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }

#ifdef AT_CMD_SET_DTR_FUNCTION_MODE_IGNORE
            // set DTR function mode: ignore
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_SET_DTR_FUNCTION_MODE_IGNORE,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error: DTR function mode", __FUNCTION__, __LINE__);
                    break;
                } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }
#endif

            // query baud rate
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_QUERY_BAUD_RATE,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error: query baud rate", __FUNCTION__, __LINE__);
                    break;
                } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }

#if (READ_BG96_CFG_CMD == 1)
#define AT_CMD_READ_BG96_CFG_CMD_1      "AT+QNVFR=\"/nv/item_files/modem/uim/gstk/proactive_feature_enable_cfg\"\r"
#define AT_CMD_READ_BG96_CFG_CMD_2      "AT+QNVR=6253,0\r"

            // set QNVFR
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_READ_BG96_CFG_CMD_1,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error: QNVFR", __FUNCTION__, __LINE__);
                    break;
                } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }

            // set QNVR
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_READ_BG96_CFG_CMD_2,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error: QNVR", __FUNCTION__, __LINE__);
                    break;
                } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }
#endif

#if (WRITE_BG96_CFG_CMD == 1)
#define AT_CMD_WRITE_BG96_CFG_CMD_1     "AT+QNVFW=\"/nv/item_files/modem/uim/gstk/proactive_feature_enable_cfg\",7FFFFF7F\r" //Need to change to 7FFFFF7F
#define AT_CMD_WRITE_BG96_CFG_CMD_2     "AT+QNVW=6253,0,\"01\"\r"   //Need to change to 01

            // set QNVFW
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_WRITE_BG96_CFG_CMD_1,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error: QNVFW", __FUNCTION__, __LINE__);
                    break;
                } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }

            // set QNVW
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_WRITE_BG96_CFG_CMD_2,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error: QNVW", __FUNCTION__, __LINE__);
                    break;
                } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }
#endif

            // query SIM card status
            bool simCardError = true;

            for (i = 0; i < MAX_QUERY_SIM_CARD_STATUS_RETRIES; i++) {
                if (Modem_SendATCommand(modem_p->handle,
                                        AT_CMD_QUERY_SIM_CARD_STATUS,
                                        modem_p->line_buffer_p,
                                        modem_p->line_buffer_size)) {
                    if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                        CY_LOGE(TAG, "%s [%d]: Sim Card error", __FUNCTION__, __LINE__);
                    } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                        //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                        simCardError = false;
                        break;
                    }
                }

                cy_rtos_delay_milliseconds(QUERY_SIM_CARD_STATUS_INTERVAL_MSEC);
            }

            if (simCardError) {
                CY_LOGE(TAG, "%s [%d]: USIM Card Error", __FUNCTION__, __LINE__);
                break;
            }
#if 0
            // IMSI number
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_IMSI,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error reading: IMSI", __FUNCTION__, __LINE__);
                    break;
                } else {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }
#endif
            // ICCID
            if (Modem_SendATCommandEx(modem_p->handle,
                                      AT_CMD_ICCID,
                                      modem_p->line_buffer_p,
                                      modem_p->line_buffer_size,
                                      ICCID_WAIT_TIME_BEFORE_READ_MSEC,
                                      ICCID_READ_TIMEOUT_MSEC,
                                      true)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error reading: ICCID", __FUNCTION__, __LINE__);
                    break;
                } else {
                    // e.g.  +ICCID: 8931070422315955905\r\n\r\nOK
                    extract_iccid(modem_p->line_buffer_p,
                                  s_iccid,
                                  sizeof(s_iccid));
                }
            }

            //cy_modem_update_gps_location(modem_p);
        }
        else {

#ifdef AT_CMD_QUERY_HOTSWAP_LEVEL
            // query SIM card Hot Swap Level
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_QUERY_HOTSWAP_LEVEL,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {
                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
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
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                }
            }
#endif
        }

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
                                AT_CMD_POWER_OFF_MODEM,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error: power down", __FUNCTION__, __LINE__);
            } else {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
    }
#endif

    modem_power_off(modem_p);

    return true;
}

const char* cy_modem_get_operator_name(void)
{
  return s_operatorName;
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
            //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            result = true;
        }
    }

    if (result) {
        result = false;

        for (i = 0; i < MAX_GPS_READY_RETRIES; i++) {
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
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);

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
#if (FEATURE_FLASH_EEPROM == ENABLE_FEATURE)
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
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
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

const char* cy_modem_get_ue_system_info(void)
{
    return s_ueSystemInfo;
}

const char* cy_modem_get_iccid(void)
{
    return s_iccid;
}

int cy_modem_get_rssi(void)
{
    return s_rssi;
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
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
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
                    //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
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
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
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

#define PDP_CONTEXT_CMD_BUF_SIZE      80

bool modem_start_ppp(cy_modem_t *modem_p,
                     const char *apn_p)
{
    do {
        bool result;
        int i;
        bool networkRegistrationError = true;

        // modem manufacturer
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_MFG,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: mfg", __FUNCTION__, __LINE__);
                break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // modem model
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_MODEL,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: model", __FUNCTION__, __LINE__);
                break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // modem IMEI
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_IMEI,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: IMEI", __FUNCTION__, __LINE__);
                break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // modem version
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_VERSION,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: Version", __FUNCTION__, __LINE__);
                break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

#ifdef AT_CMD_ENABLE_STK
        // enable SIM toolkit
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_ENABLE_STK,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error enabling SIM toolkix", __FUNCTION__, __LINE__);

                // continue even if we fail here
                //break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
#endif

#ifdef AT_CMD_IDENT_0
        // modem identity 0
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_IDENT_0,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: Identity0", __FUNCTION__, __LINE__);

                // continue even if we fail here
                //break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
#endif

#ifdef AT_CMD_IDENT_6
        // modem identity 6
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_IDENT_6,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: Identity6", __FUNCTION__, __LINE__);

                // continue even if we fail here
                //break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
#endif

#ifdef AT_CMD_IDENT_9
        // modem identity 9
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_IDENT_9,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: Identity9", __FUNCTION__, __LINE__);
                //break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
#endif

        // modem capabilities
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_CAPABILITIES,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: Capabilities", __FUNCTION__, __LINE__);
                //break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

#ifdef AT_CMD_OPERATOR_SELECTION
        // cellular operator name
        bool operatorSelectionError = true;

        for (i = 0; i < MAX_QUERY_OPERATOR_SELECTION_RETRIES; i++) {
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_OPERATOR_SELECTION,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {

                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error reading: operator name", __FUNCTION__, __LINE__);
                    break;

                } else {
                    //CY_LOGD(TAG, "[%d] %s", __LINE__, modem_p->line_buffer_p);

                    if (strstr(modem_p->line_buffer_p, ",") != NULL) {
                        // e.g.  +COPS: 0,0,"Singtel Tata",7
                        extract_operator_name(modem_p->line_buffer_p,
                                              s_operatorName,
                                              sizeof(s_operatorName));

                        operatorSelectionError = false;
                        break;
                    }
                }
            }

            cy_rtos_delay_milliseconds(QUERY_OPERATOR_SELECTION_INTERVAL_MSEC);
        }

        if (operatorSelectionError) {
            CY_LOGE(TAG, "%s [%d]: Operator Selection Error", __FUNCTION__, __LINE__);

            // continue even if we fail here
            //break;
        }

        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_OPERATOR_SELECTION_AUTO_MODE,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error setting operator selection, auto mode", __FUNCTION__, __LINE__);
                break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
#endif

#ifdef AT_CMD_TEST_GSM_NETWORK
        networkRegistrationError = true;

        // GSM network (2G)
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_TEST_GSM_NETWORK,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if ((strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) ||
                (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ABORTED) != NULL)) {

                // command is supported
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);

                if (Modem_SendATCommand(modem_p->handle,
                                        AT_CMD_SET_GSM_NETWORK_PRESENTATION,
                                        modem_p->line_buffer_p,
                                        modem_p->line_buffer_size)) {

                    if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                        CY_LOGE(TAG, "%s [%d]: Failed to set GSM network register presentation", __FUNCTION__, __LINE__);

                        // continue even if we fail here
                        //break;

                    } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                        //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                    }
                }

                for (i = 0; i < MAX_QUERY_NETWORK_REGISTRATION_RETRIES; i++) {
                    if (Modem_SendATCommand(modem_p->handle,
                                            AT_CMD_QUERY_GSM_NETWORK,
                                            modem_p->line_buffer_p,
                                            modem_p->line_buffer_size)) {

                        if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                            CY_LOGE(TAG, "%s [%d]: Failed to register to GSM network", __FUNCTION__, __LINE__);
                            break;

                        } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                            //CY_LOGD(TAG, "[%d] %s", i, modem_p->line_buffer_p);

                            if (parse_network_registration_response(modem_p->line_buffer_p)) {
                                networkRegistrationError = false;
                                break;
                            }

                            cy_rtos_delay_milliseconds(QUERY_NETWORK_REGISTRATION_INTERVAL_MSEC);
                        }
                    }
                }

                if (networkRegistrationError) {
                    CY_LOGE(TAG, "%s [%d]: 2G Network Registration Error", __FUNCTION__, __LINE__);

                    // continue even if we fail here
                    //break;
                }
            }
        }
#endif


#ifdef AT_CMD_TEST_GPRS_NETWORK
        networkRegistrationError = true;

        // GPRS network (3G)
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_TEST_GPRS_NETWORK,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if ((strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) ||
                (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ABORTED) != NULL)) {

                // command is supported
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);

                if (Modem_SendATCommand(modem_p->handle,
                                        AT_CMD_SET_GPRS_NETWORK_PRESENTATION,
                                        modem_p->line_buffer_p,
                                        modem_p->line_buffer_size)) {

                    if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                        CY_LOGE(TAG, "%s [%d]: Failed to set GSM network register presentation", __FUNCTION__, __LINE__);

                        // continue even if we fail here
                        //break;

                    } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                        //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                    }
                }

                for (i = 0; i < MAX_QUERY_NETWORK_REGISTRATION_RETRIES; i++) {
                    if (Modem_SendATCommand(modem_p->handle,
                                            AT_CMD_QUERY_GPRS_NETWORK,
                                            modem_p->line_buffer_p,
                                            modem_p->line_buffer_size)) {

                        if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                            CY_LOGE(TAG, "%s [%d]: Failed to register to GPRS network", __FUNCTION__, __LINE__);
                            break;

                        } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                            //CY_LOGD(TAG, "[%d] %s", i, modem_p->line_buffer_p);

                            if (parse_network_registration_response(modem_p->line_buffer_p)) {
                                networkRegistrationError = false;
                                break;
                            }

                            cy_rtos_delay_milliseconds(QUERY_NETWORK_REGISTRATION_INTERVAL_MSEC);
                        }
                    }
                }

                if (networkRegistrationError) {
                    CY_LOGE(TAG, "%s [%d]: 3G Network Registration Error", __FUNCTION__, __LINE__);

                    // continue even if we fail here
                    //break;
                }
            }
        }
#endif


#ifdef AT_CMD_TEST_EPS_NETWORK
        networkRegistrationError = true;

        // EPS network (LTE/4G)
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_TEST_EPS_NETWORK,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if ((strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) ||
                (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ABORTED) != NULL)) {

                // command is supported
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);

                if (Modem_SendATCommand(modem_p->handle,
                                        AT_CMD_SET_EPS_NETWORK_PRESENTATION,
                                        modem_p->line_buffer_p,
                                        modem_p->line_buffer_size)) {

                    if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                        CY_LOGE(TAG, "%s [%d]: Failed to set EPS network register presentation", __FUNCTION__, __LINE__);

                        // continue even if we fail here
                        //break;

                    } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                        //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                    }
                }

                for (i = 0; i < MAX_QUERY_NETWORK_REGISTRATION_RETRIES; i++) {
                    if (Modem_SendATCommand(modem_p->handle,
                                            AT_CMD_QUERY_EPS_NETWORK,
                                            modem_p->line_buffer_p,
                                            modem_p->line_buffer_size)) {

                        if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                            CY_LOGE(TAG, "%s [%d]: Failed to register to GPRS network", __FUNCTION__, __LINE__);
                            break;

                        } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                            //CY_LOGD(TAG, "[%d] %s", i, modem_p->line_buffer_p);

                            if (parse_network_registration_response(modem_p->line_buffer_p)) {
                                networkRegistrationError = false;
                                break;
                            }

                            cy_rtos_delay_milliseconds(QUERY_NETWORK_REGISTRATION_INTERVAL_MSEC);
                        }
                    }
                }

                if (networkRegistrationError) {
                    CY_LOGE(TAG, "%s [%d]: LTE/4G Network Registration Error", __FUNCTION__, __LINE__);

                    // continue even if we fail here
                    //break;
                }
            }
        }
#endif

        // GPRS packet domain service (3G)
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_TEST_PACKET_DOMAIN,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                // command is supported
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);

                bool attachmentError = true;

                if (Modem_SendATCommand(modem_p->handle,
                                        AT_CMD_ATTACH_PACKET_DOMAIN,
                                        modem_p->line_buffer_p,
                                        modem_p->line_buffer_size)) {

                    if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                        CY_LOGE(TAG, "%s [%d]: Failed to attach to packet domain service", __FUNCTION__, __LINE__);

                        // continue even if we fail here
                        //break;

                    } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                        //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                    }
                }

                for (i = 0; i < MAX_QUERY_PACKET_DOMAIN_RETRIES; i++) {
                    if (Modem_SendATCommand(modem_p->handle,
                                            AT_CMD_QUERY_PACKET_DOMAIN,
                                            modem_p->line_buffer_p,
                                            modem_p->line_buffer_size)) {

                        if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                            CY_LOGE(TAG, "%s [%d]: Failed to query packet domain attachment status", __FUNCTION__, __LINE__);
                            break;

                        } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                            //CY_LOGD(TAG, "[%d] %s", i, modem_p->line_buffer_p);

                            if (parse_attach_packet_domain_response(modem_p->line_buffer_p)) {
                                attachmentError = false;
                                break;
                            }

                            cy_rtos_delay_milliseconds(QUERY_PACKET_DOMAIN_INTERVAL_MSEC);
                        }
                    }
                }

                if (attachmentError) {
                    CY_LOGE(TAG, "%s [%d]: GPRS Packet Domain Attachment Error", __FUNCTION__, __LINE__);

                    // continue even if we fail here
                    //break;
                }
            }
        }

        // signal quality
        bool signalQualityError = true;

        for (i = 0; i < MAX_QUERY_SIGNAL_QUALITY_RETRIES; i++) {
            if (Modem_SendATCommand(modem_p->handle,
                                    AT_CMD_QUERY_SIGNAL_QUALITY,
                                    modem_p->line_buffer_p,
                                    modem_p->line_buffer_size)) {

                if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: Error reading: signal quality", __FUNCTION__, __LINE__);
                    break;

                } else {
                    //CY_LOGD(TAG, "[%d] %s", i, modem_p->line_buffer_p);
                }
            }

            if (parse_signal_quality_response(modem_p->line_buffer_p, &s_rssi)) {
                signalQualityError = false;
                break;
            }

            cy_rtos_delay_milliseconds(QUERY_SIGNAL_QUALITY_INTERVAL_MSEC);
        }

        if (signalQualityError) {
            CY_LOGE(TAG, "%s [%d]: Signal Quality Error", __FUNCTION__, __LINE__);
            break;
        }


#ifdef AT_CMD_QUERY_UE_INFO
        // UE Info
        if (Modem_SendATCommandEx(modem_p->handle,
                                  AT_CMD_QUERY_UE_INFO,
                                  modem_p->line_buffer_p,
                                  modem_p->line_buffer_size,
                                  UE_INFO_WAIT_TIME_BEFORE_READ_MSEC,
                                  UE_INFO_READ_TIMEOUT_MSEC,
                                  true)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error reading: UE Info", __FUNCTION__, __LINE__);
                break;

            } else {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);

                // e.g. +CPSI: NO SERVICE,Online
                if (strstr(modem_p->line_buffer_p, AT_RSP_UE_INFO_PATTERN_FAILED) != NULL) {
                    CY_LOGE(TAG, "%s [%d]: No Service", __FUNCTION__, __LINE__);
                    break;
                } else {
                    // e.g.  +CPSI: LTE,Online,525-01,0x0309,187230312,184,EUTRAN-BAND3,1300,5,5,-1488
                    extract_ue_system_info( modem_p->line_buffer_p,
                                            s_ueSystemInfo,
                                            sizeof(s_ueSystemInfo));
                }
            }
        }
#endif

        // Set Phone Functionality (Full)
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_SET_PHONE_FUNCTIONALITY_FULL,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error set phone functionality: Full", __FUNCTION__, __LINE__);
                break;

            } else {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

        // Get Phone Functionality
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_GET_PHONE_FUNCTIONALITY,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error get phone functionality", __FUNCTION__, __LINE__);
                break;

            } else {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }

#ifdef AT_CMD_TEST_PDP_CONTEXT
        /* Set PDP Context (APN) */
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_TEST_PDP_CONTEXT,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                // command is supported
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
#endif


                bool setApnError = true;

                for (i = 0; i < MAX_SET_PDP_CONTEXT_RETRIES; i++) {


                    if (Modem_SendATCommand(modem_p->handle,
                                            AT_CMD_QUERY_PDP_CONTEXT,
                                            modem_p->line_buffer_p,
                                            modem_p->line_buffer_size)) {
                        if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                            CY_LOGE(TAG, "%s [%d]: Error in reading PDP contexts", __FUNCTION__, __LINE__);

                            // continue even if we fail here
                            //break;

                        } else {
                            //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                        }
                    }

                    char temp_cmd_buffer[PDP_CONTEXT_CMD_BUF_SIZE] = "";

                    if (strlen(apn_p) > 0) {
                        SNPRINTF( temp_cmd_buffer,
                                  sizeof(temp_cmd_buffer),
                                  AT_CMD_SET_PDP_CONTEXT "=%u,\"%s\",\"%s\"\r",
                                  AT_CMD_PDP_CONTEXT_CID,
                                  AT_CMD_PDP_CONTEXT_TYPE,
                                  apn_p);
                    } else {
                        SNPRINTF( temp_cmd_buffer,
                                  sizeof(temp_cmd_buffer),
                                  AT_CMD_SET_PDP_CONTEXT "=%u,\"%s\",\r",
                                  AT_CMD_PDP_CONTEXT_CID,
                                  AT_CMD_PDP_CONTEXT_TYPE);
                    }

                    if (Modem_SendATCommand(modem_p->handle,
                                            temp_cmd_buffer,
                                            modem_p->line_buffer_p, /* response */
                                            modem_p->line_buffer_size)) {

                        if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                            CY_LOGE(TAG, "%s [%d]: PDP context failed", __FUNCTION__, __LINE__);

                        } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                            //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                            setApnError = false;
                            break;
                        }
                    }
                    cy_rtos_delay_milliseconds(SET_PDP_CONTEXT_INTERVAL_MSEC);
                }

                if (setApnError) {
                    CY_LOGE(TAG, "%s [%d]: Set PDP context error", __FUNCTION__, __LINE__);

                    // continue even if we fail here
                    //break;
                }

#ifdef AT_CMD_TEST_PDP_CONTEXT
            }
        }
#endif


#ifdef AT_CMD_ACTIVATE_PDP_CONTEXT
        // Activate PDP context
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_ACTIVATE_PDP_CONTEXT,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {
            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error activating PDP context", __FUNCTION__, __LINE__);

                // continue even if we fail here
                //break;

            } else {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
#endif
        // show pdp address
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_TEST_PDP_ADDRESS,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                // command is supported
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);

                if (Modem_SendATCommand(modem_p->handle,
                                        AT_CMD_SHOW_PDP_ADDRESS,
                                        modem_p->line_buffer_p,
                                        modem_p->line_buffer_size)) {

                    if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                        CY_LOGE(TAG, "%s [%d]: Error: show PDP address", __FUNCTION__, __LINE__);

                        // continue even if we fail here
                        //break;

                    } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                        //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
                    }
                }
            }
        }

#ifdef AT_CMD_SET_CONNECT_RESPONSE_FORMAT
        // connect response format
        if (Modem_SendATCommand(modem_p->handle,
                                AT_CMD_SET_CONNECT_RESPONSE_FORMAT,
                                modem_p->line_buffer_p,
                                modem_p->line_buffer_size)) {

            if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
                CY_LOGE(TAG, "%s [%d]: Error: set connect response format", __FUNCTION__, __LINE__);

                // continue even if we fail here
                //break;

            } else if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_OK) != NULL) {
                //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            }
        }
#endif

        /* Enter PPP mode */
        result = cy_modem_change_mode(modem_p, CY_MODEM_PPP_MODE);
        if (result) {
            modem_p->is_data_connected = true;
        }

        return result;

    } while (false);

    return false;
}
#undef PDP_CONTEXT_CMD_BUF_SIZE


bool modem_stop_ppp(cy_modem_t *modem_p)
{
    bool result;
    result = cy_modem_change_mode(modem_p, CY_MODEM_COMMAND_MODE);

    /* Hang up */
    if (Modem_SendATCommand(modem_p->handle,
                            AT_CMD_HALT_PPP_DAEMON,
                            modem_p->line_buffer_p,
                            modem_p->line_buffer_size)) {
        if (strstr(modem_p->line_buffer_p, CY_MODEM_RESULT_ERROR) != NULL) {
            CY_LOGE(TAG, "%s [%d]: hang up failed", __FUNCTION__, __LINE__);
        } else {
            //CY_LOGD(TAG, "%s", modem_p->line_buffer_p);
            result = true;
        }
        modem_p->is_data_connected = false;
    }

    result = true; // ignore errors

    return result;
}
