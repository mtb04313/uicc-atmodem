/******************************************************************************
* File Name:   cy_modem.h
*
* Description: This file is the public interface of cy_modem.c
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

#ifndef SOURCE_CY_MODEM_H_
#define SOURCE_CY_MODEM_H_

#include "feature_config.h"
#include "cyabs_rtos.h"
#include "cy_uicc_modem.h"
#include "cy_modem_mode.h"

#ifdef __cplusplus
extern "C" {
#endif


/*-- Public Definitions -------------------------------------------------*/

#define CY_MODEM_OPERATOR_NAME_MAX_LEN    80
#define CY_MODEM_GPS_MAX_LEN              80
#define CY_MODEM_UE_SYSTEM_MAX_LEN        80

typedef struct {
    cy_modem_mode_t mode;
    Modem_Handle_t handle;
    bool is_powered_on;
    bool is_data_connected;

    char* line_buffer_p;
    size_t line_buffer_size;

    void *receive_cb_ctx;
    Modem_ReadCallback_t *read_cb_p;

    char operatorName[CY_MODEM_OPERATOR_NAME_MAX_LEN];
    char ueSystemInfo[CY_MODEM_UE_SYSTEM_MAX_LEN];
    //char gpslocation[CY_MODEM_GPS_MAX_LEN];

} cy_modem_t;


/*-- Public Functions -------------------------------------------------*/

bool cy_modem_init(void);

bool cy_modem_deinit(void);

cy_modem_t * cy_modem_new(bool connect_ppp);

void cy_modem_delete(cy_modem_t *modem_p, bool power_off_modem);

bool cy_modem_powerup( cy_modem_t *modem_p, bool connect_ppp);

bool cy_modem_powerdown( cy_modem_t *modem_p);

const char* cy_modem_get_operator_name(cy_modem_t *modem_p);

bool cy_modem_update_gps_location(cy_modem_t *modem_p);

//const char* cy_modem_get_gps_location(cy_modem_t *modem_p);

const char* cy_modem_get_ue_system_info(cy_modem_t *modem_p);

bool cy_modem_change_mode(cy_modem_t *modem_p,
                          cy_modem_mode_t new_mode);

bool modem_start_ppp(cy_modem_t *modem_p,
                     const char *apn_p);

bool modem_stop_ppp(cy_modem_t *modem_p);

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_CY_MODEM_H_ */
