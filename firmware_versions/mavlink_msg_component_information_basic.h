/**
 * @file    mavlink_msg_component_information_basic.h
 * @brief   MAVLINK_MSG_COMPONENT_INFORMATION_BACIS_H
 * @version 1.0.0
 * @date    05.05.2025
 * @author  LisumLab
 */

 #ifndef MAVLINK_MSG_COMPONENT_INFORMATION_BACIS_H
 #define MAVLINK_MSG_COMPONENT_INFORMATION_BACIS_H
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /*******************************************************************************
  * Includes
  ******************************************************************************/
 
 #include <stdint.h>
 
 #include "mav.h"
 #include "task_mav.h"
 #include "types.h"
 #include "mavlink_types.h"
 
 /*******************************************************************************
  * Defines
  ******************************************************************************/
 
 /*******************************************************************************
  * Typedefs
  ******************************************************************************/
 typedef enum {
    HW_GW_GND = 0,
    HW_GW_SKY = 1,
    HW_INS_COTS = 2,
    HW_INS = 3,
    HW_BLACK_BOX = 4,
    HW_PWM_MAN = 5,
    HW_MAIN_BATT = 6,
    HW_STBY_BATT = 7,
    HW_ACT_MASTER = 8,
}Hardware_version_t;


 /*******************************************************************************
  * Variables
  ******************************************************************************/
 
 /*******************************************************************************
  * API
  ******************************************************************************/

 const char * hardwareTypeToString ( Hardware_version_t type ); 
 const char * firmwareVersionTypeToString( FIRMWARE_VERSION_TYPE type );
 void parser_git_version(const char* version_str, uint8_t* major, uint8_t* minor, uint8_t* patch)
 mavlink_component_information_basic_t initialize_mavlink_component_info(Hardware_version_t hardware_v); 
 //  char software_version_init(FIRMWARE_VERSION_TYPE firmware_version_type,
//                              );
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* MAVLINK_MSG_COMPONENT_INFORMATION_BACIS_H */