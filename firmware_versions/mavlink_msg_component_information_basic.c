/**
 * @file    mavlink_msg_component_information_basic.c
 * @brief   mavlink_msg_component_information_basic.c
 * @version 1.0.0
 * @date    05.05.2025
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

 #include <stdint.h>
 #include "mavlink_msg_component_information_basic.h"
 #include "version.h"
 
 /* Peripherals */
 
 /* External hardware drivers */
 
 /* Lib */
 
 /* Middleware */
 #include "mav.h"
 
 /*******************************************************************************
  * Defines
  ******************************************************************************/
 
 /*******************************************************************************
  * Typedefs
  ******************************************************************************/
 
 /*******************************************************************************
  * Variables
  ******************************************************************************/
 
 mavlink_component_information_basic_t data;
 mavlink_message_t msg;
 static mav_t mav_gw_sky_handle;

 /*Za svaki projekat promeni na njegovu vrednost*/ 
 const Hardware_version_t hardware_v = HW_INS; 
 uint8_t major, minor, patch;
 const char version_str[] = VERSION;

 /*******************************************************************************
  * Function Prototypes
  ******************************************************************************/
 
 /*******************************************************************************
  * Code
  ******************************************************************************/
 
 const char * hardwareTypeToString(Hardware_version_t type) {
     switch (type) {
     case HW_GW_GND: return "hw_gw_gnd";
     case HW_GW_SKY: return "hw_gw_sky";
     case HW_INS_COTS: return "hw_ins_cots";
     case HW_INS: return "hw_ins";
     case HW_BLACK_BOX: return "hw_black_box";
     case HW_PWM_MAN: return "hw_pwm_man";
     case HW_MAIN_BATT: return "hw_main_batt";
     case HW_STBY_BATT: return "hw_stby_batt";
     case HW_ACT_MASTER: return "hw_act_master";
     default: return "Unknown_hardware";
     }
 }
 
 const char * firmwareVersionTypeToString(FIRMWARE_VERSION_TYPE type) {
     switch (type) {
     case FIRMWARE_VERSION_TYPE_DEV: return "dev"; /* development release | */
     case FIRMWARE_VERSION_TYPE_ALPHA: return "alpha"; /* alpha release | */
     case FIRMWARE_VERSION_TYPE_BETA: return "beta"; /* beta release | */
     case FIRMWARE_VERSION_TYPE_RC: return "rc"; /* release candidate | */
     case FIRMWARE_VERSION_TYPE_OFFICIAL: return "official"; /* official stable release | */
     default: return "unknown";
     }
 }

 
 // Funkcija koja parsira verziju iz stringa
 void parser_git_version(version_str, &major, &minor, &patch) {
     int m, n, p;
     if (sscanf(version_str, "v%d.%d.%d", &m, &n, &p) == 3) {
         *major = (uint8_t)m;
         *minor = (uint8_t)n;
         *patch = (uint8_t)p;
     } else {
         *major = *minor = *patch = 0; // fallback vrednosti u slučaju greške
     }
 }
 
 mavlink_component_information_basic_t initialize_mavlink_component_info(Hardware_version_t hardware_v) {
    mavlink_component_information_basic_t data;

    const char model_name[MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_MODEL_NAME_LEN] = "LPC1768";
    const char vendor_name[MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_VENDOR_NAME_LEN] = "NXP Semiconductors";
    const char serial_number[MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_SERIAL_NUMBER_LEN] = "0x1A2B3C4D0x5E6F7A8B0x9C0D1E2F"; //change for real uid

    char software_version[MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_SOFTWARE_VERSION_LEN] = "";
    char hardware_version[MAVLINK_MSG_COMPONENT_INFORMATION_BASIC_FIELD_HARDWARE_VERSION_LEN] = "";

    uint8_t major = 0, minor = 0, patch = 0;
    parser_git_version(VERSION, &major, &minor, &patch);

    const char *header_of_verison_software = firmwareVersionTypeToString(FIRMWARE_VERSION_TYPE_DEV);
    snprintf(software_version, sizeof(software_version), "%s-%d.%d.%d", header_of_verison_software, major, minor, patch);

    const char *typeStr = hardwareTypeToString(hardware_v);
    strncpy(hardware_version, typeStr, sizeof(hardware_version));

    data.capabilities = 0;
    data.time_boot_ms = 0;
    data.time_manufacture_s = 0;

    strncpy(data.vendor_name, vendor_name, sizeof(data.vendor_name));
    strncpy(data.model_name, model_name, sizeof(data.model_name));
    strncpy(data.software_version, software_version, sizeof(data.software_version));
    strncpy(data.hardware_version, hardware_version, sizeof(data.hardware_version));
    strncpy(data.serial_number, serial_number, sizeof(data.serial_number));

    return data;
 }

 mavlink_component_information_basic_t data; 
 data = initialize_mavlink_component_info(HW_INS);
 mavlink_msg_component_information_basic_encode(mav_gw_sky_handle.sysid, mav_gw_sky_handle.compid, &msg, &data);

// /* initialize MAVLink handle */
// mav_init(&mav_gw_sky_handle, dev_mav_sysid, dev_mav_compid);

// /* initialize MAVLink UDP link */
// mav_link_init(&mav_gw_sky_link_udp, MAVLINK_COMM_0, mav_recv_cb, NULL,
//               (tl_t *)&mav_gw_sky_tl);

// /* connect MAVLink link to MAVLink handle */
// mav_link(&mav_gw_sky_handle, &mav_gw_sky_link_udp);

// /* track client messages */
// for (uint8_t i = 0; i < MAV_CLIENT_TRACK_COUNT; i++) {
//     mav_track_t *track;

//     track = (mav_track_t *)&mav_gw_sky_tracks[i];
//     mav_track(&mav_gw_sky_handle, track->msgid, track->sysid,
//               track->compid);
// }

//xQueueReceive(queue_member, &data, 0);

// mavlink_msg_component_information_basic_encode(mav_gw_sky_handle.sysid, mav_gw_sky_handle.compid, &msg, &data); //Ubaci se handle ka udp-komunikaciji
// mav_send(&mav_gw_sky_handle, &msg);

static inline void mavlink_msg_component_information_basic_decode(const mavlink_message_t* msg, mavlink_component_information_basic_t* component_information_basic)
