/**
 * @file    protocol_utility.c
 * @brief   Protocol utility functions
 * @version 1.0.0
 * @date    16.12.2025
 * @author  BetaTehPro
 */

/*******************************************************************************
 * System Includes
 ******************************************************************************/
#include "protocol_utility.h"

/*******************************************************************************
 * ULog Format Includes
 ******************************************************************************/

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern TaskHandle_t task_work_handle;
extern volatile uavcan_node_Mode_1_0 current_node_mode;

/* Flags */
extern volatile bool mission_planner_connected;
extern volatile bool create_file;
extern uint8_t file_sync;

extern const char vendor_name[];
extern const char model_name[];
extern const char software_version[];
extern const char hardware_version[];
extern const char serial_number[];

extern const uint8_t CYPHAL_UID_ARRAY[16];

/*******************************************************************************
 * Code
 ******************************************************************************/

#if !MAVLINK_OR_CYPHAL

void cyphal_task_priority(const node_mode_state_t current_mode)
{
	if (current_mode == NODE_MODE_MAINTENANCE) {
		if (file_sync != 0) {
			vTaskPrioritySet(task_work_handle, 2);
			file_sync = 0;
		}
	} else {
		if (file_sync != 1) {
			vTaskPrioritySet(task_work_handle, 1);
			file_sync = 1;
		}
	}
}

void cyphal_ulog_lfs_initialization(void)
{
	ESP_ERROR_CHECK_WITHOUT_ABORT(init_ulog_system());
}

void build_getinfo_response(uavcan_node_GetInfo_Response_1_0* resp)
{
	memset(resp, 0, sizeof(*resp));

	/* Protocol version */
	resp->protocol_version.major = 1;
	resp->protocol_version.minor = 0;

	/* Hardware version parsing */
	unsigned hmaj = 1, hmin = 0;
#ifdef HARDWARE_VERSION
	{
		const char* hv = hardware_version;
		if (hv) {
			const char* ver = strchr(hv, 'V');
			if (!ver)
				ver = strchr(hv, 'v');
			if (ver) {
				ver++;
				while (*ver == ' ')
					ver++;
				char* ep = NULL;
				hmaj = (unsigned)strtoul(ver, &ep, 10);
				if (ep && *ep == '.') {
					++ep;
					hmin = (unsigned)strtoul(ep, NULL, 10);
				}
			}
		}
	}
#endif
	resp->hardware_version.major = (uint8_t)hmaj;
	resp->hardware_version.minor = (uint8_t)hmin;

	/* Software semantic version */
#ifdef FLIGHT_CUSTOM_VERSION
	{
		unsigned major = 1, minor = 0;
		const char* tag = FLIGHT_CUSTOM_VERSION;
		if (tag && tag[0] == 'v') {
			const char* p = tag + 1;
			major = (unsigned)strtoul(p, (char**)&p, 10);
			if (*p == '.') {
				p++;
				minor = (unsigned)strtoul(p, NULL, 10);
			}
		}
		resp->software_version.major = (uint8_t)major;
		resp->software_version.minor = (uint8_t)minor;
	}
#else
	resp->software_version.major = 1;
	resp->software_version.minor = 0;
#endif

	/* VCS revision */
#ifdef GIT_COMMIT_HASH
	{
		uint64_t rev = 0;
		const char* h = GIT_COMMIT_HASH;
		uint8_t digits = 0;
		while (*h && digits < 16) {
			char c = *h++;
			uint8_t v = 0;
			if (c >= '0' && c <= '9')
				v = (uint8_t)(c - '0');
			else if (c >= 'a' && c <= 'f')
				v = (uint8_t)(10 + c - 'a');
			else if (c >= 'A' && c <= 'F')
				v = (uint8_t)(10 + c - 'A');
			else
				break;
			rev = (rev << 4) | v;
			digits++;
		}
		resp->software_vcs_revision_id = rev;
	}
#else
	resp->software_vcs_revision_id = 0x1ULL;
#endif

	/* Unique ID */
	memcpy(resp->unique_id, CYPHAL_UID_ARRAY, sizeof(resp->unique_id));

	/* Node name */
#ifdef FLIGHT_CUSTOM_VERSION
	{
		const char long_name[] = "Cyphel_Node:product.black_box.rev.";
		size_t n = sizeof(long_name) - 1;
		if (n > 50)
			n = 50;
		memcpy(resp->name.elements, long_name, n);
		resp->name.count = n;
	}
#else
	{
		const char name[] = "Cyphel_UAVCAN_Node__product.black_box";
		size_t name_len = sizeof(name) - 1;
		if (name_len > 50)
			name_len = 50;
		memcpy(resp->name.elements, name, name_len);
		resp->name.count = name_len;
	}
#endif

	/* Certificate of authenticity / metadata */
	resp->certificate_of_authenticity.count = 0;
#if defined(OS_CUSTOM_HASH) || defined(MIDDLEWARE_CUSTOM_HASH)
	{
		char meta[96];
		int m = snprintf(meta, sizeof(meta), "OS=%s;MW=%s",
#ifdef OS_CUSTOM_HASH
						 OS_CUSTOM_HASH
#else
						 "?"
#endif
#ifdef MIDDLEWARE_CUSTOM_HASH
						 ,
						 MIDDLEWARE_CUSTOM_HASH
#else
						 ,
						 "?"
#endif
		);
		if (m > 0) {
			if (m > (int)sizeof(resp->certificate_of_authenticity.elements))
				m = (int)sizeof(resp->certificate_of_authenticity.elements);
			resp->certificate_of_authenticity.count = (size_t)m;
			memcpy(resp->certificate_of_authenticity.elements, meta, (size_t)m);
		}
	}
#endif

	/* CRC and COA filler */
	resp->software_image_crc.count = 1;
	resp->software_image_crc.elements[0] = 0x0123456789ABCDEFULL;

	if (resp->certificate_of_authenticity.count == 0) {
		const char coa_meta[] = "OS=rtos123;MW=mid456;ID=BB";
		size_t cm = sizeof(coa_meta) - 1;
		if (cm > sizeof(resp->certificate_of_authenticity.elements))
			cm = sizeof(resp->certificate_of_authenticity.elements);
		resp->certificate_of_authenticity.count = cm;
		memcpy(resp->certificate_of_authenticity.elements, coa_meta, cm);
	}
}

void print_node_mode(uint8_t mode)
{
	switch (mode) {
	case NODE_MODE_OPERATIONAL:
		printf("NODE_MODE_OPERATIONAL\n");
		break;

	case NODE_MODE_MAINTENANCE:
		printf("NODE_MODE_MAINTENANCE\n");
		break;

	case NODE_MODE_SOFTWARE_UPDATE:
		printf("NODE_MODE_SOFTWARE_UPDATE\n");
		break;

	case NODE_MODE_IDLE:
		printf("NODE_MODE_IDLE_STATE\n");
		break;

	case NODE_MODE_CRITICAL_FAILURE:
		printf("NODE_MODE_CRITICAL_FAILURE\n");
		break;

	default:
		printf("NEPOZNAT MODE: %u\n", mode);
		break;
	}
}

#else

void mavlink_ulog_lfs_initialization(void)
{
	ESP_ERROR_CHECK_WITHOUT_ABORT(init_ulog_system());
}

#endif /* MAVLINK_OR_CYPHAL */