#include "ble.h"
#include "ble_srv_common.h"
#include <stdbool.h>
#include <stdint.h>

/* clang-format off */
// Defined as an array due to exceeding max length for an integer constant
// in reverse byte order due to little endian
#define BIKE_SERVICE_UUID_BASE {    \
    0x66,                           \
    0x9a,                           \
    0x0c,                           \
    0x20,                           \
    0x00,                           \
    0x08,                           \
    0x98,                           \
    0x95,                           \
    0xe7,                           \
    0x11,                           \
    0xf2,                           \
    0x45,                           \
    0xf0,                           \
    0x69,                           \
    0xf6,                           \
    0x0b                            \
}

// Device UUID                  "0bf669f0-45f2-11e7-9598-0800200c9a66"
#define BIKE_UUID_ADV              0x69f0
// Service UUID                 "0bf669f1-45f2-11e7-9598-0800200c9a66"
#define BIKE_UUID_SERVICE          0x69f1
// Write Characteristic UUID    "0bf669f2-45f2-11e7-9598-0800200c9a66"
#define BIKE_UUID_CHAR_WRITE       0x69f2
// Read Characteristic UUID     "0bf669f3-45f2-11e7-9598-0800200c9a66"
#define BIKE_UUID_CHAR_READ        0x69f3
// Notify Characteristic UUID   "0bf669f4-45f2-11e7-9598-0800200c9a66"
#define BIKE_UUID_CHAR_NOTIFY      0x69f4

#define BIKE_ACTION_GETDEVICEINFO           0xA1
#define BIKE_ACTION_GETERRORLOG             0xA2
#define BIKE_ACTION_GETRESISTANCERANGE      0xA3
#define BIKE_ACTION_GETCONTROLSTATE         0xA4
#define BIKE_ACTION_GETRESISTANCELEVEL      0xA5
#define BIKE_ACTION_SETCONTROLSTATE         0xB0
#define BIKE_ACTION_SETRESISTANCELEVEL      0xB1

#define BIKE_RESPONSE_DEVICEINFO            0xA1
#define BIKE_RESPONSE_ERRORLOG              0xA2
#define BIKE_RESPONSE_RESISTANCERANGE       0xA3
#define BIKE_RESPONSE_CONTROLSTATE          0xA4
#define BIKE_RESPONSE_RESISTANCELEVEL       0xA5

#define BIKE_NOTIFY_CONTROLSTATE            0xD0
#define BIKE_NOTIFY_WORKOUTSTATUS           0xD1
#define BIKE_NOTIFY_RESISTANCELEVEL         0xD2

#define BIKE_STARTBYTE                      0xF0
#define BIKE_MINIMUMCOMMANDLENGTH   4

#define BIKE_CONTROLSTATE_STOP      0
#define BIKE_CONTROLSTATE_START     1
#define BIKE_CONTROLSTATE_PAUSE     2

#define BIKE_INDEX_PREAMBLE         0
#define BIKE_INDEX_ACTION           1
#define BIKE_INDEX_RESPONSE         1
#define BIKE_INDEX_FRAMELENGTH      2
#define BIKE_INDEX_FRAMESTART       3

#define BIKE_INDEX_DEVICEINFO_MODELID           3
#define BIKE_INDEX_DEVICEINFO_HARDWARE_MAJOR    4
#define BIKE_INDEX_DEVICEINFO_HARDWARE_MINOR    5
#define BIKE_INDEX_DEVICEINFO_FIRMWARE_MAJOR    6
#define BIKE_INDEX_DEVICEINFO_FIRMWARE_MINOR    7
#define BIKE_INDEX_DEVICEINFO_FIRMWARE_PATCH    8

#define BIKE_INDEX_RESISTANCERANGE_MIN      4
#define BIKE_INDEX_RESISTANCERANGE_MAX      3

#define BIKE_INDEX_CONTROLSTATE             3
#define BIKE_INDEX_RESISTANCELEVEL          3

#define BIKE_INDEX_WORKOUTSTATUS_TIMESTAMP_BEGIN     3
#define BIKE_INDEX_WORKOUTSTATUS_TIMESTAMP_END       4
#define BIKE_INDEX_WORKOUTSTATUS_REVOLUTIONS_BEGIN   5
#define BIKE_INDEX_WORKOUTSTATUS_REVOLUTIONS_END     8
#define BIKE_INDEX_WORKOUTSTATUS_RPM_BEGIN           9
#define BIKE_INDEX_WORKOUTSTATUS_RPM_END            10
#define BIKE_INDEX_WORKOUTSTATUS_HEARTRATE          11
/* clang-format on */

/**
 * @brief   Macro for defining a ble_bike service instance
 * 
 * @param   _name   Name of the instance
 * @hideinitializer
 */
#define BLE_BIKE_DEF(_name)                         \
	static ble_bike_t _name;                        \
	NRF_SDH_BLE_OBSERVER(_name##_obs,               \
						 BLE_HRS_BLE_OBSERVER_PRIO, \
						 ble_bike_on_ble_evt, &_name)

/**
 * @brief Bike Service Event Types
 */
typedef enum
{
	BLE_BIKE_EVT_READ_NOTIFICATION_ENABLED,
	BLE_BIKE_EVT_READ_NOTIFICATION_DISABLED,
	BLE_BIKE_EVT_NOTIFY_NOTIFICATION_ENABLED,
	BLE_BIKE_EVT_NOTIFY_NOTIFICATION_DISABLED,
	BLE_BIKE_EVT_CONNECTED,
	BLE_BIKE_EVT_DISCONNECTED
} ble_bike_evt_type_t;

/**
 * @brief Bike Service Event
 */
typedef struct
{
	ble_bike_evt_type_t evt_type; //Type of event
} ble_bike_evt_t;

//Forward declare ble_bike_t
typedef struct ble_bike_t ble_bike_t;

/**
 * @brief Bike Service Event Handler
 */
typedef void (*ble_bike_evt_handler_t)(ble_bike_t *p_bike, ble_bike_evt_t *p_evt);

typedef struct
{
	uint8_t resistance_level; /** Resistance Level */
	uint8_t workout_control;  /** Workout Control State */
	uint16_t timestamp;
	uint32_t revolutions;
	uint16_t rpm;
	uint8_t heart_rate;
} ble_bike_state_t;

/**
 * @brief Bike Service - Holds state of the bike service
 */
typedef struct ble_bike_t // Identifier here again due to forward declaration
{
	ble_bike_evt_handler_t evt_handler;			  /** Event handler to be called for handling events in the Custom Service */
	uint16_t service_handle;					  /** Handle of Custom Service (as provided by the BLE stack) */
	ble_gatts_char_handles_t char_handles_read;   /** Handles related to the Read characteristic */
	ble_gatts_char_handles_t char_handles_write;  /** Handles related to the Write characteristic */
	ble_gatts_char_handles_t char_handles_notify; /** Handles related to the Notify characteristic */
	bool flag_notify_read;						  /** Flag to enable notifications for Read Characteristic */
	bool flag_notify_notify;					  /** Flag to enable notifications for Notify Characteristic */
	uint16_t conn_handle;						  /** Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection) */
	uint8_t uuid_type;							  /** UUID type */
	ble_bike_state_t bike_state;				  /** Bike State */
} ble_bike_t;

/**
 * @brief Bike Service init struct
 */
typedef struct
{
	ble_bike_evt_handler_t evt_handler;				 /** Event handler to be called for handling events in the Bike Service */
	ble_srv_cccd_security_mode_t cccd_security_mode; /** Initial security level for Custom characteristics attribute */
} ble_bike_init_t;

/**@brief Function for initializing the Bike Service
 *
 * @param[out]  p_bike          Bike Service struct. This struct will have to be supplied by
 *                              the application. It will be initialized by this function, and will later
 *                              be used to identify this particular service instance.
 * @param[in]   p_bike_init     Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_bike_init(ble_bike_t *p_bike, const ble_bike_init_t *p_bike_init);

/**@brief Function for handling the application's BLE Stack events
 *
 * @details Handles all events from the BLE stack of interest to the Bike Service
 *
 * @note 
 *
 * @param[in]   p_ble_evt  Event received from the BLE stack
 * @param[in]   p_context  Context - Should be a Bike Service struct
 */
void ble_bike_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

/**@brief Function for handling notification timer timeout
 * 
 * @details Handles notification timer timout by updating state if necessary, and sends a notification regardless of whether or not state has changed
 * 
 * @param[in]   p_context  Context - Should be a Bike Service struct
 *
 * @return      NRF_SUCCESS on successful send of notification, otherwise an error code
 */
uint32_t ble_bike_notification_timeout_handler(void *p_context);