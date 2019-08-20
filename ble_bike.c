#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_bike.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "SEGGER_RTT.h"

/**@brief Adds the Read / Write / Notify Characteristics
 *
 * @param[in]   p_bike        Bike Service structure
 * @param[in]   p_bike_init   Information needed to initialize the service
 *
 * @return      NRF_SUCCESS on success, otherwise an error code
 */
static uint32_t add_characteristics(ble_bike_t * p_bike, const ble_bike_init_t * p_bike_init)
{
    //Error if != NRF_SUCCESS
    uint32_t            err_code;

    // NOTE: Proper way to do this would be to have one characteristic
    // with read / write / notify set
    // This uses one charactersitic for each (3 total)
    // not ideal, but this emulates existing hardware

    //GATT Read Characteristic Metadata
    ble_gatts_char_md_t char_read_md;
    //GATT Write Characteristic Metadata
    ble_gatts_char_md_t char_write_md;
    //GATT Notify Characteristic Metadata
    ble_gatts_char_md_t char_notify_md;

    //GATT Read Attribute
    ble_gatts_attr_t    attr_read;
    //GATT Write Attribute
    ble_gatts_attr_t    attr_write;
    //GATT Notify Attribute
    ble_gatts_attr_t    attr_notify;

    //GATT Read Attribute Metadata
    ble_gatts_attr_md_t attr_read_md;
    //GATT Write Attribute Metadata
    ble_gatts_attr_md_t attr_write_md;
    //GATT Notify Attribute Metadata
    ble_gatts_attr_md_t attr_notify_md;

    //Read UUID
    ble_uuid_t          ble_uuid_read;
    //Write UUID
    ble_uuid_t          ble_uuid_write;
    //Notify UUID
    ble_uuid_t          ble_uuid_notify;

    //GATT Atrribute: Client Characteristic Configuration Descriptor (CCCD)
    ble_gatts_attr_md_t cccd_md;

    //Note: Writing 0x0001 to CCCD is how the client enables notifications

    //Initalize CCCD
    memset(&cccd_md, 0, sizeof(cccd_md));
    //  Read  operation on CCCD should be possible without authentication
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    //CCCD is Location on Stack
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    //Initialize Read / Write / Notify Characteristic Metadata
    memset(&char_read_md, 0, sizeof(char_read_md));
    memset(&char_write_md, 0, sizeof(char_write_md));
    memset(&char_notify_md, 0, sizeof(char_notify_md));

    //Set Read Characteristic Metadata
    char_read_md.char_props.read   = 0;
    char_read_md.char_props.write  = 0;
    char_read_md.char_props.notify = 1;
    char_read_md.p_char_user_desc  = NULL;
    char_read_md.p_char_pf         = NULL;
    char_read_md.p_user_desc_md    = NULL;
    char_read_md.p_cccd_md         = &cccd_md;
    char_read_md.p_sccd_md         = NULL;

    //Set Write Characteristic Metadata
    char_write_md.char_props.read   = 0;
    char_write_md.char_props.write  = 1;
    char_write_md.char_props.write_wo_resp = 1; // Write without response: Matches bike behavior
    char_write_md.char_props.notify = 0;
    char_write_md.p_char_user_desc  = NULL;
    char_write_md.p_char_pf         = NULL;
    char_write_md.p_user_desc_md    = NULL;
    char_write_md.p_cccd_md         = NULL;
    char_write_md.p_sccd_md         = NULL;

    //Set Notify Characteristic Metadata
    char_notify_md.char_props.read   = 0;
    char_notify_md.char_props.write  = 0;
    char_notify_md.char_props.notify = 1;
    char_notify_md.p_char_user_desc  = NULL;
    char_notify_md.p_char_pf         = NULL;
    char_notify_md.p_user_desc_md    = NULL;
    char_notify_md.p_cccd_md         = &cccd_md;
    char_notify_md.p_sccd_md         = NULL;

    //Initialize Read / Write / Notify Attribute Metadata
    memset(&attr_read_md, 0, sizeof(attr_read_md));
    memset(&attr_write_md, 0, sizeof(attr_write_md));
    memset(&attr_notify_md, 0, sizeof(attr_notify_md));

    //Set CCCD Attribute Metadata
    //NOTE: these permissions should match those set in char_read_md, but are set during service_init
    attr_read_md.read_perm  = p_bike_init->cccd_security_mode.read_perm;
    attr_read_md.write_perm = p_bike_init->cccd_security_mode.write_perm;
    attr_read_md.vloc       = BLE_GATTS_VLOC_STACK; //Attribute value location is on stack

    //Disable Read / Write authorization requirement for Read Characteristic
    attr_read_md.rd_auth    = 0;
    attr_read_md.wr_auth    = 0;
    attr_read_md.vlen       = 1;

    //Set CCCD Attribute Metadata
    //NOTE: these permissions should match those set in char_read_md, but are set during service_init
    attr_write_md.read_perm  = p_bike_init->cccd_security_mode.read_perm;
    attr_write_md.write_perm = p_bike_init->cccd_security_mode.write_perm;
    attr_write_md.vloc       = BLE_GATTS_VLOC_STACK; //Attribute value location is on stack

    //Disable Read / Write authorization requirement for Write Characteristic
    attr_write_md.rd_auth    = 0;
    attr_write_md.wr_auth    = 0;
    attr_write_md.vlen       = 1;

    //Set CCCD Attribute Metadata
    //NOTE: these permissions should match those set in char_read_md, but are set during service_init
    attr_notify_md.read_perm  = p_bike_init->cccd_security_mode.read_perm;
    attr_notify_md.write_perm = p_bike_init->cccd_security_mode.write_perm;
    attr_notify_md.vloc       = BLE_GATTS_VLOC_STACK; //Attribute value location is on stack

    //Disable Read / Write authorization requirement for Notify Characteristic
    attr_notify_md.rd_auth    = 0;
    attr_notify_md.wr_auth    = 0;
    attr_notify_md.vlen       = 1;

    //Initialize Read / Write / Notify UUIDs
    memset(&ble_uuid_read, 0, sizeof(ble_uuid_read));
    memset(&ble_uuid_write, 0, sizeof(ble_uuid_write));
    memset(&ble_uuid_notify, 0, sizeof(ble_uuid_notify));
    
    //Read UUID / Type
    ble_uuid_read.type = p_bike->uuid_type;
    ble_uuid_read.uuid = BIKE_UUID_CHAR_READ;

    //Write UUID / Type
    ble_uuid_write.type = p_bike->uuid_type;
    ble_uuid_write.uuid = BIKE_UUID_CHAR_WRITE;

    //Notify UUID / Type
    ble_uuid_notify.type = p_bike->uuid_type;
    ble_uuid_notify.uuid = BIKE_UUID_CHAR_NOTIFY;

    //Initalize Read / Write / Notify Attribute Values
    memset(&attr_read, 0, sizeof(attr_read));
    memset(&attr_write, 0, sizeof(attr_write));
    memset(&attr_notify, 0, sizeof(attr_notify));

    //Set Read Attribute Value UUID / Type
    attr_read.p_uuid    = &ble_uuid_read;
    //Set Read Attribute Metadata
    attr_read.p_attr_md = &attr_read_md;
    attr_read.init_len  = sizeof(uint8_t);
    attr_read.init_offs = 0;
    attr_read.max_len   = sizeof(uint8_t)*20;

    // NOTE: max_len to 20 bytes for 20 byte MTU

    //Set Write Attribute Value UUID / Type
    attr_write.p_uuid    = &ble_uuid_write;
    //Set Write Attribute Metadata
    attr_write.p_attr_md = &attr_write_md;
    attr_write.init_len  = sizeof(uint8_t);
    attr_write.init_offs = 0;
    attr_write.max_len   = sizeof(uint8_t)*20;

    //Set Notify Attribute Value UUID / Type
    attr_notify.p_uuid    = &ble_uuid_notify;
    //Set Notify Attribute Metadata
    attr_notify.p_attr_md = &attr_notify_md;
    attr_notify.init_len  = sizeof(uint8_t);
    attr_notify.init_offs = 0;
    attr_notify.max_len   = sizeof(uint8_t)*20;

    //NOTE: Add characteristic for Write before Read here just for clean ordering in NRF Connect

    //Add GATT Write characteristic to BLE Stack
    err_code = sd_ble_gatts_characteristic_add(p_bike->service_handle, &char_write_md,
                                               &attr_write,
                                               &p_bike->char_handles_write);
    VERIFY_SUCCESS(err_code);
    //Add GATT Read characteristic to BLE Stack
    err_code = sd_ble_gatts_characteristic_add(p_bike->service_handle, &char_read_md,
                                               &attr_read,
                                               &p_bike->char_handles_read);

    VERIFY_SUCCESS(err_code);
    //Add GATT Notify characteristic to BLE Stack
    err_code = sd_ble_gatts_characteristic_add(p_bike->service_handle, &char_notify_md,
                                               &attr_notify,
                                               &p_bike->char_handles_notify);
    return err_code;
}


uint32_t ble_bike_init(ble_bike_t * p_bike, const ble_bike_init_t * p_bike_init)
{
    //Check that arguments are valid
    if (p_bike == NULL || p_bike_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code; //error code
    ble_uuid_t ble_uuid; //BLE UUID

    // Initialize service structure
    //Set event handler
    p_bike->evt_handler               = p_bike_init->evt_handler;
    //Set connection handle to INVALID since we don't have a connection yet
    p_bike->conn_handle               = BLE_CONN_HANDLE_INVALID;

    p_bike->flag_notify_read = false;
    p_bike->flag_notify_notify = false;

    // Set Bike Initial State
    p_bike->bike_state.resistance_level = 0;
    p_bike->bike_state.workout_control = BIKE_CONTROLSTATE_STOP;
    p_bike->bike_state.timestamp = 0;
    p_bike->bike_state.revolutions = 0;
    p_bike->bike_state.rpm = 0;
    p_bike->bike_state.heart_rate = 0;

    // Add Bike Service UUID to BLE stack's table
    ble_uuid128_t base_uuid = {BIKE_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_bike->uuid_type);

    VERIFY_SUCCESS(err_code);

    //Get UUID / Type for GATT Service add
    ble_uuid.type = p_bike->uuid_type;
    ble_uuid.uuid = BIKE_UUID_SERVICE;

    // Add the Bike Service declaration to BLE stack's GATT table
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_bike->service_handle);
    VERIFY_SUCCESS(err_code);

     // Add Read / Write / Notify characteristics
    err_code = add_characteristics(p_bike, p_bike_init);
    return err_code;
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_bike      Bike Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_bike_t * p_bike, ble_evt_t const * p_ble_evt)
{
    NRF_LOG_INFO("Client Connected\n");
    //Now that connection is valid, set connection handle
    p_bike->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    //Fire Connected Event
    ble_bike_evt_t evt;
    evt.evt_type = BLE_BIKE_EVT_CONNECTED;
    p_bike->evt_handler(p_bike, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_bike      Bike Service structure
 * @param[in]   p_ble_evt   Event received from the BLE stack
 */
static void on_disconnect(ble_bike_t * p_bike, ble_evt_t const * p_ble_evt)
{
    NRF_LOG_INFO("Client Disconnected\n");
    //Not needed so macro to prevent compiler warning
    UNUSED_PARAMETER(p_ble_evt);
    //Connection is now invalid, invalidate connection handle
    p_bike->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for updating the read characteristic
 *
 * @details The application calls this function when the read value should be updated. If
 *          notification has been enabled, the read value characteristic is sent to the client
 *       
 * @param[in]   p_bike          Bike Service structure
 * @param[in]   p_value         Value to send through read characteristic - Byte Array
 * @param[in]   len             Length of p_value array
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_bike_read_update(ble_bike_t * p_bike, uint8_t * p_value, uint8_t len)
{
    NRF_LOG_INFO("Updating Read Characteristic\n"); 
    if (p_bike == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t) * len;
    gatts_value.offset  = 0; //offset is only used if larger than 20 bytes
    gatts_value.p_value = p_value;

    // Update GATT value.
    err_code = sd_ble_gatts_value_set(p_bike->conn_handle,
                                      p_bike->char_handles_read.value_handle,
                                      &gatts_value);
    VERIFY_SUCCESS(err_code);

    ble_gatts_hvx_params_t hvx_params;
    //Init Handle Value
    memset(&hvx_params, 0, sizeof(hvx_params));

    //Handle Value Parameters->Notify
    hvx_params.handle = p_bike->char_handles_read.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = gatts_value.offset;
    hvx_params.p_len  = &gatts_value.len;
    hvx_params.p_data = gatts_value.p_value;

    err_code = sd_ble_gatts_hvx(p_bike->conn_handle, &hvx_params);

    return err_code;
}

/**@brief Function for calculating the checksum
 *
 * @details The checksum algorithm is to sum all of the bytes
 *          This is then included as the last byte
 *       
 * @param[in]   p_value         The byte data array, with space for the checksum
 * @param[in]   len             Length of p_value
 *
 */
void calculateChecksum(uint8_t * p_value, uint16_t len) {
    uint8_t checksum = 0;
    // Add all of the bytes except the last one to get the checksum
    for (int i = 0; i < len - 1; ++i) {
        checksum += p_value[i];
    }
    // Set the last byte as the checksum
    p_value[len - 1] = checksum;
}

/**@brief Called to send a response using the read characteristc
 *
 * @details Generates an appropriate response using the response code
 *          adds checksum, and then sends using the read characteristic  
 *       
 * @param[in]   p_bike          Bike Service Structure
 * @param[in]   responseCode    Response code as defined in ble_bike.h
 *
 */
void sendResponseMessage(ble_bike_t * p_bike, uint8_t responseCode) {
    switch(responseCode) {
        case BIKE_RESPONSE_DEVICEINFO:
            NRF_LOG_INFO("Response: Device Info\n");
            uint8_t deviceInfo[10];
            deviceInfo[BIKE_INDEX_PREAMBLE] = BIKE_STARTBYTE;
            deviceInfo[BIKE_INDEX_RESPONSE] = BIKE_RESPONSE_DEVICEINFO;
            deviceInfo[BIKE_INDEX_FRAMELENGTH] = 6;

            deviceInfo[BIKE_INDEX_DEVICEINFO_MODELID] = 11;
            deviceInfo[BIKE_INDEX_DEVICEINFO_HARDWARE_MAJOR] = 22;
            deviceInfo[BIKE_INDEX_DEVICEINFO_HARDWARE_MINOR] = 33;

            deviceInfo[BIKE_INDEX_DEVICEINFO_FIRMWARE_MAJOR] = 44;
            deviceInfo[BIKE_INDEX_DEVICEINFO_FIRMWARE_MINOR] = 55;
            deviceInfo[BIKE_INDEX_DEVICEINFO_FIRMWARE_PATCH] = 66;
            calculateChecksum(deviceInfo, 10);
            ble_bike_read_update(p_bike, deviceInfo, 10);
            break;

        case BIKE_RESPONSE_ERRORLOG:
            NRF_LOG_INFO("Response (TODO): Error Log\n");
            // TODO
            break;
        case BIKE_RESPONSE_RESISTANCERANGE:
            NRF_LOG_INFO("Response: Resistance Range\n");
            uint8_t resistanceRange[6];
            resistanceRange[BIKE_INDEX_PREAMBLE] = BIKE_STARTBYTE;
            resistanceRange[BIKE_INDEX_RESPONSE] = BIKE_RESPONSE_RESISTANCERANGE;
            resistanceRange[BIKE_INDEX_FRAMELENGTH] = 2;

            resistanceRange[BIKE_INDEX_RESISTANCERANGE_MIN] = 0;
            resistanceRange[BIKE_INDEX_RESISTANCERANGE_MAX] = 32;

            calculateChecksum(resistanceRange, 6);
            ble_bike_read_update(p_bike, resistanceRange, 6);
            break;
        case BIKE_RESPONSE_CONTROLSTATE:
            NRF_LOG_INFO("Response: Control State\n");
            uint8_t workoutControlState[5];
            workoutControlState[BIKE_INDEX_PREAMBLE] = BIKE_STARTBYTE;
            workoutControlState[BIKE_INDEX_RESPONSE] = BIKE_RESPONSE_CONTROLSTATE;
            workoutControlState[BIKE_INDEX_FRAMELENGTH] = 1;

            workoutControlState[BIKE_INDEX_CONTROLSTATE] = p_bike->bike_state.workout_control;

            calculateChecksum(workoutControlState, 5);
            ble_bike_read_update(p_bike, workoutControlState, 5);
            break;
        case BIKE_RESPONSE_RESISTANCELEVEL:
            NRF_LOG_INFO("Response: Resistance Level\n");
            uint8_t resistanceLevel[5];
            resistanceLevel[BIKE_INDEX_PREAMBLE] = BIKE_STARTBYTE;
            resistanceLevel[BIKE_INDEX_RESPONSE] = BIKE_RESPONSE_RESISTANCELEVEL;
            resistanceLevel[BIKE_INDEX_FRAMELENGTH] = 1;

            resistanceLevel[BIKE_INDEX_RESISTANCELEVEL] = p_bike->bike_state.resistance_level;

            calculateChecksum(resistanceLevel, 5);
            ble_bike_read_update(p_bike, resistanceLevel, 5);
            break; 
    }
}


/**@brief Function for updating the notify characteristic
 *
 * @details The application calls this function when the read value should be updated. If
 *          notification has been enabled, the notify value characteristic is sent to the client
 *       
 * @param[in]   p_bike          Bike Service structure
 * @param[in]   p_value         Value to send through notify characteristic - Byte Array
 * @param[in]   len             Length of p_value array
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_bike_notify_update(ble_bike_t * p_bike, uint8_t * p_value, uint8_t len)
{
    NRF_LOG_INFO("Updating Notifiy Characteristic\n"); 
    if (p_bike == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t) * len;
    gatts_value.offset  = 0; //offset is only used if larger than 20 bytes
    gatts_value.p_value = p_value;

    // Update GATT value.
    err_code = sd_ble_gatts_value_set(p_bike->conn_handle,
                                      p_bike->char_handles_notify.value_handle,
                                      &gatts_value);
    VERIFY_SUCCESS(err_code);

    ble_gatts_hvx_params_t hvx_params;
    //Init Handle Value
    memset(&hvx_params, 0, sizeof(hvx_params));

    //Handle Value Parameters->Notify
    hvx_params.handle = p_bike->char_handles_notify.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = gatts_value.offset;
    hvx_params.p_len  = &gatts_value.len;
    hvx_params.p_data = gatts_value.p_value;

    err_code = sd_ble_gatts_hvx(p_bike->conn_handle, &hvx_params);

    // TODO - Figure out why hvx operation returns !NRF_SUCCESS
    err_code = NRF_SUCCESS;

    return err_code;
}

void extract_int16ToBytes(uint16_t num, uint8_t *data) {
    data[0] = (num >> 8) & 0xFF;
    data[1] = (num >> 0) & 0xFF;
}

void extract_int32ToBytes(uint32_t num, uint8_t *data) {
    data[0] = (num >> 24) & 0xFF;
    data[1] = (num >> 16) & 0xFF;
    data[2] = (num >>  8) & 0xFF;
    data[3] = (num >>  0) & 0xFF;
}

uint32_t sendNotificationMessage(ble_bike_t * p_bike, uint8_t notificationCode) {
    uint32_t err_code = NRF_SUCCESS;
    switch(notificationCode) {
        case BIKE_NOTIFY_RESISTANCELEVEL:
            NRF_LOG_INFO("Notify: Resistance Level\n");
            uint8_t resistanceLevel[5];
            resistanceLevel[BIKE_INDEX_PREAMBLE] = BIKE_STARTBYTE;
            resistanceLevel[BIKE_INDEX_RESPONSE] = BIKE_NOTIFY_RESISTANCELEVEL;
            resistanceLevel[BIKE_INDEX_FRAMELENGTH] = 1;

            resistanceLevel[BIKE_INDEX_RESISTANCELEVEL] = p_bike->bike_state.resistance_level;

            calculateChecksum(resistanceLevel, 5);
            ble_bike_notify_update(p_bike, resistanceLevel, 5);
        break;
        case BIKE_NOTIFY_CONTROLSTATE:
            NRF_LOG_INFO("Notify: Workout Control State\n");
            uint8_t controlstate[5];
            controlstate[BIKE_INDEX_PREAMBLE] = BIKE_STARTBYTE;
            controlstate[BIKE_INDEX_RESPONSE] = BIKE_NOTIFY_CONTROLSTATE;
            controlstate[BIKE_INDEX_FRAMELENGTH] = 1;

            controlstate[BIKE_INDEX_CONTROLSTATE] = p_bike->bike_state.workout_control;

            calculateChecksum(controlstate, 5);
            err_code = ble_bike_notify_update(p_bike, controlstate, 5);
        break;
        case BIKE_NOTIFY_WORKOUTSTATUS:
            NRF_LOG_INFO("Notify: Workout Status");
            uint8_t status[13];
            status[BIKE_INDEX_PREAMBLE] = BIKE_STARTBYTE;
            status[BIKE_INDEX_RESPONSE] = BIKE_NOTIFY_WORKOUTSTATUS;
            status[BIKE_INDEX_FRAMELENGTH] = 9;

            uint8_t timestamp[2];
            extract_int16ToBytes(p_bike->bike_state.timestamp, timestamp);
            status[BIKE_INDEX_WORKOUTSTATUS_TIMESTAMP_BEGIN] = timestamp[0];
            status[BIKE_INDEX_WORKOUTSTATUS_TIMESTAMP_END] = timestamp[1];

            uint8_t revolutions[4];
            extract_int32ToBytes(p_bike->bike_state.revolutions, revolutions);
            status[BIKE_INDEX_WORKOUTSTATUS_REVOLUTIONS_BEGIN] = revolutions[0];
            status[BIKE_INDEX_WORKOUTSTATUS_REVOLUTIONS_BEGIN + 1] = revolutions[1];
            status[BIKE_INDEX_WORKOUTSTATUS_REVOLUTIONS_BEGIN + 2] = revolutions[2];
            status[BIKE_INDEX_WORKOUTSTATUS_REVOLUTIONS_END] = revolutions[3];

            uint8_t rpm[2];
            extract_int16ToBytes(p_bike->bike_state.rpm, rpm);
            status[BIKE_INDEX_WORKOUTSTATUS_RPM_BEGIN] = rpm[0];
            status[BIKE_INDEX_WORKOUTSTATUS_RPM_END] = rpm[1];

            status[BIKE_INDEX_WORKOUTSTATUS_HEARTRATE] = p_bike->bike_state.heart_rate;

            calculateChecksum(status, 13);
            err_code = ble_bike_notify_update(p_bike, status, 13);
        break; 
    }
    return err_code;
}

void receiveWriteMessage(ble_bike_t * p_bike, const ble_gatts_evt_write_t * p_evt_write)
{
    // Check to make sure it's long enough to get length
    if (p_evt_write->len < BIKE_MINIMUMCOMMANDLENGTH) {
        NRF_LOG_ERROR("Error: Message Received Shorter than Minimum Length\n");
        return;
    }
    // Generally speaking commands should fit into one packet
    if (p_evt_write->len != BIKE_MINIMUMCOMMANDLENGTH + p_evt_write->data[BIKE_INDEX_FRAMELENGTH]) {
        NRF_LOG_ERROR("Error: Message Received Data Length Mismatch\n");
        return; 
    }
    // TODO - Validate checksum

    switch(p_evt_write->data[BIKE_INDEX_ACTION]) {
        case BIKE_ACTION_GETDEVICEINFO:
            NRF_LOG_INFO("Action: Get Device Info\n");
            sendResponseMessage(p_bike, BIKE_RESPONSE_DEVICEINFO);
            break;
        case BIKE_ACTION_GETERRORLOG:
            NRF_LOG_INFO("Action: Get Error Log\n");
            sendResponseMessage(p_bike, BIKE_RESPONSE_ERRORLOG);
            break;
        case BIKE_ACTION_GETRESISTANCERANGE:
            NRF_LOG_INFO("Action: Get Resistance Range\n");
            sendResponseMessage(p_bike, BIKE_RESPONSE_RESISTANCERANGE);
            break;
        case BIKE_ACTION_GETCONTROLSTATE:
            NRF_LOG_INFO("Action: Get Workout Control State\n");
            sendResponseMessage(p_bike, BIKE_RESPONSE_CONTROLSTATE);
            break;
        case BIKE_ACTION_GETRESISTANCELEVEL:
            NRF_LOG_INFO("Action: Get Resistance Level\n");
            sendResponseMessage(p_bike, BIKE_RESPONSE_RESISTANCELEVEL);
            break;
        case BIKE_ACTION_SETCONTROLSTATE:
            NRF_LOG_INFO("Action: Set Workout Control State\n");
            p_bike->bike_state.workout_control = p_evt_write->data[BIKE_INDEX_CONTROLSTATE];
            sendNotificationMessage(p_bike, BIKE_NOTIFY_CONTROLSTATE);
            break;
        case BIKE_ACTION_SETRESISTANCELEVEL:
            NRF_LOG_INFO("Action: Set Resistance Level\n");
            p_bike->bike_state.resistance_level = p_evt_write->data[BIKE_INDEX_RESISTANCELEVEL];
            sendNotificationMessage(p_bike, BIKE_NOTIFY_RESISTANCELEVEL);
            break;
    }
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bike       Bike Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_bike_t * p_bike, ble_evt_t const * p_ble_evt)
{
    NRF_LOG_INFO("Write Event Received");
    //Get Write Event Parameters
    const ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    // Toggle LED 2 on any write
    nrf_gpio_pin_toggle(LED_2);

    // Check if the handle passed with the event matches the Write Characteristic handle.
    if (p_evt_write->handle == p_bike->char_handles_write.value_handle)
    {
        // Debug: Print Bytes
        for (int i=0; i<p_evt_write->len; ++i) {
            NRF_LOG_DEBUG("Byte #%d: %d ", i,p_evt_write->data[i]);
        }
        NRF_LOG_DEBUG("\n");
        // Toggle LED 4 only on write for write characteristic
        nrf_gpio_pin_toggle(LED_4);
        // Process the data received
        receiveWriteMessage(p_bike, p_evt_write);
    }


    // Check if the read CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_bike->char_handles_read.cccd_handle)
        && (p_evt_write->len == 2))
    {
        // CCCD for read written, call application event handler
        if (p_bike->evt_handler != NULL)
        {
            ble_bike_evt_t evt;

            //Note: Technically the softdevice enables notifications for us 
            //However we should still pass event so we know when to start notifying
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_BIKE_EVT_READ_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_BIKE_EVT_READ_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_bike->evt_handler(p_bike, &evt);
        }
    }
    else if ((p_evt_write->handle == p_bike->char_handles_notify.cccd_handle)
        && (p_evt_write->len == 2))
    {
        // CCCD for notify written, call application event handler
        if (p_bike->evt_handler != NULL)
        {
            ble_bike_evt_t evt;

            //Note: Technically the softdevice enables notifications for us 
            //However we should still pass event so we know when to start notifying
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_BIKE_EVT_NOTIFY_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_BIKE_EVT_NOTIFY_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_bike->evt_handler(p_bike, &evt);
        }
    }

    //TODO: Handle case for incorrect length
}

uint32_t ble_bike_notification_timeout_handler(void * p_context) {
     // Typecase context to bike
    ble_bike_t * p_bike = (ble_bike_t *) p_context;
    if (p_bike == NULL) {
        return NRF_ERROR_NULL;
    }
    // Only update values if Control State == Start
    if (p_bike->bike_state.workout_control == BIKE_CONTROLSTATE_START) {
        // for RPM == 60, 1 revolution per second
        p_bike->bike_state.rpm = 60;
        ++p_bike->bike_state.revolutions;
        ++p_bike->bike_state.timestamp;
    }
    else if (p_bike->bike_state.workout_control == BIKE_CONTROLSTATE_STOP){
        p_bike->bike_state.revolutions = 0;
        p_bike->bike_state.rpm = 0;
        p_bike->bike_state.timestamp = 0;
    }

    // Notify Status regardless of control state
    return sendNotificationMessage(p_bike, BIKE_NOTIFY_WORKOUTSTATUS);
}

void ble_bike_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    NRF_LOG_INFO("Bike Event Received");
    //Typecast context pointer so we can use it
    ble_bike_t * p_bike = (ble_bike_t *) p_context;
    
    if (p_bike == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    //Handle Event
    switch (p_ble_evt->header.evt_id)
    {
        //Connect Event
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_bike, p_ble_evt);
            break;
        //Disconnect Event
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_bike, p_ble_evt);
            break;
        //GATT Characteristic Write Event
        case BLE_GATTS_EVT_WRITE:
            on_write(p_bike, p_ble_evt);
        default:
            // Events we don't care about now but might in the future
            NRF_LOG_WARNING("Unknown Bike Event ID Received: %d\n", p_ble_evt->header.evt_id);
            break;
    }
}