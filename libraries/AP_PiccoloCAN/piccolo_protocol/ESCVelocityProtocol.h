// ESCVelocityProtocol.h was generated by ProtoGen version 3.5.c

/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#ifndef _ESCVELOCITYPROTOCOL_H
#define _ESCVELOCITYPROTOCOL_H

// Language target is C, C++ compilers: don't mangle us
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \file
 * \mainpage ESCVelocity protocol stack
 * 
 * This is the ICD for the Gen-2 Currawong Engineering Electronic Speed
 * Controller (ESCVelocity). This document details the ESCVelocity command and
 * packet structure for communication with and configuration of the ESC. Note
 * that there may be some differences between this ICD and the ICD for the
 * Gen-1 ESCVelocity. Please refer to the old ICD for the Gen-1 device.
 * 
 * The protocol API enumeration is incremented anytime the protocol is changed
 * in a way that affects compatibility with earlier versions of the protocol.
 * The protocol enumeration for this version is: 88
 * 
 * The protocol version is 3.70
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>	// C string manipulation function header

//! \return the protocol API enumeration
#define getESCVelocityApi() 88

//! \return the protocol version string
#define getESCVelocityVersion() "3.70"

/*!
 * Constant values required for sending a disable (inhibit) command
 */
typedef enum
{
    ESC_DISABLE_A = 0xAA, //!< Constant value required for disabling the ESC
    ESC_DISABLE_B = 0xC3  //!< Constant value required for disabling the ESC
} ESCDisableSequence;

/*!
 * Constant values required for sending an enable command
 */
typedef enum
{
    ESC_ENABLE_A = 0xAA, //!< Constant value required for enabling the ESC
    ESC_ENABLE_B = 0x3C  //!< Constant value required for enabling the ESC
} ESCEnableSequence;

/*!
 * ESC Operational Modes
 */
typedef enum
{
    ESC_MODE_STANDBY = 0x00, //!< ESC is in standby mode - the motor is OFF but the ESC is ready to accept motor commands
    ESC_MODE_PWM,            //!< ESC is controlling motor in open-loop mode based on a 'PWM' (Pulse Width) input
    ESC_MODE_RPM,            //!< ESC is controlling motor speed based on an RPM setpoint
    ESC_MODE_VOLT,           //!< ESC is controlling DC voltage based on a provided setpoint
    ESC_VALID_MODES          //!< ESC mode counter
} ESCOperatingModes;

//! \return the label of a 'ESCOperatingModes' enum entry, based on its value
const char* ESCOperatingModes_EnumLabel(int value);

/*!
 * ESC Command Sources
 */
typedef enum
{
    ESC_COMMAND_SOURCE_NONE = 0x00, //!< No valid command has been received
    ESC_COMMAND_SOURCE_CAN,         //!< Most recent command from CAN
    ESC_COMMAND_SOURCE_PWM          //!< Most recent command from PWM
} ESCCommandSources;

//! \return the label of a 'ESCCommandSources' enum entry, based on its value
const char* ESCCommandSources_EnumLabel(int value);

/*!
 * ESC motor temperature sensor options
 */
typedef enum
{
    ESC_MOTOR_TEMP_SENSOR_OFF = 0x00, //!< No temperature sensor selected
    ESC_MOTOR_TEMP_SENSOR_KTY84,      //!< KTY84 or equivalent
    ESC_MOTOR_TEMP_SENSOR_KTY83,      //!< KTY83 or equivalent
    ESC_MOTOR_TEMP_SENSOR_NTC_SH,     //!< NTC sensor (Steinhart–Hart equation)
    ESC_MOTOR_TEMP_SENSOR_NTC_BETA    //!< NTC sensor (Beta equation)
} ESCMotorTemperatureSensor;

//! \return the label of a 'ESCMotorTemperatureSensor' enum entry, based on its value
const char* ESCMotorTemperatureSensor_EnumLabel(int value);

/*!
 * Motor direction
 */
typedef enum
{
    ESC_MOTOR_DIR_ABC = 0,   //!< Motor phase sequence A / B / C
    ESC_MOTOR_DIR_ACB = 1,   //!< Motor phase sequence A / C / B
    ESC_MOTOR_DIR_OTHER = 3  //!< Unknown / unsupported direction
} ESCMotorDirection;

//! \return the label of a 'ESCMotorDirection' enum entry, based on its value
const char* ESCMotorDirection_EnumLabel(int value);

/*!
 * Hall sensor modes
 */
typedef enum
{
    ESC_HALL_MODE_SENSORLESS = 0, //!< Sensorless control only
    ESC_HALL_MODE_SENSORED = 1,   //!< Sensored control only
    ESC_HALL_MODE_HYBRID = 2      //!< Sensored starting, running below RPM threshold and where sensorless control is unreliable.
} ESCHallSensorMode;

//! \return the label of a 'ESCHallSensorMode' enum entry, based on its value
const char* ESCHallSensorMode_EnumLabel(int value);

/*!
 * AFW operation modes
 */
typedef enum
{
    ESC_AFW_MODE_OFF = 0,     //!< AFW always off (during this motor running state)
    ESC_AFW_MODE_ON = 1,      //!< AFW always on (during this motor running state)
    ESC_AFW_MODE_DYNAMIC = 2, //!< AFW may change state
    ESC_AFW_MODE_OTHER = 3    //!< Future expansion
} ESCAFWModes;

//! \return the label of a 'ESCAFWModes' enum entry, based on its value
const char* ESCAFWModes_EnumLabel(int value);

/*!
 * PWM operation modes
 */
typedef enum
{
    ESC_PWM_FREQ_FIXED = 0, //!< PWM frequency is the specified value
    ESC_PWM_FREQ_RAMP = 1,  
    ESC_PWM_FREQ_OTHER = 3  //!< Future expansion
} ESCPWMFreqModes;

//! \return the label of a 'ESCPWMFreqModes' enum entry, based on its value
const char* ESCPWMFreqModes_EnumLabel(int value);

/*!
 * PWM operation modes
 */
typedef enum
{
    ESC_TIMING_ADVANCE_MODE_FIXED = 0, //!< Timing advance is the specified value
    ESC_TIMING_ADVANCE_MODE_RAMP = 1,  
    ESC_TIMING_ADVANCE_MODE_OTHER = 3  //!< Future expansion
} ESCTimingAdvanceModes;

//! \return the label of a 'ESCTimingAdvanceModes' enum entry, based on its value
const char* ESCTimingAdvanceModes_EnumLabel(int value);

/*!
 * ESC protection actions
 */
typedef enum
{
    ESC_PROTECTION_WARNING = 0,  //!< Warning bit is set
    ESC_PROTECTION_FOLDBACK = 1, //!< Motor duty cycle is limited
    ESC_PROTECTION_DISABLE = 2,  //!< ESC is disabled
    ESC_PROTECTION_INVALID = 7   //!< Invalid protection action
} ESCProtectionActions;

//! \return the label of a 'ESCProtectionActions' enum entry, based on its value
const char* ESCProtectionActions_EnumLabel(int value);

/*!
 * Motor beep modes enumeration
 */
typedef enum
{
    ESC_BEEP_NONE = 0b00,   
    ESC_BEEP_STATUS = 0b01, //!< Motor status beeps only
    ESC_BEEP_ERROR = 0b10,  //!< Motor error beeps only
    ESC_BEEP_ALL = 0b11     //!< All motor beeps
} ESCBeepModes;

//! \return the label of a 'ESCBeepModes' enum entry, based on its value
const char* ESCBeepModes_EnumLabel(int value);

/*!
 * ESC standby cause flags
 */
typedef enum
{
    ESC_STANDBY_CAUSE_CMD = 0x0001,               //!< ESC was put into *STANDBY* mode by a command
    ESC_STANDBY_CAUSE_INHIBIT = 0x0002,           //!< ESC was put into *STANDBY* mode by SW or HW inhibit
    ESC_STANDBY_CAUSE_TIMEOUT = 0x0004,           //!< ESC was put into *STANDBY* mode due to keepalive timeout
    ESC_STANDBY_CAUSE_HALL_SENSOR_ERROR = 0x0008, //!< ESC was put into *STANDBY* mode due to a hall sensor error
    ESC_STANDBY_CAUSE_INVALID_CMD = 0x0010,       //!< ESC was put into *STANDBY* mode due to a command being invalid
    ESC_STANDBY_CAUSE_PWM_ARM = 0x0020,           //!< PWM arming signal detected
    ESC_STANDBY_CAUSE_FAILED_START = 0x0040,      //!< ESC was put into *STANDBY* mode due to failed starting routine
    ESC_STANDBY_CAUSE_MIN_CMD = 0x0080,           //!< ESC was put into *STANDBY* mode due to the received command below minimum threshold
    ESC_STANDBY_CAUSE_FAILED_RESYNC = 0x0100,     //!< ESC was put into *STANDBY* mode due to failed resync routine
    ESC_STANDBY_CAUSE_UNEXPECTED = 0x0200,        //!< ESC was put intot *STANDBY* mode due to an unexpected internal state
    ESC_STANDBY_CAUSE_RESET = 0x8000              
} ESCStandbyCause;

//! \return the label of a 'ESCStandbyCause' enum entry, based on its value
const char* ESCStandbyCause_EnumLabel(int value);

/*!
 * ESC disable cause flags
 */
typedef enum
{
    ESC_DISABLE_CAUSE_NONE = 0x0000,              //!< Unused / blank value
    ESC_DISABLE_CAUSE_CAN_CMD = 0x0001,           //!< ESC is disabled by a CAN command
    ESC_DISABLE_CAUSE_PWM_TIMEOUT = 0x0002,       //!< PWM signal lost
    ESC_DISABLE_CAUSE_HARDWARE = 0x0004,          //!< Hardware enable signal deasserted
    ESC_DISABLE_CAUSE_OVERCURRENT = 0x0008,       //!< ESC disabled due to overcurrent
    ESC_DISABLE_CAUSE_OVERSPEED = 0x0010,         //!< ESC disabled due to overspeed
    ESC_DISABLE_CAUSE_OVERTEMP = 0x0020,          //!< ESC disabled due to overtemperature
    ESC_DISABLE_CAUSE_UNDERVOLTAGE = 0x0040,      //!< ESC disabled due to undervoltage
    ESC_DISABLE_CAUSE_FAILED_START = 0x0080,      //!< ESC disabled due to starting failure (see ESCFailedStartCause for details)
    ESC_DISABLE_CAUSE_COMMUTATION_ERROR = 0x0100, //!< ESC disabled due to commutation failure
    ESC_DISABLE_CAUSE_PHASE_VOLTAGE = 0x0200,     //!< Phase voltage issues detected
    ESC_DISABLE_CAUSE_REGEN_CURRENT = 0x0400,     //!< Regen current exceeded threshold
    ESC_DISABLE_CAUSE_INVALID_STATE = 0x2000,     //!< ESC disabled due to invalid commutation state
    ESC_DISABLE_CAUSE_RESET = 0x8000              //!< ESC is disabled by processor reset
} ESCDisableCause;

//! \return the label of a 'ESCDisableCause' enum entry, based on its value
const char* ESCDisableCause_EnumLabel(int value);

/*!
 * ESC motor OFF cause
 */
typedef enum
{
    ESC_MOTOR_OFF_STANDBY = 0x0001,      //!< Motor turned off due to system standby
    ESC_MOTOR_OFF_BEEP = 0x0002,         //!< Motor turned off due to beeping routine
    ESC_MOTOR_OFF_INITIALISE = 0x0004,   //!< Motor turned off at system initialisation
    ESC_MOTOR_OFF_INHIBITED = 0x0010,    //!< Motor turned off due to ESC being inhibited
    ESC_MOTOR_OFF_THROTTLE_MIN = 0x0020, //!< Throttle below minimum value
    ESC_MOTOR_OFF_NOT_RUNNING = 0x0040,  //!< Motor does not have valid commutation
    ESC_MOTOR_OFF_FAILED_START = 0x0080, //!< Starting routine failed
    ESC_MOTOR_OFF_INVALID = 0x8000       
} ESCMotorOffCause;

//! \return the label of a 'ESCMotorOffCause' enum entry, based on its value
const char* ESCMotorOffCause_EnumLabel(int value);

/*!
 * Failed start cause flags
 */
typedef enum
{
    ESC_FAILED_START_CAUSE_RESET = 0x0000,         //!< No failed start has been recorded
    ESC_FAILED_START_CAUSE_TIMEOUT = 0x0001,       //!< Starting procedure timed out
    ESC_FAILED_START_CAUSE_OVERSPEED = 0x0002,     //!< Commutation speed too high
    ESC_FAILED_START_CAUSE_OVERCURRENT = 0x0004,   //!< Starting current exceeded
    ESC_FAILED_START_CAUSE_SPIN_REVERSED = 0x0010, //!< Motor is already spinning, in reverse direction
    ESC_FAILED_START_CAUSE_SPIN_TOO_FAST = 0x0020, //!< Motor is already spinning, above maximum catch speed
    ESC_FAILED_START_CAUSE_INVALID = 0x8000        
} ESCFailedStartCause;

//! \return the label of a 'ESCFailedStartCause' enum entry, based on its value
const char* ESCFailedStartCause_EnumLabel(int value);

/*!
 * ESC Multi Command Packets
 */
typedef enum
{
    PKT_ESC_SETPOINT_1 = 0, //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 1 - 4
    PKT_ESC_SETPOINT_2,     //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 5 - 8
    PKT_ESC_SETPOINT_3,     //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 9 - 12
    PKT_ESC_SETPOINT_4,     //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 13 - 16
    PKT_ESC_SETPOINT_5,     //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 17 - 20
    PKT_ESC_SETPOINT_6,     //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 21 - 24
    PKT_ESC_SETPOINT_7,     //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 25 - 28
    PKT_ESC_SETPOINT_8,     //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 29 - 32
    PKT_ESC_SETPOINT_9,     //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 33 - 36
    PKT_ESC_SETPOINT_10,    //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 37 - 40
    PKT_ESC_SETPOINT_11,    //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 41 - 44
    PKT_ESC_SETPOINT_12,    //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 45 - 48
    PKT_ESC_SETPOINT_13,    //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 49 - 52
    PKT_ESC_SETPOINT_14,    //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 53 - 56
    PKT_ESC_SETPOINT_15,    //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 57 - 60
    PKT_ESC_SETPOINT_16     //!< This packet is used to send commands to multiple ESCs with sequential CAN IDs 61 - 64
} ESCMultiCommandPackets;

//! \return the label of a 'ESCMultiCommandPackets' enum entry, based on its value
const char* ESCMultiCommandPackets_EnumLabel(int value);

/*!
 * ESC Command Packets
 */
typedef enum
{
    PKT_ESC_PWM_CMD = 0x10, //!< Send a PWM (Pulse width) command to a particular ESC
    PKT_ESC_RPM_CMD,        //!< Send an RPM (Speed) command to a particular ESC
    PKT_ESC_VOLT_CMD,       //!< Send a voltage loop command to a particular ESC
    PKT_ESC_DISABLE = 0x20, //!< Send this packet to an ESC to disable the ESC
    PKT_ESC_STANDBY         //!< Send this packet to an ESC to enable the ESC and place it in Standby mode
} ESCCommandPackets;

//! \return the label of a 'ESCCommandPackets' enum entry, based on its value
const char* ESCCommandPackets_EnumLabel(int value);

/*!
 * ESC Status Packets
 */
typedef enum
{
    PKT_ESC_STATUS_A = 0x80,      //!< ESC Status A telemetry packet transmitted by the ESC at regular intervals
    PKT_ESC_STATUS_B,             //!< ESC Status B telemetry packet transmitted by the ESC at regular intervals
    PKT_ESC_STATUS_C,             //!< ESC Status C telemetry packet transmitted by the ESC at regular intervals
    PKT_ESC_STATUS_D,             //!< ECC Status D telemetry packet transmitted by the ESC at regular intervals
    PKT_ESC_ACCELEROMETER = 0x88  //!< Raw accelerometer data
} ESCStatusPackets;

//! \return the label of a 'ESCStatusPackets' enum entry, based on its value
const char* ESCStatusPackets_EnumLabel(int value);

/*!
 * ESC Packets Definitions
 */
typedef enum
{
    PKT_ESC_SYSTEM_CMD = 0x50,              //!< Send a configuration command to the ESC (followed by optional command data bytes)
    PKT_ESC_SET_TITLE = 0x51,               //!< Set the ESC descriptor title
    PKT_ESC_CONTROL_LOOP_DATA = 0x8A,       //!< Control loop output data - varies depending on the operational mode of the ESC
    PKT_ESC_HALL_SENSOR_INFO = 0x8C,        //!< Hall sensor information - detected pattern, advance, validity
    PKT_ESC_WARNINGS_ERRORS = 0x86,         //!< ESC warning / error status information.
    PKT_ESC_MOTOR_FLAGS = 0x87,             //!< Motor status flags
    PKT_ESC_EVENT = 0x8D,                   //!< Event description packet
    PKT_ESC_SERIAL_NUMBER = 0x90,           //!< ESC Serial Number and User ID information
    PKT_ESC_TITLE = 0x91,                   //!< Human-readable string descriptor (max 8 chars) of the particular ESC
    PKT_ESC_FIRMWARE = 0x92,                //!< ESC Firmware information
    PKT_ESC_SYSTEM_INFO = 0x93,             //!< ESC system information packet
    PKT_ESC_TELEMETRY_SETTINGS = 0x94,      //!< Telemetry packet configuration
    PKT_ESC_EEPROM = 0x95,                  //!< ESC non-volatile data information and settings
    PKT_ESC_EXTRA = 0x96,                   //!< Extra settings
    PKT_ESC_MOTOR_TEMP_SENSOR = 0x97,       //!< Motor temperature sensor settings
    PKT_ESC_COMMISSIONING = 0x99,           //!< ESC commissioning data (factory only)
    PKT_ESC_TELLTALES = 0x9A,               //!< ESC telltales
    PKT_ESC_GIT_HASH = 0x9B,                //!< ESC firmware git hash
    PKT_ESC_LEGACY_MOTOR_STATUS = 0xA0,     //!< ESC motor status information
    PKT_ESC_LEGACY_MOTOR_SETTINGS = 0xA5,   //!< ESC motor configuration
    PKT_ESC_LEGACY_MOTOR_SETTINGS_2 = 0xA6, //!< ESC Motor settings information
    PKT_ESC_LEGACY_MOTOR_FIRMWARE = 0xAA,   //!< ESC motor control firmware information
    PKT_ESC_MOTOR_SETTINGS = 0xA7,          //!< Motor control settings packet
    PKT_ESC_MOTOR_STARTING = 0xA8,          //!< Motor starting settings packet
    PKT_ESC_MOTOR_PARAMETERS = 0xA9,        //!< Motor and system parameters
    PKT_ESC_MOTOR_HALL_CONFIG = 0xAB,       //!< Motor hall sensor configuration
    PKT_ESC_CONFIG = 0xB0,                  //!< ESC Configuration parameters
    PKT_ESC_WARNINGS = 0xB1,                //!< Warning level values for various ESC parameters (legacy)
    PKT_ESC_PROTECTION_LEVELS = 0xB2,       //!< ESC protection values
    PKT_ESC_PROTECTION_ACTIONS = 0xB3,      //!< ESC protection actions
    PKT_ESC_VOLT_LOOP_SETTINGS = 0xB4,      //!< Voltage Control Loop Settings
    PKT_ESC_RPM_LOOP_SETTINGS = 0xB5,       //!< RPM Control Loop Settings
    PKT_ESC_STARTING_SETTINGS = 0xB6,       //!< ESC auto-starting configuration for RPM mode
    PKT_ESC_CURRENT_CALIBRATION = 0xB7,     //!< ESC current sense calibration settings
    PKT_ESC_IO_TABLE_SETTINGS = 0xC0,       //!< Configuration of the Input/Output mapping
    PKT_ESC_IO_TABLE_ELEMENT = 0xC1,        //!< A single element of the Input/Output mapping table
    PKT_ESC_THROTTLE_CURVE = 0xC2,          //!< Throttle curve calibration
    PKT_ESC_PWM_INPUT_CALIBRATION = 0xC3,   //!< PWM input calibration
    PKT_ESC_BULK_TRANSFER = 0xF0,           //!< Bulk data transfer (long packets)
    PKT_ESC_DRONECAN_SETTINGS = 0xFA        //!< DroneCAN specific settings
} ESCPackets;

//! \return the label of a 'ESCPackets' enum entry, based on its value
const char* ESCPackets_EnumLabel(int value);

/*!
 * ESC System Commands
 */
typedef enum
{
    CMD_ESC_SET_NODE_ID = 0x50,          //!< Set the CAN Node ID for the target ESC
    CMD_ESC_SET_USER_ID_A,               //!< Set user ID A value
    CMD_ESC_SET_USER_ID_B,               //!< Set user ID B value
    CMD_ESC_TARE_CURRENT = 0x60,         //!< Tare the current measurement
    CMD_ESC_IDENTIFY = 0x70,             //!< Identify the ESC with a sequence of LED flashes / beeps
    CMD_ESC_SET_MOTOR_DIRECTION = 0x90,  //!< Set the motor direction
    CMD_ESC_REQUEST_HF_DATA = 0xB0,      //!< Request high-frequency telemetry data
    CMD_ESC_CONFIGURE_IO_MAP = 0xC0,     //!< Configure the Input/Output map for the ESC
    CMD_ESC_CONFIGURE_IO_ELEMENT,        //!< Configure (or request) a particular element of the I/O map
    CMD_ESC_RESET_SETTINGS = 0xD0,       //!< Reset ESC settings to default parameters
    CMD_ESC_ENTER_DEBUG = 0xDE,          //!< Enter debug mode
    CMD_ESC_EXIT_DEBUG,                  //!< Exit debug mode
    CMD_ESC_UNLOCK_SETTINGS = 0xF5,      //!< Unlock ESC nonvolatile settings
    CMD_ESC_LOCK_SETTINGS = 0xF6,        //!< Lock ESC nonvolatile settings
    CMD_ESC_VALIDATE_SETTINGS = 0xF7,    //!< Mark the current settings as valid
    CMD_ESC_RESET_MOTOR_RUN_TIME = 0xFA, //!< Reset motor run time
    CMD_ESC_ENTER_BOOTLOADER = 0xFB,     //!< Enter bootloader mode
    CMD_ESC_RESET                        //!< Reset ESC
} ESCSystemCommands;

//! \return the label of a 'ESCSystemCommands' enum entry, based on its value
const char* ESCSystemCommands_EnumLabel(int value);

/*!
 * ESC CAN Protocols
 */
typedef enum
{
    CAN_PROTOCOL_PICCOLO = 0x00,  //!< Piccolo CAN protocol
    CAN_PROTOCOL_DRONECAN = 0x01, //!< DroneCAN CAN protocol
    CAN_PROTOCOL_NONE = 0xFF      //!< No protocol specified
} CANProtocols;

//! \return the label of a 'CANProtocols' enum entry, based on its value
const char* CANProtocols_EnumLabel(int value);

/*!
 * ESC CAN Baud rates
 */
typedef enum
{
    CAN_BAUD_RATES_INVALID = 0x00, 
    CAN_BAUD_RATES_1000K,          
    CAN_BAUD_RATES_500K,           
    CAN_BAUD_RATES_250K,           
    CAN_BAUD_RATES_125K,           
    CAN_BAUD_RATES_100K,           
    CAN_BAUD_RATES_50K             
} CANBaudRates;

//! \return the label of a 'CANBaudRates' enum entry, based on its value
const char* CANBaudRates_EnumLabel(int value);


// The prototypes below provide an interface to the packets.
// They are not auto-generated functions, but must be hand-written

//! \return the packet data pointer from the packet
uint8_t* getESCVelocityPacketData(void* pkt);

//! \return the packet data pointer from the packet, const
const uint8_t* getESCVelocityPacketDataConst(const void* pkt);

//! Complete a packet after the data have been encoded
void finishESCVelocityPacket(void* pkt, int size, uint32_t packetID);

//! \return the size of a packet from the packet header
int getESCVelocityPacketSize(const void* pkt);

//! \return the ID of a packet from the packet header
uint32_t getESCVelocityPacketID(const void* pkt);

#ifdef __cplusplus
}
#endif
#endif // _ESCVELOCITYPROTOCOL_H
