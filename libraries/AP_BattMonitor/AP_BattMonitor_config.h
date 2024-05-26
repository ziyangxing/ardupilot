#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_ESC_Telem/AP_ESC_Telem_config.h>
#include <AP_EFI/AP_EFI_config.h>
#include <AP_Generator/AP_Generator_config.h>
#include <AP_Torqeedo/AP_Torqeedo_config.h>

#ifndef AP_BATTERY_ENABLED
#define AP_BATTERY_ENABLED 1
#endif

#ifndef AP_BATTERY_BACKEND_DEFAULT_ENABLED
#define AP_BATTERY_BACKEND_DEFAULT_ENABLED AP_BATTERY_ENABLED
#endif

#ifndef AP_BATTERY_ANALOG_ENABLED
#define AP_BATTERY_ANALOG_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BATTERY_BEBOP_ENABLED
#define AP_BATTERY_BEBOP_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO)
#endif

#ifndef AP_BATTERY_EFI_ENABLED
#define AP_BATTERY_EFI_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && HAL_EFI_ENABLED
#endif

#ifndef AP_BATTERY_ESC_ENABLED
#define AP_BATTERY_ESC_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && HAL_WITH_ESC_TELEM
#endif

#ifndef AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED
#define AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED 0
#endif

#ifndef AP_BATTERY_WATT_MAX_ENABLED
#define AP_BATTERY_WATT_MAX_ENABLED AP_BATTERY_ENABLED
#endif

#ifndef AP_BATTERY_SMBUS_ENABLED
#define AP_BATTERY_SMBUS_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BATTERY_UAVCAN_BATTERYINFO_ENABLED
#define AP_BATTERY_UAVCAN_BATTERYINFO_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS
#endif

#ifndef AP_BATTERY_FUELFLOW_ENABLED
#define AP_BATTERY_FUELFLOW_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && (BOARD_FLASH_SIZE > 1024)
#endif

#ifndef AP_BATTERY_FUELLEVEL_PWM_ENABLED
#define AP_BATTERY_FUELLEVEL_PWM_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && (BOARD_FLASH_SIZE > 1024)
#endif

#ifndef AP_BATTERY_FUELLEVEL_ANALOG_ENABLED
#define AP_BATTERY_FUELLEVEL_ANALOG_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && (BOARD_FLASH_SIZE > 1024)
#endif

#ifndef AP_BATTERY_GENERATOR_ENABLED
#define AP_BATTERY_GENERATOR_ENABLED HAL_GENERATOR_ENABLED
#endif

#ifndef AP_BATTERY_INA239_ENABLED
#define AP_BATTERY_INA239_ENABLED 0  // SPI device must be specified in hwdef
#endif

#ifndef AP_BATTERY_INA2XX_ENABLED
#define AP_BATTERY_INA2XX_ENABLED (AP_BATTERY_BACKEND_DEFAULT_ENABLED && (BOARD_FLASH_SIZE > 1024))
#endif

#ifndef AP_BATTERY_LTC2946_ENABLED
#define AP_BATTERY_LTC2946_ENABLED (AP_BATTERY_BACKEND_DEFAULT_ENABLED && defined(HAL_BATTMON_LTC2946_BUS) && defined(HAL_BATTMON_LTC2946_ADDR))
#endif

#ifndef AP_BATTERY_SYNTHETIC_CURRENT_ENABLED
#define AP_BATTERY_SYNTHETIC_CURRENT_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BATTERY_SMBUS_SOLO_ENABLED
#define AP_BATTERY_SMBUS_SOLO_ENABLED 0  // turned on in hwdefs
#endif

#ifndef AP_BATTERY_SUM_ENABLED
#define AP_BATTERY_SUM_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BATTERY_TORQEEDO_ENABLED
#define AP_BATTERY_TORQEEDO_ENABLED HAL_TORQEEDO_ENABLED
#endif

#ifndef AP_BATTERY_AD7091R5_ENABLED 
#define AP_BATTERY_AD7091R5_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && (BOARD_FLASH_SIZE > 1024)
#endif

// SMBus-subclass backends:
#ifndef AP_BATTERY_SMBUS_GENERIC_ENABLED
#define AP_BATTERY_SMBUS_GENERIC_ENABLED AP_BATTERY_SMBUS_ENABLED
#endif

#ifndef AP_BATTERY_SMBUS_NEODESIGN_ENABLED
#define AP_BATTERY_SMBUS_NEODESIGN_ENABLED AP_BATTERY_SMBUS_ENABLED
#endif

#ifndef AP_BATTERY_SMBUS_SUI_ENABLED
#define AP_BATTERY_SMBUS_SUI_ENABLED AP_BATTERY_SMBUS_ENABLED
#endif

// subclasses of the SUMbus generic backend:
#ifndef AP_BATTERY_SMBUS_MAXELL_ENABLED
#define AP_BATTERY_SMBUS_MAXELL_ENABLED AP_BATTERY_SMBUS_GENERIC_ENABLED
#endif

#ifndef AP_BATTERY_SMBUS_ROTOYE_ENABLED
#define AP_BATTERY_SMBUS_ROTOYE_ENABLED AP_BATTERY_SMBUS_GENERIC_ENABLED
#endif

#ifndef AP_BATTERY_SCRIPTING_ENABLED
#define AP_BATTERY_SCRIPTING_ENABLED (AP_SCRIPTING_ENABLED && AP_BATTERY_BACKEND_DEFAULT_ENABLED)
#endif
