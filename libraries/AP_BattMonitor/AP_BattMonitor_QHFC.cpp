#include "AP_BattMonitor_config.h"

#if AP_BATTERY_QHFC_BATTERYINFO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_QHFC.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_QHFC/AP_QHFC.h>
#include <AP_BoardConfig/AP_BoardConfig.h>


#define LOG_TAG "BattMon"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_QHFC::var_info[] = {

    // @Param: CURR_MULT
    // @DisplayName: Scales reported power monitor current
    // @Description: Multiplier applied to all current related reports to allow for adjustment if no QHFC param access or current splitting applications
    // @Range: .1 10
    // @User: Advanced
    AP_GROUPINFO("KPA", 30, AP_BattMonitor_QHFC, _kpa, 1.0),

    // Param indexes must be between 30 and 39 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_QHFC::AP_BattMonitor_QHFC(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_QHFC_Type type, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _type(type)
{
    AP_Param::setup_object_defaults(this,var_info);
    _state.var_info = var_info;

    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_QHFC::init()
{
}

// return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
uint32_t AP_BattMonitor_QHFC::get_mavlink_fault_bitmask() const
{
    // // return immediately if not mppt or no faults
    // if (!_mppt.is_detected || (_mppt.fault_flags == 0)) {
    //     return 0;
    // }

    // convert mppt fault bitmask to mavlink fault bitmask
    uint32_t mav_fault_bitmask = 0;
    // gcs().send_text(MAV_SEVERITY_INFO, "get_mavlink_fault_bitmask:%ld",mav_fault_bitmask );
    // if ((_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_VOLTAGE) || (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::UNDER_VOLTAGE)) {
    //     mav_fault_bitmask |= MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE;
    // }
    // if (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_CURRENT) {
        mav_fault_bitmask |= MAV_BATTERY_FAULT_OVER_CURRENT;
    // }
    // if (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_TEMPERATURE) {
    //     mav_fault_bitmask |= MAV_BATTERY_FAULT_OVER_TEMPERATURE;
    // }
    return mav_fault_bitmask;
}

AP_BattMonitor::Failsafe AP_BattMonitor_QHFC::update_failsafes()
{
    // gcs().send_text(MAV_SEVERITY_INFO, "update_failsafes");
    // if((AP::qhfc()->GetFCFault()) || 1)
    // {
    //     gcs().send_text(MAV_SEVERITY_INFO, "qhfc Critical");
    //     return AP_BattMonitor::Failsafe::Critical;

    // }
    // else if( (AP::qhfc()->GetFCWarning() != 0))
    // {
    //     gcs().send_text(MAV_SEVERITY_INFO, "qhfc Low");
    //     return AP_BattMonitor::Failsafe::Low;
    // }
    return AP_BattMonitor::Failsafe::None;
}

// read - read the voltage and current
void AP_BattMonitor_QHFC::read()
{
    // AP_QHFC& fc = AP::qhfc();
    uint32_t tnow = AP_HAL::micros();

    // timeout after 5 seconds
    if ((tnow - _interim_state.last_time_micros) > AP_BATTMONITOR_QHFC_TIMEOUT_MICROS) {
        _interim_state.healthy = false;
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "QHFC::read");
    // Copy over relevant states over to main state
    _state.temperature = 30;

    // _state.temperature_external_use = true;
    // _state.temperature_external = 35; 
    _state.voltage = 120;//fc.GCStatus.FCVoltage;
    _state.battFC1Volt = 66;
    _state.battLIVoltage = 100;
    _state.current_amps = 6;//fc.GCStatus.FCCurrent;
    _state.consumed_mah = 2000;
    _state.consumed_wh = 50;
    _state.last_time_micros = _interim_state.last_time_micros;
    _state.healthy = true;//_interim_state.healthy;
    _state.time_remaining = 0;
    _state.has_time_remaining = 0;
    _state.is_powering_off = _interim_state.is_powering_off;
    _state.battFC1FanSpeed2 = 100;
    memset(_state.cell_voltages.cells, 0, sizeof(_state.cell_voltages));


    if ((AP_HAL::millis() - _state.temperature_time) > AP_BATT_MONITOR_TIMEOUT) {
        _has_temperature = true;
        _state.temperature_time = AP_HAL::millis();
        // gcs().send_text(MAV_SEVERITY_INFO, "QHFC::read:%d",_state.battFC1FanSpeed2 );
        return; 
    }
    _state.temperature = 10;
    
    //_has_temperature = false;//(AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
}

#endif

