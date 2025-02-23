#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#include <AP_QHFC/AP_QHFC.h>

#define AP_BATTMONITOR_QHFC_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds


class AP_BattMonitor_QHFC : public AP_BattMonitor_Backend
{
public:
    enum BattMonitor_QHFC_Type {
        QHFC_BATTERY_INFO = 0
    };

    /// Constructor
    AP_BattMonitor_QHFC(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_QHFC_Type type, AP_BattMonitor_Params &params);

    static const struct AP_Param::GroupInfo var_info[];

    void init() override;

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    bool has_temperature() const override { return _has_temperature; }

    bool has_current() const override { return true; }

    // return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
    uint32_t get_mavlink_fault_bitmask() const override;//override BattBackend

    AP_BattMonitor::Failsafe update_failsafes() override;

    /// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
    // bool capacity_remaining_pct(uint8_t &percentage) const override;
    friend class AP_QHFC;
private:
    AP_BattMonitor::BattMonitor_State _interim_state;
    BattMonitor_QHFC_Type _type;
    
    bool _has_temperature;
    AP_Float _kpa;                 // scaling multiplier applied to current reports for adjustment
};
