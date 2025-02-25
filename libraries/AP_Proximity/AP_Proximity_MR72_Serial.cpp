/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIGHTWARE_MR72_ENABLED

#include "AP_Proximity_MR72_Serial.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>   
extern const AP_HAL::HAL& hal;

#define PROXIMITY_MR72_HEADER_L                  0x54
#define PROXIMITY_MR72_HEADER_H                  0x48
#define PROXIMITY_MR72_DESIRED_OUTPUT_RATE     3
    


void AP_Proximity_MR72_Serial::MR72_ReceiveDataAnl(uint8_t *data_buffer, uint8_t datalen)
{
    uint8_t crc8;
    /* CRC8校验值计算 */
    crc8 = crc_crc8(data_buffer,datalen-1);
    /* 接收crc8校验值判断 */
    if ( crc8 != data_buffer[datalen - 1] )
    {
        gcs().send_text(MAV_SEVERITY_DEBUG, "crc error");
        return;
    }
    sector1 = (data_buffer[16] << 8) | data_buffer[17];
    sector2 = (data_buffer[2] << 8) | data_buffer[3];
    sector3 = (data_buffer[4] << 8) | data_buffer[5];
    _last_distance_received_ms = AP_HAL::millis();

    if(sector1 < sector2)
    {
        _face_distance = sector1;
        _face_yaw_deg = 1;
        if(sector1 > sector3)
        {
            _face_distance = sector3;
            _face_yaw_deg = 2;
        }
    }else
    {
        _face_distance = sector2;
        _face_yaw_deg = 1;
        if(sector2 > sector3)
        {
            _face_distance = sector3;
            _face_yaw_deg = 2;
        }
    }
    if(sector1 != 0xFFFF || sector2 != 0XFFFF || sector3 != 0XFFFF)
    {
        _face_distance_valid = true;
    }
    gcs().send_text(MAV_SEVERITY_DEBUG, "sector1:%dsector2:%d sector3:%d",sector1,sector2,sector3);
}
  
////////////////////////////////////////////////////////////////////////////////////////
// update the state of the sensor
void AP_Proximity_MR72_Serial::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // initialise sensor if necessary
    initialise();

    // process incoming messages
    process_replies();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || ((AP_HAL::millis() - _last_distance_received_ms) > PROXIMITY_MR72_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }

    process_message();
}

// initialise sensor
void AP_Proximity_MR72_Serial::initialise()
{

}

// restart sensor and re-init our state
void AP_Proximity_MR72_Serial::restart_sensor()
{

}

// check for replies from sensor
void AP_Proximity_MR72_Serial::process_replies()
{
    if (_uart == nullptr) {
        return;
    }
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        const int16_t r = _uart->read();
        if ((r < 0) || (r > 0xFF)) {
            continue;
        }
        parse_byte((uint8_t)r);
    }
}

// process one byte received on serial port
// state is stored in _msg structure
void AP_Proximity_MR72_Serial::parse_byte(uint8_t b)
{
    // check that payload buffer is large enough
    static_assert(ARRAY_SIZE(_msg.payload) == PROXIMITY_MR72_PAYLOAD_LEN_MAX, "AP_Proximity_MR72_Serial: check _msg.payload array size");
    
    // process byte depending upon current state
    switch (_msg.state) {

    case ParseState::HEADER_L:
        _msg.payload_recv = 0;
        if (b == PROXIMITY_MR72_HEADER_L) 
        {
            _msg.payload[0] = b;
            _msg.state = ParseState::HEADER_H;
        }
        break;
    case ParseState::HEADER_H:
        if (b == PROXIMITY_MR72_HEADER_H) 
        {
            _msg.payload[1] = b;
            _msg.state = ParseState::CRC;
        }else
        {
            _msg.state = ParseState::HEADER_L;
        }
        break;
    case ParseState::CRC:
        _msg.payload[2 + _msg.payload_recv] = b;
        _msg.payload_recv ++;                            //数据包的位置自增
        if ((PROXIMITY_MR72_LEN) == _msg.payload_recv )        //如果收满4个数据
        {
            // gcs().send_text(MAV_SEVERITY_DEBUG, "step:%d",_step);
            MR72_ReceiveDataAnl( _msg.payload, PROXIMITY_MR72_LEN);  //数据校验及解析
            _msg.state = ParseState::HEADER_H;
        }
        break;
    }
}

// process the latest message held in the _msg structure
void AP_Proximity_MR72_Serial::process_message()
{
    // const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(_face_yaw_deg);

    // // if point is on a new face then finish off previous face
    // if (face != _face) {
    //     // update boundary used for avoidance
        if (_face_distance_valid) {
            frontend.boundary.set_face_attributes(_face, _face_yaw_deg, _face_distance, state.instance);
            database_push(_face_yaw_deg,_face_distance);
        } else {
            // mark previous face invalid
            frontend.boundary.reset_face(_face, state.instance);
        }
        // init for new face
        // _face = face;
        _face_distance_valid = false;
    // }
}

// convert buffer to uint32, uint16
uint32_t AP_Proximity_MR72_Serial::buff_to_uint32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) const
{
    uint32_t leval = (uint32_t)b0 | (uint32_t)b1 << 8 | (uint32_t)b2 << 16 | (uint32_t)b3 << 24;
    return leval;
}

uint16_t AP_Proximity_MR72_Serial::buff_to_uint16(uint8_t b0, uint8_t b1) const
{
    uint16_t leval = (uint16_t)b0 | (uint16_t)b1 << 8;
    return leval;
}

#endif // AP_PROXIMITY_LIGHTWARE_MR72_ENABLED
